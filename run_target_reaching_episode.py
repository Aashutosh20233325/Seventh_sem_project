#!/usr/bin/env python3
"""Run exactly one supervised target-reaching episode with the observation-only baseline controller, or with
a saved PPO checkpoint (--policy) behind the LiDAR safety filter.

SITL use (only after the runbook gates pass; see runs/sar_demo_20261018/RUNBOOK_ISOLATED.md):
    ROS_DOMAIN_ID=43 .venv/bin/python -u run_target_reaching_episode.py --log <run>/logs/episode.jsonl \\
        --i-confirm-supervised-flight

Offline dry run against the kinematic stand-in (no ROS, no flight):
    .venv/bin/python run_target_reaching_episode.py --standin --log /tmp/episode.jsonl

With --policy, the policy is loaded and checked against sar_obs_v1 and the env's action space before any
environment exists; its path and sha256 are logged. A ``.npz`` (numpy_policy.export_policy) is evaluated with
numpy alone and never imports torch: the flight choice, about 300 MiB lighter. A ``.zip`` SB3 checkpoint is run
with stable-baselines3 (CPU, deterministic actions). Every policy action passes
through safety_layer.LidarSafetyFilter, which cannot be disabled here: it only reduces speed or replaces the
turn, and LiDAR of unknown or stale age (PX4UAVEnv.observation_age(), wall time of the sectors' receipt), a
malformed policy output, or invalid LiDAR commands a hover. Each step logs the original and applied action and
the filter's reason. The stand-in computes LiDAR fresh every step, so it uses age 0, as in the evaluations.

--start-yaw-offset-deg turns the start heading from the nominal east heading (NED degrees, positive = clockwise),
passed to reset(options={"start_yaw_offset": rad}); both envs validate it (sar_task.start_yaw_offset) before
anything is commanded. Without it the start is unchanged.

Per step it logs wall UTC and monotonic time, the simulation step index, the action, the observation's
target fields, the reward, and info. Privileged values appear only under "privileged" (evaluation only;
the controller never sees them). There is never an automatic retry; close() keeps the committed landing
gates (AUTO_LAND stream release, fresh landed-and-disarmed confirmation).

SITL runs add a runtime guard (RuntimeGuard), which can only stop an episode early, never extend one:
- Before reset/take-off it refuses to start unless MemAvailable >= 1.2 GiB and a full 25 s simulation-
  speed window is >= 0.35x ("guard_refuse").
- Before every step it requires MemAvailable >= 600 MiB and a 25 s simulation speed >= 0.35x, taken as the
  lower of the PX4 HRT clock (TimesyncStatus timestamp + estimated_offset, ~1 sample per simulated second)
  and the camera image-stamp clock ("guard_abort"). PX4 local-position stamps are not used for speed: PX4
  slews them toward wall time after each DDS timesync reset.
- PX4 liveness is checked separately: local-position receipt must be <= 1.0 s old. The HRT clock is stale
  after MAX_SAMPLE_GAP_S without progress, and "stalled" if the camera clock advanced >= 2 simulated
  seconds since the last HRT progress (the simulation runs but PX4 time does not). A slow simulation
  advances both clocks and is judged only by the 0.35x rate.
- Missing, stale, or gapped telemetry, or unreadable memory, counts as a failure, not as healthy.
- An abort stops sending actions and then calls env.close(), the unchanged landing path: zero action, one
  NAV_LAND, wait for fresh landed-and-disarmed, ROS kept alive until then.
- It does not act inside reset() (take-off, fly-to-start) or inside the return-to-start of a successful
  step; those keep their own bounded timeouts. Background "guard_sample" records log the measurements there.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

import sar_observation as so
import sar_task
from baseline_controller import BaselineController
from safety_layer import LidarSafetyFilter

TARGET_FIELDS = ("det_now", "det_bearing_sin", "det_bearing_cos", "det_range_valid", "det_range",
                 "est_valid", "est_bearing_sin", "est_bearing_cos", "est_range", "est_age", "height")


SPEED_FLOOR = 0.35              # x real time, same as the runbook readiness gate
SPEED_WINDOW_S = 25.0           # s wall, same as the pre-flight stability windows
MEM_START_MIB = 1.2 * 1024      # MemAvailable required before reset/take-off
MEM_FLOOR_MIB = 600.0           # MemAvailable required before every step
PX4_MAX_AGE_S = 1.0             # PX4UAVEnv.TAKEOFF_TELEMETRY_MAX_AGE; PX4 link (local-position receipt) freshness
CAMERA_MAX_AGE_S = 1.5          # PX4UAVEnv.PERCEPTION_MAX_AGE
MAX_SAMPLE_GAP_S = 5.0          # a longer telemetry gap inside the window invalidates it
# PX4 HRT updates about once per simulated second (~2.9 s wall at 0.35x). Its progress may be no older than
# the window's own gap limit; a longer silence fails the window anyway.
PX4_HRT_MAX_AGE_S = MAX_SAMPLE_GAP_S
# Camera-clock advance (simulated s) since the last PX4 HRT progress that means PX4 time has stopped while the
# simulation runs: two nominal HRT periods. Independent of simulation speed.
PX4_HRT_STALL_CAMERA_S = 2.0
SAMPLE_PERIOD_S = 1.0
LOG_SAMPLE_PERIOD_S = 5.0
PREFLIGHT_WAIT_S = 60.0         # wall time allowed for the first full speed window
POLICY_TORCH_THREADS = 1        # policy inference only; leave the CPU to the simulator

_log_lock = threading.Lock()


def mem_available_mib(path="/proc/meminfo"):
    """MemAvailable in MiB, or None if it cannot be read."""
    try:
        with open(path) as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024.0
    except (OSError, ValueError, IndexError):
        return None
    return None


def px4_telemetry(env):
    """Latest clocks from PX4UAVEnv's node, each (sim_seconds, monotonic_received_at) or None if absent:
    "px4" = PX4 HRT (reconstructed from TimesyncStatus), "camera" = perception image stamp. "px4_link" is the
    monotonic receipt time of the latest PX4 local position (liveness only; its stamps are slewed)."""
    node = env.node

    def read():
        with node.condition:
            px4 = None
            if node.px4_sim_time_us is not None and node.px4_sim_time_received_at is not None:
                px4 = (node.px4_sim_time_us / 1_000_000, node.px4_sim_time_received_at)
            camera = None
            perception = node.latest_perception
            if perception is not None and node.perception_received_at is not None:
                camera = (float(perception["image_stamp"]), node.perception_received_at)
            link = node.position_received_at
        return {"px4": px4, "camera": camera, "px4_link": link}

    return read


class RuntimeGuard:
    """Rolling simulation-speed and memory guard; see the module docstring.

    Each stored sample pairs a clock's simulation time with the monotonic time that message was received,
    so a stale value never stands for a later instant. A rate needs a fresh latest sample, a start sample at
    least SPEED_WINDOW_S older, and no gap above MAX_SAMPLE_GAP_S between them.
    """

    STREAMS = {"px4": PX4_HRT_MAX_AGE_S, "camera": CAMERA_MAX_AGE_S}

    def __init__(self, telemetry, memory=mem_available_mib, clock=time.monotonic):
        self.telemetry = telemetry
        self.memory = memory
        self.clock = clock
        self.samples = {name: deque() for name in self.STREAMS}
        self.px4_link_at = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def sample(self):
        values = self.telemetry()
        with self._lock:
            link = values.get("px4_link")
            if link is not None and (self.px4_link_at is None or link > self.px4_link_at):
                self.px4_link_at = float(link)
            for name in self.STREAMS:
                value = values.get(name)
                if value is None or value[0] is None or value[1] is None:
                    continue
                sim_s, received = float(value[0]), float(value[1])
                history = self.samples[name]
                if history and received <= history[-1][0]:
                    continue                                   # no new message since the last sample
                if history and sim_s == history[-1][1]:
                    continue                                   # duplicate: no progress, freshness not refreshed
                if history and sim_s < history[-1][1]:
                    history.clear()                            # clock went backwards: start over
                history.append((received, sim_s))
                while len(history) > 1 and history[1][0] <= received - SPEED_WINDOW_S - MAX_SAMPLE_GAP_S:
                    history.popleft()

    def _rate(self, name, now):
        history = list(self.samples[name])
        if not history:
            return None, "missing"
        latest_wall, latest_sim = history[-1]
        if now - latest_wall > self.STREAMS[name]:
            return None, "stale"
        start = None
        for i in range(len(history) - 1, -1, -1):
            if history[i][0] <= latest_wall - SPEED_WINDOW_S:
                start = i
                break
        if start is None:
            return None, "insufficient_history"
        window = history[start:]
        if max(b[0] - a[0] for a, b in zip(window, window[1:])) > MAX_SAMPLE_GAP_S:
            return None, "gap"
        wall0, sim0 = window[0]
        return (latest_sim - sim0) / (latest_wall - wall0), "ok"

    def _px4_link_status(self, now):
        if self.px4_link_at is None:
            return "missing"
        return "stale" if now - self.px4_link_at > PX4_MAX_AGE_S else "ok"

    def _px4_clock_stalled(self):
        """True if the camera clock advanced >= PX4_HRT_STALL_CAMERA_S since the last PX4 HRT progress.

        The camera reference at that instant is the first camera sample received at or after it, which can
        only understate the advance (no false stall from sampling granularity)."""
        px4, camera = self.samples["px4"], self.samples["camera"]
        if not px4 or not camera:
            return False
        last_px4_at = px4[-1][0]
        after = [sim for received, sim in camera if received >= last_px4_at]
        return bool(after) and after[-1] - after[0] >= PX4_HRT_STALL_CAMERA_S

    def check(self, memory_floor_mib=MEM_FLOOR_MIB):
        self.sample()
        now = self.clock()
        memory = self.memory()
        with self._lock:
            link_status = self._px4_link_status(now)
            px4_rate, px4_status = self._rate("px4", now)
            camera_rate, camera_status = self._rate("camera", now)
            stalled = self._px4_clock_stalled()
        verdict = {
            "ok": False, "reason": None,
            "px4_rate": px4_rate, "px4_status": px4_status, "px4_link_status": link_status,
            "camera_rate": camera_rate, "camera_status": camera_status, "speed": None,
            "mem_available_mib": memory,
            "thresholds": {"speed_floor": SPEED_FLOOR, "speed_window_s": SPEED_WINDOW_S,
                           "mem_floor_mib": memory_floor_mib, "px4_max_age_s": PX4_MAX_AGE_S,
                           "camera_max_age_s": CAMERA_MAX_AGE_S, "max_sample_gap_s": MAX_SAMPLE_GAP_S,
                           "px4_hrt_max_age_s": PX4_HRT_MAX_AGE_S, "px4_hrt_stall_camera_s": PX4_HRT_STALL_CAMERA_S},
        }
        if link_status == "ok" and px4_status == "ok" and camera_status == "ok":
            verdict["speed"] = min(px4_rate, camera_rate)
        if memory is None:
            verdict["reason"] = "memory_unavailable"
        elif memory < memory_floor_mib:
            verdict["reason"] = "memory_below_floor"
        elif link_status != "ok":
            verdict["reason"] = f"px4_telemetry_{link_status}"
        elif px4_status != "ok":
            verdict["reason"] = f"px4_clock_{px4_status}"
        elif stalled:
            verdict["reason"] = "px4_clock_stalled"
        elif camera_status != "ok":
            verdict["reason"] = f"camera_clock_{camera_status}"
        elif verdict["speed"] < SPEED_FLOOR:
            verdict["reason"] = "speed_below_floor"
        else:
            verdict["ok"] = True
        return verdict

    def preflight(self, wait_s=PREFLIGHT_WAIT_S, sleep=time.sleep):
        """Start gate: memory first, then wait (bounded) for the first full speed window."""
        deadline = self.clock() + wait_s
        while True:
            verdict = self.check(memory_floor_mib=MEM_START_MIB)
            # Only "not yet enough data" waits; any other failure refuses at once. Waiting never passes
            # by itself: the loop ends in a refusal at the deadline.
            history_pending = verdict["reason"] in ("px4_telemetry_missing", "px4_clock_missing", "camera_clock_missing",
                                                    "px4_clock_insufficient_history",
                                                    "camera_clock_insufficient_history")
            if verdict["ok"] or not history_pending or self.clock() >= deadline:
                return verdict
            sleep(SAMPLE_PERIOD_S)

    def start(self, log=None):
        """Background sampling (read-only; it never commands the vehicle)."""
        def loop():
            next_log = self.clock()
            while not self._stop.wait(SAMPLE_PERIOD_S):
                try:
                    if log is not None and self.clock() >= next_log:
                        record(log, "guard_sample", guard=self.check())
                        next_log = self.clock() + LOG_SAMPLE_PERIOD_S
                    else:
                        self.sample()
                except Exception as exc:                    # sampling must never take the runner down
                    print(f"guard sampling error: {exc}", file=sys.stderr, flush=True)
        self._thread = threading.Thread(target=loop, name="runtime_guard", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)


def jsonable(value):
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [jsonable(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(value) else str(value)
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def record(log, event, **fields):
    line = {"event": event, "wall_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "mono": time.monotonic(), **fields}
    with _log_lock:
        log.write(json.dumps(jsonable(line)) + "\n")
        log.flush()
        print(json.dumps(jsonable({k: v for k, v in line.items() if k != "observation"})), flush=True)


def obs_fields(obs):
    return {name: float(obs[so.SLICES[name]][0]) for name in TARGET_FIELDS}


def check_policy_spaces(observation_space, action_space):
    """Raise ValueError unless a policy's spaces are exactly sar_obs_v1 and the env's [speed, turn] action."""
    problems = []
    if tuple(observation_space.shape) != (so.DIM,):
        problems.append(f"observation shape {tuple(observation_space.shape)} != ({so.DIM},)")
    elif not (np.array_equal(observation_space.low, so.LOW) and np.array_equal(observation_space.high, so.HIGH)):
        problems.append("observation bounds differ from sar_obs_v1")
    if tuple(action_space.shape) != (2,) or not (np.array_equal(action_space.low, [0.0, -1.0])
                                                and np.array_equal(action_space.high, [1.0, 1.0])):
        problems.append("action space is not [speed 0..1, turn -1..1]")
    if problems:
        raise ValueError("policy does not match the env interface: " + "; ".join(problems))


def load_policy(path):
    """predict(obs) -> action for a numpy_policy .npz or a stable-baselines3 PPO .zip (CPU, deterministic),
    checked first."""
    if Path(path).suffix == ".npz":
        from numpy_policy import NumpyPolicy

        policy = NumpyPolicy.load(path)
        check_policy_spaces(policy.observation_space, policy.action_space)
        return policy.predict
    import torch
    from stable_baselines3 import PPO

    torch.set_num_threads(POLICY_TORCH_THREADS)
    model = PPO.load(str(path), device="cpu")
    check_policy_spaces(model.observation_space, model.action_space)
    return lambda obs: model.predict(obs, deterministic=True)[0]


def file_sha256(path):
    import hashlib

    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


class PolicyController:
    """A policy's actions through the LiDAR safety filter; `last` holds the latest decision for the step log."""

    def __init__(self, predict, lidar_age, safety_filter=None):
        self.predict = predict
        self.lidar_age = lidar_age
        self.safety_filter = safety_filter if safety_filter is not None else LidarSafetyFilter()
        self.last = None

    def act(self, obs):
        age = self.lidar_age()
        raw = np.asarray(self.predict(obs), dtype=np.float32)
        if raw.shape != (2,) or not np.all(np.isfinite(raw)):
            # A malformed output is never clipped into a command: hover, as for invalid LiDAR.
            self.last = {"lidar_age_s": age, "original": jsonable(raw.reshape(-1)), "reason": "policy_invalid",
                         "intervened": True, "front_clear_m": None, "min_clear_m": None}
            return np.zeros(2, dtype=np.float32)
        action, info = self.safety_filter.filter(obs, raw, age)
        self.last = {"lidar_age_s": age, **info}
        return action


def run(env, log, max_steps, guard=None, controller=None, reset_options=None):
    if controller is None:
        controller = BaselineController()
    if guard is not None:
        verdict = guard.preflight()
        record(log, "guard_preflight", guard=verdict)
        if not verdict["ok"]:
            record(log, "guard_refuse", reason=verdict["reason"], guard=verdict)
            record(log, "episode_end", reason="guard_refuse")
            return "guard_refuse"
    record(log, "reset_start", options=reset_options)
    obs, info = env.reset(options=reset_options) if reset_options else env.reset()
    record(log, "reset_done", fields=obs_fields(obs), info=info)
    result = None
    for step in range(max_steps):
        if guard is not None:
            verdict = guard.check()
            if not verdict["ok"]:
                # Stop commanding; the caller's env.close() performs the unchanged landing.
                record(log, "guard_abort", step=step, reason=verdict["reason"], guard=verdict)
                result = "guard_abort"
                break
        action = controller.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        policy = {"policy": controller.last} if isinstance(controller, PolicyController) else {}
        record(log, "step", step=step, action=action, reward=reward, terminated=terminated,
               truncated=truncated, fields=obs_fields(obs), min_lidar=float(np.min(obs[so.SLICES["lidar"]]) * 30),
               info=info, **policy)
        if terminated or truncated:
            result = info.get("reason")
            break
    record(log, "episode_end", reason=result)
    return result


def execute(env, log, max_steps, guard=None, controller=None, reset_options=None):
    """One episode, then always env.close() (landing gates) and stop the guard. Never retries."""
    try:
        if guard is not None:
            guard.start(log)
        return run(env, log, max_steps, guard, controller, reset_options)
    finally:
        record(log, "close_start")
        if hasattr(env, "close"):
            env.close()
        record(log, "close_returned")
        if guard is not None:
            guard.stop()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--standin", action="store_true", help="offline kinematic stand-in instead of PX4 SITL")
    p.add_argument("--i-confirm-supervised-flight", action="store_true",
                   help="required for SITL: confirms the runbook preflight gates passed and a human is supervising")
    p.add_argument("--max-steps", type=int, default=600)
    p.add_argument("--start-yaw-offset-deg", type=float, default=None,
                   help="turn the start heading by this many degrees from the nominal east heading (NED, positive = "
                        "clockwise/right, within +-180); default: nominal start")
    p.add_argument("--terrain-following", action="store_true",
                   help="SITL only: hold the take-off height above the current ground (downward range, /uav/agl_range)")
    p.add_argument("--policy", type=Path,
                   help="policy behind the always-on LiDAR safety filter: numpy_policy .npz (torch-free, for flight) "
                        "or SB3 PPO .zip (default: baseline controller)")
    args = p.parse_args(argv)
    if not args.standin and not args.i_confirm_supervised_flight:
        p.error("SITL flight requires --i-confirm-supervised-flight after the runbook gates pass")
    reset_options = None
    if args.start_yaw_offset_deg is not None:
        if not (math.isfinite(args.start_yaw_offset_deg) and abs(args.start_yaw_offset_deg) <= 180.0):
            p.error("--start-yaw-offset-deg must be finite and within +-180")
        reset_options = {"start_yaw_offset": math.radians(args.start_yaw_offset_deg)}

    with args.log.open("x") as log:                 # never overwrite an existing log
        predict = None
        if args.policy is not None:                 # before any environment exists: a bad checkpoint never flies
            try:
                predict = load_policy(args.policy)
            except Exception as exc:
                record(log, "policy_load_failed", path=str(args.policy), error=repr(exc))
                raise
            f = LidarSafetyFilter()
            numpy_meta = {}
            if args.policy.suffix == ".npz":
                from numpy_policy import NumpyPolicy
                meta = NumpyPolicy.load(args.policy).meta
                numpy_meta = {"source": meta["source"], "source_sha256": meta["source_sha256"],
                              "layer_shapes": meta["layer_shapes"], "activations": meta["activations"]}
            record(log, "policy_loaded", path=str(args.policy.resolve()), sha256=file_sha256(args.policy),
                   format="numpy" if numpy_meta else "sb3", numpy_export=numpy_meta or None,
                   torch_loaded="torch" in sys.modules,
                   torch_threads=None if numpy_meta else POLICY_TORCH_THREADS, mem_available_mib=mem_available_mib(),
                   safety_filter={"stop_clearance_m": f.stop_clearance, "slow_clearance_m": f.slow_clearance,
                                  "emergency_clearance_m": f.emergency_clearance, "max_age_s": f.max_age_s})
        world = sar_task.selected_world_sdf()       # Stage C: SAR_WORLD_VARIANT, unset = default world
        target = sar_task.load_target(world)
        record(log, "world", variant=sar_task.selected_world_variant(), path=str(world),
               sha256=sar_task.file_sha256(world), target_ned=[target.x_ned, target.y_ned],
               terrain_following=bool(args.terrain_following) and not args.standin)
        if args.standin:
            from sar_standin_env import SarStandInEnv
            env, guard = SarStandInEnv(), None      # offline: no simulator clock or memory to guard
            lidar_age = lambda: 0.0                 # noqa: E731  (stand-in LiDAR is computed every step)
        else:
            from px4_uav_env import PX4UAVEnv
            env = PX4UAVEnv(terrain_following=args.terrain_following)
            guard = RuntimeGuard(px4_telemetry(env))
            lidar_age = env.observation_age
        controller = None if predict is None else PolicyController(predict, lidar_age)
        reason = execute(env, log, args.max_steps, guard, controller, reset_options)
    print(f"EPISODE RESULT: {reason}", flush=True)
    return 0 if reason == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
