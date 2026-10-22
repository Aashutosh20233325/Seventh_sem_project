#!/usr/bin/env python3
"""Run exactly one supervised target-reaching episode with the observation-only baseline controller.

SITL use (only after the runbook gates pass; see runs/sar_demo_20261018/RUNBOOK_ISOLATED.md):
    ROS_DOMAIN_ID=43 .venv/bin/python -u run_target_reaching_episode.py --log <run>/logs/episode.jsonl \\
        --i-confirm-supervised-flight

Offline dry run against the kinematic stand-in (no ROS, no flight):
    .venv/bin/python run_target_reaching_episode.py --standin --log /tmp/episode.jsonl

Per step it logs wall UTC and monotonic time, the simulation step index, the action, the observation's
target fields, the reward, and info. Privileged values appear only under "privileged" (evaluation only;
the controller never sees them). There is never an automatic retry; close() keeps the committed landing
gates (AUTO_LAND stream release, fresh landed-and-disarmed confirmation).

SITL runs add a runtime guard (RuntimeGuard), which can only stop an episode early, never extend one:
- Before reset/take-off it refuses to start unless MemAvailable >= 1.2 GiB and a full 25 s simulation-
  speed window is >= 0.35x ("guard_refuse").
- Before every step it requires MemAvailable >= 600 MiB and a 25 s simulation speed >= 0.35x, taken as the
  lower of the PX4 local-position clock and the camera image-stamp clock ("guard_abort").
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
from baseline_controller import BaselineController

TARGET_FIELDS = ("det_now", "det_bearing_sin", "det_bearing_cos", "det_range_valid", "det_range",
                 "est_valid", "est_bearing_sin", "est_bearing_cos", "est_range", "est_age", "height")


SPEED_FLOOR = 0.35              # x real time, same as the runbook readiness gate
SPEED_WINDOW_S = 25.0           # s wall, same as the pre-flight stability windows
MEM_START_MIB = 1.2 * 1024      # MemAvailable required before reset/take-off
MEM_FLOOR_MIB = 600.0           # MemAvailable required before every step
PX4_MAX_AGE_S = 1.0             # PX4UAVEnv.TAKEOFF_TELEMETRY_MAX_AGE
CAMERA_MAX_AGE_S = 1.5          # PX4UAVEnv.PERCEPTION_MAX_AGE
MAX_SAMPLE_GAP_S = 5.0          # a longer telemetry gap inside the window invalidates it
SAMPLE_PERIOD_S = 1.0
LOG_SAMPLE_PERIOD_S = 5.0
PREFLIGHT_WAIT_S = 60.0         # wall time allowed for the first full speed window

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
    """Latest (sim_seconds, monotonic_received_at) per clock from PX4UAVEnv's node, or None if absent."""
    node = env.node

    def read():
        with node.condition:
            px4 = None
            if node.sim_clock_us is not None and node.position_received_at is not None:
                px4 = (node.sim_clock_us / 1_000_000, node.position_received_at)
            camera = None
            perception = node.latest_perception
            if perception is not None and node.perception_received_at is not None:
                camera = (float(perception["image_stamp"]), node.perception_received_at)
        return {"px4": px4, "camera": camera}

    return read


class RuntimeGuard:
    """Rolling simulation-speed and memory guard; see the module docstring.

    Each stored sample pairs a clock's simulation time with the monotonic time that message was received,
    so a stale value never stands for a later instant. A rate needs a fresh latest sample, a start sample at
    least SPEED_WINDOW_S older, and no gap above MAX_SAMPLE_GAP_S between them.
    """

    STREAMS = {"px4": PX4_MAX_AGE_S, "camera": CAMERA_MAX_AGE_S}

    def __init__(self, telemetry, memory=mem_available_mib, clock=time.monotonic):
        self.telemetry = telemetry
        self.memory = memory
        self.clock = clock
        self.samples = {name: deque() for name in self.STREAMS}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def sample(self):
        values = self.telemetry()
        with self._lock:
            for name in self.STREAMS:
                value = values.get(name)
                if value is None or value[0] is None or value[1] is None:
                    continue
                sim_s, received = float(value[0]), float(value[1])
                history = self.samples[name]
                if history and received <= history[-1][0]:
                    continue                                   # no new message since the last sample
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

    def check(self, memory_floor_mib=MEM_FLOOR_MIB):
        self.sample()
        now = self.clock()
        memory = self.memory()
        with self._lock:
            px4_rate, px4_status = self._rate("px4", now)
            camera_rate, camera_status = self._rate("camera", now)
        verdict = {
            "ok": False, "reason": None,
            "px4_rate": px4_rate, "px4_status": px4_status,
            "camera_rate": camera_rate, "camera_status": camera_status, "speed": None,
            "mem_available_mib": memory,
            "thresholds": {"speed_floor": SPEED_FLOOR, "speed_window_s": SPEED_WINDOW_S,
                           "mem_floor_mib": memory_floor_mib, "px4_max_age_s": PX4_MAX_AGE_S,
                           "camera_max_age_s": CAMERA_MAX_AGE_S, "max_sample_gap_s": MAX_SAMPLE_GAP_S},
        }
        if px4_status == "ok" and camera_status == "ok":
            verdict["speed"] = min(px4_rate, camera_rate)
        if memory is None:
            verdict["reason"] = "memory_unavailable"
        elif memory < memory_floor_mib:
            verdict["reason"] = "memory_below_floor"
        elif px4_status != "ok":
            verdict["reason"] = f"px4_clock_{px4_status}"
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
            history_pending = verdict["reason"] in ("px4_clock_missing", "camera_clock_missing",
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


def run(env, log, max_steps, guard=None):
    controller = BaselineController()
    if guard is not None:
        verdict = guard.preflight()
        record(log, "guard_preflight", guard=verdict)
        if not verdict["ok"]:
            record(log, "guard_refuse", reason=verdict["reason"], guard=verdict)
            record(log, "episode_end", reason="guard_refuse")
            return "guard_refuse"
    record(log, "reset_start")
    obs, info = env.reset()
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
        record(log, "step", step=step, action=action, reward=reward, terminated=terminated,
               truncated=truncated, fields=obs_fields(obs), min_lidar=float(np.min(obs[so.SLICES["lidar"]]) * 30),
               info=info)
        if terminated or truncated:
            result = info.get("reason")
            break
    record(log, "episode_end", reason=result)
    return result


def execute(env, log, max_steps, guard=None):
    """One episode, then always env.close() (landing gates) and stop the guard. Never retries."""
    try:
        if guard is not None:
            guard.start(log)
        return run(env, log, max_steps, guard)
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
    args = p.parse_args(argv)
    if not args.standin and not args.i_confirm_supervised_flight:
        p.error("SITL flight requires --i-confirm-supervised-flight after the runbook gates pass")

    with args.log.open("x") as log:                 # never overwrite an existing log
        if args.standin:
            from sar_standin_env import SarStandInEnv
            env, guard = SarStandInEnv(), None      # offline: no simulator clock or memory to guard
        else:
            from px4_uav_env import PX4UAVEnv
            env = PX4UAVEnv()
            guard = RuntimeGuard(px4_telemetry(env))
        reason = execute(env, log, args.max_steps, guard)
    print(f"EPISODE RESULT: {reason}", flush=True)
    return 0 if reason == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
