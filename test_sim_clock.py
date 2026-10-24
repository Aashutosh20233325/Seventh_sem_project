"""Simulation-clock regression tests (offline): camera-clock action steps and HRT-timed fly-to.

The recorded fixture (fixtures/clock_slew_20261023) is an excerpt of
runs/sar_demo_20261018/restart_20261023_split2/logs/telemetry_full.jsonl: PX4 local-position stamps
that PX4 slews toward wall time after each timesync reset, TimesyncStatus records (PX4 HRT =
timestamp + estimated_offset) and RGB image header stamps (Gazebo time). The retained position-stamp
clock (sim_clock_us, now only the runtime guard's input) is replayed alongside to document the defect.
"""
import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_px4_uav_env_safety import load_environment, make_environment

ROOT = Path(__file__).resolve().parent
FIXTURE = ROOT / "fixtures/clock_slew_20261023/telemetry_excerpt.jsonl"
FULL_RUN = ROOT / "runs/sar_demo_20261018/restart_20261023_split2/logs/telemetry_full.jsonl"
STEP_US = 100_000


# ---------- helpers ----------

def bare_node(module):
    """A PX4ControllerNode with only the state its clock callbacks use (no ROS)."""
    node = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    node.condition = threading.Condition()
    node.x = node.y = node.z = node.heading = node.vx = node.vy = 0.0
    node.heading_valid = node.position_valid = False
    node.position_received_at = None
    node.sim_clock_us = None
    node._last_position_stamp = None
    node.px4_sim_time_us = None
    node.px4_sim_time_received_at = None
    node.reset_camera_clock()
    return node


def camera_msg(stamp_us):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=stamp_us // 1_000_000, nanosec=(stamp_us % 1_000_000) * 1_000)))


def position_msg(stamp_us):
    return SimpleNamespace(x=0.0, y=0.0, z=-1.8, heading=0.0, vx=0.0, vy=0.0, heading_good_for_control=True,
                           xy_valid=True, z_valid=True, v_xy_valid=True, timestamp=stamp_us)


def load_rows(path):
    rows = []
    with open(path) as f:
        next(f)                                                     # header
        for line in f:
            rows.append(json.loads(line))
    return rows


def full_run_rows():
    """The whole preserved run in fixture row format (skipped if runs/ is absent)."""
    import target_perception as tp
    if not FULL_RUN.is_file():
        pytest.skip(f"{FULL_RUN} not present (runs/ is git-ignored)")
    rows = []
    with open(FULL_RUN) as f:
        for line in f:
            r = json.loads(line)
            if r["event"] == "position":
                rows.append(["p", r["mono"], int(r["msg"]["timestamp"])])
            elif r["event"] == "timesync":
                rows.append(["t", r["mono"], int(r["msg"]["timestamp"]), int(r["msg"]["estimated_offset"])])
            elif r["event"] == "target_perception":
                d = tp.decode(r["msg"]["data"])
                if d:
                    rows.append(["c", r["mono"], float(d["image_stamp"])])
    rows.sort(key=lambda x: x[1])
    return rows


def replay(monkeypatch, rows):
    """Drive the real callbacks in receipt order; return per-event clock samples."""
    module = load_environment(monkeypatch)
    now = [0.0]
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now[0], sleep=time.sleep))
    node = bare_node(module)
    samples = []                                                    # (mono, kind, old_us, camera_us, hrt_us)
    for row in rows:
        now[0] = row[1]
        if row[0] == "p":
            module.PX4ControllerNode.position_callback(node, position_msg(row[2]))
        elif row[0] == "t":
            module.PX4ControllerNode.timesync_status_callback(
                node, SimpleNamespace(timestamp=row[2], estimated_offset=row[3]))
        else:
            module.PX4ControllerNode.camera_info_callback(node, camera_msg(int(round(row[2] * 1e6))))
        samples.append((row[1], row[0], node.sim_clock_us, node.camera_clock_us, node.px4_sim_time_us))
    return module, node, samples


def drift_vs_hrt(samples):
    """Advance of the old and camera clocks between HRT samples, minus the HRT advance (s).

    Only HRT samples received while camera frames were still arriving are used (the preserved run ends
    with PX4 timesync records sent during the gated stop, after the camera stream had stopped).
    """
    last_camera = max(s[0] for s in samples if s[1] == "c")
    hrt = [s for s in samples if s[1] == "t" and s[2] is not None and s[3] is not None and s[0] <= last_camera]
    a, b = hrt[0], hrt[-1]
    true = (b[4] - a[4]) / 1e6
    return true, (b[2] - a[2]) / 1e6 - true, (b[3] - a[3]) / 1e6 - true


def truth_curve(samples, kind):
    """Receipt-time -> simulation-time interpolant from camera ("c") or HRT ("t") samples."""
    idx = 3 if kind == "c" else 4
    pts = [(s[0], s[idx]) for s in samples if s[1] == kind and s[idx] is not None]
    mono, sim = np.array(pts, dtype=float).T
    return lambda t: np.interp(t, mono, sim) / 1e6


def step_lengths(samples, index, truth):
    """Simulate back-to-back 0.1 s steps on clock[index] (each ends when the clock passes start + 0.1 s);
    return each step's length in simulation seconds according to the independent `truth` interpolant."""
    pts = [(s[0], s[index]) for s in samples if s[index] is not None]
    lengths, start = [], None
    for mono, clock in pts:
        if start is None:
            start = (mono, clock)
        elif clock >= start[1] + STEP_US:
            lengths.append(truth(mono) - truth(start[0]))
            start = (mono, clock)
    return np.array(lengths)


# ---------- recorded slew replay: old position-stamp clock vs new camera clock ----------

def test_recorded_slew_old_clock_runs_fast_camera_clock_tracks_px4_hrt(monkeypatch):
    _, node, samples = replay(monkeypatch, load_rows(FIXTURE))
    true, old_drift, camera_drift = drift_vs_hrt(samples)
    assert true > 30.0
    assert old_drift / true > 0.10                       # old: ~+14% (4.6 s over 32.5 s): the defect
    assert abs(camera_drift) <= 0.3                      # new: tracks PX4 HRT
    assert node.camera_clock_invalid is None             # recorded frames never trip the fail-closed rules
    hrt = [s[4] for s in samples if s[1] == "t"]
    assert all(b >= a for a, b in zip(hrt, hrt[1:]))     # reconstructed HRT is monotonic


def test_full_recorded_run_reproduces_the_measured_drift(monkeypatch):
    rows = full_run_rows()
    _, node, samples = replay(monkeypatch, rows)
    first = next(s for s in samples if s[2] is not None and s[3] is not None)
    last = samples[-1]
    # The investigation's figure: old clock vs camera clock over the whole log, 12.1 s over 86 s of camera time
    # (this includes the shutdown tail, when positions kept arriving after the camera stopped).
    old_vs_camera = (last[2] - first[2]) / 1e6 - (last[3] - first[3]) / 1e6
    assert (last[3] - first[3]) / 1e6 == pytest.approx(86.0, abs=0.5)
    assert old_vs_camera == pytest.approx(12.1, abs=0.3)
    # Against PX4 HRT while the camera was live: old ~+11%, camera within 0.5 s.
    true, old_drift, camera_drift = drift_vs_hrt(samples)
    assert old_drift / true > 0.08 and abs(camera_drift) <= 0.5
    assert node.camera_clock_invalid is None


def test_recorded_slew_old_clock_makes_short_steps_camera_clock_does_not(monkeypatch):
    _, _, samples = replay(monkeypatch, load_rows(FIXTURE))
    camera_time = truth_curve(samples, "c")
    old = step_lengths(samples, 2, camera_time)
    new = step_lengths(samples, 3, camera_time)
    assert (old < 0.07).mean() > 0.10                    # old: ~17% of "0.1 s" steps are < 0.07 s
    assert new.min() >= 0.1 - 1e-6                       # new: never shorter than 0.1 s of Gazebo time
    assert np.median(new) < 0.2


def test_slewed_position_stamps_never_advance_the_action_clock(monkeypatch):
    module = load_environment(monkeypatch)
    node = bare_node(module)
    module.PX4ControllerNode.camera_info_callback(node, camera_msg(5_000_000))
    stamp = 1_000_000_000
    for inc in (24_000, 64_000, 69_000, 85_000, 61_000, 75_000, 22_000):   # recorded slew increments
        stamp += inc
        module.PX4ControllerNode.position_callback(node, position_msg(stamp))
    assert node.camera_clock_us == 5_000_000             # action clock untouched
    assert node.sim_clock_us == 400_000 - 24_000         # old clock counted the slew (first sample only anchors)


# ---------- camera-clock state machine ----------

def clock_node(monkeypatch):
    module = load_environment(monkeypatch)
    return module, bare_node(module)


def test_camera_stamp_units_and_steady_advance(monkeypatch):
    module, node = clock_node(monkeypatch)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=345_678_912)))
    module.PX4ControllerNode.camera_info_callback(node, msg)
    assert node.camera_clock_us == 12_345_678 and node.camera_clock_samples == 1
    for k in range(1, 31):
        node._advance_camera_clock(12_345_678 + k * 33_333, 100.0 + k)
    assert node.camera_clock_us == 12_345_678 + 30 * 33_333
    assert node.camera_clock_samples == 31 and node.camera_clock_received_at == 130.0
    assert node.camera_clock_invalid is None


def test_duplicate_frames_do_not_advance_or_refresh_the_clock(monkeypatch):
    _, node = clock_node(monkeypatch)
    node._advance_camera_clock(1_000_000, 10.0)
    node._advance_camera_clock(1_033_000, 11.0)
    node._advance_camera_clock(1_033_000, 12.0)
    assert node.camera_clock_us == 1_033_000 and node.camera_clock_received_at == 11.0
    assert node.camera_clock_samples == 2 and node.camera_clock_invalid is None


def test_backward_stamp_invalidates_until_reset(monkeypatch):
    _, node = clock_node(monkeypatch)
    node._advance_camera_clock(5_000_000, 1.0)
    node._advance_camera_clock(4_999_999, 2.0)
    assert "backwards" in node.camera_clock_invalid
    node._advance_camera_clock(6_000_000, 3.0)           # sticky: later frames are ignored
    assert node.camera_clock_us == 5_000_000 and node.camera_clock_received_at == 1.0
    node.reset_camera_clock()
    assert node.camera_clock_invalid is None and node.camera_clock_us is None
    node._advance_camera_clock(100_000, 4.0)             # a reset simulation starts a new clock
    assert node.camera_clock_us == 100_000


@pytest.mark.parametrize("gap, invalid", [(500_000, False), (500_001, True), (20_000_000, True)])
def test_large_forward_jump_invalidates(monkeypatch, gap, invalid):
    _, node = clock_node(monkeypatch)
    node._advance_camera_clock(1_000_000, 1.0)
    node._advance_camera_clock(1_000_000 + gap, 2.0)
    assert (node.camera_clock_invalid is not None) is invalid
    assert node.camera_clock_us == (1_000_000 if invalid else 1_000_000 + gap)


def test_camera_callback_wakes_waiters(monkeypatch):
    module, node = clock_node(monkeypatch)
    woke = []

    def waiter():
        with node.condition:
            woke.append(node.condition.wait_for(lambda: node.camera_clock_us is not None, timeout=2.0))
    th = threading.Thread(target=waiter)
    th.start()
    time.sleep(0.05)
    module.PX4ControllerNode.camera_info_callback(node, camera_msg(2_000_000))
    th.join(timeout=2.0)
    assert woke == [True]


# ---------- action steps ----------

@pytest.fixture
def real_clock(monkeypatch):
    """The env module; fake nodes made by the helpers below use its real camera-clock update."""
    return load_environment(monkeypatch)


def with_real_clock_update(module, env):
    node = env.node
    node.CAMERA_CLOCK_MAX_FRAME_GAP_US = module.PX4ControllerNode.CAMERA_CLOCK_MAX_FRAME_GAP_US
    node._advance_camera_clock = module.PX4ControllerNode._advance_camera_clock.__get__(node)
    node.reset_camera_clock = module.PX4ControllerNode.reset_camera_clock.__get__(node)
    return env


def step_environment(module):
    env = with_real_clock_update(module, make_environment(module))
    env._wait_for_observation = module.PX4UAVEnv._wait_for_observation.__get__(env)
    env.node.arming_state = 2                                # airborne: the safe state must land
    return env


def deliver(env, stamps_us, period_s=0.033, slew=False):
    """Background camera frames through the real clock update (None = no frame), each with a LiDAR
    observation; with slew=True the retained position-stamp clock races ahead at the same time."""
    node = env.node

    def run():
        for stamp in stamps_us:
            time.sleep(period_s)
            with node.condition:
                if stamp is not None:
                    node._advance_camera_clock(stamp, time.monotonic())
                if slew:
                    node.sim_clock_us += 70_000              # what the old clock would have counted
                node.observation_counter += 1
                node.observation_received_at = time.monotonic()
                node.condition.notify_all()
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


def deliver_after_action(env, first_us, then_us, period_s=0.02):
    """Deterministic frame delivery for one step: `first_us` only after step() has applied the action,
    the rest only after the step has visibly anchored. Returns (thread, log of (stamp, delivered_at))."""
    node = env.node
    log = []

    def frame(stamp):
        with node.condition:
            node._advance_camera_clock(stamp, time.monotonic())
            node.observation_counter += 1
            node.observation_received_at = time.monotonic()
            log.append((stamp, time.monotonic()))
            node.condition.notify_all()

    def wait_for(predicate, limit=1.5):
        deadline = time.monotonic() + limit
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.002)

    def run():
        wait_for(lambda: getattr(env, "_step_applied_at", None) is not None)
        time.sleep(0.01)
        frame(first_us)
        wait_for(lambda: getattr(env, "_step_anchor_us", None) is not None)
        for stamp in then_us:
            time.sleep(period_s)
            frame(stamp)
    env._step_applied_at = None
    env._step_anchor_us = None
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th, log


@pytest.mark.parametrize("problem", ["missing", "invalid", "stale", "never_validated"])
def test_step_refuses_before_applying_the_action_without_a_valid_clock(real_clock, problem):
    env = step_environment(real_clock)
    node = env.node
    if problem == "missing":
        node.camera_clock_us = None
    elif problem == "invalid":
        node.camera_clock_invalid = "camera stamp moved backwards"
    elif problem == "stale":
        node.camera_clock_received_at = time.monotonic() - 5.0
    else:
        node.camera_clock_samples = 0
    with pytest.raises(RuntimeError, match="Cannot time the action step"):
        env.step(np.array([0.6, 0.3], dtype=np.float32))
    np.testing.assert_array_equal(node.action, [0.0, 0.0])   # action never applied
    assert node.land_calls == 1 and not env._episode_active


def test_step_completes_only_after_a_tenth_of_a_second_from_its_own_anchor(real_clock):
    env = step_environment(real_clock)
    node = env.node
    node.camera_clock_us = 1_000_000
    node.camera_clock_received_at = time.monotonic()
    frames = [1_000_000 + k * 33_333 for k in range(1, 9)]       # 30 Hz frames after the action
    th, log = deliver_after_action(env, frames[0], frames[1:])
    env.step(np.array([0.2, 0.0], dtype=np.float32))
    returned_at = time.monotonic()
    th.join(timeout=2.0)
    anchor, completed = env.last_step_clock
    assert anchor == frames[0]                                   # the first frame after the action
    assert completed == next(s for s in frames if s >= anchor + STEP_US)   # first frame >= anchor + 0.1 s
    assert STEP_US <= completed - anchor < STEP_US + 33_333
    delivered = dict(log)
    assert returned_at >= delivered[completed]                   # did not return before that frame arrived
    assert node.land_calls == 0


def test_step_anchors_on_the_first_frame_after_the_action_not_an_old_fresh_one(real_clock):
    """The latest frame is old but still fresh (0.8 s < CAMERA_CLOCK_MAX_AGE) when the action is applied,
    and the next frame shows the simulation 0.4 s further on. The old gate (last stamp + 0.1 s) would end
    the step on that first frame; the step must instead anchor there and wait 0.1 s more camera time."""
    env = step_environment(real_clock)
    node = env.node
    node.camera_clock_us = 1_000_000
    node.camera_clock_received_at = time.monotonic() - 0.8
    assert env._camera_clock_problem(time.monotonic()) is None   # fresh enough to start the step
    th, log = deliver_after_action(env, 1_400_000, [1_433_333, 1_466_666, 1_499_999, 1_500_000, 1_533_333])
    env.step(np.array([0.2, 0.0], dtype=np.float32))
    returned_at = time.monotonic()
    th.join(timeout=2.0)
    assert 1_400_000 >= 1_000_000 + STEP_US                      # the old gate would have completed here
    assert env.last_step_clock == (1_400_000, 1_500_000)         # anchor after the action; +0.1 s exactly
    delivered = dict(log)
    assert delivered[1_400_000] > env._step_applied_at           # the anchor arrived after the action
    assert returned_at >= delivered[1_500_000] > delivered[1_499_999]   # 1.499999 s did not complete it
    assert node.land_calls == 0 and env._episode_active


def test_camera_stall_times_out_on_the_wall_clock_without_advancing_time(real_clock):
    env = step_environment(real_clock)
    node = env.node
    node.camera_clock_us = 1_000_000
    node.camera_clock_received_at = time.monotonic()
    th = deliver(env, [1_033_000] + [None] * 60, period_s=0.033, slew=True)   # one frame, then a stall
    started = time.monotonic()
    with pytest.raises(TimeoutError, match="simulation-time"):
        env.step(np.array([0.2, 0.0], dtype=np.float32))
    th.join(timeout=3.0)
    assert 1.9 <= time.monotonic() - started < 3.0           # existing 2 s wall timeout
    assert node.camera_clock_us == 1_033_000                 # never advanced artificially
    assert node.land_calls == 1 and not env._episode_active


def test_clock_invalidation_mid_step_fails_fast_and_does_not_resume(real_clock):
    env = step_environment(real_clock)
    node = env.node
    node.camera_clock_us = 1_000_000
    node.camera_clock_received_at = time.monotonic()
    th = deliver(env, [1_033_000, 900_000])                  # backward frame mid-step
    started = time.monotonic()
    with pytest.raises(RuntimeError, match="clock invalid"):
        env.step(np.array([0.2, 0.0], dtype=np.float32))
    th.join(timeout=2.0)
    assert time.monotonic() - started < 1.0 and node.land_calls == 1
    with pytest.raises(RuntimeError, match="not active"):
        env.step(np.array([0.2, 0.0], dtype=np.float32))
    assert node.camera_clock_invalid is not None             # still invalid until a new reset()


# ---------- reset ----------

def reset_environment(module):
    env = with_real_clock_update(module, make_environment(module))
    env._episode_active = False
    env._wait_until_landed = lambda timeout: None
    env.CAMERA_CLOCK_WAIT_TIMEOUT = 0.3
    env.takeoffs = []
    env._takeoff = lambda: env.takeoffs.append(time.monotonic())
    env._privileged_info = lambda: {}
    return env


def test_reset_refuses_before_arming_without_a_camera_clock(real_clock):
    env = reset_environment(real_clock)
    with pytest.raises(RuntimeError, match="Cannot take off: simulation clock unavailable"):
        env.reset()
    assert env.takeoffs == [] and env.node.arm_calls == 0 and not env._episode_active


def test_reset_clears_an_invalid_clock_and_requires_fresh_frames(real_clock):
    env = reset_environment(real_clock)
    env.node.camera_clock_invalid = "camera stamp moved backwards"
    th = deliver(env, [7_000_000, 7_033_000, 7_066_000], period_s=0.02)
    env.reset()
    th.join(timeout=2.0)
    assert len(env.takeoffs) == 1 and env.node.camera_clock_invalid is None
    assert env.node.camera_clock_samples >= env.CAMERA_CLOCK_MIN_SAMPLES and env._episode_active


def test_reset_refuses_when_frames_go_backwards_during_validation(real_clock):
    env = reset_environment(real_clock)
    th = deliver(env, [7_000_000, 6_000_000], period_s=0.02)
    with pytest.raises(RuntimeError, match="invalid"):
        env.reset()
    th.join(timeout=2.0)
    assert env.takeoffs == []


# ---------- observation ageing uses Gazebo time ----------

def test_observation_rejects_an_invalid_or_stale_clock(real_clock):
    from test_px4_uav_env_safety import observation_environment
    env = observation_environment(real_clock)
    env.node.camera_clock_invalid = "camera stamp jumped forward"
    with pytest.raises(ValueError, match="simulation clock"):
        env._convert_observation(np.full(49, 7.5, dtype=np.float32))
    env.node.camera_clock_invalid = None
    env.node.camera_clock_received_at = time.monotonic() - 5.0
    with pytest.raises(ValueError, match="stalled"):
        env._convert_observation(np.full(49, 7.5, dtype=np.float32))


def test_target_estimate_ages_in_camera_time_not_position_stamp_time(real_clock):
    from test_px4_uav_env_safety import observation_environment
    env = observation_environment(real_clock)
    node = env.node
    obs = env._convert_observation(np.full(49, 7.5, dtype=np.float32))
    assert obs[__import__("sar_observation").SLICES["est_valid"]][0] == 1.0
    node.perception_received_at = time.monotonic() - 5.0     # no new detection
    node.camera_clock_us += 9_000_000                         # 9 s of Gazebo time later
    node.camera_clock_received_at = time.monotonic()
    node.sim_clock_us += 1_000_000                            # the old clock is irrelevant now
    obs = env._convert_observation(np.full(49, 7.5, dtype=np.float32))
    assert obs[__import__("sar_observation").SLICES["est_valid"]][0] == 0.0   # expired after 8 s


# ---------- fly-to on the PX4 HRT clock ----------

def hrt_feed(env, values_us, period_s=0.02):
    node = env.node

    def run():
        for value in values_us:
            time.sleep(period_s)
            with node.condition:
                node.px4_sim_time_us = value
                node.px4_sim_time_received_at = time.monotonic()
                node.position_received_at = time.monotonic()
                node.status_received_at = time.monotonic()
                node.condition.notify_all()
    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


def fly(module):
    from test_px4_uav_env_safety import fly_environment
    env = fly_environment(module)
    env.node.x = 3.0                                          # never arrives
    env.REPOSITION_SIM_TIMEOUT = 3.0
    env.REPOSITION_WALL_TIMEOUT = 5.0
    env.TAKEOFF_CLOCK_STALL_TIMEOUT = 0.5
    return env


def test_fly_to_timeout_cannot_fire_early_from_a_coarse_or_pre_start_sample(real_clock):
    env = fly(real_clock)
    env.node.px4_sim_time_us = 10_000_000                     # received before fly-to starts: never the anchor
    env.node.px4_sim_time_received_at = time.monotonic() - 0.1
    th = hrt_feed(env, [11_000_000, 12_000_000, 13_000_000, 13_900_000, 14_000_000, 15_000_000])
    with pytest.raises(TimeoutError, match="not reached") as err:
        env._fly_to(0.0, 0.0, -1.8, 1.5707963)
    th.join(timeout=2.0)
    # Anchored on 11.0 s: fires at 14.0 s (3.0 s elapsed), not at 13.0 s (as a pre-start anchor at 10.0 s would).
    assert env.node.px4_sim_time_us >= 14_000_000 and "sim 3.0 s" in str(err.value)


@pytest.mark.parametrize("feed", [[], [11_000_000]])
def test_fly_to_stops_when_the_px4_clock_never_starts_or_freezes(real_clock, feed):
    env = fly(real_clock)
    keep = threading.Event()

    def keep_position_fresh():
        while not keep.is_set():
            time.sleep(0.05)
            with env.node.condition:
                env.node.position_received_at = time.monotonic()
                env.node.status_received_at = time.monotonic()
    t = threading.Thread(target=keep_position_fresh, daemon=True)
    t.start()
    hrt_feed(env, feed)
    started = time.monotonic()
    try:
        with pytest.raises(TimeoutError, match="clock-stall"):
            env._fly_to(0.0, 0.0, -1.8, 1.5707963)
    finally:
        keep.set()
    assert time.monotonic() - started < 2.0                   # no indefinite wait


def test_fly_to_rejects_a_backward_px4_clock(real_clock):
    env = fly(real_clock)
    hrt_feed(env, [11_000_000, 12_000_000, 11_500_000])
    with pytest.raises(RuntimeError, match="backwards"):
        env._fly_to(0.0, 0.0, -1.8, 1.5707963)


def test_fly_to_wall_ceiling_still_bounds_a_slow_simulation(real_clock):
    env = fly(real_clock)
    env.REPOSITION_WALL_TIMEOUT = 1.0
    hrt_feed(env, [11_000_000 + k * 100_000 for k in range(1, 40)], period_s=0.05)   # 2x slower than needed
    started = time.monotonic()
    with pytest.raises(TimeoutError, match="not reached"):
        env._fly_to(0.0, 0.0, -1.8, 1.5707963)
    assert 0.9 <= time.monotonic() - started < 1.6


def test_fly_to_arrival_needs_no_px4_clock_sample(real_clock):
    from test_px4_uav_env_safety import fly_environment
    env = fly_environment(real_clock)
    env.node.px4_sim_time_us = None
    env._fly_to(0.0, 0.0, -1.8, 1.5707963)
    assert env.node.phase == "REPOSITION"


# ---------- compatibility ----------

def test_standin_environment_does_not_use_the_live_clock():
    src = (ROOT / "sar_standin_env.py").read_text()
    assert "px4_uav_env" not in src and "camera_clock" not in src


def test_runtime_guard_reads_px4_hrt_and_position_receipt_not_slewed_stamps():
    """B3: the guard's PX4 speed term is the reconstructed HRT; local position supplies only liveness."""
    src = (ROOT / "run_target_reaching_episode.py").read_text()
    assert "node.px4_sim_time_us / 1_000_000, node.px4_sim_time_received_at" in src
    assert "node.sim_clock_us" not in src
    assert "SPEED_FLOOR = 0.35" in src and "PX4_MAX_AGE_S = 1.0" in src
