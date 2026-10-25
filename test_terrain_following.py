"""Terrain following, 3D step 2 option A: the env holds the take-off height above the current ground from a downward
single-beam range, relative to the ground under the flat start point. Offline, with the ROS/PX4 shims."""
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_px4_uav_env_safety import load_environment, make_environment


def node(module, z=-1.8, agl=1.85, age=0.05, ref=0.05, hold=-1.8, roll=0.0, pitch=0.0, tf=True):
    n = module.PX4ControllerNode.__new__(module.PX4ControllerNode)
    n.condition = threading.Condition()
    n.z, n.roll, n.pitch = z, roll, pitch
    n.agl_range = agl
    n.agl_received_at = None if age is None else time.monotonic() - age
    n.terrain_ref_ground_z, n._terrain_last_t, n.hold_z, n.terrain_following = ref, None, hold, tf
    return n


def run(n, seconds, dt=0.05):
    t0 = time.monotonic()
    for k in range(int(seconds / dt)):
        n.agl_received_at = t0 + k * dt
        n.hold_z = n.terrain_hold_z(t0 + k * dt)
    return n.hold_z


def test_ground_z_from_a_fresh_range_with_tilt(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, z=-1.8, agl=2.0, roll=0.1, pitch=-0.2)
    assert n.ground_z(time.monotonic()) == pytest.approx(-1.8 + 2.0 * math.cos(0.1) * math.cos(-0.2))


@pytest.mark.parametrize("kw", [{"age": 0.6}, {"age": None}, {"agl": None}])
def test_stale_missing_or_no_return_gives_no_ground(monkeypatch, kw):
    m = load_environment(monkeypatch)
    assert node(m, **kw).ground_z(time.monotonic()) is None


def test_flat_ground_keeps_exactly_the_takeoff_height(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, z=-1.8, agl=1.85, ref=0.05)                       # ground = -1.8 + 1.85 = 0.05 = reference
    assert run(n, 3.0) == pytest.approx(m.PX4ControllerNode.TAKEOFF_TARGET_Z)


def test_a_hill_raises_the_target_by_its_height_at_a_limited_rate(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, z=-1.8, agl=1.40, ref=0.05)                       # ground 0.45 m higher than at the start
    after_half_s = run(n, 0.5)
    assert after_half_s == pytest.approx(-1.8 - 0.5 * 0.5 * (9 / 10), abs=0.03)   # rate limit 0.5 m/s
    assert run(n, 3.0) == pytest.approx(-2.25, abs=1e-6)


def test_the_target_is_clamped_to_one_metre(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, z=-1.8, agl=0.3, ref=0.05)                        # implausible 1.55 m ground step
    assert run(n, 10.0) == pytest.approx(-1.8 - 1.0)
    n = node(m, z=-1.8, agl=3.5, ref=0.05)                        # ground 1.65 m lower: target at most 1 m down
    assert run(n, 10.0) == pytest.approx(-1.8 + 1.0)


@pytest.mark.parametrize("kw", [{"age": 0.6}, {"agl": None}, {"ref": None}])
def test_without_a_fresh_range_or_reference_the_target_is_unchanged(monkeypatch, kw):
    m = load_environment(monkeypatch)
    n = node(m, hold=-2.0, **kw)
    assert n.terrain_hold_z(time.monotonic()) == -2.0


def test_velocity_phase_applies_terrain_following_only_when_enabled(monkeypatch):
    m = load_environment(monkeypatch)
    calls = []
    for tf, expect in ((False, -1.8), (True, None)):
        n = node(m, z=-1.8, agl=1.40, ref=0.05, tf=tf)
        n.phase, n.action = "VELOCITY", np.zeros(2, np.float32)
        n.publish_offboard_mode = lambda **k: None
        n.publish_velocity_setpoint = lambda hold_z=None: calls.append(hold_z)
        n._terrain_last_t = time.monotonic() - 0.1
        m.PX4ControllerNode.control_loop(n)
        if expect is not None:
            assert calls[-1] == expect
        else:
            assert calls[-1] < -1.8                                # moved up toward the hill
    assert len(calls) == 2


def test_enter_velocity_phase_records_the_reference_or_refuses(monkeypatch):
    m = load_environment(monkeypatch)
    env = make_environment(m)
    env.terrain_following = True
    env.node.condition = threading.Condition()
    env.node.z, env.node.roll, env.node.pitch = -1.8, 0.0, 0.0
    env.node.TAKEOFF_TARGET_Z = m.PX4ControllerNode.TAKEOFF_TARGET_Z
    env.node.ground_z = m.PX4ControllerNode.ground_z.__get__(env.node)
    env.node.AGL_MAX_AGE = m.PX4ControllerNode.AGL_MAX_AGE
    env.node.agl_range, env.node.agl_received_at = None, None
    env.node.phase = "REPOSITION"                                  # the phase before the policy phase
    with pytest.raises(RuntimeError, match="fresh downward range"):
        env._enter_velocity_phase()
    assert env.node.phase == "REPOSITION"
    env.node.agl_range, env.node.agl_received_at = 1.85, time.monotonic()
    env._enter_velocity_phase()
    assert env.node.phase == "VELOCITY" and env.node.terrain_ref_ground_z == pytest.approx(0.05)
    env.terrain_following = False                                  # off: never needs the range
    env.node.agl_range = None
    env._enter_velocity_phase()


def test_observation_height_is_height_above_the_current_ground(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, z=-2.25, agl=1.85, ref=0.05)                       # flying 0.45 m higher over a 0.45 m hill
    now = time.monotonic()
    assert m.PX4UAVEnv._observation_z(n, now) == pytest.approx(-1.8)   # as on the start pad
    n.terrain_following = False
    assert m.PX4UAVEnv._observation_z(n, now) == -2.25
    n.terrain_following, n.agl_received_at = True, now - 1.0       # stale: fall back to the EKF z
    assert m.PX4UAVEnv._observation_z(n, now) == -2.25


def test_agl_callback_keeps_the_nearest_valid_return(monkeypatch):
    m = load_environment(monkeypatch)
    n = node(m, agl=None, age=None)
    m.PX4ControllerNode.agl_callback(n, SimpleNamespace(ranges=[float("inf"), 2.3, 0.05], range_min=0.1, range_max=100.0))
    assert n.agl_range == 2.3 and n.agl_received_at is not None
    m.PX4ControllerNode.agl_callback(n, SimpleNamespace(ranges=[float("inf")], range_min=0.1, range_max=100.0))
    assert n.agl_range is None                                     # no return is not a range


def test_runner_flag_is_recorded(tmp_path):
    import json
    import run_target_reaching_episode as runner
    log = tmp_path / "e.jsonl"
    runner.main(["--standin", "--terrain-following", "--log", str(log)])
    w = next(json.loads(l) for l in log.read_text().splitlines() if '"event": "world"' in l)
    assert w["terrain_following"] is False                         # the stand-in has no terrain to follow
