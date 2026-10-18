import ast
import math
from pathlib import Path

import numpy as np
import pytest

import sar_observation as so
import sar_task

LIDAR = np.full(49, 12.0)


def ego(x=0.0, y=0.0, z=-1.8, vx=0.0, vy=0.0, heading=0.0, yaw_rate=0.0):
    return so.EgoState(x, y, z, vx, vy, heading, yaw_rate)


def det(bearing, rng=None, stamp=1.0):
    return {"detected": True, "bearing": bearing, "range_valid": rng is not None,
            "horizontal_range": math.nan if rng is None else rng, "image_stamp": stamp}


def field(obs, name):
    v = obs[so.SLICES[name]]
    return float(v[0]) if v.size == 1 else v


def test_contract_layout_and_bounds():
    assert so.DIM == 65 and so.LOW.shape == so.HIGH.shape == (65,)
    assert so.SLICES["lidar"] == slice(0, 49) and so.SLICES["est_age"] == slice(64, 65)
    obs = so.ObservationBuilder().build(LIDAR, [0.4, -0.2], ego(), None, None, 0.0)
    assert obs.dtype == np.float32 and obs.shape == (65,)
    assert np.all(obs >= so.LOW) and np.all(obs <= so.HIGH)
    assert "| 64 | est_age |" in so.describe()


def test_body_velocity_yaw_rate_and_height_conventions():
    # Heading east, flying east at 1.5 m/s, drifting south (= right) at 0.75 m/s, turning left.
    obs = so.ObservationBuilder().build(
        LIDAR, [0, 0], ego(vx=-0.75, vy=1.5, heading=math.pi / 2, yaw_rate=0.5, z=-1.5), None, None, 0.0)
    assert field(obs, "vel_forward") == pytest.approx(1.0)
    assert field(obs, "vel_left") == pytest.approx(-0.5)
    assert field(obs, "yaw_rate") == pytest.approx(0.5)
    assert field(obs, "height") == pytest.approx(0.5)


def test_fresh_detection_sets_current_fields_and_memory():
    b = so.ObservationBuilder()
    obs = b.build(LIDAR, [0, 0], ego(), det(math.radians(30), 4.0), 0.2, 10.0)
    assert field(obs, "det_now") == 1 and field(obs, "det_range_valid") == 1
    assert field(obs, "det_bearing_sin") == pytest.approx(0.5)
    assert field(obs, "det_range") == pytest.approx(0.4)
    assert field(obs, "est_valid") == 1 and field(obs, "est_age") == pytest.approx(0.0)
    assert math.degrees(math.atan2(field(obs, "est_bearing_sin"), field(obs, "est_bearing_cos"))) \
        == pytest.approx(30.0)


def test_estimate_is_dead_reckoned_through_motion_and_rotation():
    b = so.ObservationBuilder()
    b.build(LIDAR, [0, 0], ego(heading=0.0), det(0.0, 4.0), 0.1, 0.0)    # target 4 m north
    moved = b.build(LIDAR, [0, 0], ego(x=2.0), None, None, 1.0)
    assert field(moved, "est_range") == pytest.approx(0.2)
    assert field(moved, "est_bearing_cos") == pytest.approx(1.0)
    turned = b.build(LIDAR, [0, 0], ego(x=2.0, heading=math.pi / 2), None, None, 2.0)
    assert field(turned, "est_bearing_sin") == pytest.approx(1.0)       # north is to the left when facing east
    assert field(turned, "det_now") == 0
    assert field(turned, "est_age") == pytest.approx(2.0 / 8.0)


def test_estimate_expires_in_simulation_time():
    b = so.ObservationBuilder(memory_expiry_s=8.0)
    b.build(LIDAR, [0, 0], ego(), det(0.0, 4.0), 0.1, 0.0)
    assert field(b.build(LIDAR, [0, 0], ego(), None, None, 8.0), "est_valid") == 1
    expired = b.build(LIDAR, [0, 0], ego(), None, None, 8.01)
    assert field(expired, "est_valid") == 0 and field(expired, "est_age") == 1.0
    assert field(expired, "est_range") == 0 and field(expired, "est_bearing_cos") == 0


@pytest.mark.parametrize("age", [1.01, -0.1, math.nan, None])
def test_stale_or_unknown_age_perception_is_ignored(age):
    b = so.ObservationBuilder(perception_max_age_s=1.0)
    obs = b.build(LIDAR, [0, 0], ego(), det(0.3, 4.0), age, 0.0)
    assert field(obs, "det_now") == 0 and field(obs, "est_valid") == 0


def test_bearing_only_detection_does_not_create_estimate():
    b = so.ObservationBuilder()
    obs = b.build(LIDAR, [0, 0], ego(), det(0.2), 0.1, 0.0)
    assert field(obs, "det_now") == 1 and field(obs, "det_range_valid") == 0
    assert field(obs, "det_range") == 0 and field(obs, "est_valid") == 0


def test_same_image_does_not_refresh_estimate_age():
    b = so.ObservationBuilder()
    b.build(LIDAR, [0, 0], ego(), det(0.0, 4.0, stamp=5.0), 0.1, 0.0)
    obs = b.build(LIDAR, [0, 0], ego(), det(0.0, 4.0, stamp=5.0), 0.9, 0.5)
    assert field(obs, "est_age") == pytest.approx(0.5 / 8.0)


def test_reset_clears_memory():
    b = so.ObservationBuilder()
    b.build(LIDAR, [0, 0], ego(), det(0.0, 4.0), 0.1, 0.0)
    b.reset()
    assert field(b.build(LIDAR, [0, 0], ego(), None, None, 0.1), "est_valid") == 0


@pytest.mark.parametrize("bad", [np.full(48, 1.0), np.r_[np.full(48, 1.0), np.nan]])
def test_invalid_lidar_is_rejected(bad):
    with pytest.raises(ValueError):
        so.ObservationBuilder().build(bad, [0, 0], ego(), None, None, 0.0)


def test_non_finite_ego_is_rejected():
    with pytest.raises(ValueError):
        so.ObservationBuilder().build(LIDAR, [0, 0], ego(vx=math.nan), None, None, 0.0)


def test_observation_module_cannot_reach_ground_truth():
    tree = ast.parse(Path(so.__file__).read_text())
    imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    imported |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert imported <= {"__future__", "math", "dataclasses", "numpy"}
    params = ast.parse(Path(so.__file__).read_text())
    build = next(n for n in ast.walk(params) if isinstance(n, ast.FunctionDef) and n.name == "build")
    assert [a.arg for a in build.args.args] == [
        "self", "lidar_m", "prev_action", "ego", "perception", "perception_age_s", "now_s"]


# ---- privileged task module ----

def test_target_loaded_from_versioned_world_matches_audited_ned_target():
    t = sar_task.load_target()
    assert (t.x_ned, t.y_ned, t.z_enu, t.radius) == (-3.0, 3.0, 0.055, 0.5)


def test_reward_progress_success_collision_and_timeout():
    cfg = sar_task.TaskConfig()
    plain = sar_task.step_outcome(4.0, 3.9, False, 1, cfg)
    assert plain.reward == pytest.approx(-0.1 + 0.1) and not plain.terminated and plain.reason is None
    success = sar_task.step_outcome(0.8, 0.75, False, 5, cfg)
    assert success.terminated and success.reason == "success"
    assert success.reward == pytest.approx(-0.1 + 0.05 + 100.0)
    both = sar_task.step_outcome(0.8, 0.5, True, 5, cfg)
    assert both.reason == "collision" and both.reward == pytest.approx(-0.1 + 0.3 - 100.0)
    timeout = sar_task.step_outcome(4.0, 4.0, False, cfg.max_steps, cfg)
    assert timeout.truncated and not timeout.terminated and timeout.reason == "timeout"
    with pytest.raises(ValueError):
        sar_task.step_outcome(math.nan, 1.0, False, 1, cfg)


def test_world_target_mismatch_is_detected(tmp_path):
    bad = tmp_path / "w.sdf"
    bad.write_text('<sdf><world name="w"><model name="target"><pose>1 2 0 0 0 0</pose>'
                   '<link><collision><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>'
                   '<visual><geometry><sphere><radius>0.4</radius></sphere></geometry></visual></link>'
                   '</model></world></sdf>')
    with pytest.raises(ValueError, match="radii disagree"):
        sar_task.load_target(bad)
