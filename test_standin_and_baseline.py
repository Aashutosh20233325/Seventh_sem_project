import ast
import math
from pathlib import Path

import numpy as np
import pytest

import baseline_controller as bc
import sar_observation as so
from sar_standin_env import SarStandInEnv


def place(env, x, y, heading):
    env.x, env.y, env.heading = x, y, heading
    env.vx = env.vy = env.yaw_rate_ned = 0.0


def test_forward_action_follows_heading_and_positive_turn_is_clockwise():
    env = SarStandInEnv(perception_noise=False)
    env.reset(seed=0)
    place(env, 0.0, 0.0, math.pi / 2)                 # facing east
    for _ in range(20):
        env.step([1.0, 0.0])
    assert env.y > 1.0 and abs(env.x) < 1e-6         # moved east
    heading0 = env.heading
    env.step([0.0, 1.0])
    assert so.wrap(env.heading - heading0) > 0        # NED heading increased = clockwise


def test_lidar_sector_order_is_right_to_left():
    env = SarStandInEnv()
    env.reset(seed=0)
    place(env, 0.0, 2.5, 0.0)                         # facing north, box1 wall (NED y 4.5) is 2 m to the right
    sectors = env._lidar()
    right = sectors[np.argmin(np.abs(bc.SECTOR_ANGLES + math.pi / 2))]
    left = sectors[np.argmin(np.abs(bc.SECTOR_ANGLES - math.pi / 2))]
    assert right == pytest.approx(2.0, abs=0.05) and left > 2.0
    place(env, 0.0, 2.5, math.pi / 2)                 # facing east: wall straight ahead
    assert min(env._lidar()[bc.FRONT]) == pytest.approx(2.0, abs=0.05)


@pytest.mark.parametrize("offset,expect_visible,sign", [
    ((0.0, -4.0), True, 0),        # 4 m behind the target in -east: target dead ahead when facing east
    ((-1.5, -4.0), True, +1),      # target to the north = left when facing east
    ((0.0, 4.0), False, 0),        # target behind
    ((0.0, -0.8), False, 0),       # too close: below the camera's view
])
def test_camera_model_visibility_and_bearing_sign(offset, expect_visible, sign):
    env = SarStandInEnv(perception_noise=False, detection_prob=1.0)
    env.reset(seed=0)
    t = env.target
    place(env, t.x_ned + offset[0], t.y_ned + offset[1], math.pi / 2)
    p = env._perceive()
    assert p["detected"] == expect_visible
    if expect_visible:
        assert np.sign(round(p["bearing"], 3)) == sign
        assert p["horizontal_range"] == pytest.approx(math.hypot(*offset))


def test_rewards_sum_to_progress_and_success_triggers_at_radius():
    env = SarStandInEnv(perception_noise=False)
    _, info = env.reset(seed=0)
    start = info["privileged"]["distance_to_target"]
    ctl = bc.BaselineController()
    total, steps = 0.0, 0
    while True:
        _, reward, terminated, truncated, info = env.step(ctl.act(env._observe(env._lidar())))
        total += reward
        steps += 1
        if terminated or truncated:
            break
    final = info["privileged"]["distance_to_target"]
    assert info["reason"] == "success" and final <= 0.75
    assert total == pytest.approx(start - final - 0.1 * steps + 100.0, abs=1e-6)


def test_flying_into_a_wall_is_a_collision():
    env = SarStandInEnv()
    env.reset(seed=0)
    place(env, 0.0, 3.8, math.pi / 2)                 # 0.7 m from box1, facing it
    for _ in range(30):
        _, reward, terminated, _, info = env.step([1.0, 0.0])
        if terminated:
            break
    assert info["reason"] == "collision" and reward < -90


@pytest.mark.parametrize("randomize,episodes", [(False, 10), (True, 25)])
def test_baseline_reaches_target_without_collisions(randomize, episodes):
    env = SarStandInEnv(randomize_start=randomize)
    ctl = bc.BaselineController()
    reasons = []
    for seed in range(episodes):
        obs, _ = env.reset(seed=seed)
        ctl.reset()
        while True:
            obs, _, terminated, truncated, info = env.step(ctl.act(obs))
            if terminated or truncated:
                break
        reasons.append(info["reason"])
    assert reasons.count("success") == episodes


def test_baseline_stops_and_turns_away_at_close_obstacle():
    obs = np.zeros(so.DIM, np.float32)
    obs[so.SLICES["lidar"]] = 1.0
    obs[so.SLICES["lidar"]][bc.FRONT] = 0.8 / so.LIDAR_RANGE_MAX
    obs[so.SLICES["lidar"]][bc.SECTOR_ANGLES > math.radians(40)] = 0.6 / so.LIDAR_RANGE_MAX   # left blocked
    obs[so.SLICES["est_valid"]] = 1.0
    obs[so.SLICES["est_bearing_cos"]] = 1.0                           # target straight ahead
    obs[so.SLICES["est_range"]] = 0.5
    speed, turn = bc.BaselineController().act(obs)
    assert speed == 0.0 and turn > 0                  # positive turn = right, toward the clearer side


def test_baseline_searches_instead_of_following_expired_or_absent_target():
    obs = np.zeros(so.DIM, np.float32)
    obs[so.SLICES["lidar"]] = 1.0
    obs[so.SLICES["est_age"]] = 1.0                   # expired estimate: est_valid = 0
    ctl = bc.BaselineController()
    actions = [ctl.act(obs) for _ in range(ctl.search_rotation_steps + 5)]
    assert all(a[0] == 0.0 and a[1] == pytest.approx(ctl.search_turn) for a in actions[:ctl.search_rotation_steps])
    assert actions[-1][0] > 0                         # then moves to a new viewpoint


def test_standin_observation_follows_contract_and_hides_truth():
    env = SarStandInEnv()
    obs, info = env.reset(seed=3)
    assert obs.shape == (so.DIM,) and np.all(obs >= so.LOW) and np.all(obs <= so.HIGH)
    assert "distance_to_target" in info["privileged"]


def test_baseline_has_no_privileged_access():
    tree = ast.parse(Path(bc.__file__).read_text())
    imported = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    imported |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert imported <= {"__future__", "math", "numpy", "sar_observation"}
