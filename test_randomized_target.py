"""Randomized-target training option: valid sampling, per-episode consistency, actual-target perception,
held-out exclusion, and no target leakage into the observation."""
import json
import math

import numpy as np
import pytest

import baseline_controller as bc
import sar_observation as so
import sar_standin_env as se
from sar_standin_env import SarStandInEnv


def test_targets_are_sampled_valid_varied_and_recorded():
    env = SarStandInEnv(randomize_start=True, randomize_target=True)
    seen = []
    for seed in range(60):
        _, info = env.reset(seed=seed)
        t = env.target
        assert info["privileged"]["target_ned"] == (t.x_ned, t.y_ned)
        assert se.valid_random_target(t.x_ned, t.y_ned, env.walls)
        assert all(se.box_distance(t.x_ned, t.y_ned, w) >= se.TARGET_WALL_CLEARANCE for w in env.walls)
        assert all(math.hypot(t.x_ned - hx, t.y_ned - hy) > se.HELD_OUT_RADIUS for hx, hy in se.HELD_OUT_TARGETS)
        assert info["privileged"]["distance_to_target"] > env.task_config.success_radius + 0.3
        assert (t.z_enu, t.radius) == (env.nominal_target.z_enu, env.nominal_target.radius)
        seen.append((t.x_ned, t.y_ned))
    assert env.sampled_targets == seen
    xs, ys = np.array(seen).T
    assert len(set(seen)) == 60 and np.ptp(xs) > 5.0 and np.ptp(ys) > 5.0     # spread over the region


def test_target_is_fixed_within_an_episode_and_reproducible_by_seed():
    env = SarStandInEnv(randomize_target=True)
    env.reset(seed=7)
    first = env.target
    for _ in range(50):
        env.step(np.array([0.5, 0.2], np.float32))
        assert env.target == first
    env.reset(seed=8)
    assert env.target != first
    env.reset(seed=7)
    assert env.target == first


def test_default_env_keeps_the_nominal_world_target():
    env = SarStandInEnv(randomize_start=True)
    for seed in range(5):
        env.reset(seed=seed)
        assert env.target == env.nominal_target and env.sampled_targets == []


def test_held_out_positions_are_valid_locations_but_never_sampled():
    env = SarStandInEnv()
    for hx, hy in se.HELD_OUT_TARGETS:
        assert se.valid_random_target(hx, hy, env.walls, held_out=())     # valid world position
        assert not se.valid_random_target(hx, hy, env.walls)               # but excluded from training
    assert (env.nominal_target.x_ned, env.nominal_target.y_ned) in se.HELD_OUT_TARGETS


def test_perception_and_reward_follow_the_actual_sampled_target():
    env = SarStandInEnv(randomize_start=True, randomize_target=True, perception_noise=False, detection_prob=1.0)
    ctl = bc.BaselineController()
    for seed in range(6):
        obs, _ = env.reset(seed=seed)
        ctl.reset()
        while True:
            obs, _, term, trunc, info = env.step(ctl.act(obs))
            if obs[so.SLICES["det_now"]][0] and obs[so.SLICES["det_range_valid"]][0]:
                bearing, rng = env._true_relative()                         # actual target, not nominal
                measured = math.atan2(obs[so.SLICES["det_bearing_sin"]][0], obs[so.SLICES["det_bearing_cos"]][0])
                assert abs(so.wrap(measured - bearing)) < 0.35                # latency only
            if term or trunc:
                break
        assert info["reason"] == "success"
        assert info["privileged"]["distance_to_target"] <= 0.75


def test_observation_carries_no_target_information_without_a_detection():
    """With the camera blind, two worlds that differ only in target position give identical observations."""
    a = SarStandInEnv(randomize_start=True, detection_prob=0.0)
    b = SarStandInEnv(randomize_start=True, detection_prob=0.0)
    b.target = se.dataclasses.replace(b.nominal_target, x_ned=2.0, y_ned=-3.0)
    oa, _ = a.reset(seed=3)
    ob, _ = b.reset(seed=3)
    np.testing.assert_array_equal(oa, ob)
    rng = np.random.default_rng(0)
    for _ in range(200):
        act = rng.uniform([0, -1], [1, 1]).astype(np.float32)
        oa, _, ta, tra, _ = a.step(act)
        ob, _, tb, trb, _ = b.step(act)
        np.testing.assert_array_equal(oa, ob)
        if ta or tra or tb or trb:
            break


sb3 = pytest.importorskip("stable_baselines3")
import train_ppo  # noqa: E402


def test_training_with_randomized_targets_records_draws(tmp_path):
    run_dir = tmp_path / "ppo"
    train_ppo.main([
        "--total-timesteps", "256", "--n-envs", "2", "--n-steps", "128", "--batch-size", "64",
        "--checkpoint-freq", "256", "--eval-freq", "256", "--eval-episodes", "1",
        "--final-eval-episodes", "2", "--seed", "3", "--run-dir", str(run_dir),
        "--progress-weight", "10", "--randomize-target",
    ])
    config = json.loads((run_dir / "config.json").read_text())
    assert config["randomize_target"] is True and config["train_task_config"]["progress_weight"] == 10.0
    targets = json.loads((run_dir / "sampled_targets.json").read_text())["per_env"]
    assert len(targets) == 2 and all(targets) and config["sampled_target_count"] == sum(map(len, targets))
