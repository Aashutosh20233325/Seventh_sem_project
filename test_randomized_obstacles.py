"""Randomized interior obstacles (opt-in): validity, reproducibility, non-triviality, sensing, no target cue."""
import dataclasses
import json

import numpy as np
import pytest

import sar_observation as so
import sar_standin_env as se
from sar_standin_env import SarStandInEnv


def make(**kw):
    return SarStandInEnv(randomize_start=True, randomize_target=True, randomize_obstacles=True, **kw)


def test_layouts_satisfy_every_placement_constraint():
    env = make()
    for seed in range(300):
        env.reset(seed=seed)
        obs_boxes = env.obstacles
        assert se.OBSTACLE_COUNT[0] <= len(obs_boxes) <= se.OBSTACLE_COUNT[1]
        assert env.walls == env.base_walls + obs_boxes
        t = env.target
        for i, b in enumerate(obs_boxes):
            long_side = max(b[1] - b[0], b[3] - b[2])
            short_side = min(b[1] - b[0], b[3] - b[2])
            assert se.OBSTACLE_LENGTH[0] - 1e-9 <= long_side <= se.OBSTACLE_LENGTH[1] + 1e-9
            assert short_side == pytest.approx(se.OBSTACLE_THICKNESS)
            cx, cy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
            x0, x1, y0, y1 = se.OBSTACLE_CENTRE_REGION
            assert x0 <= cx <= x1 and y0 <= cy <= y1
            assert se.box_box_distance(b, se.START_SQUARE) >= se.OBSTACLE_START_CLEARANCE
            assert se.box_distance(t.x_ned, t.y_ned, b) >= se.OBSTACLE_TARGET_CLEARANCE
            for other in env.base_walls + obs_boxes[:i] + obs_boxes[i + 1:]:
                assert se.box_box_distance(b, other) >= se.OBSTACLE_GAP
        assert float(np.min(env._lidar())) > 0.25            # never starts in collision


def test_layouts_are_reproducible_by_seed_and_recorded():
    a, b = make(), make()
    for seed in (3, 99, 1234):
        a.reset(seed=seed); b.reset(seed=seed)
        assert a.obstacles == b.obstacles and a.target == b.target
    assert a.sampled_layouts == b.sampled_layouts and len(a.sampled_layouts) == 3
    json.dumps(a.sampled_layouts)                              # serializable for the run record
    a.reset(seed=4)
    assert a.obstacles != b.obstacles


def test_default_env_never_changes_walls_and_keeps_evaluation_overrides():
    env = SarStandInEnv(randomize_start=True, randomize_target=True)
    base = list(env.walls)
    env.reset(seed=1)
    assert env.walls == base and env.obstacles == [] and env.sampled_layouts == []
    env.walls = base + [(-2.4, -2.1, 0.5, 4.0)]                # evaluation scripts override walls directly
    env.reset(seed=2)
    assert env.walls[-1] == (-2.4, -2.1, 0.5, 4.0)


def test_default_rng_stream_is_unchanged_by_the_new_option():
    a = SarStandInEnv(randomize_start=True, randomize_target=True)
    b = SarStandInEnv(randomize_start=True, randomize_target=True, randomize_obstacles=False)
    oa, _ = a.reset(seed=8); ob, _ = b.reset(seed=8)
    np.testing.assert_array_equal(oa, ob)
    assert (a.x, a.y, a.heading, a.target) == (b.x, b.y, b.heading, b.target)


def test_scenarios_are_not_trivial():
    env = make()
    blocked = 0
    for seed in range(300):
        env.reset(seed=seed)
        blocked += any(se.segment_hits_box(env.x, env.y, env.target.x_ned, env.target.y_ned, o) for o in env.obstacles)
    assert blocked / 300 >= 0.08                               # measured 0.127 at these settings


def test_obstacles_are_sensed_by_lidar_and_cause_collisions():
    env = make(perception_noise=False)
    env.reset(seed=0)
    b = env.obstacles[0]
    cx, cy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
    # Place the vehicle 1 m from the obstacle's long face, facing it.
    if b[1] - b[0] > b[3] - b[2]:
        env.x, env.y, env.heading = cx, b[3] + 1.0, -np.pi / 2
    else:
        env.x, env.y, env.heading = b[1] + 1.0, cy, np.pi
    env.vx = env.vy = env.yaw_rate_ned = 0.0
    front = env._lidar()[so.LIDAR_SECTORS // 2]
    assert front == pytest.approx(1.0, abs=0.05)
    reason = None
    for _ in range(50):
        _, _, term, trunc, info = env.step(np.array([1.0, 0.0], np.float32))
        if term or trunc:
            reason = info["reason"]
            break
    assert reason == "collision"


def test_obstacles_are_not_preferentially_between_start_and_target():
    """No layout cue: the line of sight to the real target is blocked no more often than the line to a phantom
    target drawn from the same sampler (target clearance can only make the real line *less* blocked)."""
    env = make()
    rng = np.random.default_rng(5)
    real = phantom = 0
    n = 600
    for seed in range(n):
        env.reset(seed=seed)
        while True:
            px, py = rng.uniform(*se.TARGET_REGION[:2]), rng.uniform(*se.TARGET_REGION[2:])
            if se.valid_random_target(px, py, env.base_walls):
                break
        real += any(se.segment_hits_box(env.x, env.y, env.target.x_ned, env.target.y_ned, o) for o in env.obstacles)
        phantom += any(se.segment_hits_box(env.x, env.y, px, py, o) for o in env.obstacles)
    assert real / n <= phantom / n + 0.03


def test_blind_observations_depend_on_obstacles_not_on_target():
    def blind(target):
        env = SarStandInEnv(randomize_start=True, randomize_obstacles=True, detection_prob=0.0, camera_occlusion=True)
        env.target = dataclasses.replace(env.nominal_target, x_ned=target[0], y_ned=target[1])
        obs, _ = env.reset(seed=33)
        rng = np.random.default_rng(0)
        out = [obs]
        for _ in range(100):
            obs, _, t, tr, _ = env.step(rng.uniform([0, -1], [1, 1]).astype(np.float32))
            out.append(obs)
            if t or tr:
                break
        return np.array(out), env.obstacles
    (o1, l1), (o2, l2) = blind((-3.0, 3.0)), blind((-2.8, 3.1))
    if l1 == l2:                                               # same layout -> identical observations
        np.testing.assert_array_equal(o1, o2)


sb3 = pytest.importorskip("stable_baselines3")
import train_ppo  # noqa: E402


def test_training_with_obstacles_and_occlusion_records_layouts(tmp_path):
    run_dir = tmp_path / "ppo"
    train_ppo.main([
        "--total-timesteps", "256", "--n-envs", "2", "--n-steps", "128", "--batch-size", "64",
        "--checkpoint-freq", "256", "--eval-freq", "256", "--eval-episodes", "1",
        "--final-eval-episodes", "2", "--seed", "3", "--run-dir", str(run_dir),
        "--progress-weight", "10", "--randomize-target", "--camera-occlusion", "--randomize-obstacles",
    ])
    config = json.loads((run_dir / "config.json").read_text())
    assert config["camera_occlusion"] and config["randomize_obstacles"] and config["randomize_target"]
    layouts = json.loads((run_dir / "sampled_layouts.json").read_text())["per_env"]
    assert len(layouts) == 2 and all(layouts) and config["sampled_layout_count"] == sum(map(len, layouts))


def test_layout_generation_never_fails_and_retries_whole_layouts(monkeypatch):
    env = make()
    for seed in range(20000):                                  # the training crash happened in ~1 of ~7000 resets
        env.reset(seed=seed)
    # Force the per-obstacle failure path: the first layout attempt fails, a later one must succeed.
    calls = {"n": 0}
    original = SarStandInEnv._valid_obstacle

    def flaky(self, box, placed):
        calls["n"] += 1
        return False if calls["n"] <= 500 else original(self, box, placed)
    monkeypatch.setattr(SarStandInEnv, "_valid_obstacle", flaky)
    env.reset(seed=1)
    assert len(env.obstacles) >= se.OBSTACLE_COUNT[0] and calls["n"] > 500


def test_impossible_layouts_raise_instead_of_looping(monkeypatch):
    env = make()
    monkeypatch.setattr(SarStandInEnv, "_valid_obstacle", lambda self, box, placed: False)
    monkeypatch.setattr(SarStandInEnv, "OBSTACLE_LAYOUT_TRIES", 3)
    with pytest.raises(RuntimeError, match="layout"):
        env.reset(seed=0)
