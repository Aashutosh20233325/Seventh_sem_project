"""Training-only reward weight and start-state curriculum: reward, termination, and observation separation."""
import dataclasses
import json
import math

import numpy as np
import pytest

import baseline_controller as bc
import sar_observation as so
import sar_task
from sar_standin_env import SarStandInEnv


def rollout(env, seed, actions):
    obs, info = env.reset(seed=seed)
    out = [(obs, None, False, False, info)]
    for a in actions:
        step = env.step(a)
        out.append(step)
        if step[2] or step[3]:
            break
    return out


def test_progress_weight_changes_only_the_reward():
    actions = [np.array([0.6, 0.3 * math.sin(i / 7)], np.float32) for i in range(300)]
    base = rollout(SarStandInEnv(randomize_start=True), 4, actions)
    heavy = rollout(SarStandInEnv(randomize_start=True,
                                  task_config=dataclasses.replace(sar_task.TaskConfig(), progress_weight=10.0)),
                    4, actions)
    assert len(base) == len(heavy)
    for (o1, r1, t1, tr1, i1), (o2, r2, t2, tr2, i2) in zip(base, heavy):
        np.testing.assert_array_equal(o1, o2)                       # observation unaffected
        assert (t1, tr1, i1.get("reason")) == (t2, tr2, i2.get("reason"))
    # Per step: reward difference is exactly 9 x progress (terminal bonuses are unchanged).
    for k in range(1, len(base)):
        progress = base[k - 1][4]["privileged"]["distance_to_target"] - base[k][4]["privileged"]["distance_to_target"]
        assert heavy[k][1] - base[k][1] == pytest.approx(9.0 * progress, abs=1e-9)


def test_progress_reward_telescopes_so_oscillation_earns_nothing():
    config = dataclasses.replace(sar_task.TaskConfig(), progress_weight=10.0, step_cost=0.0)
    env = SarStandInEnv(perception_noise=False, task_config=config)
    _, info = env.reset(seed=0)
    start = info["privileged"]["distance_to_target"]
    total = 0.0
    for i in range(200):                                        # forward/back by turning around repeatedly
        _, r, term, trunc, info = env.step(np.array([0.5, 1.0 if (i // 20) % 2 else 0.0], np.float32))
        total += r
        if term or trunc:
            break
    bonus = config.success_bonus if info["reason"] == "success" else 0.0
    assert info["reason"] in (None, "success")
    assert total - bonus == pytest.approx(10.0 * (start - info["privileged"]["distance_to_target"]), abs=1e-6)


def test_terminal_ordering_is_preserved_at_any_progress_weight():
    """Progress collected before a choice is sunk, so compare options from the same state.

    Crashing at some point is worse than flying to that point and hovering until timeout (the collision
    penalty exceeds the largest remaining step cost), and reaching the target beats hovering (positive
    bonus, non-negative remaining progress). Neither comparison involves progress_weight."""
    config = sar_task.TaskConfig()
    assert config.collision_penalty > config.step_cost * config.max_steps
    assert config.success_bonus > 0


def test_default_task_is_unchanged():
    assert sar_task.TaskConfig() == sar_task.TaskConfig(
        success_radius=0.75, step_cost=0.1, progress_weight=1.0, collision_penalty=100.0,
        success_bonus=100.0, max_steps=600)


@pytest.mark.parametrize("r_min, r_max, err_deg", [(1.0, 1.75, 30.0), (1.0, 3.0, 90.0), (1.0, 4.5, 180.0)])
def test_curriculum_starts_are_in_range_clear_and_not_already_successful(r_min, r_max, err_deg):
    env = SarStandInEnv()
    env.set_start_range(r_min, r_max, math.radians(err_deg))
    for seed in range(40):
        obs, info = env.reset(seed=seed)
        d = info["privileged"]["distance_to_target"]
        assert r_min - 1e-9 <= d <= r_max + 1e-9 and d > env.task_config.success_radius
        bearing_to_target = math.atan2(env.target.y_ned - env.y, env.target.x_ned - env.x)
        assert abs(so.wrap(env.heading - bearing_to_target)) <= math.radians(err_deg) + 1e-9
        assert float(np.min(env._lidar())) >= env.START_CLEARANCE
        assert env.observation_space.contains(obs)


def test_clearing_the_curriculum_restores_the_default_start_distribution():
    env, reference = SarStandInEnv(randomize_start=True), SarStandInEnv(randomize_start=True)
    env.set_start_range(1.0, 2.0, 0.5)
    env.reset(seed=3)
    env.set_start_range(None)
    o1, i1 = env.reset(seed=11)
    o2, i2 = reference.reset(seed=11)
    np.testing.assert_array_equal(o1, o2)
    assert i1["privileged"] == i2["privileged"]


def test_curriculum_rejects_starts_inside_the_success_radius():
    with pytest.raises(ValueError):
        SarStandInEnv().set_start_range(0.5, 2.0, 0.1)


def test_curriculum_changes_start_state_only_not_the_observation_contract():
    env = SarStandInEnv()
    env.set_start_range(1.0, 1.5, 0.2)
    obs, info = env.reset(seed=2)
    assert obs.shape == (so.DIM,) and set(info) == {"privileged"}
    obs2, _, _, _, info2 = env.step(np.zeros(2, np.float32))
    assert "privileged" in info2 and env.observation_space.contains(obs2)


def test_baseline_succeeds_from_curriculum_starts():
    env = SarStandInEnv()
    env.set_start_range(1.0, 3.0, math.radians(90))
    ctl = bc.BaselineController()
    for seed in range(5):
        obs, _ = env.reset(seed=seed)
        ctl.reset()
        while True:
            obs, _, term, trunc, info = env.step(ctl.act(obs))
            if term or trunc:
                break
        assert info["reason"] == "success"


# ---------- training plumbing ----------

sb3 = pytest.importorskip("stable_baselines3")
import train_ppo  # noqa: E402


class FakeModel:
    def __init__(self, env):
        self.env = env

    def get_env(self):
        return self.env


class FakeVecEnv:
    def __init__(self):
        self.calls = []

    def env_method(self, name, *args):
        self.calls.append((name, args))


def test_curriculum_callback_advances_stages_and_ends_on_the_default_distribution():
    cb = train_ppo.CurriculumCallback(1000)
    env = FakeVecEnv()
    cb.model = FakeModel(env)
    applied = []
    for t in range(0, 1001, 50):
        cb.num_timesteps = t
        before = len(env.calls)
        cb._apply()
        if len(env.calls) > before:
            applied.append((t, env.calls[-1][1]))
    assert [t for t, _ in applied] == [0, 250, 500, 700]
    assert applied[0][1][:2] == (1.0, 1.75) and applied[-1][1] == (None, None, None)


def test_training_flags_affect_training_env_only_and_are_recorded(tmp_path):
    run_dir = tmp_path / "ppo"
    train_ppo.main([
        "--total-timesteps", "256", "--n-envs", "1", "--n-steps", "128", "--batch-size", "64",
        "--checkpoint-freq", "128", "--eval-freq", "128", "--eval-episodes", "1",
        "--final-eval-episodes", "2", "--seed", "3", "--run-dir", str(run_dir),
        "--progress-weight", "10", "--curriculum",
    ])
    config = json.loads((run_dir / "config.json").read_text())
    assert config["train_task_config"]["progress_weight"] == 10.0
    assert config["eval_task_config"]["progress_weight"] == 1.0
    assert config["curriculum"] and config["curriculum_history"][0] == {"stage": 0, "timesteps": 0}
