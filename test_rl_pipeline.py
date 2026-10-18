import json

import numpy as np
import pytest

pytest.importorskip("stable_baselines3")

import evaluate_policies  # noqa: E402
import train_ppo  # noqa: E402
from sar_standin_env import SarStandInEnv  # noqa: E402


def test_short_training_writes_artifacts_and_evaluates_deterministically(tmp_path):
    run_dir = tmp_path / "ppo"
    train_ppo.main([
        "--total-timesteps", "256", "--n-envs", "1", "--n-steps", "128", "--batch-size", "64",
        "--checkpoint-freq", "128", "--eval-freq", "128", "--eval-episodes", "1",
        "--final-eval-episodes", "2", "--seed", "3", "--run-dir", str(run_dir),
    ])

    config = json.loads((run_dir / "config.json").read_text())
    assert config["observation_version"] == "sar_obs_v1" and config["observation_dim"] == 65
    assert (run_dir / "final_model.zip").is_file() and list((run_dir / "checkpoints").glob("*.zip"))
    summary = json.loads((run_dir / "eval_summary.json").read_text())
    assert set(summary) == {"ppo_final", "baseline"}
    for metrics in summary.values():
        assert 0.0 <= metrics["success_rate"] <= 1.0 and np.isfinite(metrics["mean_return"])

    act, reset = evaluate_policies.ppo_policy(run_dir / "final_model.zip")
    runs = [evaluate_policies.run_episodes(act, reset, SarStandInEnv(randomize_start=True), 2, 77)
            for _ in range(2)]
    assert runs[0] == runs[1]                         # deterministic policy + seeded env


def test_rollout_episode_boundaries_and_finite_rewards():
    env = SarStandInEnv(randomize_start=True, max_steps=40)
    rng = np.random.default_rng(0)
    obs, _ = env.reset(seed=5)
    episodes, steps_in_episode = 0, 0
    for _ in range(200):
        obs, reward, terminated, truncated, info = env.step(rng.uniform([0, -1], [1, 1]))
        steps_in_episode += 1
        assert np.isfinite(reward) and np.all(np.isfinite(obs))
        assert env.observation_space.contains(obs)
        if terminated or truncated:
            assert steps_in_episode <= 40
            assert truncated == (info["reason"] == "timeout")
            episodes += 1
            steps_in_episode = 0
            obs, _ = env.reset()
    assert episodes >= 4
