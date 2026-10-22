#!/usr/bin/env python3
"""Reproducible PPO training on the offline stand-in environment (sar_obs_v1, same action space as SITL).

Each run writes to a new, git-ignored directory (default ``runs/rl/ppo_<UTC>``):
- ``config.json``: arguments, seeds, library versions, observation version.
- ``checkpoints/``: periodic models.
- ``best_model.zip``: best deterministic evaluation.
- ``final_model.zip``.
- ``monitor/``: per-episode train CSV.
- ``evaluations.npz``: periodic evaluation.
- ``eval_summary.json``: final deterministic evaluation of PPO vs the baseline (success, collision,
  timeout, final distance, duration, return).

Training never touches PX4/Gazebo. Torch threads are capped so a running SITL stack is not starved.

Usage (short smoke run):
    .venv/bin/python train_ppo.py --total-timesteps 20000 --seed 1
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import gymnasium
import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

import evaluate_policies
import sar_observation
import sar_task
from sar_standin_env import SarStandInEnv

PROJECT = Path(__file__).resolve().parent


# Start-state curriculum: (end of stage as a fraction of total timesteps, r_min, r_max, heading error deg).
# The last stage (None) is the default start distribution, which is also the evaluation distribution.
CURRICULUM = (
    (0.25, 1.0, 1.75, 30.0),
    (0.50, 1.0, 3.0, 90.0),
    (0.70, 1.0, 4.5, 180.0),
    (1.00, None, None, None),
)


class CurriculumCallback(BaseCallback):
    """Advance the training envs' start range with training progress (evaluation envs are untouched)."""

    def __init__(self, total_timesteps, stages=CURRICULUM):
        super().__init__()
        self.total = total_timesteps
        self.stages = stages
        self.stage = None
        self.history = []

    def _apply(self):
        fraction = self.num_timesteps / self.total
        stage = next(i for i, s in enumerate(self.stages) if fraction < s[0] or i == len(self.stages) - 1)
        if stage != self.stage:
            _, r_min, r_max, err = self.stages[stage]
            self.training_env.env_method(
                "set_start_range", r_min, r_max, None if err is None else np.radians(err))
            self.stage = stage
            self.history.append({"stage": stage, "timesteps": int(self.num_timesteps)})

    def _on_training_start(self):
        self._apply()

    def _on_step(self):
        self._apply()
        return True


def make_env(seed, randomize_start, monitor_dir=None, task_config=None):
    def _init():
        env = SarStandInEnv(randomize_start=randomize_start, seed=seed, task_config=task_config)
        env = Monitor(env, str(monitor_dir / f"env_{seed}") if monitor_dir else None,
                      info_keywords=("reason",))
        env.reset(seed=seed)
        env.action_space.seed(seed)
        return env
    return _init


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--total-timesteps", type=int, default=20_000)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--n-envs", type=int, default=2)
    p.add_argument("--randomize-start", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--n-steps", type=int, default=1024)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--learning-rate", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--checkpoint-freq", type=int, default=10_000)
    p.add_argument("--eval-freq", type=int, default=5_000)
    p.add_argument("--eval-episodes", type=int, default=10)
    p.add_argument("--final-eval-episodes", type=int, default=30)
    p.add_argument("--torch-threads", type=int, default=2)
    p.add_argument("--run-dir", type=Path)
    p.add_argument("--progress-weight", type=float,
                   help="training-only reward weight per metre of progress (default: the task's)")
    p.add_argument("--curriculum", action="store_true", help="staged start-state curriculum (CURRICULUM)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    run_dir = args.run_dir or PROJECT / "runs" / "rl" / f"ppo_{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}"
    run_dir.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(args.torch_threads)
    config = {
        "args": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "run_dir": str(run_dir),
        "observation_version": sar_observation.VERSION,
        "observation_dim": sar_observation.DIM,
        "environment": "sar_standin_env.SarStandInEnv (offline kinematic stand-in)",
        "versions": {"python": platform.python_version(), "stable_baselines3": stable_baselines3.__version__,
                     "torch": torch.__version__, "gymnasium": gymnasium.__version__, "numpy": np.__version__},
        "started_utc": datetime.now(timezone.utc).isoformat(),
    }
    (run_dir / "config.json").write_text(json.dumps(config, indent=2) + "\n")

    train_config = sar_task.TaskConfig()
    if args.progress_weight is not None:
        train_config = dataclasses.replace(train_config, progress_weight=args.progress_weight)
    config["train_task_config"] = dataclasses.asdict(train_config)
    config["eval_task_config"] = dataclasses.asdict(sar_task.TaskConfig())
    config["curriculum"] = CURRICULUM if args.curriculum else None
    (run_dir / "config.json").write_text(json.dumps(config, indent=2) + "\n")

    monitor_dir = run_dir / "monitor"
    monitor_dir.mkdir()
    train_env = DummyVecEnv([make_env(args.seed + i, args.randomize_start, monitor_dir, train_config)
                             for i in range(args.n_envs)])
    eval_env = DummyVecEnv([make_env(args.seed + 1000, args.randomize_start)])
    model = PPO(
        "MlpPolicy", train_env, n_steps=args.n_steps, batch_size=args.batch_size,
        learning_rate=args.learning_rate, gamma=args.gamma, seed=args.seed, device="cpu", verbose=0,
    )
    callbacks = [
        CheckpointCallback(max(args.checkpoint_freq // args.n_envs, 1), str(run_dir / "checkpoints"), "ppo"),
        EvalCallback(eval_env, n_eval_episodes=args.eval_episodes, eval_freq=max(args.eval_freq // args.n_envs, 1),
                     best_model_save_path=str(run_dir), log_path=str(run_dir), deterministic=True),
    ]
    curriculum = CurriculumCallback(args.total_timesteps) if args.curriculum else None
    if curriculum is not None:
        callbacks.append(curriculum)
    model.learn(total_timesteps=args.total_timesteps, callback=callbacks, progress_bar=False)
    model.save(run_dir / "final_model")
    if curriculum is not None:
        config["curriculum_history"] = curriculum.history

    # Final deterministic comparison against the baseline on held-out seeds.
    summary = {}
    policies = {"ppo_final": evaluate_policies.ppo_policy(run_dir / "final_model.zip")}
    if (run_dir / "best_model.zip").is_file():
        policies["ppo_best"] = evaluate_policies.ppo_policy(run_dir / "best_model.zip")
    policies["baseline"] = evaluate_policies.baseline_policy()
    for name, (act, reset) in policies.items():
        env = SarStandInEnv(randomize_start=args.randomize_start)
        summary[name] = evaluate_policies.summarize(
            evaluate_policies.run_episodes(act, reset, env, args.final_eval_episodes, 10_000))
    config["finished_utc"] = datetime.now(timezone.utc).isoformat()
    (run_dir / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    (run_dir / "eval_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"run_dir": str(run_dir), **summary}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
