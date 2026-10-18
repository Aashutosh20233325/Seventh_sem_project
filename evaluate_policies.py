#!/usr/bin/env python3
"""Evaluate target-reaching policies on the offline stand-in (or any env with the same interface).

Metrics (ground truth is used for evaluation only): success rate, collision rate, timeout rate, mean
final target distance, mean episode duration (simulation s), and mean return. Reward alone is never
used to claim success.

Usage:
    .venv/bin/python evaluate_policies.py --episodes 50 [--ppo runs/rl/<run>/best_model.zip]
                                          [--randomize-start] [--out results.json]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from baseline_controller import BaselineController
from sar_standin_env import DT, SarStandInEnv


def run_episodes(policy, reset_policy, env, episodes, seed):
    records = []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        reset_policy()
        total, info = 0.0, {}
        while True:
            obs, reward, terminated, truncated, info = env.step(policy(obs))
            if not math.isfinite(reward):
                raise ValueError(f"non-finite reward in episode {i}")
            total += reward
            if terminated or truncated:
                break
        records.append({
            "reason": info["reason"], "steps": info["step"], "return": total,
            "final_distance": info["privileged"]["distance_to_target"],
        })
    return records


def summarize(records):
    n = len(records)
    reasons = [r["reason"] for r in records]
    return {
        "episodes": n,
        "success_rate": reasons.count("success") / n,
        "collision_rate": reasons.count("collision") / n,
        "timeout_rate": reasons.count("timeout") / n,
        "mean_final_distance_m": float(np.mean([r["final_distance"] for r in records])),
        "mean_episode_duration_s": float(np.mean([r["steps"] for r in records]) * DT),
        "mean_return": float(np.mean([r["return"] for r in records])),
    }


def baseline_policy():
    controller = BaselineController()
    return controller.act, controller.reset


def ppo_policy(model_path):
    from stable_baselines3 import PPO

    model = PPO.load(model_path, device="cpu")
    return (lambda obs: model.predict(obs, deterministic=True)[0]), (lambda: None)


def random_policy(seed):
    rng = np.random.default_rng(seed)
    return (lambda obs: rng.uniform([0.0, -1.0], [1.0, 1.0]).astype(np.float32)), (lambda: None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=10_000, help="evaluation seeds (disjoint from training)")
    parser.add_argument("--randomize-start", action="store_true")
    parser.add_argument("--ppo", type=Path, action="append", default=[], help="PPO model .zip (repeatable)")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    policies = {"baseline": baseline_policy(), "random": random_policy(args.seed)}
    for path in args.ppo:
        policies[f"ppo:{path}"] = ppo_policy(path)
    results = {}
    for name, (act, reset) in policies.items():
        env = SarStandInEnv(randomize_start=args.randomize_start)
        results[name] = summarize(run_episodes(act, reset, env, args.episodes, args.seed))
        print(name, json.dumps(results[name]))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps({"randomize_start": args.randomize_start, "episodes": args.episodes,
                                        "seed": args.seed, "results": results}, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
