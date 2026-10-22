# Offline PPO experiments (stand-in simulator) — 2026-10-22

Offline only: `sar_standin_env.SarStandInEnv`. There was no Gazebo/PX4 and no flight. The live task defaults (`sar_task.TaskConfig`), the `sar_obs_v1` observation contract, the live env, and the episode runner are unchanged. Run artifacts are in `runs/rl/exp20261022/` (git-ignored, preserved): `run_matrix.sh`, `generalization_eval.py`, per-run `config.json`, `eval_summary.json`, `evaluations.npz`, `monitor/`, models, and logs.

## Why the 20k-step smoke run stood still

Reward per step: `-0.1 + progress_weight * (d_prev - d)`, plus `+100` on success (d <= 0.75 m) or `-100` on collision. Timeout comes at 600 steps (60 s).

- **Progress is small:** it telescopes to `w * (d0 - d_end)`. At w = 1 and d0 ≈ 4.24 m that is at most about 3.5 per episode, around 3% of the ±100 terminal terms.
- **Standing still beats exploring:** standing still returns −60 (timeout). A uniform random policy averaged **−67.7** (47% collision, 17% success, 37% timeout over 30 episodes), so early collisions push PPO to slow down. In the smoke run, successes went 2 → 0 and timeouts 7 → 18 between the two halves.
- **The run was too short:** 20k steps is about 46 episodes.
- **The task design is not flawed:** progress is sunk once collected. From any state, crashing (−100) is worse than hovering to timeout (≥ −60, since 600 × 0.1 < 100), and reaching the target (+100) beats hovering. Neither comparison depends on the progress weight, and telescoping means oscillation earns nothing. Tests in `test_rl_training_options.py` cover this.

## Matrix (code: `4f06f6e`)

- **Variants:**
  - E0: control.
  - E1: `--progress-weight 10`, training env only.
  - E2: `--curriculum`, a staged start range around the target: 1–1.75 m/±30° → 1–3 m/±90° → 1–4.5 m/±180° → default starts for the last 30%.
- **Training:** PPO, 100k steps, 2 envs, training seeds 1/2/3.
- **Evaluation:** always the default task and default randomised start, 50 deterministic episodes on seeds 10000–10049.

Final model, mean over 3 seeds (per-seed values in parentheses):

| | Success | Collision | Timeout | Duration (s) | Return (default task) |
|---|---|---|---|---|---|
| Baseline controller | 1.00 | 0 | 0 | 14.8 | 88.7 |
| E0 control | 0.71 (0.78 / 1.00 / 0.36) | 0 | 0.29 | 27.3 | 46.8 |
| **E1 progress ×10** | **1.00 (1.00 / 1.00 / 1.00)** | 0 | 0 | **8.1** | **95.4** |
| E2 curriculum | 0.19 (0.12 / 0.04 / 0.40) | 0.02 | 0.79 | 50.3 | −32.5 |

- **E0:** at 100k steps the control mostly no longer stands still, but it is seed-sensitive.
- **E1:** reliable on the training distribution in every seed, and faster than the baseline.
- **E2 (as staged):** worse. It spends 70% of training near the target and gets only 30k steps on the evaluation distribution, so it never learns to search. Not pursued.

## Generalization check (evaluation only, `generalization_eval.json`)

| Success rate | Nominal | Camera blind (detection 0) | Target moved A (−3,−2.5) | B (2,−3) | C (−1,3) |
|---|---|---|---|---|---|
| Baseline | 1.00 | 0.00 | 1.00 | 1.00 | 1.00 |
| E0 (s1/s2/s3) | .78/1.0/.36 | 0/0/0 | 0/0/0 | 0/0/0 | .22/.12/.22 |
| E1 (s1/s2/s3) | 1/1/1 | 0/0/0 | .76/.02/0 | .54/0/0 | .94/.78/.54 |

- **Camera blind:** every policy fails, so the PPO policies do depend on camera perception and are not blind dead reckoning.
- **Target moved:** success collapses, while the baseline stays at 100%. The PPO policies were trained with **one fixed target location**. LiDAR in a fixed map is enough to localise, so they learned to turn and fly toward where the target always is and use the camera mainly for the final approach.
- **Conclusion:** E1 is reliable only for this exact world. It has not learned general search-and-reach. That is a training-distribution limitation, not a reward bug. The principled next step is to randomise the target position (and possibly the walls) during training, with evaluation on held-out positions. Neither has been implemented yet.

## E1R: E1 plus randomized targets (code `16f6814`)

**Training:**
- `train_ppo.py --total-timesteps 100000 --seed {1,2,3} --progress-weight 10 --randomize-target --final-eval-episodes 50 --torch-threads 2`: PPO MlpPolicy, 2 envs, n_steps 1024, batch 256, lr 3e-4, gamma 0.99, randomized start.
- Each episode draws a new target in the open corner inside the north/east walls: NED x, y ∈ [−6, 3.5], ≥ 1 m from walls, 2.5–6 m from the start.
- Six held-out positions are never sampled: the world's nominal target (−3, 3) and A (−3, −2.5), B (2, −3), C (−1, 3), D (−5.5, 0.5), E (1.5, 2.5).
- Draws are recorded in `sampled_targets.json`: 363 / 537 / 217 episodes. Seed 1's draws span x −5.86…3.48 and y −5.96…3.45. The only duplicate is the doubly-seeded first reset.
- Run directory: `runs/rl/exp20261022_randtarget/` (scripts, logs, models, `evaluation.json`).

**Evaluation (`evaluate_matrix.py`, final models):**
- Every policy uses the same 50 episode seeds (10000–10049) and the default task reward.
- `random_world` uses the same 50 sampled targets for every policy (asserted).
- For E1R, the nominal target is a held-out position. For E1 it is the training target.

Success rate per condition. Collisions were 0 in every cell.

| Policy | Nominal | A | B | C | D | E | Held-out mean (min) | Random world | Camera blind |
|---|---|---|---|---|---|---|---|---|---|
| Baseline | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 (1.00) | 1.00 | 0.00 |
| E1 fixed s1 | 1.00\* | .76 | .54 | .94 | .98 | .64 | .77 (.54) | .56 | 0 |
| E1 fixed s2 | 1.00\* | .02 | .00 | .78 | .00 | .04 | .17 (.00) | .20 | 0 |
| E1 fixed s3 | 1.00\* | .00 | .00 | .54 | .94 | .04 | .30 (.00) | .20 | 0 |
| E1R s1 | .56 | .72 | .94 | .82 | .36 | .94 | .72 (.36) | .72 | 0 |
| **E1R s2** | .84 | 1.00 | .98 | 1.00 | 1.00 | 1.00 | **.97 (.84)** | **1.00** | 0 |
| E1R s3 | .00 | .00 | .26 | .00 | .00 | .04 | .05 (.00) | .14 | 0 |

\* E1's training target, so not a generalization result. Held-out mean is over the six held-out positions (E1R) or A–E (E1).

- **Episode length:** baseline 13–17 s. E1R s2 7–10 s on held-out positions and 8.7 s on random targets. E1R s1 12–43 s. E1R s3 46–60 s. Full return and length figures per cell are in `evaluation.json`.
- **Random-world success:** E1R 0.72 / 1.00 / 0.14 (mean 0.62, sd 0.36 across seeds) against E1 fixed 0.56 / 0.20 / 0.20 (mean 0.32). Randomizing the target improves generalization on average, but it is not reliable across seeds. The baseline is 1.00 everywhere.

**Failure diagnosis (30 random-world episodes per seed):**
- **E1R s1 timeouts (8/30):** every one detected the target and then circled (|turn| ≈ 0.9, speed ≈ 0.1) at **0.8–0.9 m**, just outside the 0.75 m success radius.
  - The camera model cannot see the target closer than about **1.05 m** (30° down-pitch, lower FOV edge). So the last ~0.3 m must be flown on the dead-reckoned estimate (`est_*`, 8 s memory), as the baseline does.
  - This is an intended property of the sensor geometry (the live OAK-D mount is the same), not a reward bug.
- **E1R s3 timeouts (26/30):** 11 never detected the target and barely moved (speed ≈ 0), the original stand-still optimum. 15 detected it but stalled 0.8–3 m away.
  - Its evaluation curve stayed at about −59 for all 100k steps. s1 and s2 were still rising at 95k (s1 reached 76, s2 95).
- **Interpretation:** at 100k steps the outcome depends on whether a seed escapes the stand-still optimum early. Seeds that escape then still have to learn the memory-based final approach.
