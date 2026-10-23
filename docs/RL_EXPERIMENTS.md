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

## E1R at 300k steps: training duration only (code `16f6814`)

**Training:** identical to E1R except `--total-timesteps 300000`. Seeds 1/2/3, progress weight 10, randomized targets and starts, unchanged PPO settings and target sampler. Run directory: `runs/rl/exp20261022_randtarget300k/` (`run_train.sh`, logs, checkpoints every 10k steps, `sampled_targets.json`, `evaluate_matrix.py`/`.out`, `evaluation.json`, `approach_diagnostic.py`/`.json`/`.out`). The 100k runs are untouched.

**Learning curves** (`evaluations.npz`: 10 deterministic episodes every 5k steps on random targets):

| Seed | Return at 100k | First ≥ 90 | Minimum over 200–300k |
|---|---|---|---|
| s1 | 61.2 | 135k | 94.3 |
| s2 | 94.3 | 95k | 92.8 |
| s3 | −43.8 | 185k | 78.4 |

- **s3** stayed in the stationary policy (about −59) until about 145k, rose from 160k, and ended at about 95.
- **Reproducibility:** same-seed runs are not bit-identical. s1's curve at 100k differs from the separate 100k run (CPU torch nondeterminism), so the 300k run's 100k checkpoint is not the earlier 100k model.

**Evaluation:**
- Same matrix as the 100k evaluation: 50 episodes per cell, seeds 10000–10049, default task reward, final models.
- The random-world targets are asserted identical to the 100k evaluation's.
- Training draws stayed outside every held-out exclusion disc. The closest approaches were 0.7510 / 0.7508 / 0.7501 m against the 0.75 m radius.

Success rate:

| Policy | Nominal | A | B | C | D | E | Random world | Camera blind |
|---|---|---|---|---|---|---|---|---|
| Baseline | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 |
| E1R 100k s1 | .56 | .72 | .94 | .82 | .36 | .94 | .72 | 0 |
| E1R 100k s2 | .84 | 1.00 | .98 | 1.00 | 1.00 | 1.00 | 1.00 | 0 |
| E1R 100k s3 | .00 | .00 | .26 | .00 | .00 | .04 | .14 | 0 |
| **E1R 300k s1** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | 0 |
| **E1R 300k s2** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | 0 |
| **E1R 300k s3** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | **1.00** | 0 |

- **Collisions:** 0 in every cell.
- **E1R 300k episode length:** 6.2–9.7 s across seeds and positions; random world 8.6–8.9 s. Return is 93.8–96.2 in every success cell. The baseline takes 13.3–16.9 s (return 87.1–89.7).
- **Camera blind:** 0% for every policy (return −60 to −61.5). Expected, and a diagnostic only.
- **Changes from 100k:** no cell regressed. Every 100k cell below 1.00 rose to 1.00, including s2's nominal (0.84 → 1.00).

**Final-approach diagnosis** (`approach_diagnostic.py`, 50 random-world episodes per model):
- Each failure is classified from the per-step detection and estimate history, not assumed: never detected; stalled while the target was visible (never within the camera's 1.05 m near limit); or lost inside 1.05 m.

| Model | Success | Never detected | Stalled visible | Lost in blind zone |
|---|---|---|---|---|
| Baseline | 50/50 | – | – | – |
| E1R 100k s1 | 36/50 | 0 | 0 | 14 |
| E1R 100k s2 | 50/50 | – | – | – |
| E1R 100k s3 | 7/50 | 15 | 18 | 10 |
| E1R 300k s1 / s2 / s3 | 50/50 each | – | – | – |

- **What the blind-zone failures (100k) show:**
  - **Entry:** they entered the near zone off-axis. The target was 53° (s1) or 33° (s3) from the nose, so it left the side of the camera view (half-HFOV about 34°) as well as the bottom.
  - **The estimate was not the problem:** it was valid at entry in 100% of cases and accurate where valid (range error 0.02 m, bearing error 3.5° for s1 and 1.1° for s3).
  - **The policy did not use it:** s1 spun in place (|turn| 0.83 over the first 10 steps, then 0.99, speed ≈ 0). s3 froze (speed ≈ 0, |turn| ≈ 0.04).
  - **Result:** the 8 s estimate then expired (valid in only 18% of blind-zone steps), leaving the vehicle at 0.83–0.85 m, just outside the 0.75 m success radius.
  - **Conclusion:** the near-field visibility limit triggers the failure, but the cause is the policy's response (no forward motion on a valid estimate). Missing or wrong information is not the cause.
- **s3 at 100k also showed the stand-still optimum:** 15 episodes never detected the target and 18 stalled about 2.6 m away with the target visible, both with speed ≈ 0. All three failure classes are absent at 300k.

**Limitations:**
- **Weak held-out test:** the exclusion radius (0.75 m) is small, and training targets came within 0.751 m of held-out positions. Those positions are therefore inside the trained region, and the test measures interpolation, not extrapolation to unseen areas.
- **Single map:** one fixed wall layout and one target size, in the idealized stand-in.
- **Not validated live:** nothing here is validated in Gazebo/PX4, where the live episode is still blocked by the readiness gates.

## Stronger generalization test of E1R 300k (evaluation only, no training)

**Run:** `runs/rl/exp20261022_generalization2/` (`evaluate_generalization.py`/`.out`, `evaluation.json` with all scenarios and episodes, and `trajectories/*.npz` with per-step x, y, heading, distance, det_now, est_valid, speed, turn). No model, checkpoint, training setting, or tracked code was changed.

**Why held-out regions must lie beyond 6 m:** every 30° bearing sector of the training band (2.5–6 m from the start) contains training draws (8107 draws, 100k + 300k runs). No unseen region exists at training distances, so the held-out regions lie beyond it. **Distance is therefore a confound:** these targets are also farther than any training target.

**Scenarios** (50 each; target i is paired with episode seed 10000+i, which fixes the start, identically for every policy):

| Scenario | Targets | Notes |
|---|---|---|
| `region_W` | x ∈ [−8.5, −7.5], y ∈ [−1.5, 2.5] | 7.6–8.7 m from start; ≥ 1.63 m from every training draw |
| `region_S` | x ∈ [−1.5, 2.5], y ∈ [−8.5, −7.5] | 7.5–8.7 m; ≥ 1.52 m |
| `region_SW` | x, y ∈ [−8.5, −6.5] | 9.5–11.9 m; ≥ 3.49 m |
| `reference` | the 50 earlier random-world targets | training layout |
| `layout_open` | same targets | all walls removed |
| `layout_interior` | seeded training-sampler draws ≥ 1 m from the new wall | training walls plus an unseen interior wall x ∈ [−2.4, −2.1], y ∈ [0.5, 4.0]; the wall blocks the start-to-target line in 4/50 scenarios |

- **Exclusion buffer:** 1.5 m (twice the success radius) from every recorded training draw. It is asserted.
- **Region targets:** sampled with seed 20261022 and recorded in `evaluation.json`.
- **Leakage check:** camera-blind observations were identical for two different targets in every scenario (asserted before evaluation).

| Success / collision / timeout | region_W | region_S | region_SW | reference | layout_open | layout_interior |
|---|---|---|---|---|---|---|
| Baseline | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .88/0/.12 |
| E1R 300k s1 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .98/0/.02 | .94/**.06**/0 |
| E1R 300k s2 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .86/**.14**/0 |
| E1R 300k s3 | .94/0/.06 | 1/0/0 | .86/0/.14 | 1/0/0 | .96/0/.04 | .90/**.10**/0 |

Mean episode length (s) and return:

| | region_W | region_S | region_SW | reference | layout_open | layout_interior |
|---|---|---|---|---|---|---|
| Baseline | 20.3 / 87.1 | 20.3 / 87.0 | 22.1 / 87.7 | 15.7 / 87.6 | 15.7 / 87.6 | 21.1 / 70.0 |
| s1 | 12.3 / 95.1 | 12.8 / 94.5 | 14.9 / 94.9 | 9.1 / 94.3 | 15.8 / 85.5 | 8.9 / 82.6 |
| s2 | 12.2 / 95.2 | 12.5 / 94.8 | 13.8 / 96.0 | 8.6 / 94.8 | 8.6 / 94.8 | 7.7 / 67.7 |
| s3 | 16.0 / 85.4 | 12.9 / 94.4 | 29.5 / 66.2 | 9.1 / 94.2 | 17.6 / 81.7 | 8.9 / 74.5 |

Every policy detected the target in 100% of episodes in every scenario.

**Findings (verified from the trajectories):**
- **Unseen regions:** s1 and s2 reached 100% in all three, with no collisions, faster than the baseline. s3 reached 0.94 / 1.00 / 0.86. Each s3 timeout (3 in W, 7 in SW, 2 in the open layout) is the known final-approach failure: the target was detected, the vehicle got to 0.75–0.93 m, then spun in place (|turn| = 1.0, speed 0 over the last 200 steps). The 300k s3 policy's final approach is still fragile off-distribution.
- **Open layout** (LiDAR sees nothing): 0.98 / 1.00 / 0.96. The policies do not depend on the wall layout for search.
- **Interior wall: the first PPO collisions in any evaluation, 6–14%.** 14 of the 15 PPO collisions are with the new interior wall, at final target distances of 1.7–4.3 m. Most occur in the scenarios where the wall blocks the direct line (episodes 9, 14, 25, 33 for every seed).
  - The baseline never collides. Its explicit obstacle stop holds it at the wall, and it times out instead (6/50, stuck behind the wall at 3.4–5.2 m; it has no path planning).
  - Cause: training targets were always in open space, with walls only at the periphery, so PPO never learned to avoid an obstacle on the way to the target. This is a safety-relevant gap.
- **Comparability limitations:**
  - The stand-in camera model has no occlusion, so a target behind the interior wall stays "visible" to both policies. That inflates the pull toward the wall compared with a real camera.
  - Walls were replaced as evaluation-only data; the observation and reward code is unchanged.
  - Region targets are farther than training targets, which confounds distance with novelty.

**Tests:** 264 passed (full offline suite; 53 in the RL/stand-in/observation subset). No tracked code changed.

## LiDAR safety filter around E1R 300k (evaluation only; filter code `081211e`)

**Filter (`safety_layer.LidarSafetyFilter`):** the baseline controller's obstacle rule applied to the policy's action, using only the observation's 49 LiDAR sectors. First match wins:
1. **Invalid LiDAR** (missing, wrong length, non-finite, outside [0, 1]): hover (0, 0).
2. **Stale or unknown LiDAR age** (`None` or > 0.5 s): hover (0, 0).
3. **Any sector < 0.5 m** (emergency): speed 0, turn ±0.8 toward the clearer half (larger mean range).
4. **Front clearance < 1.2 m** (minimum over |angle| ≤ 35°): speed 0, turn ±0.8 toward the clearer half. This is the baseline's stop rule.
5. **Front clearance < 3.0 m:** speed capped at 0.8·(front − 1.2)/1.8, keeping the policy's turn. This is the baseline's slow rule.
6. **Otherwise:** the action passes unchanged.

The filter never increases speed. The original action, the applied action, and an intervention code are logged per step.

**Evaluation:** `runs/rl/exp20261022_safety_layer/` (`evaluate_safety_layer.py`/`.out`, `evaluation.json`, `trajectories/*.npz`).
- **Scenarios:** loaded unchanged from the generalization evaluation: the same targets, walls, and episode seeds for all 7 policies (baseline, PPO s1–3 unfiltered, PPO s1–3 + filter), 50 episodes each.
- **LiDAR age:** the stand-in computes LiDAR every step, so `lidar_age_s = 0.0`.
- **Reproducibility:** the unfiltered PPO and baseline numbers reproduce the generalization evaluation exactly.

Success / collision / timeout:

| | reference | region_W | region_S | region_SW | layout_open | layout_interior |
|---|---|---|---|---|---|---|
| Baseline | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .88/0/.12 |
| s1 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .98/0/.02 | .94/**.06**/0 |
| s1 + filter | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .98/0/.02 | .94/**0**/.06 |
| s2 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .86/**.14**/0 |
| s2 + filter | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | 1/0/0 | .92/**0**/.08 |
| s3 | 1/0/0 | .94/0/.06 | 1/0/0 | .86/0/.14 | .96/0/.04 | .90/**.10**/0 |
| s3 + filter | **.98**/0/.02 | .94/0/.06 | 1/0/0 | .86/0/.14 | .96/0/.04 | .92/**0**/.08 |

Interventions (mean per episode / episodes with any, of 50):

| | reference | region_W | region_S | region_SW | layout_open | layout_interior |
|---|---|---|---|---|---|---|
| s1 + filter | 3.4 / 11 | 0.1 / 2 | 0 / 0 | 0 / 0 | 0 / 0 | 43.4 / 22 |
| s2 + filter | 4.0 / 12 | 0 / 0 | 0.4 / 6 | 0 / 0 | 0 / 0 | 67.7 / 40 |
| s3 + filter | 3.9 / 12 | 0.7 / 3 | 0 / 0 | 0 / 0 | 0 / 0 | 54.8 / 23 |

- **By reason:** only `slow` outside the interior layout. In the interior layout, `slow` 1297 / 2653 / 1913 and `stop` 871 / 731 / 829 across all steps; one `emergency` (s1). No invalid or stale LiDAR (not expected in the stand-in).
- **Mean duration of successful episodes (s):** unchanged where the filter rarely acts (all regions and the open layout identical). Reference +0.1–0.2 s. Interior wall 9.2 → 10.2 (s1), 8.0 → 10.3 (s2), 9.0 → 10.4 (s3). The baseline is 15.7–22.1 s.

**Findings:**
- **Collisions eliminated:** all 15 interior-wall collisions (6 / 14 / 10%) became 0, and no collisions appeared anywhere else. Interior-wall success: s1 .94 → .94, s2 .86 → .92, s3 .90 → .92. Of the 15 avoided collisions, 5 became successes and 10 became timeouts.
- **The filter is not path planning.** Every new interior timeout ends with the vehicle held about 1.2 m from the interior wall (0.6 m in two s3/s2 cases), the target 2.4–5.2 m away behind it, and stop interventions in 31–56% of the last 200 steps. The policy keeps pushing toward the target (visible through the wall, since the stand-in camera has no occlusion), the filter keeps stopping it, and the vehicle stays stuck. The unfiltered baseline fails the same way (6 timeouts).
- **One regression:** s3 reference episode 20 went from success to timeout. Its only interventions were `slow` caps. Near the end no filter action was active (applied speed 0 came from the policy), and it ended at 0.86 m. The perturbed approach exposed s3's known final-approach spin, not a safety stop.
- **Limitations:**
  - This is the idealized stand-in: point-mass dynamics with a 0.45 s velocity lag, fresh noiseless LiDAR every step, no camera occlusion.
  - The 1.2 m stop distance and 0.5 m emergency margin are the baseline's and have not been validated against PX4 braking in SITL.
  - Stale and invalid LiDAR handling is unit-tested only.
  - The filter is not wired into the live env or the flight runner.

## Camera occlusion (stand-in; code `dd9924d`) and re-evaluation of the 300k policies

All results in this section are **stand-in results only**. They say nothing about PX4/Gazebo flight readiness.

**Model:** `SarStandInEnv(camera_occlusion=True)`. After the field-of-view check, the target is undetectable when all three sightlines from the camera (body x offset 0.19 m) cross a wall box: to the target centre, and to both horizontal edges (offset by the 0.5 m radius perpendicular to the line of sight). This is a 2D test; walls are modelled as taller than the camera, which holds for the world's 15 m walls against a 1.87 m camera. A partly visible target stays detectable. The option is off by default. 20 tests cover geometry edge cases, blocked/clear/re-visible, partial visibility, determinism, exact reproduction when off, and no leakage. Full suite: 304 passed.

**Evaluation:** `runs/rl/exp20261023_occlusion_eval/`. Occlusion on, the same six scenarios, targets, and seeds as the safety-filter evaluation, 7 policies × 50 episodes. Occluded-frame counts and path lengths are recorded per episode.
- **Occlusion acted only where intended.** It triggered in 0 episodes in the reference, held-out-region, and open scenarios, which reproduce the no-occlusion evaluation episode for episode (35/35 cells identical). In the interior-wall layout it triggered in 2–6 of 50 episodes, and only those cells changed.

Interior-wall layout, success / collision / timeout, without → with occlusion:

| | Without | With occlusion |
|---|---|---|
| Baseline | .88/0/.12 | .88/0/.12 (5 episodes changed internally; same totals) |
| s1 | .94/.06/0 | **1.00/0/0** |
| s1 + filter | .94/0/.06 | .94/0/.06 |
| s2 | .86/.14/0 | .88/**.12**/0 |
| s2 + filter | .92/0/.08 | .96/0/.04 |
| s3 | .90/.10/0 | .90/.02/.08 |
| s3 + filter | .92/0/.08 | .90/0/.10 |

**Per-episode diagnosis (trajectories):**
- **s1:** its 3 former collisions became successes. With the target hidden, it stopped being pulled straight into the wall, then saw the target around the wall's end.
- **s3:** 4 former collisions became timeouts with **0 detections**. The target stayed hidden and its search never found it.
- **s2 still collides in 6 episodes.** Two of them had 0–1 occluded frames: the target was visible past the wall's end, and the vehicle cut the corner into the wall. The others had short detections followed by a direct approach into the wall.
- **Conclusion (confirmed):** occlusion removes the see-through artefact. The remaining collisions show that the 300k policies lack obstacle avoidance independently of the camera model. The safety filter still prevents every collision.

## Randomized-obstacle training, E2O (stand-in only; code `505048d`, generator fix `7b88a64`)

All results here are **stand-in results**. They do not establish PX4/Gazebo flight readiness.

**Generator** (`SarStandInEnv(randomize_obstacles=True)`, opt-in):
- **Draw:** 4–7 axis-aligned walls per episode, after the target and before the start. Each is 2.0–4.5 m long and 0.3 m thick (tall, so occlusion is 2D), with centres in NED x, y ∈ [−6.5, 3.5].
- **Clearances:** ≥ 0.8 m from the whole start square [−1, 1]², ≥ 1.0 m from the target centre, and ≥ 1.2 m gap to every other wall. Separated convex obstacles cannot enclose a region, so the target is always reachable.
- **Collision geometry:** the existing LiDAR proximity check (< 0.25 m).
- **Independence:** placement is independent of the target except for its clearance. Line of sight is blocked in 12.5% of episodes, against 26.7% for a phantom target from the same sampler, so obstacles are *not* preferentially between start and target. Residual bias: the target clearance keeps targets in slightly more open space.
- **Density:** an obstacle lies within 1 m of the straight path in about 33% of episodes. The first density setting (2–4 obstacles) gave only 4.7% blocked and was rejected as near-trivial before any training.
- **Reproducibility:** layouts are recorded (`sampled_layouts.json`) and reproducible by seed.
- **Fix `7b88a64`:** the generator could rarely fail to place an obstacle, which crashed seed 2 at about 60k steps. It now redraws the whole layout instead. This is verified identical to `505048d` over 3000 seeds wherever the old code succeeded, so seeds 1 and 3 are unaffected. The crashed partial run is kept in `E2O_s2/` with `CRASHED_README.txt`; seed 2 was rerun as `E2O_s2r`.
- **Tests:** 11 in `test_randomized_obstacles.py`. Full suite: 315 passed.

**Training:** `runs/rl/exp20261023_obstacles300k/`. `run_train.sh` with `CODE_REVISION` (505048d) for s1/s3 and `CODE_REVISION_s2r` (7b88a64) for s2r.
- From scratch (random init), 300k steps, seeds 1/2/3.
- E1R settings (progress weight 10, randomized target and start, unchanged PPO hyperparameters) plus `--camera-occlusion --randomize-obstacles` for training, best-model selection, and final evaluation.
- Checkpoints: `E2O_s1/`, `E2O_s2r/`, `E2O_s3/` (`final_model.zip`, `best_model.zip`, `checkpoints/` every 10k).
- 1673 / 1849 / 1876 training layouts.
- Learning curves stayed noisy to 300k (s1 swings 41–95 after 200k, s2r ends at 28 after 95 at 200k, s3 58–95), unlike E1R's plateau at about 95.

**Evaluation:** `evaluation/evaluate_obstacles.py`, `evaluation/evaluation.json`, `evaluation/trajectories/`.
- **Occlusion on** for every policy and scenario. 50 episodes per cell; target i is paired with episode seed 10000+i.
- **Scenarios:** the six earlier scenarios; `layout_twowall` (held-out hand design: interior walls (−2.4, −2.1, 0.5, 4.0) and (−4.9, −4.6, −4.0, −0.5), sampler targets ≥ 1 m clear, seed 20261023); and `gen_heldout` (50 generator layouts at episode seeds 10000–10049, disjoint from the training streams, asserted identical across policies and recorded).
- **Policies:** baseline, E1R 300k ± filter, E2O 300k ± filter.

Success / **collision** / timeout:

| Policy | reference | region_W | region_S | region_SW | layout_open | layout_interior | layout_twowall | gen_heldout |
|---|---|---|---|---|---|---|---|---|
| baseline | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.88/0/0.12 | 0.80/0/0.20 | 0.76/0/0.24 |
| E1R_s1 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.98/0/0.02 | 1.00/0/0.00 | 0.86/**0.14**/0.00 | 0.82/**0.04**/0.14 |
| E1R_s1+filter | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.98/0/0.02 | 0.94/0/0.06 | 0.82/0/0.18 | 0.76/0/0.24 |
| E1R_s2 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.88/**0.12**/0.00 | 0.86/**0.14**/0.00 | 0.82/**0.06**/0.12 |
| E1R_s2+filter | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.96/0/0.04 | 0.86/0/0.14 | 0.72/0/0.28 |
| E1R_s3 | 1.00/0/0.00 | 0.94/0/0.06 | 1.00/0/0.00 | 0.86/0/0.14 | 0.96/0/0.04 | 0.90/**0.02**/0.08 | 0.80/**0.08**/0.12 | 0.62/**0.24**/0.14 |
| E1R_s3+filter | 0.98/0/0.02 | 0.94/0/0.06 | 1.00/0/0.00 | 0.86/0/0.14 | 0.96/0/0.04 | 0.90/0/0.10 | 0.84/0/0.16 | 0.72/0/0.28 |
| E2O_s1 | 0.84/0/0.16 | 1.00/0/0.00 | 0.80/0/0.20 | 0.92/0/0.08 | 0.98/0/0.02 | 0.74/**0.06**/0.20 | 0.78/**0.04**/0.18 | 0.80/**0.04**/0.16 |
| E2O_s1+filter | 0.84/0/0.16 | 1.00/0/0.00 | 0.82/0/0.18 | 0.92/0/0.08 | 0.98/0/0.02 | 0.72/0/0.28 | 0.78/0/0.22 | 0.76/0/0.24 |
| E2O_s2 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.90/**0.06**/0.04 | 0.86/**0.08**/0.06 | 0.80/**0.06**/0.14 |
| E2O_s2+filter | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 1.00/0/0.00 | 0.90/0/0.10 | 0.82/0/0.18 | 0.74/0/0.26 |
| E2O_s3 | 1.00/0/0.00 | 1.00/0/0.00 | 0.64/0/0.36 | 0.66/0/0.34 | 0.00/0/1.00 | 0.92/0/0.08 | 0.82/**0.02**/0.16 | 0.80/**0.06**/0.14 |
| E2O_s3+filter | 0.98/0/0.02 | 1.00/0/0.00 | 0.64/0/0.36 | 0.66/0/0.34 | 0.00/0/1.00 | 0.90/0/0.10 | 0.82/0/0.18 | 0.76/0/0.24 |

Path length, episode length, return, interventions, and occluded frames per cell are in `evaluate_obstacles.out` and `evaluation.json`.

**Confirmed findings:**
- **E2O did not deliver reliable obstacle avoidance.** Unfiltered collision rates, E2O vs E1R per seed:
  - interior wall: .06/.06/0 vs 0/.12/.02;
  - two-wall: .04/.08/.02 vs .14/.14/.08;
  - generated held-out layouts: .04/.06/.06 vs .04/.06/.24.

  Collisions fall on the two-wall layout and for s3 on generated layouts, but none reach zero, and interior-wall s1 got worse.
- **The safety filter still prevents every collision**, for E1R and E2O, in every scenario. It converts them into timeouts (for example gen_heldout E2O + filter: .76/.74/.76 success with .24/.26/.24 timeouts).
- **Success on obstacle layouts:** E2O 0.80 / 0.80 / 0.80 against E1R 0.82 / 0.82 / 0.62 and the baseline 0.76.
- **Regressions:** E2O s1 reference 0.84 (E1R 1.00); E2O s1 region_S 0.80; E2O s3 region_S/SW 0.64/0.66; **E2O s3 no-wall layout 0.00 (50/50 timeouts)**.
- **Cause of every one of those regression failures (trajectories):** the final-approach failure. The target was detected and approached to 0.75–0.95 m, just outside the success radius and inside the camera's ~1.05 m near limit, then the vehicle circled (mean |turn| 0.6–0.98, speed ≤ 0.15) until timeout. E1R at 300k had overcome this; E2O, on a harder task, has not within the same 300k steps.

**Hypotheses (not verified):**
- E2O s3's total failure without walls suggests its final approach depends on LiDAR returns that were always present in training. The open layout is out of distribution for it.
- E2O is under-trained (non-converged curves). Longer training might recover both the final approach and the avoidance, but this experiment does not show that.
- The task geometry makes the final approach intrinsically hard. A 0.75 m success radius sits inside the camera's ~1.05 m blind range, so the last metres depend on the 8 s dead-reckoned estimate. Changing the task would require explicit approval.

**Status:** the best stand-in configuration remains E1R 300k + the LiDAR safety filter: 0 collisions everywhere, 1.00 on open-space scenarios, 0.72–0.96 on obstacle layouts where it times out instead of colliding. None of this is validated in PX4/Gazebo, where the live episode is still blocked by the readiness gates.
