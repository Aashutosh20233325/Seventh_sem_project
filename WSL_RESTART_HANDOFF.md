# WSL restart handoff — 2026-10-23 (prepared ~17:20 UTC)

Prepared before a manual `.wslconfig` change (memory 4 GB → 5 GB) and `wsl --shutdown`. **No flight was authorized
or performed as part of this task**, and nothing was armed. Nothing in this document is a secret.

## Repository

- **Location:** `/home/hp/uav_ros`, branch `master`.
- **Commit:** the commit that adds this file. Its parent is `74f783b` ("Document split-rendering option and per-process render-check in the demo checklist").
- **Working tree:** clean after that commit. No stashes, no uncommitted or untracked source changes.
- **Recent milestones:**
  - `2574773`: runtime speed/memory guard in the episode runner.
  - `081211e`: LiDAR safety filter (evaluation only; not wired live).
  - `dd9924d`: stand-in camera occlusion (opt-in).
  - `505048d` + `7b88a64`: randomized obstacles (opt-in).
  - `7f27985`: checkpoint/live interface tests.
  - `b9b4588`: `--split-rendering` + `render-check`.
  - `e277170`, `74f783b`: `docs/DEMO_CHECKLIST.md`.
- **Full offline suite at `74f783b`:** 337 passed (ROS sourced).
- **Other repos (not modified by this project):**
  - `/home/hp/PX4-Autopilot` at `2fc441493e` has a **pre-existing** modification inside the `Tools/simulation/gz` submodule. Left untouched.
  - `/home/hp/Micro-XRCE-DDS-Agent` (`7362281`) and `/home/hp/px4_ros2_ws/src/px4_msgs` (`72fcfaa`) are clean.

## Where evidence lives (all on the persistent home filesystem; nothing only in memory)

| What | Path |
|---|---|
| Worklog (live runs, gates, memory) | `runs/sar_demo_20261018/worklog.md` (committed) |
| Runbook / demo checklist | `runs/sar_demo_20261018/RUNBOOK_ISOLATED.md`, `docs/DEMO_CHECKLIST.md` |
| RL experiment report | `docs/RL_EXPERIMENTS.md` |
| SITL run directories (logs, telemetry, screenshots; git-ignored, ~5.1 GB) | `runs/sar_demo_20261018/restart_*` (latest: `restart_20261023_split1`) |
| RL runs, checkpoints, evaluations (git-ignored, ~115 MB) | `runs/rl/` (see below) |
| External experiment evidence | `/home/hp/uav_ros_experiments/` (preflight_guard_20261022, preflight_split_20261023, mesa_d3d12_leak_20261022, gz_memory_growth_20261018, wsl_memory_20261018, …) |
| Archived session scratchpad (`/tmp` may be cleared at boot) | `/home/hp/uav_ros_experiments/session_scratchpad_archive_20261023/` |
| `.wslconfig` backups / 5 GB proposal | `/home/hp/uav_ros_experiments/wsl_memory_20261018/` (`wslconfig_backup_20261018T222129Z` = `memory=4GB swap=4GB`; `wslconfig_proposed_5GB` = `memory=5GB swap=4GB`) |

**Key RL checkpoints** (`final_model.zip`, `best_model.zip`, `checkpoints/`, `config.json`, `eval_summary.json`):
- **E1R 300k (best stand-in policy):** `runs/rl/exp20261022_randtarget300k/E1R_s{1,2,3}/`. Recommended: s2. All seeds score 1.00 on held-out targets and regions, but have no obstacle avoidance without the filter.
- **Evaluations:** `runs/rl/exp20261022_generalization2/`, `exp20261022_safety_layer/`, `exp20261023_occlusion_eval/`.
- **E2O (randomized obstacles):** `runs/rl/exp20261023_obstacles300k/E2O_s1/`, `E2O_s2r/`, `E2O_s3/`. Not better than E1R + filter. `E2O_s2/` is a crashed partial run kept as evidence.
- **Earlier:** `runs/rl/exp20261022/` (E0/E1/E2, 100k), `exp20261022_randtarget/` (E1R 100k), `ppo_smoke_20261018_s1/`.

## Demo status and blockers

**Goal:** a visible Gazebo/PX4 run in which the drone takes off, detects the red target with the camera, flies to it, reaches it, and lands. The live runner (`run_target_reaching_episode.py`) flies the **baseline controller**; it has no PPO path.

**Latest measurements:**
- **2026-10-22, GPU GUI, `restart_20261022_preflight1`:**
  - Stability windows 0/6: camera sim speed **0.334–0.347×**, below the unchanged **0.35×** gate (PX4 clock ~0.40×). All other gates passed.
  - MemAvailable fell **~137 MiB/min** (1517 → 1036 MiB in 3.5 min); gz server about +50 MiB/min, gz GUI about +80 MiB/min.
- **2026-10-23, `--split-rendering`, `restart_20261023_split1`:**
  - The renderer split is verified manually: server on d3d12/NVIDIA, GUI on llvmpipe.
  - The GUI shows the scene, but it probably does not auto-follow the drone.
  - `render-check` failed because of a **parser bug**: gz's NUL-padded single-string cmdline. The fix is not yet approved.
  - Speed was **not measured**.
  - MemAvailable reached **1118 MiB at +60 s**, below the runtime guard's **1.2 GiB start threshold**; the llvmpipe GUI uses ~830 MiB.
- **Implication for the WSL change:** at 4 GB, both GUI modes fail the memory start threshold quickly. A 5 GB VM adds about 1 GiB of headroom. That helps the memory gate but **does not by itself fix the 0.35× camera-speed gate**.

## After Ubuntu restarts (setup and next steps)

```bash
cd /home/hp/uav_ros
git log --oneline -1 && git status --short            # expect this handoff commit, clean tree
free -m                                               # confirm the new VM memory (expect ~5 GB total if applied)
source /opt/ros/jazzy/setup.bash
source /home/hp/px4_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
  SAR_LIDAR_TOPIC=/world/sar_walls/model/x500_lidar_2d_oakd_1/link/link/sensor/lidar_2d_v2/scan
.venv/bin/python -m pytest -q -p no:cacheprovider     # offline suite; expect 337 passed
```

Relaunch QGroundControl. Its GCS heartbeat is required for PX4 preflight (`NAV_DLL_ACT=2`), so never close it during a run. The previous launch, recorded in `runs/sar_demo_20261018/restart_20261018_oakd2/qgroundcontrol.process.json`, ran from `runs/sar_demo_20261018/restart_20261018/` with `XDG_CONFIG_HOME=$PWD/qgc_config XDG_CACHE_HOME=$PWD/qgc_cache` under `tools/owned_process_group.py`, using `./QGroundControl-x86_64.AppImage --appimage-extract-and-run`. Record the new PIDs in a **new** run directory; do not overwrite old records.

**Recommended order (each needs explicit user approval):**
1. Fix the `render-check` parser (code + test from the real cmdline format). Offline only.
2. One no-flight readiness run with the documented gates. Choose `--split-rendering` or the default GPU GUI, measuring camera/PX4 speed per window and memory against the unchanged 0.35× and 1.2 GiB thresholds. Orchestration copies: `/home/hp/uav_ros_experiments/preflight_split_20261023/run_preflight_split.sh` (split) and `preflight_guard_20261022/run_preflight.sh` (default). Use a **new** run name; never reuse a run directory.
3. Only if every gate passes: present the evidence and wait for flight approval. The episode command is in `docs/DEMO_CHECKLIST.md`, marked DO NOT RUN WITHOUT EXPLICIT APPROVAL.

## Processes at handoff

- **Not running:** PX4, Gazebo, XRCE agent, bridges, ROS consumers, recorder, training, evaluation. The last stack was stopped through the telemetry-gated stop at 2026-10-23 17:02:26 UTC: disarmed, landed, at rest.
- **Will be ended by `wsl --shutdown` (all safe to lose):**
  - QGroundControl (owned group 58696, QGC 58699), idle;
  - an idle `ros2` CLI daemon for domain 43 (started by read-only `ros2` queries);
  - Ubuntu's unattended-upgrades shutdown helper.
- **Windows side:** nothing in this project needs a Windows application to keep running. Close or save anything unrelated in other WSL distributions before `wsl --shutdown`, because it stops all of them.
