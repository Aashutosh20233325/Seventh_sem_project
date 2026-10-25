# UAV search-and-reach with PPO in PX4 SITL + Gazebo: project state report

**Date:** 2026-10-25 · **Repository:** `/home/hp/uav_ros` · **Git tag for this state:** `demo-2026-10-25` (on
`master`) · **Demo kit commit:** `80ed7e5` · **Code through Stage C:** `cc9d3fa`

This report records where the project stands, what was shown in live (simulated) flight, and exactly how to return to
this state and repeat the demo.

## 1. What the system does

A PX4 x500 quadcopter in Gazebo Harmonic (a 2D LiDAR and an OAK-D Lite camera pitched 30° down) takes off to about
1.8 m, flies to a start point, then **searches for and flies to a red target sphere** using a reinforcement-learning
policy (PPO). It returns to the start, lands and disarms.

- **Policy:** E1R 300k seed 2, a PPO MLP (65 → 64 → 64 → 2, tanh) trained in a fast 2D stand-in simulator with
  randomized targets.
- **Observation (`sar_obs_v1`, 65 values):** 49 LiDAR sectors, previous action, body velocity and height, current
  camera detection, and an 8 s dead-reckoned target estimate. No target coordinates are given to the policy.
- **Action:** forward speed and turn rate.
- **For flight:** the policy runs as a torch-free numpy export, matching stable-baselines3 to within 6×10⁻⁷, behind a
  **LiDAR safety filter**. The filter only reduces speed or replaces the turn, and it hovers if the LiDAR data is
  invalid, unknown or stale (older than 0.5 s).
- **Safety around each flight:**
  - readiness gates: telemetry, world target, world consistency, a strict check that nothing else publishes to PX4,
    and the renderer;
  - six 25 s simulation-speed windows (≥ 0.35×);
  - a fail-closed pre-flight state check;
  - a runtime guard (simulation speed and memory) during the episode;
  - landing confirmation, and a gated shutdown that never uses SIGKILL.

## 2. Results (all live in PX4 SITL + Gazebo, 2026-10-25)

| Stage | What changed | Runs | Result |
|---|---|---|---|
| Baseline | first live episode (baseline controller) | `restart_20261025_baseline1` | success, 64 steps |
| A: repeatability | PPO, same start and target | `ppo1`–`ppo4` | **4/4 success**, 38–41 steps |
| B: turned start headings | start rotated +90°, 180°, −90° | `stageb1`–`stageb3` (PPO), `stageb4` (baseline) | **3/3 PPO success** (36 / 86 / 62 steps); baseline 205 |
| C: moved targets | target at three new positions (world variants) | `stagec_*_passive` (3, no flight), `stagec_c2_sw_ppo`, `c5_w_ppo`, `c1_ne_ppo`, `c2_sw_baseline` | passive 3/3; **PPO 3/3 success** (61 / 66 / 29 steps); baseline 101 |
| Demo kit live test | the kit in this report, end to end | `kittest_1_ppo_default_0`, `kittest2_1_ppo_default_180` | **2/2 success** (38 / 90 steps) |

**In total:** 13 of 13 live PPO episodes succeeded (11 before the kit test, 2 through the kit). There were no collisions, no runtime-guard
aborts, and landing and disarm were confirmed every time. PPO needed 1.6–7× fewer steps than the baseline in
comparable cases. When the target started out of view, the policy searched (turning right) and then approached
directly.

**Findings that matter for the demo:**
- **Host memory decides whether the simulation is fast enough.** Windows has 7.86 GB of RAM and WSL is capped at
  5 GB. With browsers open, Windows pages heavily and the simulation drops to about 0.25–0.3× (below the 0.35×
  floor). With them closed it runs at about 0.5×.
- **The live camera sees wider than the stand-in models.** It detects the target about 41–46° off-axis, so targets
  about 45° to one side are sometimes already visible at the start (b1, c1_ne).
- **Limitations:**
  - one wall layout with no interior obstacles;
  - the policy is a memoryless reflex (no map), so it can't plan around obstacles;
  - validation was in simulation only;
  - live results are per episode, not statistics. Statistics (50 episodes per case) exist only for the stand-in.

## 3. Key commits (branch `master`)

| Commit | Content |
|---|---|
| `837a876` | Step timing, fly-to and runtime guard from validated simulation clocks |
| `5df1273` | Safe `close()` when flight was never commanded |
| `18ebe00` | Runner `--policy` path behind the mandatory LiDAR safety filter; `PX4UAVEnv.observation_age()` |
| `1fac16b` | Torch-free numpy policy export (`numpy_policy.py`), automatic in training |
| `4da38f5` | Validated start-heading offset (`--start-yaw-offset-deg`) in both envs |
| `231fc3a` | World variants (target moved), `SAR_WORLD_VARIANT` selector, `--world-variant`, world-consistency gate |
| `9e66198`, `ec6d337`, `6eb3882`, `8a6fd34`, `cc9d3fa` | Runbook, Stage C plan, and records of every live stage (`runs/sar_demo_20261018/worklog.md`) |
| `80ed7e5` | **Demo kit** (`tools/demo/`) and the flown policy files (`models/e1r_300k_s2/`) |
| tag `demo-2026-10-25` | this report and the kit's live-test record |

The full test suite (`.venv/bin/python -m pytest`, with ROS sourced) passes at the tag.

## 4. How to reproduce the demo

### Prerequisites (outside git, already on this machine)
- **OS and simulators:** WSL2 Ubuntu 24.04, ROS 2 Jazzy, PX4-Autopilot SITL build (`/home/hp/PX4-Autopilot`),
  `px4_ros2_ws` with `px4_msgs` (`/home/hp/px4_ros2_ws`), Gazebo Harmonic, the Python venv `/home/hp/uav_ros/.venv`.
- **QGroundControl** v5.1.5 AppImage with its settings in `runs/sar_demo_20261018/restart_20261018/` (git-ignored).
- **Windows:** close Chrome and other memory-heavy applications before flying. If memory is short anyway, the
  stability windows catch the slow simulation and the run stops before flying.

### Commands
```bash
cd /home/hp/uav_ros
git checkout demo-2026-10-25             # or master at/after this tag; the tree must be clean
bash tools/demo/start_qgc.sh             # starts QGroundControl under its owned guard unless already running

# one supervised flight (arms and flies: a human must watch Gazebo/QGroundControl)
bash tools/demo/run_demo.sh profdemo1 ppo default 180 > profdemo1.log 2>&1
.venv/bin/python tools/demo/check_episode.py profdemo1.log runs/sar_demo_20261018/restart_profdemo1/logs/episode.jsonl default ppo 180

# or a series, back to back, with the stop rules applied after each run
bash tools/demo/run_demo_series.sh profdemo "ppo default 0" "ppo default 180" "ppo c2_sw 0" "baseline default 0"
```

**Arguments of `run_demo.sh <name> <mode> [world] [offset]`:**
- **mode:** `ppo`, `baseline`, or `passive` (all checks, never flies).
- **world:** `default`, `c1_ne`, `c2_sw` or `c5_w` (target moved).
- **offset:** `0`, `90`, `180` or `−90` degrees (start heading, positive = right).

Each run takes about 7 minutes: 3.5 min of checks, 1–1.5 min of flight, and about 1.7 min of shutdown.

**To abort during a flight:** `kill -INT $(cat runs/sar_demo_20261018/restart_<name>/logs/runner.pid)`. The runner's
`close()` then lands and confirms disarm. Never use SIGKILL.

**Where the evidence goes:** the run directory `runs/sar_demo_20261018/restart_<name>/` holds `logs/episode.jsonl`
(every step: action, filter decision, detection), the gates, the stability windows and the shutdown record.

### Live test of this kit (before it was saved)
The kit was flown live from the `demo-kit` branch, from commits `80ed7e5` and `f40b56f`, before it was saved to `master`:

| Run | Command (series) | Result |
|---|---|---|
| `restart_kittest_1_ppo_default_0` | `run_demo_series.sh kittest "ppo default 0" …` | success, 38 steps, filter: 9 speed caps, landed and disarmed |
| `restart_kittest2_1_ppo_default_180` | `run_demo_series.sh kittest2 "ppo default 180" …` | success, 90 steps, filter: 10 speed caps, landed and disarmed |

**Not flown through the kit:** the moved-target world and the baseline mode. Flying stopped there at the user's
request. Both were flown live the same day through the identical procedure (`run_stagec_lean.sh`, of which
`run_demo.sh` is a line-for-line port, checked by `test_demo_kit.py`): `stagec_c1_ne_ppo` and
`stagec_c2_sw_baseline`, both successes.

### What each file in `tools/demo/` is

| File | Purpose |
|---|---|
| `run_demo.sh` | One supervised run with every gate. It is the flown procedure line for line, with repo paths, argument checks and the optional start offset |
| `run_demo_series.sh` | Several runs back to back; stop rules after each; one re-attempt after a readiness failure; no retry of a flown episode |
| `check_episode.py` | Stop rules: success, no collision, no guard refusal or abort, only `slow`/`stop` filter interventions, numpy policy without torch, confirmed landing, stack stopped, correct world and start heading |
| `start_qgc.sh` | Starts QGroundControl under an owned process-group guard |
| `guard_observer.py`, `measure_light.py`, `evaluate_window.py`, `fmu_in_gate_parse.py`, `preflight_state.py` | Passive observer, stability-window measurement and evaluation, strict `/fmu/in` gate parser, pre-flight state check |

## 5. Steps to return to this state

Starting in `/home/hp/uav_ros`:

> Restore and demonstrate the supervised PPO search-and-reach demo from `docs/PROJECT_STATE_REPORT.md`. Check out git
> tag `demo-2026-10-25` with a clean tree, and confirm the tests pass (`.venv/bin/python -m pytest` with
> `/opt/ros/jazzy/setup.bash` and `/home/hp/px4_ros2_ws/install/setup.bash` sourced). Make sure QGroundControl is
> running (`tools/demo/start_qgc.sh`), and remind me to close memory-heavy Windows applications (never close them
> yourself). Do not add any extra checks, waits or recording without asking me. Ask for my explicit approval before any flight.
> After I approve, run `tools/demo/run_demo_series.sh <name> "ppo default 0" "ppo default 180" "ppo c2_sw 0"` and
> report each result from `check_episode.py`. Never use SIGKILL or broad kill commands, never retry a flown episode,
> and stop and report on any stop rule.

## 6. Next step (planned, not started)

The next step is a more general 3D environment with terrain and dense obstacles (ruined buildings, rubble, wrecks,
trees) for a resupply / search-and-rescue mission. It needs:
- **an environment generator** that produces matching Gazebo worlds and stand-in layouts, with solvability checks;
- **terrain-relative height**, with climb and descend actions;
- **3D perception** (depth camera);
- **memory or mapping**, and path planning: a hybrid planner plus learned detection is the suggested route;
- **a delivery-and-return mission.**

See `docs/STAGE_C_PLAN.md` and the planning notes from 2026-10-25.
