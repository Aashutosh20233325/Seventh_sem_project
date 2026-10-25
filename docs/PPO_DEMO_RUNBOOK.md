# Supervised PPO demo runbook (PX4 SITL + Gazebo, WSL2)

How to run the visible search-and-reach demo that flew 7/7 on 2026-10-25 (records:
`runs/sar_demo_20261018/worklog.md`, runs `restart_20261025_ppo1`–`ppo4` and `stageb1`–`stageb3`). Every live
flight needs **explicit approval** from the person supervising. Background, gates and history:
`docs/DEMO_CHECKLIST.md`.

## What the audience sees

The x500 (LiDAR plus OAK-D camera) takes off to about 1.8 m, flies to the start point and turns to the start heading.
The PPO policy then searches (it turns right when the red sphere is out of view) and flies to the sphere. Within
0.75 m the episode succeeds, the drone returns to the start and lands, and it disarms.

| Start offset | Drone faces | Target | Live PPO steps | Episode (wall) |
|---|---|---|---|---|
| 0° (default) | east | 45° right, in view | 38–41 | about 22 s |
| −90° | north | 135° right, out of view | 62 | about 29 s |
| 180° | west | 135° left, out of view | 86 | about 34 s |

The baseline controller searches the same way but takes about 1.6–2× the steps (64 at 0°). For a demo that shows
search, use 180° or −90°. +90° (target 45° left) is at the live camera's edge of view, so it may or may not need a search.

## 1. Before anything (Windows host)

1. **Close Chrome and other memory-heavy applications.** The host has 7.86 GB RAM, and WSL is capped at 5 GB. With
   browsers open, Windows pages heavily and the simulation drops below the 0.35× readiness floor.
2. **QGroundControl must be running** (its GCS heartbeat is required; see `RUNBOOK_ISOLATED.md`). Never close it during a run.
3. Repo at a committed state with a clean tree (`git status`), and no PX4/gz processes running.

(If Windows is short of memory anyway, the six stability windows of each run catch the slow simulation and the run
stops before flying.)

## 2. One supervised flight

Everything is in the repo under `tools/demo/` (see `docs/PROJECT_STATE_REPORT.md`):

```bash
cd /home/hp/uav_ros
bash tools/demo/start_qgc.sh                         # only if QGroundControl is not running yet
bash tools/demo/run_demo.sh profdemo1 ppo default 180 > profdemo1.log 2>&1
.venv/bin/python tools/demo/check_episode.py profdemo1.log runs/sar_demo_20261018/restart_profdemo1/logs/episode.jsonl default ppo 180
# several runs back to back, stop rules applied after each:
bash tools/demo/run_demo_series.sh profdemo "ppo default 0" "ppo default 180" "ppo c2_sw 0" "baseline default 0"
```

`run_demo.sh <name> <ppo|baseline|passive> [default|c1_ne|c2_sw|c5_w] [0|90|180|-90]`: mode, world (target position)
and start-heading offset.

`run_demo.sh` does, in order, and stops fail-closed at any failure:
1. Checks the policy file's sha256 (`models/e1r_300k_s2/final_policy_numpy.npz`, `fbcacf93…0511`).
2. Starts the WSL memory watchdog.
3. Starts the stack with split rendering (and the chosen world).
4. Runs the readiness gates: topics, world target, world consistency, strict `/fmu/in`, and render-check.
5. Runs six 25 s stability windows (≥ 0.35×; gap limits).
6. Stops the passive observer and runs the fail-closed pre-flight state check.
7. Runs the runner **once** with `--policy` (numpy, LiDAR safety filter always on), with no retry.
8. Runs the gated stop.

Timeline: about 3.5 min of readiness, 1–1.5 min of flight, about 1.7 min of shutdown. The policy's flight process
uses about 66 MiB (torch is never loaded).

## 3. During the flight: what to watch, and how to abort

- **Gazebo:** smooth take-off, the turn at the start, then the search and approach. The policy keeps at least about
  2 m from walls; the filter caps speed below 3 m.
- **Abort:** if anything looks wrong, send SIGINT to the runner's exact PID (`logs/runner.pid` in the run
  directory). The runner then calls `close()`, which lands and waits for a confirmed disarm. **Never use SIGKILL**, broad
  kills, or a force-disarm.
- The runtime guard aborts on its own (then lands) if speed falls below 0.35× or MemAvailable below 600 MiB.

## 4. After the flight

- `check_episode.py` must print `ok …`. Anything else means stop and investigate before another flight.
- **Normal shutdown pattern:** gated stop attempt 1 confirms fresh landed, at-rest and disarmed telemetry, then reports
  "Bounded shutdown incomplete" (PX4/gz outlast the bounded stop; no SIGKILL). Attempts 2–5 refuse (no telemetry left).
  The stack then exits on its own, and the script prints "stack stopped".
- **Several flights in a row:** `tools/demo/run_demo_series.sh` runs them back to back. It checks before each run that no stack
  is running, QGroundControl is up and the tree is clean. It applies the stop rules after each run, re-attempts once after a
  readiness failure (no flight), and never retries a flown episode.

## 5. If something fails

| Symptom | Likely cause | Action |
|---|---|---|
| Stability windows below 0.35× or long gaps | Windows short of memory (browsers open) | Close the applications and start a new run |
| `guard_refuse` before take-off | MemAvailable below 1.2 GiB or speed below 0.35× | Do not retry immediately; check host memory |
| `POLICY CHECK FAILED` / `OFFSET REFUSED` | Wrong file or argument | Nothing started; fix the argument |
| Gated stop refuses before PX4 exits | Telemetry not landed/disarmed | Do not stop anything; wait, and follow `DEMO_CHECKLIST.md` recovery |

## Known limits

- Every live episode so far used one start point and one target position. Different target positions are Stage C (`docs/STAGE_C_PLAN.md`).
- The policy was statistically validated only in the stand-in. Live results are 7 episodes.
- The live camera detects about 41–46° off-axis, wider than the stand-in's model (about 41° including the sphere's size).
