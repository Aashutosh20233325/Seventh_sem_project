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
3. **Host check (20 s):** proceed only if the median Pages/sec is below 500 and the page file is below 10 %:
   ```bash
   /mnt/c/Windows/System32/typeperf.exe "\Memory\Available MBytes" "\Memory\Pages/sec" "\Paging File(_Total)\% Usage" -si 1 -sc 20
   ```
4. Repo at a committed state with a clean tree (`git status`), and no PX4/gz processes running.

## 2. One supervised flight

The flown orchestration lives outside the repo, in `/home/hp/uav_ros_experiments/`. Verify its hash before use.

| Purpose | Script (sha256) |
|---|---|
| PPO, default start | `ppo_flight_20261025/run_ppo_flight.sh <run_name>` (`03297200…7473`) |
| PPO or baseline, turned start (90, 180, −90) | `stageb_prep_20261025/run_stageb_flight.sh <run_name> <offset> <ppo\|baseline>` (`d44c21c0…f987`) |

```bash
cd /home/hp/uav_ros
X=/home/hp/uav_ros_experiments/stageb_prep_20261025
bash $X/run_stageb_flight.sh restart_<YYYYMMDD>_<name> 180 ppo > $X/restart_<YYYYMMDD>_<name>.log 2>&1
python3 $X/check_episode.py $X/restart_<...>.log runs/sar_demo_20261018/restart_<...>/logs/episode.jsonl 180 ppo
```

Each script does, in order, and stops fail-closed at any failure:
1. Checks the policy file's sha256 (`runs/rl/exp20261022_randtarget300k/E1R_s2/final_policy_numpy.npz`, `fbcacf93…0511`, read-only).
2. Starts the bounded samplers.
3. Starts the stack with split rendering.
4. Runs the readiness gates: topics, world target, strict `/fmu/in`, and render-check.
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
- **Between flights:** wait at least 2–3 min after the stack stops, let the previous run's bounded samplers finish,
  then repeat the host check. Shutdown causes a Windows paging burst. It usually settles within 30–60 s, but once lasted about 10 min.
- Several flights in a row: `stageb_prep_20261025/run_series_b.sh` is the pattern (all pre-checks, stop rules, at most one
  re-attempt after a readiness failure, never a retried flight).

## 5. If something fails

| Symptom | Likely cause | Action |
|---|---|---|
| Stability windows below 0.35× or long gaps | Windows host paging (browsers open) | Close the applications, repeat the host check, start a new run |
| Host check above 500 pages/s right after a run | Post-shutdown paging | Wait a few minutes and check again |
| `guard_refuse` before take-off | MemAvailable below 1.2 GiB or speed below 0.35× | Do not retry immediately; check host memory |
| `POLICY CHECK FAILED` / `OFFSET REFUSED` | Wrong file or argument | Nothing started; fix the argument |
| Gated stop refuses before PX4 exits | Telemetry not landed/disarmed | Do not stop anything; wait, and follow `DEMO_CHECKLIST.md` recovery |

## Known limits

- Every live episode so far used one start point and one target position. Different target positions are Stage C (`docs/STAGE_C_PLAN.md`).
- The policy was statistically validated only in the stand-in. Live results are 7 episodes.
- The live camera detects about 41–46° off-axis, wider than the stand-in's model (about 41° including the sphere's size).
