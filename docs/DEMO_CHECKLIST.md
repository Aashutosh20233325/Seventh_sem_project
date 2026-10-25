# Visible Gazebo/PX4 target-reaching demo — checklist

Status (2026-10-25): **one supervised baseline flight succeeded** (`restart_20261025_baseline1`, code `5df1273`, split
rendering): 6/6 stability windows at 0.497–0.512×, target reached in 64 steps, landed and disarmed. B1/B2/B3 are
committed (`837a876`). Readiness depends on free memory on the Windows host (see "Host memory" below). **One
supervised PPO flight succeeded** (`restart_20261025_ppo1`, code `1fac16b`): E1R 300k seed 2 as a torch-free numpy
export behind the LiDAR safety filter, target reached in 38 steps, landed and disarmed. That is one episode, not a
reliability measure. No flight is authorized by this document. Every
step that could arm or take off is marked **DO NOT RUN WITHOUT EXPLICIT APPROVAL**. Detailed procedure and history:
`runs/sar_demo_20261018/RUNBOOK_ISOLATED.md` and `runs/sar_demo_20261018/worklog.md`.

## What the demo is

PX4 SITL (instance 1, MAV_SYS_ID 2) in Gazebo Harmonic with the `sar_oakd` profile:
- **World:** `sim/worlds/sar_walls.sdf`, PX4's walls world plus a **red sphere, radius 0.5 m, at Gazebo ENU (3, −3)**, which is PX4 local NED (−3, 3), 4.24 m from the spawn point.
- **Vehicle:** `sim/models/x500_lidar_2d_oakd`, an x500 with a 2D LiDAR and an OAK-D Lite camera pitched 30° down.

The episode runner (`run_target_reaching_episode.py`) drives `PX4UAVEnv` with the **observation-only baseline controller** (`baseline_controller.py`):
1. Take off to 1.8 m.
2. Fly to the start pose: NED (0, 0), heading east. The target is then about 45° to the right.
3. Rotate right until the camera detects the red sphere.
4. Approach with altitude hold until the target is within 0.75 m (horizontal).
5. Return to start, NAV_LAND, auto-disarm.

With `--policy <policy.npz | checkpoint.zip>` the runner drives a saved PPO policy instead (flown once as `.npz`, 2026-10-25):
- **Loading:** CPU inference with deterministic actions and 1 torch thread. The checkpoint is checked against `sar_obs_v1` and the action space before any env exists, and its sha256 is logged.
- **Safety filter:** every action passes through `safety_layer.LidarSafetyFilter` (stop at 1.2 m, slow below 3.0 m, emergency below 0.5 m). There is no option to disable it.
- **Hover cases:** unknown or stale LiDAR (> 0.5 s wall since the returned sectors were received, from `PX4UAVEnv.observation_age()`), invalid LiDAR, or a malformed policy output all command a hover.
- **Logging:** each step logs the original and applied action and the filter's reason.

The E1R 300k checkpoints are validated only in the stand-in: there they succeed with the filter, but some time out behind interior walls.

## Former blocker (measured 2026-10-22, run `restart_20261022_preflight1`; superseded, kept as a record)

- **Speed:** the camera simulation speed was **0.334–0.347×** in 0/6 stability windows, against the **unchanged 0.35× gate**. The PX4 clock ran at about 0.40×, and every other gate passed. (That PX4 figure came from local-position stamps, which PX4 slews toward wall time; it reads roughly 10–15% high. The camera figure is the valid one.)
- **Memory:** with the gz GUI on D3D12, MemAvailable falls about 137 MiB/min (1517 → 1036 MiB in 3.5 min). The server grows about 50 MiB/min and the GUI about 80 MiB/min. After start-up, gates, and six windows (~4.5 min), MemAvailable is below the runtime guard's 1.2 GiB start threshold, so `guard_refuse` would trigger even with passing speed.
- **Not caused by:** stale telemetry, the target position, LiDAR, the observation/action interface, or the guard (its live read-only validation matched an independent computation).
- **Since then:** WSL memory was raised to 5 GB and split rendering was adopted. In that configuration (its log shows 4915 MiB total, so it ran after the WSL change) `restart_20261023_split2` still failed 2/6 windows (camera 0.316× and 0.341×); `restart_20261023_clockcheck1`, `restart_20261024_guardcheck2` and `restart_20261024_guardcheck3` then passed 6/6 each (camera 0.374–0.411×), with MemAvailable ≥ 1664 MiB. So 3 of 4 runs passed the speed gate, and the margin is small: the lowest passing window was 0.374× (7% above 0.35×) and the lowest 25 s guard speed 0.365× (4%).

## Host memory (measured 2026-10-24/25)

The Windows host has 7.86 GB RAM and WSL is capped at 5 GB. With browsers open, Windows pages heavily and the stack
drops to 0.25–0.31× (`restart_20261024_hostmem1`, 0/6 windows). With Chrome closed it ran at about 0.50×
(`restart_20261025_hostmem2` 6/6, `restart_20261025_baseline1` 6/6 and flown). Before a live run:
- Take a 30 s host baseline (`typeperf "\Memory\Pages/sec" "\Memory\Available MBytes" "\Paging File(_Total)\% Usage" -si 1 -sc 30`).
- If paging is high, ask the operator to close memory-heavy applications. Never close them automatically, and do not raise
  the WSL limit (that worsens host paging).

Inside WSL, MemAvailable falls about 60–70 MiB/min while Gazebo runs (the gz server grows), so start the episode soon
after the windows.

Between live runs, wait at least 2–3 minutes after the stack stops, then repeat the host check. Shutdown causes a
paging burst on Windows (WSL hands memory back, Defender scans the new logs). It usually settles within 30–60 s, but
once (`restart_20261025_ppo3`) about 2,200 pages/s lasted about 10 minutes. Flights themselves showed little paging
(medians 1–15 pages/s).

## Diagnosing rendering overhead without system changes

| Option | Status |
|---|---|
| `--software-rendering` (launcher flag, reversible) | Measured 2026-10-18: memory flat but **~0.17×**, so the gate fails. Diagnostic only. |
| `--px4-server-config` (restores GstCameraSystem) | Adds load; diagnostic only. |
| `HEADLESS=1` (PX4-supported; not exposed by the launcher) | Removes the GUI, which is about two thirds of the leak plus its GPU rendering. Previously authorized for diagnosis only, and a headless run is not a visible demo. |
| GUI on llvmpipe, server on D3D12 ("split rendering") | **Current readiness configuration** (`b9b4588`; render-check fix `8a73737`): `start --split-rendering`. PX4 runs with `HEADLESS=1` (server only, still d3d12 from `gz_env.sh`); the launcher starts and records `gz_gui` from `gz_gui_env.sh` (llvmpipe). No-flight readiness passed in the runs listed above. Not yet flown. |

None of these change `.wslconfig`, Mesa, packages, PX4 parameters, or gates.

## Startup (only with approval for a live-stack start)

One terminal, in this order:

```bash
cd /home/hp/uav_ros
source /opt/ros/jazzy/setup.bash
source /home/hp/px4_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
  SAR_LIDAR_TOPIC=/world/sar_walls/model/x500_lidar_2d_oakd_1/link/link/sensor/lidar_2d_v2/scan
# QGroundControl must already be running. Its GCS heartbeat is required (NAV_DLL_ACT=2); never close it.
.venv/bin/python tools/isolated_sitl.py start --profile sar_oakd --split-rendering --run-dir runs/sar_demo_20261018/restart_<YYYYMMDD>_<name>
.venv/bin/python -u tools/telemetry_recorder.py runs/sar_demo_20261018/restart_<...>/logs/telemetry_full.jsonl 3600 &
```

Split rendering (`--split-rendering`, above) is the configuration that passed readiness. Without it the GUI also
renders on D3D12, and the 2026-10-22 speed and memory blocker applies. Verify the renderers per process (read-only; do
not use the shared `~/.gz/rendering/ogre2.log`):

```bash
.venv/bin/python tools/isolated_sitl.py render-check --run-dir runs/sar_demo_20261018/restart_<...>   # exit 0 = server d3d12 + GUI llvmpipe
```

**Expected:**
- A Gazebo GUI window (`gz sim -g`) showing the walls world, the x500 on the ground at the world origin (PX4 local origin within ~5 cm), and the red sphere at ENU (3, −3), about 4.2 m away. Screenshots from earlier runs: `runs/sar_demo_20261018/restart_20261018_oakd3_gpu/logs/gui_after_d3d12.png`.
- `~/.gz/rendering/ogre2.log` reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`.
- No `llvmpipe` threads in the gz server.
- With `--split-rendering`: `render-check` reports the server with `d3d12_mapped: true` and 0 llvmpipe threads, and the GUI with `d3d12_mapped: false` and >0 llvmpipe threads. The GUI view may not auto-follow the drone (PX4's follow message is sent before the launcher's GUI exists); use the GUI's own follow control if needed.

## Readiness gates (all must pass; abort on any failure)

```bash
.venv/bin/python tools/verify_isolated_topics.py --duration 8   # PASS (re-running the unchanged gate is allowed)
.venv/bin/python tools/verify_world_target.py                   # live target == sim/worlds/sar_walls.sdf
for t in offboard_control_mode vehicle_command trajectory_setpoint; do   # daemon-independent discovery, 5 s
  timeout 30 ros2 topic info --no-daemon --spin-time 5 -v /fmu/in/$t
done
# PASS per topic only if: exit 0; one "Publisher count: 0" and no PUBLISHER endpoint; one "Subscription count: N"
# matching N endpoint records; the expected px4_msgs type; and >= 1 subscription whose node name and namespace are
# _CREATED_BY_BARE_DDS_APP_ with type hash INVALID (the PX4 XRCE-DDS agent's reader). Our own recorder's subscription
# (passive_telemetry_recorder, RIHS01_ hash) does not count. The three PX4-side readers must share one DDS participant.
# "Unknown topic", a timeout, or any other output is a FAIL, never "0 publishers".
# Automated and logged (at most 3 attempts per topic; any publisher fails at once):
#   /home/hp/uav_ros_experiments/guard_validation_20261024/guardcheck3/run_guardcheck3.sh (gate block) + fmu_in_gate_parse.py
# Six 25 s light stability windows (no image subscriptions), each evaluated with the established gates:
#   /home/hp/uav_ros_experiments/gst_camera_perf_20261018/measure_light.py
#   /home/hp/uav_ros_experiments/preflight_guard_20261022/evaluate_window.py
# (current orchestration: guardcheck3/run_guardcheck3.sh above; note that it ends with the gated stop)
```

**Expected fresh topics:**
- `/fmu/out/vehicle_status_v4`: disarmed (arming_state 1), nav 4 (Hold), failsafe false, preflight true, GCS connected.
- `/fmu/out/vehicle_land_detected`: landed and at rest.
- `/fmu/out/vehicle_local_position_v1`: valid, about 16 Hz wall.
- `/fmu/out/vehicle_odometry`.
- `/uav/lidar_sectors`: 49 values, about 10 Hz.
- `/uav/camera/{rgb,depth}/{image,camera_info}`: about 10 Hz wall.
- `/uav/target_perception`: about 7 Hz, publishing; `detected=0` on the ground is expected.

**Each window must show:**
- Simulation speed **≥ 0.35×** on the RGB, depth, and PX4 terms. RGB and depth are the binding terms: the window's PX4 term comes from local-position stamps, which PX4 slews toward wall time after each timesync reset, so it reads about 10–15% high (for example 0.44× against 0.39× camera).
- Status gap ≤ 2.0 s, land gap ≤ 4.0 s, perception gap ≤ 1.5 s.
- LiDAR ≥ 5 Hz.
- Vehicle disarmed, Hold, landed, preflight true, no failsafe, GCS connected.

**Memory:**
- MemAvailable must be ≥ 1.2 GiB when the runner starts.
- Swap must not be approaching exhaustion.

**Abort if:** any gate fails, any window is below 0.35×, memory is below the start threshold, any `/fmu/in` publisher exists or the PX4-side reader is not positively identified, the vehicle is not landed and disarmed, or the renderer is not as expected.

## Policy interface (verified offline)

- **Observation `sar_obs_v1`:** 65 float32 values (`docs/OBSERVATION_CONTRACT.md`):
  - 49 LiDAR sectors / 30 m;
  - previous action (2);
  - body velocity, yaw rate, and height (4);
  - current detection (5);
  - dead-reckoned estimate (5).

  It contains no target coordinates.
- **Action:** `[speed ∈ [0, 1], turn ∈ [−1, 1]]`. Speed 1 = 1.0 m/s along the heading. Turn +1 = +0.5 rad/s NED yaw rate (clockwise, right). The altitude is held at the take-off height. Forward speed is forced to 0 without a valid heading.
- **Step:** 0.1 s of simulation time. Success: horizontal distance ≤ 0.75 m. Collision: LiDAR < 0.25 m, attitude > 1.2 rad, or a PX4 failsafe. Timeout: 600 steps (60 s simulated).
- **Known live/stand-in difference:** perception freshness is 1.5 s **wall** time live (about 0.5 s of simulation time at 0.35×) and 1.0 s of simulation time in the stand-in.

## The demo episode — DO NOT RUN WITHOUT MY EXPLICIT APPROVAL

```bash
# DO NOT RUN WITHOUT EXPLICIT APPROVAL — this arms and takes off.
ROS_DOMAIN_ID=43 .venv/bin/python -u run_target_reaching_episode.py \
  --log runs/sar_demo_20261018/restart_<...>/logs/episode.jsonl --i-confirm-supervised-flight
```

The command refuses to run without `--i-confirm-supervised-flight`, refuses to overwrite an existing log, and never retries.
A PPO episode adds `--policy runs/rl/exp20261022_randtarget300k/E1R_s2/final_policy_numpy.npz` (explicit approval
required each time; flown orchestration `/home/hp/uav_ros_experiments/ppo_flight_20261025/run_ppo_flight.sh`).
Memory: the runner peaks at about 65–90 MiB with the baseline or a numpy `.npz` policy (`numpy_policy.py`, never
imports torch), and about 360–410 MiB with an SB3 `.zip` (torch and stable-baselines3). The policy loads before the guard's 1.2 GiB start check, so too little memory gives
`guard_refuse` before arming.
The flown orchestration (gates, windows, observer stopped by exact PID, fail-closed pre-flight state check, one runner
call, gated stop) is `/home/hp/uav_ros_experiments/baseline_flight_20261025/run_flight.sh`.

## Evidence to review before authorizing a flight

1. A fresh `verify_isolated_topics` PASS, `verify_world_target` PASS, the strict `/fmu/in` gate PASS (0 publishers and the PX4-side reader identified on all three topics), and the renderer check, from the same run.
2. All six stability-window results: per-clock speed, gaps, and vehicle state, each ≥ 0.35×.
3. MemAvailable and swap at the end of the windows, with the trend since start (it must still be ≥ 1.2 GiB when the runner would start).
4. The commit hash and a clean tree, with the full offline suite passing at that commit.
5. QGroundControl connected (`gcs_connection_lost = false`).

## Recovery, landing, and aborts (guard clock inputs from B3, `837a876`; flown once, 2026-10-25)

- **Runtime guard** (`run_target_reaching_episode.py`):
  - **Before take-off:** refuses (`guard_refuse`, no arming) unless MemAvailable ≥ 1.2 GiB and a full 25 s window has speed ≥ 0.35×.
  - **Before each step:** aborts (`guard_abort`) if MemAvailable < 600 MiB, if speed < 0.35× (lower of the PX4 HRT clock and the camera image-stamp clock), or if telemetry is missing, stale, or gapped.
  - **PX4 HRT clock:** `TimesyncStatus.timestamp + estimated_offset`, about one sample per simulated second. PX4 local-position stamps are not used for speed: PX4 slews them toward wall time after each DDS timesync reset.
  - **Separate PX4 checks:** local-position receipt older than 1.0 s → `px4_telemetry_stale`; no HRT progress for 5 s → `px4_clock_stale` (repeated identical HRT values do not refresh it); camera clock advanced ≥ 2 simulated s since the last HRT progress → `px4_clock_stalled`. A backward clock restarts its window (fails until a full window rebuilds).
- **Episode endings:**
  - Success or timeout: return to start, then land.
  - Collision or failsafe: land in place.
  - Any failure during the return: land in place.
- **Abort path:** the runner stops commanding and calls `close()`. That sets zero action and sends one NAV_LAND, then waits for fresh landed-and-disarmed telemetry. If that is not confirmed within 30 s, ROS stays alive in a non-daemon monitor until it is.
- **Refusal before any flight command** (for example `guard_refuse`, or `reset()` failing before take-off): if the environment has never commanded flight, `close()` first waits up to 10 s for fresh disarmed and landed telemetry and, if confirmed, sends nothing (no NAV_LAND, no Offboard stream). Fresh armed status, missing or stale telemetry after 10 s, or any flight already commanded falls back to the abort path above.
- **Not covered by the guard:** it cannot interrupt take-off (≤ 45 s wall) or fly-to-start (≤ 120 s wall); those have their own bounded timeouts, and PX4's offboard-loss failsafe applies if the stream stops.
- **Never:** force-disarm, retry automatically, or stop the stack unless fresh telemetry shows landed and disarmed. Stop only with:

```bash
.venv/bin/python tools/isolated_sitl.py stop --run-dir runs/sar_demo_20261018/restart_<...>   # gated; re-run the unchanged gate if it refuses on a stale sample
```
