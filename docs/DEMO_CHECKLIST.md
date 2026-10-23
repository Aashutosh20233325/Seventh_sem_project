# Visible Gazebo/PX4 target-reaching demo — checklist

Status (2026-10-23): **the demo is blocked at the readiness stage.** No flight is authorized by this document. Every
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

The runner has **no PPO path**. Flying a PPO checkpoint live would be a new, separately reviewed change. The E1R 300k checkpoints match the interface (`test_policy_interface_compat.py`), but the LiDAR safety filter is not wired into the live system.

## Current blocker (measured 2026-10-22, run `restart_20261022_preflight1`)

- **Speed:** the camera simulation speed was **0.334–0.347×** in 0/6 stability windows, against the **unchanged 0.35× gate**. The PX4 clock ran at about 0.40×, and every other gate passed.
- **Memory:** with the gz GUI on D3D12, MemAvailable falls about 137 MiB/min (1517 → 1036 MiB in 3.5 min). The server grows about 50 MiB/min and the GUI about 80 MiB/min. After start-up, gates, and six windows (~4.5 min), MemAvailable is below the runtime guard's 1.2 GiB start threshold, so `guard_refuse` would trigger even with passing speed.
- **Not caused by:** stale telemetry, the target position, LiDAR, the observation/action interface, or the guard (its live read-only validation matched an independent computation).

## Diagnosing rendering overhead without system changes

| Option | Status |
|---|---|
| `--software-rendering` (launcher flag, reversible) | Measured 2026-10-18: memory flat but **~0.17×**, so the gate fails. Diagnostic only. |
| `--px4-server-config` (restores GstCameraSystem) | Adds load; diagnostic only. |
| `HEADLESS=1` (PX4-supported; not exposed by the launcher) | Removes the GUI, which is about two thirds of the leak plus its GPU rendering. Previously authorized for diagnosis only, and a headless run is not a visible demo. |
| GUI on llvmpipe, server on D3D12 ("split rendering") | **Not implemented.** PX4 starts server and GUI from the same `gz_env.sh`, so this needs a launcher option (`HEADLESS=1` for PX4 plus an owned `gz sim -g` started with software rendering). Expected to remove the GUI leak and free GPU time for the cameras; the speed effect is **unmeasured**. Needs approval and one no-flight readiness run. |

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
.venv/bin/python tools/isolated_sitl.py start --profile sar_oakd --run-dir runs/sar_demo_20261018/restart_<YYYYMMDD>_<name>
.venv/bin/python -u tools/telemetry_recorder.py runs/sar_demo_20261018/restart_<...>/logs/telemetry_full.jsonl 3600 &
```

**Expected:**
- A Gazebo GUI window (`gz sim -g`) showing the walls world, the x500 on the ground at the world origin (PX4 local origin within ~5 cm), and the red sphere at ENU (3, −3), about 4.2 m away. Screenshots from earlier runs: `runs/sar_demo_20261018/restart_20261018_oakd3_gpu/logs/gui_after_d3d12.png`.
- `~/.gz/rendering/ogre2.log` reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`.
- No `llvmpipe` threads in the gz server.

## Readiness gates (all must pass; abort on any failure)

```bash
.venv/bin/python tools/verify_isolated_topics.py --duration 8   # PASS (re-running the unchanged gate is allowed)
.venv/bin/python tools/verify_world_target.py                   # live target == sim/worlds/sar_walls.sdf
ros2 topic info /fmu/in/offboard_control_mode                   # Publisher count: 0
ros2 topic info /fmu/in/trajectory_setpoint                     # Publisher count: 0
ros2 topic info /fmu/in/vehicle_command                         # Publisher count: 0
# Six 25 s light stability windows (no image subscriptions), each evaluated with the established gates:
#   /home/hp/uav_ros_experiments/gst_camera_perf_20261018/measure_light.py
#   /home/hp/uav_ros_experiments/preflight_guard_20261022/evaluate_window.py   (orchestration: run_preflight.sh there)
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
- Simulation speed **≥ 0.35×** on the RGB, depth, and PX4 clocks.
- Status gap ≤ 2.0 s, land gap ≤ 4.0 s, perception gap ≤ 1.5 s.
- LiDAR ≥ 5 Hz.
- Vehicle disarmed, Hold, landed, preflight true, no failsafe, GCS connected.

**Memory:**
- MemAvailable must be ≥ 1.2 GiB when the runner starts.
- Swap must not be approaching exhaustion.

**Abort if:** any gate fails, any window is below 0.35×, memory is below the start threshold, any `/fmu/in` publisher exists, the vehicle is not landed and disarmed, or the renderer is not as expected.

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

## Evidence to review before authorizing a flight

1. A fresh `verify_isolated_topics` PASS, `verify_world_target` PASS, `/fmu/in` publisher counts of 0, and the renderer check, from the same run.
2. All six stability-window results: per-clock speed, gaps, and vehicle state, each ≥ 0.35×.
3. MemAvailable and swap at the end of the windows, with the trend since start (it must still be ≥ 1.2 GiB when the runner would start).
4. The commit hash and a clean tree, with the full offline suite passing at that commit.
5. QGroundControl connected (`gcs_connection_lost = false`).

## Recovery, landing, and aborts (existing behaviour; unchanged)

- **Runtime guard** (`run_target_reaching_episode.py`):
  - **Before take-off:** refuses (`guard_refuse`, no arming) unless MemAvailable ≥ 1.2 GiB and a full 25 s window has speed ≥ 0.35×.
  - **Before each step:** aborts (`guard_abort`) if MemAvailable < 600 MiB, if speed < 0.35× (lower of the PX4 and camera clocks), or if telemetry is missing, stale, or gapped.
- **Episode endings:**
  - Success or timeout: return to start, then land.
  - Collision or failsafe: land in place.
  - Any failure during the return: land in place.
- **Abort path:** the runner stops commanding and calls `close()`. That sets zero action and sends one NAV_LAND, then waits for fresh landed-and-disarmed telemetry. If that is not confirmed within 30 s, ROS stays alive in a non-daemon monitor until it is.
- **Not covered by the guard:** it cannot interrupt take-off (≤ 45 s wall) or fly-to-start (≤ 120 s wall); those have their own bounded timeouts, and PX4's offboard-loss failsafe applies if the stream stops.
- **Never:** force-disarm, retry automatically, or stop the stack unless fresh telemetry shows landed and disarmed. Stop only with:

```bash
.venv/bin/python tools/isolated_sitl.py stop --run-dir runs/sar_demo_20261018/restart_<...>   # gated; re-run the unchanged gate if it refuses on a stale sample
```
