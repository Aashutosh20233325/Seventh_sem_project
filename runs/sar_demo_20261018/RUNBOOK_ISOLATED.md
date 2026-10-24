# Isolated PX4/Gazebo SAR validation

This runbook describes the separately owned PX4 instance 1 validation stack.
It leaves the pre-existing PX4 instance 0, its rootfs, ULog archive, XRCE
agent, Gazebo partition, bridge, and observation publisher untouched. It does
not authorize flight by itself. Supervised smoke-test attempt 1 (run
`restart_20261018`) eventually landed and PX4 auto-disarmed after a prolonged
AUTO_LAND; that run was stopped through the gated shutdown on 2026-10-18 at
13:16 UTC. The current run is `restart_20261018_visible3` (see "Landing
setpoint interleaving" below). Always re-verify live state before acting.

## Isolation layout

- PX4 instance: `1` (`MAV_SYS_ID=2`, model `x500_lidar_2d_1`).
- PX4 startup data: `/home/hp/PX4-Autopilot/build/px4_sitl_default/etc`.
- PX4 writable rootfs: each run's `px4_rootfs/` directory, including its own
  `fs/parameters.bson`, `fs/dataman`, and `fs/log/`.
- ROS 2 DDS domain: `43`, set by PX4 `ROS_DOMAIN_ID` to
  `UXRCE_DDS_DOM_ID` and by the ROS processes directly.
- XRCE UDP transport: port `18890`. This port is distinct from the DDS domain.
- Gazebo transport partition: `uav_sar_validation_20261018_instance1`.
- LiDAR bridge and observation publisher use the instance-specific model
  `x500_lidar_2d_1`; ROS topics retain the `/fmu/...` names because PX4's
  automatically selected `px4_1` namespace is explicitly cleared within the
  isolated ROS domain.
- Each component has a manifest-owned process session and a separate log.

The verified active run is
`/home/hp/uav_ros/runs/sar_demo_20261018/restart_20261018_visible3/` (started
2026-10-18 13:42 UTC with the Gazebo GUI). The previous runs `restart_20261018/` and
`restart_20261018_smoke2/` are stopped and preserved.
Check its manifest before doing anything. Do not start another run if its
components are alive.

## Read-only inspection

```bash
cd /home/hp/uav_ros
source /opt/ros/jazzy/setup.bash
source /home/hp/px4_ros2_ws/install/setup.bash
source .venv/bin/activate

python tools/isolated_sitl.py status \
  --run-dir runs/sar_demo_20261018/restart_20261018
ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
SAR_UAV_MODEL_NAME=x500_lidar_2d_1 \
SAR_LIDAR_TOPIC=/world/walls/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan \
python tools/verify_isolated_topics.py --duration 8
```

The active GCS is the official QGroundControl v5.1.5 x86_64 AppImage. Its
verified SHA-256 is
`9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344`.
The binary, private config/cache, log, and process record are stored inside
the active run directory. PX4's startup log reports its Normal MAVLink link
uses localhost UDP 14550, which QGroundControl binds. Treat GCS as connected
only when fresh PX4 status has `gcs_connection_lost=false`; ROS discovery alone
does not establish this. The QGroundControl process sent a working heartbeat,
but its GUI window was not visible in this WSL session; startup logged Mesa/EGL
renderer warnings.

The telemetry probe must report fresh status, land detection, local position,
LiDAR, and exactly 54 finite observation values; it additionally requires
landed/at-rest/disarmed state and no PX4 failsafe. That validates communications
and observations, not readiness to fly. Before any future flight experiment,
separately require `pre_flight_checks_pass=true`, `failsafe=false`, fresh valid
local position and initially aligned/observable yaw with healthy estimator
flags, a fresh landed/disarmed state, and GCS connectivity. With this PX4 EKF
configuration the final `heading_good_for_control` flag is only expected after
the bounded vertical ascent clears the in-flight magnetometer alignment
threshold; the controller must hold position until that flag is fresh and
true before horizontal velocity control. Keep the one-attempt-per-experiment
rule and monitor ACKs, actual altitude, setpoint continuity, and landing
telemetry. An accepted command ACK is not proof of vehicle movement.

The optional `tools/ground_idle_controller.py` process subscribes to telemetry
and creates PX4 input publishers while remaining in `GROUND`; its timer
publishes no control messages in this phase. Run it only once, after sourcing
ROS Jazzy and the workspace:

```bash
ROS_DOMAIN_ID=43 .venv/bin/python tools/ground_idle_controller.py
```

This verifies DDS subscriber matching, not a continuous Offboard stream. The
stream is only verified in the takeoff sequence, which must remain gated by
preflight and estimator health.

Before a PX4 command, the environment requires an observed `VehicleStatus`
system ID. Offboard and arm commands additionally require status no older than
1 second; takeoff waits up to 2 seconds for a fresh replacement sample before
requesting Offboard. `NAV_LAND` remains a best-effort safety command and may use
the previously observed system ID if status has gone stale. Its ACK and fresh
landed/disarmed telemetry still need separate confirmation.

Read-only cadence measurements on the current slow SITL were 1.62–1.71 seconds
for `VehicleStatus` and 3.28–3.33 seconds for `VehicleLandDetected`. The
bounded telemetry probe uses 2-second status and 4-second land freshness
limits, leaving a small margin above those measured receipt intervals. The
takeoff status-replacement wait is 2 seconds while state decisions still
require status no older than 1 second. The 15-second simulation-time takeoff
deadline, 45-second wall ceiling, 5-second clock-stall watchdog, and −0.8 m
NED altitude gate are unchanged.

## Start a new isolated run

The launcher refuses non-empty run directories, occupied XRCE ports, active
ROS domains, and active Gazebo partitions. Use a new run directory and review
the process table first:

```bash
ps -eo pid,ppid,pgid,sid,tty,stat,etimes,args | \
  rg 'px4|gz sim|MicroXRCEAgent|ros_gz_bridge|observation_publisher|test_px4_uav'

python tools/isolated_sitl.py start \
  --run-dir runs/sar_demo_YYYYMMDD/run_name
```

The launcher starts instance 1 with PX4's `-d` no-shell option, but does not
arm or publish vehicle-control commands. After startup, first run `status`,
inspect that run's PX4 log and rootfs, then run the read-only telemetry probe.
If startup fails, the launcher leaves its owned groups running for inspection.
For the specifically verified error `PX4 server already running for instance`
before the PX4 startup script begins, the narrow
`cleanup-failed-start --run-dir ...` command only cleans the manifest-owned
groups after verifying the PX4 child exited and no simulator child remains.
Do not use it for any other failure.

## Bounded shutdown

`stop --run-dir ...` first spends up to 15 seconds collecting multiple fresh
land-detection and vehicle-status samples in the ROS domain recorded by that
run's manifest. It refuses shutdown unless telemetry is fresh, landed,
at-rest, disarmed, and no control publishers remain. It then sends bounded
SIGINT/SIGTERM only to exact manifest-owned process groups and never sends
SIGKILL. If the gate fails, leave PX4/Gazebo and telemetry running and inspect
the vehicle state; do not bypass the gate.

If the idle controller is running, stop its exact group recorded in
`ground_controller.process.json` after fresh landed/disarmed telemetry is
confirmed, then run the manifest-gated SITL stop. QGroundControl is recorded
separately in `qgroundcontrol.process.json`; stop that exact group only after
the PX4 stack has stopped. It is not part of the SITL manifest.

```bash
python tools/isolated_sitl.py stop \
  --run-dir runs/sar_demo_YYYYMMDD/run_name
```

## Current result and evidence

The active run has fresh status/position/land telemetry, fresh Gazebo LiDAR,
and a fresh finite 54-value observation. Its PX4 ULog and parameters are under
that run's `px4_rootfs/fs/`, not the shared build rootfs. With QGroundControl
connected, a fresh read-only
sample showed system ID 2, disarmed, `nav_state=4`, `failsafe=false`,
`pre_flight_checks_pass=true`, and `gcs_connection_lost=false`; landed, at-rest,
and ground-contact were true. Position was valid and the LiDAR/observation
checks passed. `heading_good_for_control=false` is expected while grounded in
this EKF alignment mode, as detailed below; its transition during a vertical
ascent has not yet been verified live.

Fresh estimator flags report tilt/yaw alignment complete, magnetic heading
fusion active, heading observable/consistent, and no bad-heading fault, while
`cs_mag_aligned_in_flight=false` and `cs_in_air=false`. This agrees with PX4's
current false `heading_good_for_control` value while disarmed. Do not replace
that PX4 flag with a weaker estimate or alter arming checks. The offboard-signal
loss flag is expected until a controller streams setpoints.

PX4 source defines `heading_good_for_control` as final yaw alignment. With
magnetometer heading enabled, final alignment also requires the in-flight
magnetometer alignment flag to complete; the current ULog has that flag false
while disarmed. PX4 sets that flag only after detecting flight and exceeding
1.5 m height above ground, then waits more than one second after its magnetic
alignment reset. The ULog confirms healthy magnetic fusion and no heading or
magnetometer fault; the false final-heading flag is therefore an expected
ground-state result, not an evidenced sensor failure or current preflight
failure.

The prior controller held NED z at -1.0 m and left position-hold takeoff as
soon as z reached -0.8 m. That sequence could not cross PX4's 1.5 m in-flight
magnetic alignment threshold, so the environment's body-forward velocity gate
could remain closed for the whole episode. The controller now holds z=-1.8 m
until fresh PX4 `heading_good_for_control=true` arrives, while retaining the
existing -0.8 m altitude gate and all current timeouts. Initial preflight must
still pass with fresh landed/disarmed state, valid position, aligned and
observable yaw, healthy magnetometer flags, GCS connected, and no failsafe.
The final heading flag is verified during the bounded vertical ascent before
the environment enters velocity control. The first supervised attempt below
confirmed this transition, but landing confirmation failed.

The ULog's IMU and magnetometer inputs are fresh and non-faulted: combined IMU
data is recorded at 250 Hz, magnetometer readings are finite with zero error
count and are fused, and heading/mag innovation checks pass. `dist_bottom_valid`
is false because no range/terrain aid is active; this is separate from the
heading-validity condition. The ROS domain does not currently publish a
`VehicleMagnetometer` sample, so magnetometer freshness is evidenced from the
PX4 ULog, not a ROS subscription. No PX4 parameter or arming check was changed.

The project restart on 2026-10-18 stopped both previously running SITL stacks
only after fresh landed/at-rest/disarmed telemetry. The original stack exited
on Ctrl-C through its identified PX4 terminal; its exact XRCE, ROS-Gazebo
bridge, and observation-publisher groups exited on SIGINT. The isolated stack
was stopped through its manifest-gated shutdown. The new active run above was
then started with the dedicated rootfs. Its first topic probe ran before DDS
discovery completed and saw no samples; PX4 subsequently synchronized, created
its DDS writers, and later bounded probes passed. The idle GROUND-phase
controller finds one subscriber each on PX4's Offboard mode and trajectory
setpoint topics and publishes no control messages. No Offboard stream, command
ACK, arming, or takeoff test has been attempted.

The initial isolated launch omitted PX4's `-i 1` and was refused because
instance 0 was already running. Its log and manifest are preserved. A
separately verified cleanup removed only that failed attempt's XRCE agent and
idle session guards. The first successful no-shell launch produced an
excessive prompt log; it was stopped only after the telemetry gate passed, and
its 7.3 MB ULog is preserved. The active restart run's ULog continues to be
written under its isolated rootfs.

## Supervised smoke-test attempt 1 — landing unresolved

- The user authorized one supervised test. The existing `test_px4_uav_env.py` ran exactly once against instance 1. No parameters, code, stack, or test retry were changed during the run.
- Fresh preflight passed before start: system ID 2, disarmed, landed/at-rest/ground-contact, preflight true, failsafe false, GCS connected, valid local position, initial yaw alignment/observable consistent mag fusion, LiDAR 1080 beams, and 54 finite observations. PX4 had one subscriber on each Offboard mode and trajectory setpoint input.
- PX4 ULog records accepted Offboard command 176 at PX4 time 732.812 s and accepted arm command 400 at 732.836 s (result 0). Status changed to Offboard at 732.824 s and armed at 732.836 s. It reached local NED z minimum -1.6308 m at PX4 time 741.552 s, crossing the unchanged -0.8 m gate. `heading_good_for_control` became true during this ascent. Reset returned a 54-value observation; the existing test executed all ten low-speed actions with no reported collision. These confirm command acceptance and motion separately.
- PX4 accepted the environment's normal NAV_LAND command 21 (result 0) at PX4 time 741.756 s; PX4 logged “Landing at current position” and status entered AUTO_LAND at 741.768 s. `close()` timed out waiting for fresh landed/disarmed telemetry and warned that ROS control remains active. The non-daemon environment cleanup monitor is still running, with zero-velocity Offboard setpoints continuing at about 20 Hz.
- Latest read-only sample at 2026-10-18 06:04:02 UTC: status system ID 2, armed (`arming_state=2`), `nav_state=18` AUTO_LAND, failsafe false, GCS connected; land flags `landed=false`, `at_rest=false`, `ground_contact=false`. Local position was near the origin (`z=0.032 m`, `vz=0.0055 m/s`) with final heading-good true. Near-ground position is not being treated as proof of landing. PX4 ULog also shows the land flags remained false through latest data at simulation time 887.156 s; the Gazebo model root pose was steady at approximately `(-0.0836, 0.0217, -0.0130)` across three read-only samples. No safe disarmed confirmation exists.
- Telemetry log: `logs/takeoff_watch_after_close_20261018T055853Z.jsonl` (continuous 20 Hz Offboard/setpoint samples and PX4 state after recorder restart). Smoke output: `logs/supervised_smoke_20261018T055722Z.log`. Initial recorder `logs/takeoff_watch_20261018T055722Z.jsonl` has an early gap because the recorder crashed when serializing NumPy float32 setpoint fields; that error is known and the ULog supplements command ACK and climb evidence. PX4 log: `logs/px4_gazebo.log`. Active ULog: `px4_rootfs/fs/log/2026-10-18/05_27_19.ulg`.
- Recovery is unresolved. Preserve the active environment cleanup monitor, telemetry recorder, PX4/Gazebo, XRCE, ROS bridge, observation publisher, and QGroundControl. Do not send more commands, force-disarm, stop processes, or retry without a separately approved recovery plan.
- *Superseded outcome (recorded later):* PX4 ULog shows sustained `ground_contact=true` at PX4 time 917.308 s, `landed`/`at_rest` at 917.980 s, and automatic disarm at 920.000 s (“Landing detected”, “Disarmed by landing”; no forced disarm). After disarm PX4 reverted to its user-intended mode, `nav_state=14` (OFFBOARD), not Hold. The environment's cleanup monitor then confirmed landed/disarmed and exited.

## Landing setpoint interleaving and preflight diagnosis — 2026-10-18

Isolated native baseline (`/home/hp/uav_ros_experiments/landing_baseline_20261018_03/RESULTS.md`): stock `gz_x500`, PX4-native `commander takeoff`/`land`, no Offboard stream. NAV_LAND to auto-disarm took 7.412 PX4 s with one ground-contact rising edge.

**Confirmed by PX4 source:**
- `/fmu/in/trajectory_setpoint` is bridged onto the same uORB `trajectory_setpoint` topic that FlightModeManager publishes in AUTO modes (`dds_topics.yaml:173`, `FlightModeManager.cpp:364`). `mc_pos_control` uses whichever sample arrived last (`MulticopterPositionControl.cpp:435`).
- The MC land detector only allows ground contact while the latest trajectory setpoint commands descent (`vz >= 1.1*LNDMC_Z_VEL_MAX`, `MulticopterLandDetector.cpp:244-254`).
- `pre_flight_checks_pass` is `canArm(current nav_state)` (`Commander.cpp:2064`). With `nav_state=14` and no Offboard signal, `check_modes_offboard_signal_lost` fails (`offboardCheck.cpp:104-114`).

**Confirmed by logs:** In the attempt-1 ULog, between NAV_LAND (741.748 s) and sustained contact (917.308 s), 232 of 878 logged `trajectory_setpoint` samples had exactly `vz=0` (the environment's LAND stream) interleaved with 646 descending samples. During that time `flag_control_offboard_enabled=0`, auto=1, and there were 14 ground-contact rising edges. The native baseline had 0 zero-velocity samples. The previous run's post-flight `pre_flight_checks_pass=false` matched the OFFBOARD-without-signal mode check. After a fresh boot in `nav_state=4`, `restart_20261018_smoke2` reports `pre_flight_checks_pass=true`.

**Confirmed in live SITL with the fix:** Two supervised runs passed. In `restart_20261018_smoke2` (13:30 UTC), NAV_LAND to auto-disarm took 5.932 PX4 s. In `restart_20261018_visible3` (13:45 UTC, watched in the Gazebo GUI), it took 5.940 PX4 s. Both runs had no environment setpoints after AUTO_LAND, a single ground-contact edge, and a `close()` that returned without a timeout (see worklog). After every auto-disarm PX4 reverts to OFFBOARD, so a standalone probe reports `preflight=false` until PX4 is in Hold or the stack is restarted.

**Still unverified:** The ULog subsamples setpoints at about 5 Hz, so the true interleave ratio is not measured. Host load and simulation slowdown were not measured and may also contribute. Two flights are a small sample.

**Code change (`px4_uav_env.py`):** In the LAND phase the environment keeps the Offboard heartbeat and zero-velocity setpoint while fresh status reports OFFBOARD. When a status sample reports `NAVIGATION_STATE_AUTO_LAND` during LAND, it latches `land_setpoints_released` and stops publishing both `OffboardControlMode` and `TrajectorySetpoint`. Stale or later-changed status, including the post-disarm reversion to OFFBOARD, does not restart the stream. Only a new `land()` request or a new takeoff clears the latch. The single NAV_LAND request, the fresh landed-and-disarmed completion gate, the non-daemon cleanup monitor, and close semantics are unchanged. If PX4 left AUTO_LAND for OFFBOARD while still airborne, PX4's own offboard-loss failsafe would act instead of a zero-velocity hover.

**Tests:** 11 new offline tests in `test_px4_uav_env_safety.py`; the full suite has 106 passing. Five of them fail against the pre-change code.

**Analysis scripts and outputs:** `/home/hp/uav_ros_experiments/landing_interleave_analysis_20261018/` (`analyze_landing_setpoints.py`, `primary_05_27_19.txt`, `baseline_06_34_31.txt`, `telemetry_recorder.py`, `estimator_flags_probe.py`).

## Supervised target-reaching test with OAK-D perception (prepared; not yet flown)

**Prerequisites:** explicit user approval, the full offline suite passing on the current HEAD, one attempt only, no automatic retry. Every command below is in the repository root with ROS Jazzy and `px4_ros2_ws` sourced.

### 1. Release the current stack (only if fresh telemetry is landed and disarmed)

```bash
ROS_DOMAIN_ID=43 .venv/bin/python tools/verify_shutdown_state.py --duration 5 --expected-system-id 2
.venv/bin/python tools/isolated_sitl.py stop --run-dir runs/sar_demo_20261018/<current run>
```

### 2. Start the project-asset profile

`sar_oakd` uses `sim/server_no_gstreamer.config`: PX4's Gazebo server config without the GstCameraSystem UDP video streamer, which the camera, ROS, and perception path does not use. To restore PX4's default config, add `--px4-server-config`. The profile also renders the gz server and GUI on the RTX 3050 through Mesa D3D12 (`GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, set in the run-local `gz_env.sh`). Verify this with `~/.gz/rendering/ogre2.log` (`GL_RENDERER = D3D12 (NVIDIA ...)`) and the absence of `llvmpipe` threads. To revert to CPU rendering, add `--software-rendering`.

It uses the versioned world and model, the camera bridge, the perception node, and the GUI.

Readiness has passed only with split rendering (`--split-rendering`: gz server on D3D12, GUI on llvmpipe; check with
`isolated_sitl.py render-check --run-dir <run>`, exit 0). See `docs/DEMO_CHECKLIST.md`.

```bash
.venv/bin/python tools/isolated_sitl.py start --profile sar_oakd --split-rendering \
  --run-dir runs/sar_demo_20261018/restart_<YYYYMMDD>_oakd1
```

### 3. Readiness gates

All must pass; otherwise stop and report.

```bash
export ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
  SAR_LIDAR_TOPIC=/world/sar_walls/model/x500_lidar_2d_oakd_1/link/link/sensor/lidar_2d_v2/scan
.venv/bin/python tools/verify_isolated_topics.py --duration 8      # PASS, preflight=True, disarmed, landed, 49 sectors
.venv/bin/python tools/verify_world_target.py                      # live target matches sim/worlds/sar_walls.sdf
ros2 topic hz /uav/camera/rgb/image     # record; expect about 10-12 Hz wall (split rendering; 1-2 Hz under software rendering)
ros2 topic hz /uav/camera/depth/image
ros2 topic hz /uav/target_perception    # must be publishing (detected=0 is expected on the ground: the target is ~45 deg right of the spawn heading)
gz topic -e -n 10 -t /world/sar_walls/stats | grep real_time_factor   # record only; not a gate (unreliable here)
```

- **Simulation speed:** ≥ **0.35×** in each of six 25 s stability windows, on the RGB and depth camera-stamp clocks (`measure_light.py` + `evaluate_window.py`; see `docs/DEMO_CHECKLIST.md`). The windows' PX4 term uses local-position stamps, which PX4 slews toward wall time; it reads about 10–15% high and is not the binding term. The takeoff gate allows 15 s of simulation time within a 45 s wall ceiling.
- **Control inputs:** `/fmu/in/*` must have 0 publishers, **and** the PX4-side reader must be positively discovered. Per topic, `timeout 30 ros2 topic info --no-daemon --spin-time 5 -v /fmu/in/<topic>` must exit 0 and show `Publisher count: 0`, no PUBLISHER endpoint, and a subscription with node name and namespace `_CREATED_BY_BARE_DDS_APP_` and type hash `INVALID`; the three readers must share one DDS participant. An unknown topic, a timeout, or any other output fails the gate. Automated version: `/home/hp/uav_ros_experiments/guard_validation_20261024/guardcheck3/`.
- **Estimator:** must be healthy.

### 4. Record and fly once

```bash
.venv/bin/python -u tools/telemetry_recorder.py runs/sar_demo_20261018/<run>/logs/telemetry_full.jsonl 14400 &
.venv/bin/python -u run_target_reaching_episode.py --log runs/sar_demo_20261018/<run>/logs/episode.jsonl \
  --i-confirm-supervised-flight
```

**Expected sequence:**
1. Takeoff gates pass.
2. Speed-limited fly-to the start pose (NED 0,0, −1.8 m, heading east).
3. The baseline rotates right, detects the target, and approaches with altitude hold.
4. Success at ≤ 0.75 m.
5. Return to start, NAV_LAND, AUTO_LAND stream release, auto-disarm, and `close()`.

If takeoff, the start-pose flight, or landing confirmation fails: do not retry, do not force-disarm, and leave the stack running if landed/disarmed is not confirmed.

**Runtime guard (`run_target_reaching_episode.py`, SITL only).** This adds to the readiness gates above and replaces none of them. The clock inputs below are those of the B3 change, which is **uncommitted** (at `8a73737` the guard still used local-position stamps):
- **Before reset/take-off:** refuses (`guard_refuse`, no arming) unless `MemAvailable` ≥ 1.2 GiB and a full 25 s window shows simulation speed ≥ 0.35×. It waits at most 60 s for that window.
- **Before every step:** aborts (`guard_abort`) if `MemAvailable` < 600 MiB or the 25 s speed is < 0.35×. Speed is the lower of the PX4 HRT clock (`TimesyncStatus.timestamp + estimated_offset`, about one sample per simulated second) and the camera image-stamp clock. PX4 local-position stamps are not used for speed: PX4 slews them toward wall time after each DDS timesync reset.
- **Missing data fails:** missing or stale telemetry, a gap > 5 s inside the window, or unreadable memory counts as a failure, never as healthy. Freshness limits: PX4 local-position receipt 1.0 s (`px4_telemetry_stale`); PX4 HRT progress 5 s (`px4_clock_stale`; identical repeated HRT values do not refresh it); camera 1.5 s (`camera_clock_stale`).
- **PX4 clock stall:** if the camera clock advances ≥ 2 simulated s since the last HRT progress, the guard fails with `px4_clock_stalled`. At speeds up to about 0.4× the 5 s HRT staleness limit normally fires first; live, only staleness has been observed (`restart_20261024_guardcheck2/3`), and the stall path is covered by offline tests.
- **What an abort does:** stops actions and calls the unchanged `close()`: zero action, one NAV_LAND, wait for fresh landed-and-disarmed, ROS kept alive until confirmed. There is no retry.
- **Logging:** each record carries measurements, thresholds and timestamps. `guard_sample` records every 5 s also cover reset and close, where the guard only logs.
- **Not covered:** the guard cannot interrupt `reset()` (take-off ≤ 45 s wall, fly-to-start ≤ 120 s wall) or a step's return-to-start; those keep their own bounded timeouts.

### Unverified live

- The altitude-hold setpoint form (position z + xy velocity under the Offboard position flag).
- The carrot fly-to.
- The simulation-time step clock (B1: RGB `camera_info` Gazebo stamps, uncommitted) and the HRT-based fly-to timeout (B2, uncommitted).
- The runtime guard during an episode (B3: validated passively only, with synthetic dropouts, in `restart_20261024_guardcheck2` and `restart_20261024_guardcheck3`).
- Live camera rates and perception latency with the camera on the flying model.
