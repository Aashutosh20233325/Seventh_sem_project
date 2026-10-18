# Isolated PX4/Gazebo SAR validation

This runbook describes the separately owned PX4 instance 1 validation stack.
It leaves the pre-existing PX4 instance 0, its rootfs, ULog archive, XRCE
agent, Gazebo partition, bridge, and observation publisher untouched. It does
not authorize flight by itself. The currently verified isolated instance is
disarmed, landed, and publishes fresh telemetry with passing preflight checks.
PX4's final heading-good flag is expected to remain false on the ground with
the configured in-flight magnetometer alignment; **do not start a flight until
the supervised test is explicitly approved**.

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
`/home/hp/uav_ros/runs/sar_demo_20261018/restart_20261018/`.
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
the environment enters velocity control. This code path has not been tested
in flight; the live instance remains grounded and no control command has been
sent.

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
