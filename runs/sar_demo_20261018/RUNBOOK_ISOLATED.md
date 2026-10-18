# Isolated PX4/Gazebo SAR validation

This runbook describes the separately owned PX4 instance 1 validation stack.
It leaves the pre-existing PX4 instance 0, its rootfs, ULog archive, XRCE
agent, Gazebo partition, bridge, and observation publisher untouched. It does
not authorize flight by itself. The currently verified isolated instance is
disarmed and publishes fresh telemetry, but fails preflight checks and does
not have a valid heading reference; **do not arm it** until those gates pass.

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
`/home/hp/uav_ros/runs/sar_demo_20261018/isolated_sitl_instance1_noshell/`.
Check its manifest before doing anything. Do not start another run if its
components are alive.

## Read-only inspection

```bash
cd /home/hp/uav_ros
source /opt/ros/jazzy/setup.bash
source /home/hp/px4_ros2_ws/install/setup.bash
source .venv/bin/activate

python tools/isolated_sitl.py status \
  --run-dir runs/sar_demo_20261018/isolated_sitl_instance1_noshell
ROS_DOMAIN_ID=43 GZ_PARTITION=uav_sar_validation_20261018_instance1 \
SAR_UAV_MODEL_NAME=x500_lidar_2d_1 \
SAR_LIDAR_TOPIC=/world/walls/model/x500_lidar_2d_1/link/link/sensor/lidar_2d_v2/scan \
python tools/verify_isolated_topics.py --duration 8
```

The telemetry probe must report fresh status, land detection, local position,
LiDAR, and exactly 54 finite observation values; it additionally requires
landed/at-rest/disarmed state and no PX4 failsafe. That validates communications
and observations, not readiness to fly. Before any future flight experiment,
separately require `pre_flight_checks_pass=true`, `failsafe=false`, fresh valid
local position, `heading_good_for_control=true`, no active controller, and a
fresh landed/disarmed state. Keep the one-attempt-per-experiment rule and
monitor ACKs, actual altitude, setpoint continuity, and landing telemetry.
An accepted command ACK is not proof of vehicle movement.

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

```bash
python tools/isolated_sitl.py stop \
  --run-dir runs/sar_demo_YYYYMMDD/run_name
```

## Current result and evidence

The no-shell run has fresh status/position/land telemetry, fresh Gazebo LiDAR,
and a fresh finite 54-value observation. Its PX4 ULog and parameters are under
that run's `px4_rootfs/fs/`, not the shared build rootfs. The last read-only
diagnostic showed disarmed, `failsafe=false`, but
`pre_flight_checks_pass=false`, `gcs_connection_lost=true`,
`heading_good_for_control=false`, and `dist_bottom_valid=false`. The isolated
ULog records `NAV_DLL_ACT=2`; the local `rcAndDataLinkCheck` code blocks all
arming modes when this parameter requires a GCS and `gcs_connection_lost` is
true. This is a confirmed readiness blocker. A QGroundControl executable or
running GCS was not found in the checked WSL/Windows paths; do not change the
PX4 parameter to bypass the gate. The offboard-signal-lost flag is expected
before a controller publishes any Offboard setpoints.

PX4 source defines `heading_good_for_control` as final yaw alignment. With
magnetometer heading enabled, final alignment also requires the in-flight
magnetometer alignment flag to complete; the current ULog has that flag false
while disarmed. Thus the current false heading flag is consistent with a
grounded vehicle and does not by itself establish the preflight failure. The
environment suppresses body-forward velocity until the flag becomes valid.
PX4 startup logs also included `ekf2 missing data` and `no heading reference`
during startup; the later estimator stream shows yaw alignment true, but a
complete current arming-check report is still needed after a GCS is available.
No flight command or smoke test has been issued against the isolated run.

The initial isolated launch omitted PX4's `-i 1` and was refused because
instance 0 was already running. Its log and manifest are preserved. A
separately verified cleanup removed only that failed attempt's XRCE agent and
idle session guards. The first successful no-shell launch produced an
excessive prompt log; it was stopped only after the telemetry gate passed, and
its 7.3 MB ULog is preserved. The current no-shell ULog continues to be written
under the isolated rootfs.
