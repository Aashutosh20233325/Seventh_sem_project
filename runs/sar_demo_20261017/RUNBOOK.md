# PX4/Gazebo UAV smoke demo

This is the existing single-vehicle SITL workflow. It is not a PPO training
procedure, and it does not validate a UGV or multi-agent behavior. Check the
process table before opening any terminal: do not start a second agent, bridge,
publisher, PX4, or Gazebo instance for this workflow.

For the isolated PX4 instance 1 workflow, use
[`../sar_demo_20261018/RUNBOOK_ISOLATED.md`](../sar_demo_20261018/RUNBOOK_ISOLATED.md).
The shared instance 0 below is not approved for flight testing.

## Terminals

1. PX4 SITL and Gazebo:

   ```bash
   cd /home/hp/PX4-Autopilot
   export PX4_GZ_WORLD=walls
   make px4_sitl_default gz_x500_lidar_2d
   ```

2. Micro XRCE-DDS Agent:

   ```bash
   cd /home/hp/Micro-XRCE-DDS-Agent
   ./build/MicroXRCEAgent udp4 -p 8888
   ```

3. ROS 2 Gazebo LiDAR bridge:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/hp/px4_ros2_ws/install/setup.bash
   ros2 run ros_gz_bridge parameter_bridge \
     '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
   ```

4. Search and rescue observation publisher:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/hp/px4_ros2_ws/install/setup.bash
   cd /home/hp/uav_ros
   python3 observation_publisher.py
   ```

5. Only after all gates below pass, run one bounded environment smoke test:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/hp/px4_ros2_ws/install/setup.bash
   cd /home/hp/uav_ros
   source .venv/bin/activate
   set -o pipefail
   python -u test_px4_uav_env.py 2>&1 | tee /tmp/px4_uav_smoke_$(date -u +%Y%m%dT%H%M%SZ).log
   ```

## Gates before the test

- Verify the model and world are the expected `x500_lidar_2d` and `walls`.
- Confirm fresh status, local position, and land-detection messages. Require
  `landed=true`, `at_rest=true`, and `arming_state=1` before reset.
- Require `pre_flight_checks_pass=true` and `failsafe=false` before allowing the
  test to issue an arm request. Do not override a failing check.
- Confirm the LiDAR scan and `/uav/observation` are fresh at about 10 Hz. The
  observation must contain 54 finite values.
- Confirm PX4 subscribers are discovered on both OffboardControlMode and
  TrajectorySetpoint inputs. The environment now gates the mode request on this
  DDS graph condition and a sustained local 20 Hz setpoint stream.
- Capture status, ACK, local-position, and land-detection telemetry before and
  during the single run. An accepted ACK alone is not a takeoff result.

If any gate fails, do not run the test. Preserve PX4/Gazebo and telemetry while
diagnosing. After a test, require a fresh landed=true and disarmed status before
declaring cleanup complete. Never use forced disarm as automatic recovery.

## Current validation limitation

The original shared stack remains running alongside the isolated instance. It
was disarmed and landed, but PX4 reported
`pre_flight_checks_pass=false`; no flight was attempted. A second isolated SITL
configuration is now available; current state and readiness limits are in the
new isolated runbook. Do not run PX4's multi-instance helper against the shared
stack: the local script uses `pkill -x px4`. Do not start a second make target
against the same build rootfs.

Isolation planning note: Gazebo's local helper supports a separate
`GZ_PARTITION`, and another XRCE agent can listen on a distinct UDP port.
However, `MicroXRCEAgent -d` selects its discovery port; it does not set the DDS
domain. Before creating an isolated second client, confirm the local PX4
UXRCE-DDS client domain/namespace configuration and provide a distinct PX4
working/rootfs path for parameters and logs. The multi-instance helper is unsafe
for this shared session because it runs `pkill -x px4`.

The final read-only status sample (about 12:41:31 UTC on 2026-10-17) was fresh
and reported `arming_state=1`, `nav_state=14`, `failsafe=false`,
`pre_flight_checks_pass=false`. Land detection reported `landed=true`,
`at_rest=true`, and `ground_contact=true`. Local position was valid and near the
origin, but also reported `heading_good_for_control=false` and
`dist_bottom_valid=false`. These conditions were not overridden; they require
diagnosis before flight readiness can be claimed.
