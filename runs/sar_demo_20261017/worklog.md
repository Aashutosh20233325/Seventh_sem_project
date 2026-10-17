# SAR SITL demo engineering run — 2026-10-17

## Objective
Deliver the strongest safely verifiable PX4/Gazebo UAV demo. Existing running stack is not to be commandeered; only one PX4/Gazebo instance is available and current read-only telemetry shows preflight false.

## Baseline
- Project: `/home/hp/uav_ros`; no Git repository existed at task start.
- Baseline archive: `/home/hp/uav_ros_baseline_20261017/project_baseline.tar` (excluded `.venv`, caches).
- Existing ULog and prior smoke-test logs are preserved in place.
- Existing stack PIDs: PX4 7124, Gazebo 7325/7326, XRCE agent 7044, ROS-Gazebo bridge 7872, observation publisher 7894. No test/cleanup controller process observed.
- Current read-only status: disarmed, nav state 14, failsafe false, preflight checks false; landed/at_rest/ground_contact true. No flight experiment authorized against this non-isolated instance.

## Hypotheses
1. Prior reset aborted on one-second VehicleStatus freshness despite ~1.1-second observed interval.
2. Accepted Offboard/arm ACKs did not yield climb; command acceptance does not prove setpoint reception or physical response.
3. Cleanup's first 20-second wait expired before later disarm; background cleanup subsequently observed it.

## Actions/results
- Experiment IDs: none. No SITL experiment or flight was started during this run.
- 2026-10-17: read-only process, topic, code, and preserved log inspection.
- Baseline safety suite: `source .venv/bin/activate && python -m pytest -q test_px4_uav_env_safety.py` — 43 passed before changes.
- Read-only input rates: `/uav/observation` and Gazebo LiDAR scan both ~10 Hz; the process graph showed no publishers on PX4 Offboard/setpoint/command input topics before testing.
- Preserved failed attempt: VehicleStatus receipt interval median 1.1355 s, max 1.2312 s; accepted mode/arm ACKs were followed by ~1 s with NED z near zero, then NAV_LAND. The preserved setpoint/thrust trace is absent, so accepted ACKs do not establish why climb did not occur.
- Preserved evidence paths: `/tmp/px4_smoke_test_repeat_20261017T.log`, `/tmp/px4_smoke_watch_repeat_20261017T.jsonl`, and `/home/hp/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/2026-10-17/10_32_41.ulg`. Their sizes/mtimes were unchanged after this offline work.
- Preserved cleanup timing: NAV_LAND ACK 12:17:14.482Z; fresh disarmed status 12:17:38.402Z (~23.9 s later). The former 20 s close wait expired first; background cleanup remained active.
- Current read-only telemetry at ~12:41:31Z: disarmed, landed, at_rest and ground_contact true; `failsafe=false`; `pre_flight_checks_pass=false`; `nav_state=14`; local x/y/z about -0.067/-0.124/+0.031 m with valid XY/Z, but heading_good_for_control=false and dist_bottom_valid=false. This fails the flight-readiness gate. No command was issued.
- Current stack is shared and is the only running PX4/Gazebo instance. A second isolated SITL is technically possible but not launch-ready: using the current build rootfs risks shared params/logs, and PX4's local multi-instance helper contains `pkill -x px4`. No experiment was launched.
- Implemented bounded 0.5 s source freshness/validity gating in the observation publisher, consistent Gazebo world XY geometry, sensor-relative target bearing, and ROS-free helper/integration tests.
- Environment hardening: serialized control publications and safe-state transitions; takeoff checks fresh valid position and holds current NED XY; checks PX4 subscriptions before mode request; rejects stale/nonfinite observations; converted walls.sdf target ENU (3,-3) to PX4 NED (-3,3) for reward using the local Gazebo bridge conversion; removed unconditional positive-speed reward.
- Landing wait is now 30 s (previous trace took ~24 s from NAV_LAND to disarmed), remains bounded, and still requires fresh landed and disarmed telemetry. Async cleanup behavior remains as fallback.
- Added regression tests for publisher frame math and integrated path/freshness gate, safe-state synchronization, invalid/stale telemetry, target NED mapping, speed-reward removal, setpoint subscriber readiness, and observed cleanup timing.
- Full repository test discovery: `source /opt/ros/jazzy/setup.bash && source /home/hp/px4_ros2_ws/install/setup.bash && source .venv/bin/activate && python -m pytest -q` — **72 passed**. Initial unsourced discovery failed importing `px4_msgs`; rerun with the installed ROS/PX4 workspace sourced passed.
- Syntax checks: `python -m py_compile` for all project scripts and new tests — passed. `git diff --check` — passed.
- No ROS control topic was published; no flight command, arm, takeoff, disarm, parameter change, simulator restart, or training was performed.

## Changed files
- `px4_uav_env.py`
- `observation_publisher.py`
- `observation_geometry.py` (new)
- `test_px4_uav_env_safety.py`
- `test_observation_geometry.py` (new)
- `test_observation_publisher.py` (new)
- `runs/sar_demo_20261017/RUNBOOK.md` (new)
- `.gitignore` (local venv/cache exclusions)

## Next action
Continue by preparing an isolated PX4 working/rootfs and startup path, then verify the changed publisher and control behavior in one supervised experiment only after fresh preflight passes. Current shared instance has preflight false and must not be used for flight. Do not use the local multi-instance helper because it kills PX4 processes.
