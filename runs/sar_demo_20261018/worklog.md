# Isolated SITL validation — 2026-10-18

## Baseline and safety boundary

- Starting repository commit: `8521397 Document isolated SITL validation constraints`; working tree was clean.
- Recoverable source baseline: `runs/sar_demo_20261018/uav_ros_baseline.bundle`.
- Full PX4 shared-rootfs snapshot: `runs/sar_demo_20261018/px4_rootfs_baseline.tar` (3.30 GB). The original shared rootfs and ULogs were left in place.
- Existing shared stack remains on instance 0: PX4 PID 7124, Gazebo PIDs 7325/7326, XRCE agent 7044 on UDP 8888, bridge 7872, publisher 7894. Its terminals/sessions were not restarted or signaled.
- No control topics, flight commands, mode changes, arming, takeoff, disarm, parameters, or PPO training were used in this run.

## Isolation evidence

Source/configuration inspection established:

- PX4's local CLI supports `-i <instance>` and `-w <working_directory>`; its per-instance daemon endpoint/lock is keyed by instance ID. PX4 startup scripts take the positional `etc` directory for startup configuration.
- In this PX4 build, `rcS` maps `ROS_DOMAIN_ID` into `UXRCE_DDS_DOM_ID`, honors `PX4_UXRCE_DDS_PORT`, and by default prefixes nonzero instances with `px4_<instance>`; explicitly setting `PX4_UXRCE_DDS_NS=""` clears that prefix. The XRCE UDP port is only the transport endpoint, not the DDS domain.
- `px4-rc.gzsim` derives the model name suffix from the instance ID (`x500_lidar_2d_1` for instance 1). Gazebo reads inherited `GZ_PARTITION`.
- `sitl_multiple_run.sh` was not used; it contains a `pkill -x px4` call.
- The isolation manager records exact process-session leaders, process start times, commands, and per-component logs. Shutdown validates landed/at-rest/disarmed telemetry in the ROS domain named by that run's manifest and refuses while control publishers remain. It uses bounded SIGINT/SIGTERM and no SIGKILL.

## Startup attempt A — diagnosed and cleaned safely

- Run path: `runs/sar_demo_20261018/isolated_sitl/`.
- Configuration: domain 42, UDP 18889, unique Gazebo partition, independent rootfs.
- Failure: PX4 logged exactly `INFO [px4] PX4 server already running for instance 0`; its child exited before starting a server or Gazebo. This confirmed that a separate `-w` path does not bypass PX4's instance-0 singleton lock.
- The startup failure path did not signal any processes. After read-only verification confirmed the specific pre-server refusal, the recorded PX4 child was gone, its owned group contained only its session guard, and no simulator descendant existed, `cleanup-failed-start` stopped only the manifest-owned groups. The original instance 0 was not signaled. The failure logs/manifest remain preserved.

## Startup attempt B — instance 1, interactive-log issue

- Run path: `runs/sar_demo_20261018/isolated_sitl_instance1/`.
- PX4 instance 1 launched in its own rootfs; PX4 spawned a separate `walls` Gazebo world in partition `uav_sar_validation_20261018_instance1` and model `x500_lidar_2d_1`.
- Domain 43 and UDP 18890 were separate from the shared system. Fresh ROS telemetry initially took time to discover; a second bounded probe passed.
- Read-only validation: status `arming_state=1`, `nav_state=4`, `failsafe=false`, `pre_flight_checks_pass=false`; land detection `landed=true`, `at_rest=true`, `ground_contact=true`; local position valid; LiDAR 1080 beams and 54-value observation. At one sample the counts were scan=70, status=4, land=3, position=117, observation=80.
- Logs showed the PX4 shell's `pxh>` prompt redrawing continuously because output was captured to a file (about 629 MB by shutdown). PX4 source supports `-d` to run without the interactive shell. The run was stopped only after a fresh 8-second telemetry gate reported disarmed, landed and at rest; its ULog (about 7.3 MB) and log are preserved.

## Startup attempt C — current no-shell isolated stack

- Run path: `runs/sar_demo_20261018/isolated_sitl_instance1_noshell/`.
- PX4 command: `px4 -i 1 -d -w <run>/px4_rootfs /home/hp/PX4-Autopilot/build/px4_sitl_default/etc`.
- Environment: `ROS_DOMAIN_ID=43`, `PX4_UXRCE_DDS_PORT=18890`, empty `PX4_UXRCE_DDS_NS`, unique `GZ_PARTITION=uav_sar_validation_20261018_instance1`, `PX4_GZ_WORLD=walls`, `PX4_SIM_MODEL=gz_x500_lidar_2d`.
- A bounded read-only probe passed after DDS discovery: `arming_state=1`, `nav_state=4`, `failsafe=false`; `landed=true`, `at_rest=true`, `ground_contact=true`; local position valid; scan=1080 beams (about 805 finite), observation=54 finite values. Sample counts: scan=74, status=4, land=2, position=69, observation=44. PX4 parameters, dataman, and ULog were confirmed under this run's own `px4_rootfs/fs/`; captured PX4 startup log was about 8 KB with no prompt redraw.
- A follow-up six-second read-only diagnostic got fresh status/local position/failsafe/estimator samples. It reported `pre_flight_checks_pass=false`, `gcs_connection_lost=true`, `heading_good_for_control=false`, `dist_bottom_valid=false`; failsafe flags included offboard-signal-lost, manual-control-signal-lost, GCS-lost, and remote-ID-unhealthy, while local-position and altitude invalid flags were false. The isolated ULog has `NAV_DLL_ACT=2`; `rcAndDataLinkCheck.cpp` confirms that when GCS loss is configured as a failsafe (`NAV_DLL_ACT>0`), no GCS link blocks all arming modes. This is a confirmed readiness blocker. QGroundControl was not found in checked WSL/Windows paths; no package was installed. Do not bypass this check by changing a parameter. Startup logs included `ekf2 missing data`, `no heading reference`, and `No connection to the GCS`. `heading_good_for_control` is assigned from EKF `isYawFinalAlignComplete()`; with mag heading enabled that also requires `mag_aligned_in_flight`, which remains false in this disarmed ULog. So the false heading flag is consistent with ground state and is not proven to be the preflight blocker. Offboard-signal-lost is expected without a controller.
- A longer read-only cadence sample measured `VehicleStatus` intervals of 1.616–1.712 s (8 samples) and `VehicleLandDetected` intervals of 3.276–3.332 s (4 samples). A probe with the earlier 1.5/3.0 s status/land freshness cutoffs was correctly rejected once the land sample reached 3.108 s age. The bounded *observation/cleanup probes* now use 2/4 s freshness limits to accommodate measured sample intervals. The controller retains <=1 s age for decisions and has a separate 2 s bounded wait for a replacement status; 15 s simulation deadline, 45 s wall ceiling, 5 s clock-stall watchdog, and -0.8 m NED gate are unchanged. An offline regression reproduces a 1.72 s receipt gap with about 0.30x sim-clock progression.
- The current instance remains running in its owned sessions so telemetry and the ULog remain available. No reset or flight command has been sent.

## Offline investigation and code changes

- The matching prior ULog shows upward NED position setpoints with `z=-1` and accepted Offboard/arm ACKs. The environment's cleanup NAV_LAND arrived about 0.47 seconds of PX4 simulation time after arming; the setpoint was then switched to zero-velocity landing control before the logged 1-second spool-up and 3-second takeoff ramp completed. This supports the prior stale-`VehicleStatus` abort as the reason the previous attempt produced no climb. It does not establish that the updated 2-second replacement-status wait will pass live.
- `px4_uav_env.py` sends `VehicleCommand.target_system=1` in the baseline. PX4 `rcS` sets `MAV_SYS_ID=px4_instance+1`; instance 1 therefore reports system ID 2 and Commander discards commands targeted to another system. `VehicleStatus.system_id` is now stored and used for commands; sending is refused before a system ID is received. Offline regression tests cover target ID 2 and the missing-ID refusal. This fix is required before any isolated control attempt.
- `observation_publisher.py` now accepts `SAR_UAV_MODEL_NAME` and `SAR_LIDAR_TOPIC`, preserving the old instance-0 defaults and enabling the isolated `_1` LiDAR/pose topics.
- `tools/isolated_sitl.py` now uses PX4 instance 1, `-d`, domain 43, port 18890, unique Gazebo partition, isolated writable rootfs, instance-specific model topics, exact process manifests, and safe bounded stop. Shutdown uses the domain in each run's manifest (regression-tested), rather than a global domain. It has a narrow cleanup path only for an evidenced pre-server singleton refusal.
- Added `tools/owned_process_group.py`, `tools/verify_isolated_topics.py`, `tools/verify_shutdown_state.py`, and `test_isolated_sitl.py`.
- PX4 controller hardening also gates body-forward velocity on valid heading (yaw-rate remains available), avoids claiming ground solely from disarmed state, retries a failed landing-command publication, and clears landing completion when fresh telemetry no longer confirms landed/disarmed. Existing observation layout, Gymnasium API, reward, and takeoff altitude gate remain unchanged.

## Verification

- Full offline suite after cadence, target-system, launcher, and cleanup-gate changes: **91 passed**.
- Python syntax checks passed for the environment, publisher, launch/verification tools, and safety tests.
- `git diff --check` passed.
- Final 8-second isolated telemetry probe passed with fresh landed/disarmed samples, `system_id=2`, no failsafe, valid local position, LiDAR, and 54 finite observation values. The separate read-only shutdown gate also passed with status age 0.189 s and land-detection age 2.353 s. The latest probe still reported `pre_flight_checks_pass=false`, `gcs_connection_lost=true`, and `heading_good_for_control=false`.
- Live validation is at the **isolated startup + fresh telemetry/LiDAR/observation** level only. No Offboard stream from the environment, takeoff, bounded movement, or landing test has been run.

## Remaining gates / next action

1. Keep the current instance and logs available; do not launch a duplicate.
2. Provide or make available a standard GCS (for example QGroundControl) on the isolated PX4's MAVLink endpoint, without changing PX4 parameters. Then recheck fresh status and the full arming-check report; the simulator is currently not flight-ready.
3. Re-run offline tests after any code change.
4. Only after fresh preflight passes, heading is valid, the command target resolves to system ID 2, and the independent safety review signs off, consider one bounded isolated flight attempt. Verify actual climb past NED z `-0.8 m`, continuous setpoints, bounded action, and fresh landed/disarmed telemetry. Do not retry an unexplained failure.

## Final offline review and current read-only state — 2026-10-18

- The isolated validation workflow and code changes are committed as `25ec608` (`Add isolated PX4 SITL validation workflow`).
- An independent review identified a stale command-target telemetry gap. `send_command()` now refuses Offboard/arm commands unless a `VehicleStatus` sample is no older than 1 second, in addition to refusing commands before any PX4 system ID has been observed. `_takeoff()` allows up to the existing bounded 2-second status replacement interval before requesting Offboard. The best-effort `NAV_LAND` path may use a previously observed system ID when status is stale, so loss of a status update does not suppress the safety landing request; landing remains unconfirmed until fresh landed-and-disarmed telemetry arrives. Regression tests cover stale command refusal, known-ID best-effort landing, and stale-status safe-state behavior.
- Latest full offline run: **91 passed in 6.82 s**. Python compilation and `git diff --check` passed.
- Latest 8-second read-only probe passed topic freshness/shape checks: system ID 2, disarmed (`arming_state=1`), nav state 4, `failsafe=false`; `landed=true`, `at_rest=true`, `ground_contact=true`; local NED position `(0.001, 0.035, -0.050)` valid; `heading_good_for_control=false`, `dist_bottom_valid=false`; 1080 LiDAR beams (805 finite), exactly 54 finite observation values. PX4 still reports `preflight=false` and `gcs_connection_lost=true`. No flight command was issued.
- `tools/isolated_sitl.py status` confirms the four manifest-owned components remain alive: PX4/Gazebo PID 21076, XRCE agent PID 21068, LiDAR bridge PID 21467, and observation publisher PID 21494. The original shared instance was not affected. Current run logs and its live ULog remain under `runs/sar_demo_20261018/isolated_sitl_instance1_noshell/`.
- Flight validation is blocked by PX4's configured GCS-loss arming gate (`NAV_DLL_ACT=2`) and absent GCS. Do not change that parameter or attempt a flight until a supported GCS connection is available and fresh preflight passes. The current false heading-valid flag is also a no-go for body-forward control; its exact recovery should be assessed from fresh estimator/arming diagnostics after GCS availability.
