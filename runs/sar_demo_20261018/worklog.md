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

## Remaining gates / next action (before full restart; superseded below)

1. Keep the current instance and logs available; do not launch a duplicate.
2. Provide or make available a standard GCS (for example QGroundControl) on the isolated PX4's MAVLink endpoint, without changing PX4 parameters. Then recheck fresh status and the full arming-check report; the simulator is currently not flight-ready.
3. Re-run offline tests after any code change.
4. Only after fresh preflight passes, heading is valid, the command target resolves to system ID 2, and the independent safety review signs off, consider one bounded isolated flight attempt. Verify actual climb past NED z `-0.8 m`, continuous setpoints, bounded action, and fresh landed/disarmed telemetry. Do not retry an unexplained failure.

## Pre-restart offline review (historical state) — 2026-10-18

- The isolated validation workflow and code changes are committed as `25ec608` (`Add isolated PX4 SITL validation workflow`).
- An independent review identified a stale command-target telemetry gap. `send_command()` now refuses Offboard/arm commands unless a `VehicleStatus` sample is no older than 1 second, in addition to refusing commands before any PX4 system ID has been observed. `_takeoff()` allows up to the existing bounded 2-second status replacement interval before requesting Offboard. The best-effort `NAV_LAND` path may use a previously observed system ID when status is stale, so loss of a status update does not suppress the safety landing request; landing remains unconfirmed until fresh landed-and-disarmed telemetry arrives. Regression tests cover stale command refusal, known-ID best-effort landing, and stale-status safe-state behavior.
- Full offline run at that point: **91 passed in 6.82 s**. Python compilation and `git diff --check` passed.
- Latest 8-second read-only probe passed topic freshness/shape checks: system ID 2, disarmed (`arming_state=1`), nav state 4, `failsafe=false`; `landed=true`, `at_rest=true`, `ground_contact=true`; local NED position `(0.001, 0.035, -0.050)` valid; `heading_good_for_control=false`, `dist_bottom_valid=false`; 1080 LiDAR beams (805 finite), exactly 54 finite observation values. PX4 still reports `preflight=false` and `gcs_connection_lost=true`. No flight command was issued.
- At that point `tools/isolated_sitl.py status` confirmed the four manifest-owned components. That former run was stopped during the process restart below; its logs/ULog remain under `runs/sar_demo_20261018/isolated_sitl_instance1_noshell/`.
- Flight validation is blocked by PX4's configured GCS-loss arming gate (`NAV_DLL_ACT=2`) and absent GCS. Do not change that parameter or attempt a flight until a supported GCS connection is available and fresh preflight passes. The current false heading-valid flag is also a no-go for body-forward control; its exact recovery should be assessed from fresh estimator/arming diagnostics after GCS availability.

## Full process restart and GCS readiness — 2026-10-18

- Before stopping anything, the original shared PX4 (PID 7124, process group 7057) and isolated PX4 (PID 21076, group 21075) each had fresh telemetry confirming disarmed and landed/at-rest. The shared instance was in nav state 14 but disarmed; both reported GCS lost and preflight false at that initial sample. No controller, smoke test, cleanup monitor, or GCS process was found.
- Stopped the previous isolated stack via `tools/isolated_sitl.py stop --run-dir runs/sar_demo_20261018/isolated_sitl_instance1_noshell`; its 15-second read-only gate passed (`system_id=2`, disarmed, landed/at-rest/ground-contact). For the original stack, sent one Ctrl-C through the verified original PX4 terminal session 92899/TTY `pts/4`; PX4 replied `PX4 Exiting...`, `pxh> Exiting NOW.`, and make reported its build interrupted. Then sent SIGINT only to the separately inventoried process groups 7044 (XRCE), 7854 (ROS-Gazebo bridge), and 7894 (observation publisher); all groups exited within 5 seconds. All prior PX4/Gazebo/agent/bridge/publisher processes were verified stopped. Existing source, bundles, ULogs, and traces were preserved.
- Started a new isolated run at `runs/sar_demo_20261018/restart_20261018/`: PX4 instance 1/system ID 2, independent `px4_rootfs`, ROS domain 43, XRCE UDP 18890, Gazebo partition `uav_sar_validation_20261018_instance1`, `walls` world, model `x500_lidar_2d_1`, LiDAR bridge, and observation publisher. Ownership manifest and per-component logs reside inside the run directory. An initial probe preceded DDS discovery and saw no samples; after PX4 synchronized and created its DDS writers, two subsequent probes passed with fresh status, land, position, 1080-beam scan (about 805 finite), and 54 finite observation values.
- No GCS executable was installed. Downloaded the official QGroundControl v5.1.5 x86_64 AppImage from the project release page to the run artifact directory and verified SHA-256 `9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344`. It runs as PID 24818 with QGroundControl PID 24821, isolated XDG config/cache and log, and binds UDP 14550. PX4 status then changed to `gcs_connection_lost=false` and `pre_flight_checks_pass=true`, confirming the GCS heartbeat. `xwininfo` found no visible QGroundControl window; its log has Mesa/EGL warnings, although its process and MAVLink UDP listener remain alive.
- An 8-second read-only PX4ControllerNode probe remained in `GROUND`, observed system ID 2, fresh status, position validity, landed state, and one PX4 subscriber each for OffboardControlMode and TrajectorySetpoint. It published zero control messages and received no command ACKs. The reproducible idle controller script was added and is now running in the current instance's separate process group (PID 25309, child PID 25310). It publishes no control messages while in `GROUND`; continuous Offboard streaming itself has not been started.
- A separate 8-second estimator probe recorded fresh `cs_tilt_align=true`, `cs_yaw_align=true`, `cs_mag_hdg=true`, `cs_mag_heading_consistent=true`, `cs_heading_observable=true`, `fs_bad_hdg=false`; `cs_in_air=false` and `cs_mag_aligned_in_flight=false`. `VehicleLocalPosition.heading_good_for_control` remains false. This is the unmet flight-readiness gate; no flight was attempted. No parameter or arming-check changes were made.
- Full offline safety suite after adding the idle-controller regression: **92 passed in 7.02 s**. Python syntax compilation passed for the environment, observation publisher, all launch/probe tools including `ground_idle_controller.py`, and both safety test files. `git diff --check` passed.
- Latest 8-second read-only probe passed: PX4 system ID 2, `arming_state=1`, `nav_state=4`, `failsafe=false`, `preflight=true`, `gcs_connection_lost=false`; landed/at-rest/ground-contact true; local NED `(0.031,0.014,-0.016)` valid; 1080 LiDAR beams (805 finite); 54 finite observations. The idle controller also reports fresh status and landed state. `heading_good_for_control=false` remains the unmet no-flight gate.
- Active process/log ownership: isolated core components are recorded in `processes.json`; QGroundControl process group 24818/child 24821 in `qgroundcontrol.process.json`; idle controller group 25309/child 25310 in `ground_controller.process.json`. Run data, AppImage, per-component logs, isolated parameters, and current ULog are preserved under `runs/sar_demo_20261018/restart_20261018/` and excluded from Git.
- The ground-only controller, regression test, runbook, and restart record are committed as `0c7bdb9` (`Add safe ground-only controller startup`). Preceding implementation/workflow commits are `25ec608` and `eaa8a57`.

## Heading-validity investigation and takeoff hold — 2026-10-18

- Read `runs/sar_demo_20261018/RUNBOOK_ISOLATED.md` and this worklog before diagnosis. The active instance and its logs were preserved. Read-only manifest status confirmed PX4/Gazebo, XRCE, LiDAR bridge, observation publisher, QGroundControl, and the GROUND idle controller remained running; no flight-control command was sent.
- Independent PX4-source review confirmed `EKF2.cpp:1758` publishes `heading_good_for_control` from `Ekf::isYawFinalAlignComplete()` (`EKF/ekf.h:268-279`). With mag fusion active, final alignment requires yaw alignment plus `mag_aligned_in_flight` and more than one second after that alignment starts. `mag_control.cpp:382-394` only requests the magnetic alignment reset when EKF detects in-air and its ground-relative altitude exceeds 1.5 m. `mag_control.cpp:218-227` sets the in-flight alignment flag after that reset. Thus `cs_mag_aligned_in_flight=false` while grounded/disarmed is expected and explains the false final heading flag; it is not evidence of a failed sensor.
- The ULog confirms PX4 has selected `EKF2_MAG_TYPE=0` (AUTO), `SIM_GZ_EN_MAG=1`, and `SYS_HAS_MAG=1`. Heading became false at PX4 time 5.804 s when `cs_mag_hdg` became active, following a brief true interval during initial yaw alignment. `cs_yaw_align`, `cs_heading_observable`, mag-heading consistency, and tilt alignment are true; bad heading/magnetometer, disturbed-field, and estimator health/fault checks are clear. Magnetometer samples are finite, error_count=0, and fused; `sensor_combined` is present at 250 Hz in PX4 time with no clipping. ROS DDS currently provides no `VehicleMagnetometer` samples, so magnetometer freshness is established from the active ULog rather than ROS.
- The confirmed repository mismatch was that PX4’s final-heading alignment needs >1.5 m HAGL, while the controller held at NED z=-1.0 m and left position hold at the existing -0.8 m gate. That sequence cannot reach the estimator’s alignment condition; horizontal body-forward velocity could therefore remain suppressed for the episode. No PX4 parameter or safety check was changed.
- Implemented the minimal environment correction: the TAKEOFF phase now position-holds at NED z=-1.8 m and `_wait_for_airborne_state()` requires fresh position telemetry with final `heading_good_for_control=true` before switching to velocity control. The -0.8 m altitude gate, 15-second simulation deadline, 45-second wall ceiling, and 5-second clock-stall watchdog are unchanged. Timeout diagnostics name missing final heading alignment. Regression tests cover the 1.8 m setpoint, continued TAKEOFF waiting until heading alignment, and a bounded timeout when the flag remains false.
- Current fresh read-only probe: system_id=2, disarmed, nav_state=4, failsafe=false, preflight=true, GCS connected; landed/at_rest/ground_contact=true; local position valid and near origin; final heading flag false as expected on ground; LiDAR=1080 beams (805 finite) and observation=54 finite values. The idle controller reports one PX4 subscriber for each required setpoint input and confirms no control messages published in GROUND. Readiness is sufficient for an explicitly supervised vertical takeoff experiment, but the post-ascent magnetic alignment transition and the changed hold behavior remain unverified live. No takeoff was attempted.
- Offline verification: the full suite initially failed collection when run without ROS Jazzy/workspace setup (`px4_msgs` unavailable); rerun with both setup files sourced passed **95 tests in 4.98 s** after all changes. Python syntax compilation with warnings-as-errors and `git diff --check` passed.
- Active logs remain under `runs/sar_demo_20261018/restart_20261018/logs/`; active ULog remains under `runs/sar_demo_20261018/restart_20261018/px4_rootfs/fs/log/2026-10-18/05_27_19.ulg`. PX4/Gazebo and all supporting processes remain running.

## Supervised smoke-test attempt 1 — landing unconfirmed — 2026-10-18

- User authorized exactly one supervised test after preflight. Inspected active process ownership; the isolated PX4/Gazebo stack, XRCE agent, LiDAR bridge, observation publisher, QGroundControl, and GROUND idle controller were active. No prior smoke-test or cleanup controller was found. Existing stack and ULogs were preserved.
- Read-only preflight immediately before launch: system ID 2, disarmed (`arming_state=1`), nav state 4, failsafe=false, preflight=true, GCS loss=false; landed/at_rest/ground_contact=true; valid local NED; current heading-good=false as expected on ground, while `cs_yaw_align`, `cs_mag_hdg`, `cs_heading_observable`, and mag consistency were true and bad-heading/mag faults false. LiDAR 1080 beams (805 finite), 54 finite observation values. EstimatorFlags sample interval was slow (~2.5 s); latest receipt age 1.70 s. Idle controller showed one matched subscriber per PX4 setpoint topic and no GROUND publications.
- Started read-only telemetry recorder and ran `.venv/bin/python -u test_px4_uav_env.py` once, in ROS domain 43 with the isolated model/LiDAR topic configuration. Mode and arm command ACKs were both PX4 result 0 in the active ULog. PX4 state transitioned to Offboard then armed. ULog maximum ascent was NED z=-1.6308 m at PX4 sim time 741.552 s; reset returned an observation only after heading-good became true. The smoke script executed ten low-speed `[0.2, 0.0]` actions; each reported no collision. No extra maneuvers or retry occurred.
- Existing cleanup sent one normal NAV_LAND (command 21, ACK result 0). PX4 logged “Landing at current position” and entered `nav_state=18` AUTO_LAND. `close()` timed out and warned that ROS control remained active; its background landing monitor and 20 Hz zero-velocity Offboard stream remain alive.
- Latest read-only telemetry at 2026-10-18 06:04:02 UTC: armed_state=2, nav_state=18, failsafe=false, GCS connected, preflight true; landed=false, at_rest=false, ground_contact=false. Local NED z=+0.032 m and vz=+0.0055 m/s; heading-good true. PX4 ULog latest status at sim time 886.72 s remains armed/AUTO_LAND; land flags are false through sim time 887.156 s. Gazebo dynamic pose samples at sim time 889.948–889.972 s show the model root steady at approximately `(-0.0836, 0.0217, -0.0130)`; this near-ground pose does not prove contact. The controller process remains active (test PID 27080); passive recorder continues at `takeoff_watch_after_close_20261018T055853Z.jsonl`.
- Recorder caveat: the first read-only recorder stopped when its JSON encoder encountered a NumPy float32 trajectory-setpoint field. Its log ends at the beginning of Offboard streaming. A corrected recorder was started immediately after the close timeout; its events are continuous afterward. PX4 ULog fills ACK/takeoff evidence during the gap.
- Logs: `runs/sar_demo_20261018/restart_20261018/logs/supervised_smoke_20261018T055722Z.log`, `takeoff_watch_20261018T055722Z.jsonl`, `takeoff_watch_after_close_20261018T055853Z.jsonl`, `px4_gazebo.log`, and `px4_rootfs/fs/log/2026-10-18/05_27_19.ulg`.
- No further command, parameter change, process stop, or flight attempt was made. Continue preserving the active control/telemetry processes and wait for review of a separate recovery action; do not declare landing success.
- *Superseded (recorded later):* the ULog shows PX4 auto-disarmed at PX4 time 920.000 s, after sustained ground contact at 917.308 s and landed/at-rest at 917.980 s. PX4 then reverted to `nav_state=14` (OFFBOARD). See the next section.

## Landing interleaving analysis, LAND-phase fix, and restart — 2026-10-18

- Start state at 13:19:53 UTC: `/home/hp/uav_ros` on `master` at `3203183`, clean, no untracked files.
- Isolated native-X500 baseline (`/home/hp/uav_ros_experiments/landing_baseline_20261018_03/`): NAV_LAND to auto-disarm took 7.412 PX4 s with no Offboard stream. This result was not modified.
- **Source-confirmed mechanism:** `/fmu/in/trajectory_setpoint` shares uORB `trajectory_setpoint` with PX4's native AUTO_LAND setpoints. The position controller uses the latest sample. The land detector forces `ground_contact=false` unless the latest setpoint commands descent. File and line references are in `RUNBOOK_ISOLATED.md`.
- **Log-confirmed:** in attempt 1, 232 of 878 logged setpoints between NAV_LAND and sustained contact were exactly `vz=0`, there were 14 ground-contact rising edges, and Offboard control was disabled. The baseline had 0 zero-velocity samples and 1 rising edge. Script and outputs: `/home/hp/uav_ros_experiments/landing_interleave_analysis_20261018/`.
- **Unverified:** the true interleave rate (the ULog samples at 5 Hz) and any contribution from host load or simulation slowdown.
- **Preflight `false` after attempt 1, diagnosed:** `pre_flight_checks_pass = canArm(nav_state)`. PX4 was in OFFBOARD after the auto-disarm, and the GROUND-phase controller publishes no Offboard signal, so the Offboard mode check failed. This was not a sensor or estimator fault, and no check was bypassed.
- **Recorder caveat:** the old recorder's `timestamp` field steps forward 15–30 s about every 30 s wall time (XRCE timesync epoch adjustment) and advances at about 0.43x wall between steps. Do not use it as a simulation-rate clock.
- **Code change:** `px4_uav_env.py` now latches `land_setpoints_released` when status reports AUTO_LAND during the LAND phase, and stops both Offboard publishers from then on. The latch is cleared only by a new `land()` request or `_takeoff()`. 11 new offline tests; the full suite has **106 passing** with ROS Jazzy and the overlay sourced. `py_compile -W error` and `git diff --check` pass. Five of the new tests fail against the pre-change code.
- **Shutdown of `restart_20261018`:** this was performed by a concurrent session, not by this session. At 13:16:07 `verify_shutdown_state.py` refused because control publishers were active. At 13:16:17 that session sent SIGINT to the exact ground-controller group 25309. At 13:16:26 the gate passed (status 0.117 s old, disarmed, `nav_state=14`, landed/at_rest/ground_contact true). From 13:16:28 to 13:16:49 that session ran the manifest-gated `isolated_sitl.py stop`. That session then ended, and its passive recorder (last sample 13:16:47) and stray `rg`/`gz topic` shell were terminated. All logs and the ULog are preserved.
- **Restart:** at 13:27:09 UTC `isolated_sitl.py start --run-dir runs/sar_demo_20261018/restart_20261018_smoke2` started the stack (instance 1, system ID 2, domain 43, UDP 18890, partition `uav_sar_validation_20261018_instance1`, own rootfs; ULog `px4_rootfs/fs/log/2026-10-18/13_27_28.ulg`). QGroundControl PID 24821 was kept running as the GCS.
- **Readiness probe at 13:27:46:** PASS. `system_id=2`, disarmed, `nav_state=4`, `failsafe=false`, `preflight=true`, GCS connected; landed/at_rest/ground_contact true; LiDAR 1080 beams (805 finite); 54 observation values. Estimator: tilt and yaw aligned, mag heading fused, consistent and observable, no mag or heading faults; `cs_mag_aligned_in_flight=false` as expected on the ground.
- New passive recorder: `restart_20261018_smoke2/logs/telemetry_full_20261018T1327Z.jsonl`. It records every message field, with wall UTC, monotonic time, and the PX4 timestamp kept separate, and it subscribes to `/fmu/in/*` to observe the environment's stream.

## Supervised smoke-test attempt 2 (LAND-phase fix) — passed — 2026-10-18

- One attempt, no retry. Gate at 13:30:08–13:30:19.9 UTC (`restart_20261018_smoke2/logs/preflight_gate_attempt2.txt`): PASS with `system_id=2`, disarmed, `nav_state=4`, `failsafe=false`, `preflight=true`, GCS connected; landed/at_rest/ground_contact true; LiDAR 1080 beams (805 finite); 54 observation values. No other publishers on the `/fmu/in` control topics. The unmodified `test_px4_uav_env.py` was then started.
- PX4 ULog `13_27_28.ulg`, sim seconds:
  - Offboard: command 176 at 91.366, ACK result 0 at 91.380, `nav_state=14` at 91.392.
  - Arm: command 400 at 91.567, ACK result 0 at 91.584, armed at 91.584.
  - `heading_good_for_control` true at 100.244; minimum NED z = −1.634 m at 100.404.
  - NAV_LAND: command 21 at 100.736, ACK result 0 at 100.740, AUTO_LAND (`nav_state=18`) at 100.752.
  - First `ground_contact` at 103.972 (+3.236 s); `landed`/`at_rest` at 104.660 (+3.924 s).
  - Automatic disarm at 106.668 (+5.932 s from the request; 2.008 s after landed). PX4 logged “Landing detected” and “Disarmed by landing”.
- Wall clock (host receipt, UTC): stream start 13:30:25.30; reset returned at 13:30:42.36 (z = −1.626 m); ten `[0.2, 0.0]` actions ran 13:30:42.46–13:30:43.36 with no collision and rewards about −0.1; NAV_LAND published at 13:30:43.362, ACK at 13:30:43.366, AUTO_LAND status received at 13:30:43.630; first ground contact at 13:30:50.126; landed/at_rest at 13:30:51.585; disarmed status received at 13:30:55.773; the smoke process exited with code 0 at 13:30:55.994.
- The Offboard stream stopped as designed. The recorder saw 0 `in_setpoint` and 0 `in_offboard_mode` messages after the AUTO_LAND status was received; the last of each arrived 0.03 s earlier. ULog landing window: 0 of 30 logged trajectory setpoints had `vz=0`, versus 232 of 878 in attempt 1; there was 1 ground-contact rising edge, versus 14 in attempt 1. Output: `/home/hp/uav_ros_experiments/landing_interleave_analysis_20261018/smoke2_13_27_28.txt`.
- Exactly one NAV_LAND was sent (environment commands seen: 176, 400, 21). There was no forced disarm and no extra command. `close()` returned without a timeout warning, and no environment process or `/fmu/in` publisher remains.
- Post-flight state: disarmed, landed/at_rest/ground_contact true. PX4 again reverted to `nav_state=14`, so `pre_flight_checks_pass=false` with the same Offboard-signal mode check. This is expected and does not block a later `reset()`, which streams setpoints before requesting the mode. A standalone readiness probe will report preflight false until PX4 is in Hold or the stack is restarted.
- Caveat: this is one sample. Landing timing was comparable to the native baseline; host load was not separately measured.

## Visible end-to-end smoke test (attempt 3) — passed — 2026-10-18

- Initial state at 13:40:44 UTC: the full `restart_20261018_smoke2` stack was still running (PX4 39114, Gazebo server 39321 and GUI 39322, XRCE 39106, bridge 39501, publisher 39607, recorder 39835, QGC 24821). PX4 was disarmed and landed, but `nav_state=14` gave `preflight=false` (the diagnosed post-disarm OFFBOARD reversion). A Gazebo GUI screenshot through `/gui/screenshot` showed it was rendering the X500 with LiDAR.
- Gated restart instead of a mode command:
  - `verify_shutdown_state.py` PASS at 13:41:47.
  - `isolated_sitl.py stop --run-dir runs/sar_demo_20261018/restart_20261018_smoke2` PASS at 13:41:59; it exited 0 at 13:42:18 and all 10 manifest PIDs exited.
  - The recorder (PID 39835) was closed with SIGINT.
  - `isolated_sitl.py start --run-dir runs/sar_demo_20261018/restart_20261018_visible3` ran 13:42:34–13:42:48.
  - Processes: XRCE 40813/40814, PX4 40821/40822 with Gazebo server 41028 and GUI 41029 (pgid 40821), bridge 41202/41214, publisher 41312/41329, recorder 41334, QGC 24821 kept.
- Pre-flight evidence:
  - `verify_isolated_topics` PASS (13:43:06 and 13:45:10): `system_id=2`, disarmed, `nav_state=4`, `failsafe=false`, `preflight=true`, GCS connected; landed/at_rest/ground_contact true; LiDAR 1080 beams (805 finite); 54 observation values.
  - Estimator flags healthy. The Gazebo model list contains only `x500_lidar_2d_1` (plus static boxes).
  - `/fmu/in` control topics had 0 publishers. `px4_uav_env.__file__` resolved to `/home/hp/uav_ros/px4_uav_env.py`, which contains the latch.
  - GUI screenshot rendered the drone. The user confirmed the window was visible before launch.
- An initial launch at 13:44:49 aborted inside the gate (a shell-variable bug in the wrapper); no ROS message or command was sent. The corrected gate passed at 13:45:20.970 and the unmodified `test_px4_uav_env.py` ran once.
- PX4 ULog `restart_20261018_visible3/px4_rootfs/fs/log/2026-10-18/13_42_52.ulg`, sim seconds:
  - Offboard: command 176 at 80.428, ACK 0 at 80.436, OFFBOARD at 80.448.
  - Arm: command 400 at 80.604, ACK 0 at 80.628, armed at 80.628.
  - `heading_good` at 89.308; minimum z = −1.642 m at 89.444.
  - NAV_LAND: command 21 at 89.776, ACK 0 at 89.784, AUTO_LAND at 89.796.
  - `ground_contact` at 93.012; landed/at_rest at 93.700.
  - Motor outputs reached ~0.002 by 94.088. Auto-disarm at **95.716 (+5.940 s from the request)**, logged as “Disarmed by landing”.
- Wall clock (UTC):
  - Stream start 13:45:23.28; Offboard ACK 13:45:24.37; arm ACK 13:45:24.77; liftoff about 13:45:28.4.
  - Reset returned 13:45:42.46 (z = −1.637 m). Ten `[0.2, 0.0]` actions ran 13:45:42.53–43.42 with no collisions.
  - NAV_LAND 13:45:43.425; AUTO_LAND received 13:45:43.707.
  - Ground contact 13:45:50.06; landed/at_rest 13:45:51.44; disarmed received 13:45:55.50. Exit code 0 at 13:45:55.724, with no warnings.
- Stream release: the recorder saw 0 `in_setpoint` and 0 `in_offboard_mode` messages after AUTO_LAND was received (the last arrived 0.027 s before). The only logged `vz=0` setpoint in the window, at 89.780, came before AUTO_LAND at 89.796. There was 1 ground-contact edge. Environment commands seen: 176, 400, 21 only.
- Propellers: after landing, PX4's motor-speed command to Gazebo was `0,0,0,0`, and rotor_0's pose was unchanged across 3 s at 13:46:26.
- Visual evidence: 40 GUI frames in `restart_20261018_visible3/logs/gui_frames/` (airborne about 13:45:30–13:45:47, landed at 13:46:00).
- Analysis output: `/home/hp/uav_ros_experiments/landing_interleave_analysis_20261018/visible3_13_42_52.txt`.

## Landing-fix commit and target/reward audit — 2026-10-18

- Committed `e10b92e` ("Stop Offboard setpoints once PX4 confirms AUTO_LAND") with exactly four files. Before committing: 106 tests passed, `py_compile -W error` and `git diff --check` were clean. Run data stays gitignored.
- Audit report: `runs/sar_demo_20261018/TARGET_REWARD_AUDIT.md`. Probes and outputs: `/home/hp/uav_ros_experiments/target_reward_audit_20261018/`.
- Key measured findings:
  - The frames agree: Gazebo (3, −3) ENU equals the env's (−3, 3) NED.
  - PX4 heading is offset about 4° from the Gazebo yaw.
  - The target is detected by the LiDAR on the ground but **never in flight**, because the scan plane at hover (about 1.9 m) passes over the 0.555 m sphere top.
  - The observation has no ego-position or target vector.
  - Simulation runs at about 0.49× real time, so one env step is about 0.05 s sim, wall-clocked.
  - The smoke actions moved the vehicle at most 0.045 m/s.
  - `reset()` does not reposition the vehicle.
  - The target model is an uncommitted edit to the shared PX4 `walls.sdf`.
  - SB3 and torch are not installed.
- No environment, reward, or observation change was made. Proposals are in §6 of the audit and await review.

## Implementation plan: camera perception to target-reaching RL — 2026-10-18 14:55 UTC

- Start state: `master` at `e2eb209`, clean. The live `restart_20261018_visible3` stack is running (disarmed, landed) and is not modified by this work. Stand-experiment evidence stays at `/home/hp/uav_ros_experiments/oakd_sensor_rig_20261018/`; visible smoke-test evidence stays in `restart_20261018_visible3/`.
- Milestones, each committed separately once its offline tests pass:
  1. Versioned simulation assets: `sim/worlds/sar_walls.sdf` (red target) and `sim/models/x500_lidar_2d_oakd` (x500 + horizontal LiDAR + OAK-D Lite pitched 30° down). The launcher gains an opt-in `--profile sar_oakd` with a run-local `gz_env.sh` and `PX4_SYS_AUTOSTART=4013`. The legacy profile stays the default.
  2. Target perception: a ROS-free `target_perception.py` (RGB red segmentation, depth mapping, known-radius sphere fit, edge handling, validity) and `target_perception_node.py` publishing a versioned Float64MultiArray with an image timestamp.
  3. Observation contract `sar_obs_v1`: a shared `sar_observation.py` builder (LiDAR sectors, previous action, EKF body velocity, yaw rate, height, current detection, and a dead-reckoned last-seen memory with expiry). Ground truth lives only in `sar_task.py` (reward and success). The publisher becomes LiDAR-only (no Gazebo pose).
  4. Episode and control changes: fly to a fixed start pose after takeoff, return to start before landing after success or timeout, sim-time step duration, altitude hold during velocity control, and a world/target consistency check.
  5. A kinematic stand-in environment that uses the same builder and task code, plus a baseline controller and offline tests.
  6. PPO: SB3 and CPU torch with pinned gymnasium/numpy; training and evaluation entry points; a short smoke run on the stand-in; comparison with the baseline.
  7. Preparation for one supervised SITL target-reaching test (gated; not flown without review).

## Camera perception to target-reaching RL: offline implementation — 2026-10-18 15:30 UTC

The live `restart_20261018_visible3` stack was not modified, and no flight was made. Commits, each with the full suite passing:

| Commit | Content |
|---|---|
| `d8c9fe6` | Versioned `sim/worlds/sar_walls.sdf` (target sphere) and `sim/models/x500_lidar_2d_oakd` (stock LiDAR mount + OAK-D Lite at 30° down). Opt-in `isolated_sitl.py --profile sar_oakd` (run-local `gz_env.sh`, airframe 4013, camera bridge remapped to `/uav/camera/*`). The model loaded and merged in a paused, sensorless Gazebo load test. |
| `5e8f797` | `target_perception.py` (ROS-free; RGB segmentation, depth mapping, sphere fit, edge and invalid-depth handling, versioned message) and `target_perception_node.py`. Tests use synthetic ray-cast frames and three recorded Gazebo stand frames (RGB-only bearing within 0.2°). |
| `64782e5` | Observation contract `sar_obs_v1` (65 values, `docs/OBSERVATION_CONTRACT.md`), privileged `sar_task.py`, LiDAR-only publisher, and environment integration. A leak test shows that moving the ground-truth target leaves the observation identical. |
| `12d0fee` | Fly-to start pose, return to start before landing after success or timeout, altitude hold, simulation-time 0.1 s steps (local-position stamp clock; timesync is only ~1 Hz sim), and the `verify_world_target.py` gate. |
| `631db3f` | Offline kinematic stand-in (same contract and task code) and the observation-only baseline. Baseline: 10/10 fixed-start and 25/25 randomized-start successes in tests. |
| `1b41b92` | PPO pipeline: SB3 2.9.0 + torch 2.9.1+cpu installed, gymnasium 1.3.0 and numpy 1.26.4 unchanged. 20k-step smoke run in `runs/rl/ppo_smoke_20261018_s1` (git-ignored). |

**Smoke-run results** (30 held-out episodes):

| Policy | Fixed start | Randomized start |
|---|---|---|
| Baseline | 100% success, 8.4 s | 100% success, 14.0 s |
| Random | 100% collision | 17% success, 47% collision |
| PPO, 20k steps | 0% success, 100% timeout | 0% success, 100% timeout |

During training, PPO had 2 successes, 19 collisions, and 25 timeouts. It converged to not moving (avoiding the −100 collision penalty against a sparse +100 success). Reward shaping or curriculum work is needed before extended training.

**Offline validation:** the real `PX4UAVEnv`, `target_perception_node`, and LiDAR publisher were constructed with real rclpy in an empty ROS domain (49). Subscriptions and publishers match the PX4 DDS topic names.

**Next step:** the gated supervised SITL test procedure is in `RUNBOOK_ISOLATED.md`; it has not been run.

## Supervised target-reaching SITL attempt (sar_oakd): stopped at readiness gates, no flight — 2026-10-18

- **Approved:** exactly one experiment. Start state: `master` at `27a70b4`, clean.
- **Release:**
  - Shutdown gate PASS at 16:15:23 (disarmed, `nav_state=14`, landed/at_rest/ground_contact true).
  - `isolated_sitl.py stop --run-dir .../restart_20261018_visible3` PASS; all manifest processes exited. Its recorder (PID 41334) was stopped with SIGINT.
  - A leftover from this session was found and stopped: the paused, sensorless model load-test Gazebo server (PID 52093; partition `sar_loadtest_52088`; config in a temporary folder), running since milestone 1 at about 14% CPU. It was stopped with SIGINT to its own process group.
- **Start:** 16:16:22–16:16:37 UTC, `isolated_sitl.py start --profile sar_oakd --run-dir runs/sar_demo_20261018/restart_20261018_oakd1`.
  - PX4 used `SYS_AUTOSTART 4013` (from the environment), world `sar_walls` from `sim/`, model `x500_lidar_2d_oakd_1`, and the GUI.
  - All 5 manifest components were alive. Camera topics were remapped onto `/uav/camera/*`.
  - Recorder: `logs/telemetry_full.jsonl`. ULog: `px4_rootfs/fs/log/2026-10-18/16_16_42.ulg`.
- **Gate results** (`logs/gate_*.txt`, `logs/cpu_snapshot.txt`):
  - **Simulation speed FAIL.** Gazebo `/stats` reports RTF 0.023, steady across six samples from 16:17:39 to 16:17:44. The true simulation rate from sensor stamps is **0.157** (camera image stamps over 105 s wall) to **0.189** (local-position stamp clock over 139 s wall). Gazebo `/stats` under-reports under this load. Both are below the required 0.35.
  - **Load:** the Gazebo server was at 369% CPU and the GUI at 208%; the GPU was idle (software rendering). PX4's server config also loads `GstCameraSystem`, which attached a GStreamer pipeline to the OAK-D RGB camera (GLib `GstNvAutoGp` message in `px4_gazebo.log`).
  - **Topic probe FAIL**, as a consequence of the slow simulation: status 2.3 s and land detection 5.3 s stale against the 2 s / 4 s probe limits. The latest status was disarmed, `nav_state=4`, `preflight=true`, no failsafe, GCS connected.
  - **World target:** the first run reported a false FAIL ("found 2"). `gz topic -e -n 1` occasionally prints two messages; the gate tool now parses only the first message, and the live check then passed 5/5. The target is at ENU (3, −3, 0.055), matching `sim/worlds/sar_walls.sdf`.
  - **Camera and perception, on the ground only:**

| Topic | Wall rate |
|---|---|
| `/uav/camera/rgb/image` | 1.8–3.1 Hz |
| `/uav/camera/depth/image` | 2.2–4.0 Hz |
| `/uav/target_perception` | 1.8–3.0 Hz (a message for every RGB frame) |
| `/uav/lidar_sectors` | 10 Hz |

    Perception processing took about 6 ms per frame. `detected=0` and status 1 (no target), as expected with the target about 45° right of the spawn heading. The pipeline is publishing and the data is fresh, but no detection was exercised live.
- **Outcome:** no flight command was sent and the vehicle was never armed. There was no retry. The stack was left running, disarmed and landed.

## GstCameraSystem investigation, project server config, and camera verification (no flight) — 2026-10-18

- **Start (16:38 UTC):** `master` at `ce3e319`. The `restart_20261018_oakd1` stack was running: disarmed, Hold, landed/at_rest, no failsafe; gz server at 439% CPU.
- **GstCameraSystem (read-only findings):**
  - It is loaded only by PX4's `src/modules/simulation/gz_bridge/server.config`, through `GZ_SIM_SERVER_CONFIG_PATH` in `gz_env.sh`.
  - Source (`src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.cpp`): it discovers new `components::Camera` sensors (RGB only, not the depth camera), subscribes to their Gazebo image topic, and H.264-encodes the frames for UDP:5600 (or RTMP) ground-station video.
  - It does not render, publish to ROS, touch depth, or feed perception. The project path is: sensors system render → Gazebo topics → `ros_gz_bridge` → `/uav/camera/*` → `target_perception_node` → `/uav/target_perception` → `PX4UAVEnv`.
  - Runtime evidence: `libgstnvcodec`, `libnvidia-encode` and `libcuda` were mapped (NVENC on the GPU). GStreamer threads used about 8% CPU against **353% for Mesa llvmpipe** threads plus 91% for the gz main loop.
- **Change (reversible):** `sim/server_no_gstreamer.config` is PX4's active `server.config` (01193bf86b) with only the `custom::GstCameraSystem` entry removed; a test enforces this. The `sar_oakd` profile exports it in the run-local `gz_env.sh`. `isolated_sitl.py start --px4-server-config` restores PX4's default. PX4 files, the camera model, resolution, frame rates, depth and LiDAR are unchanged.
- **Restart:**
  - Shutdown gate PASS at 16:49:44 (`--duration 10`).
  - The manifest-gated stop passed (status 1.8 s, land 0.7 s old); all processes exited, and the recorder was stopped with SIGINT.
  - `restart_20261018_oakd2` started 16:52:38–16:52:53 with GUI. The gz server (PID 58064) uses `sim/server_no_gstreamer.config` with no GStreamer libraries or threads.
- **WSLg incident:** the `weston` compositor segfaulted at 16:48:57 (dmesg; not an OOM kill), and WSLg restarted it. QGroundControl (PID 24821) exited ("Wayland connection broke"). It was relaunched from the SHA-256-verified AppImage with the same private config, in its own process group (session leader 58696, QGC 58699, UDP 14550). Records: `restart_20261018_oakd2/qgroundcontrol.process.json`, log `restart_20261018_oakd2/logs/qgroundcontrol.log`. The weston crash cause is unknown; it occurred while both gz windows were software-rendering.
- **Readiness (oakd2):**
  - PX4: disarmed, Hold, `preflight=true`, `gcs_connection_lost=false`, landed/at_rest, "Ready for takeoff".
  - 49 LiDAR sectors; the live target matches the versioned world.
  - The topic probe still FAILS on land-detection freshness (5.6 s old; land samples arrive every ~6.3 s wall at this simulation rate).
- **Performance** (`/home/hp/uav_ros_experiments/gst_camera_perf_20261018/`, 60 s windows, simulation rate from sensor stamps):

| Metric | Before (with GStreamer) | After (without) |
|---|---|---|
| Sim rate (RGB / depth / PX4 stamps) | 0.177 / 0.178 / 0.194 | 0.161 / 0.163 / 0.239 |
| gz server CPU | 459% | 425% |
| RGB wall rate | 1.56 Hz | 2.50 Hz |
| Depth wall rate | 2.26 Hz | 2.43 Hz |
| Perception wall rate | 1.76 Hz | 2.60 Hz |
| Max perception gap | 2.87 s | 1.55 s |
| PX4 status gap, median | 2.9 s | 3.2 s |
| GPU | 3–4% / 316 MiB (NVENC) | 0% / 0 MiB |
| RAM / swap used | 2275 / 1135 MiB | 3037 / 782 MiB (VM capped at 4 GB by `.wslconfig`) |

  The **0.35 gate still fails.**
- **Positive ground-only detection** (vehicle untouched, disarmed; target fixture moved only):
  - At 16:56:15 the static target was moved with `set_pose` to ENU (2.2, 0.4, 0.055), about 2.2 m ahead and left of the parked drone. All 109 of 109 messages over 40 s reported detected.
  - Range-valid (sphere-fit) messages: bearing **+10.33° to +10.34°** against +10.30° ground truth; horizontal range **2.235 m** against 2.236 m.
  - Messages without a depth frame paired within 0.1 s fell back to bearing-only (`range_valid=0`, confidence 0.2, +14.78°: edge-cut centroid bias), as designed.
  - The target was restored to (3, −3, 0.055); `verify_world_target.py` passed 3/3.
  - The GUI screenshot does not show the temporarily moved static sphere; the GUI scene does not appear to refresh a static-model `set_pose`, while the server-side camera did render it.
- **Root cause and recommendation:** the Gazebo server and GUI each run 16 Mesa llvmpipe threads. `/dev/dri` is absent; only `/dev/dxg` exists. `libd3d12.so` resolves through `ldconfig` but is never loaded, so Mesa's D3D12 (GPU) driver is not selected. Next step for approval: select Mesa D3D12 for the gz processes (`GALLIUM_DRIVER=d3d12`, `sar_oakd` profile only, reversible), validated first, plus consider a larger WSL memory cap.
- **State:** no arming, takeoff, or flight command. The `oakd2` stack, QGC, and the recorder are running; the vehicle is disarmed and landed.

## D3D12 GPU rendering for sar_oakd (no flight) — 2026-10-18 21:43–21:51 UTC

- **Start:** `master` at `236c4df` plus uncommitted d3d12 launcher changes. The `oakd2` stack was disarmed, landed, and software-rendered.
- **Probe:** `GALLIUM_DRIVER=d3d12` alone selects the Intel UHD iGPU, which offers only GL 4.1 core. The user chose the RTX 3050 (`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, GL 4.6 core).
- **Change:**
  - For the camera profile only, `tools/isolated_sitl.py` writes both variables into the run-local `gz_env.sh`. `px4-rc.gzsim` sources that file before launching the gz server and GUI, so no other process, profile, or PX4 file is affected.
  - `start --software-rendering` reverts to the default llvmpipe renderer.
  - The manifest records `gz_rendering`.
  - The full suite passes (222 tests).
- **Gated restart:**
  - Shutdown gate PASS at 21:45:17.
  - The first `isolated_sitl.py stop` was **refused** (status 2.12 s old against its 2.0 s limit; with the slow simulation, status arrived about every 3 s). Nothing was stopped by that attempt.
  - The unchanged gated stop passed on the retry (21:46:14: status 0.55 s, land 2.65 s old; disarmed and landed). All manifest groups exited.
  - The recorder (PID 58707) was stopped with SIGINT before the first refusal, so the `oakd2` telemetry log ends at about 21:45:30.
  - QGC (session 58696) was kept running.
  - `restart_20261018_oakd3_gpu` started 21:46:46–21:47:00 with the GUI. Recorder: `logs/telemetry_full.jsonl`. Ogre2 log copy: `logs/ogre2_d3d12.log`.
- **Renderer verified:** both gz processes map `libd3d12` and `libnvwgf2umx` and run no llvmpipe threads. Ogre2 reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`, GL 4.6 core. The RTX 3050 runs at 36–51% (578 MiB). There are no rendering errors and no new compositor faults.
- **Performance** (60 s windows, simulation rate from sensor stamps; `/home/hp/uav_ros_experiments/gst_camera_perf_20261018/`):

| Metric | CPU rendering (oakd2) | D3D12 window 1 | D3D12 window 2 |
|---|---|---|---|
| Sim rate (RGB / depth / PX4) | 0.161 / 0.163 / 0.239 | **0.376 / 0.375 / 0.397** | **0.373 / 0.374 / 0.419** |
| gz server CPU | 425% | 114% | 114% |
| gz GUI CPU | ~206% | — | 111% |
| RGB / depth (wall) | 2.5 / 2.4 Hz | 7.1 / 7.9 Hz | 7.0 / 8.4 Hz |
| Perception (wall), max gap | 2.6 Hz, 1.55 s | 7.7 Hz, 0.52 s | 7.6 Hz, 0.44 s |
| Status gap max / land gap max | 3.7 / 6.7 s | 1.7 / 3.1 s | 1.7 / 3.1 s |
| RAM / swap used | 3037 / 782 MiB | 2825 / 567 MiB | 3079 / 634 MiB |

- **Readiness:** `verify_isolated_topics` PASS (twice): disarmed, Hold, `preflight=true`, GCS connected, landed/at_rest, 49 LiDAR sectors. `verify_world_target` PASS.
- **Positive ground-only detection** (vehicle untouched; static target fixture moved to ENU (2.2, 0.4, 0.055) at 21:49:04, then restored):
  - 201 of 201 messages detected after the first 3 frames; 198 were range-valid.
  - Bearing +10.34° against +10.30° ground truth; range 2.235 m against 2.236 m (0.03%). There were 3 bearing-only fallbacks.
  - After the restore, 60 messages reported 0 detections and the gate passed 3/3.
  - The GUI displayed the moved static sphere late (after the server-side restore); the server-side camera and pose were authoritative.
- **Gate:** the simulation rate is ≥ 0.35 in both windows, but the margin is small (0.373–0.376 on the camera stamps). No arming, takeoff, or flight command was sent.

## Pre-flight stability check before the supervised target-reaching episode: FAILED, no flight — 2026-10-18 21:55–21:58 UTC

- **Start:** `master` at `ce65e44`, clean. `restart_20261018_oakd3_gpu` was running on D3D12/RTX 3050 with the GUI and recorder; the vehicle was disarmed, Hold, landed/at_rest, `preflight=true`, GCS connected.
- **Check:** six consecutive 25 s windows (`measure.py`; results in `logs/stability_w1..6.json` and `logs/stability_check.txt`). Required in every window:
  - simulation rate ≥ 0.35 from RGB, depth, and PX4-position stamps;
  - status gap ≤ 2.0 s, land gap ≤ 4.0 s, perception gap ≤ 1.5 s;
  - camera, depth, and perception flowing, and LiDAR ≥ 5 Hz;
  - vehicle disarmed, Hold, landed/at_rest, preflight true, no failsafe, GCS connected.

| Window | Sim rate (RGB / depth / PX4) | Result |
|---|---|---|
| 1 | 0.377 / 0.372 / 0.417 | PASS |
| 2 | 0.379 / 0.378 / 0.380 | PASS |
| 3 | 0.375 / 0.375 / 0.432 | PASS |
| 4 | 0.360 / 0.362 / 0.415 | PASS |
| 5 | **0.3499 / 0.3489** / 0.414 | **FAIL** (camera-stamp rate below 0.35) |
| 6 | 0.360 / 0.360 / 0.414 | PASS |

- **Other gates:** passed in all windows. Status gap ≤ 1.78 s, land gap ≤ 3.10 s, perception gap ≤ 0.60 s; RGB 6.8–7.6 Hz, depth 6.5–7.8 Hz, perception 7.2–8.1 Hz, LiDAR 10 Hz; gz server CPU 111–117%, GPU 35–51%.
- **Outcome:** the approved episode was **not** run. No arming, takeoff, or flight command was sent, and the threshold was not changed.
- **Note:** the measurement subscribes to the 1080p RGB stream itself, which adds load. The rates show a downward drift over the check (0.377 → 0.35–0.36).
