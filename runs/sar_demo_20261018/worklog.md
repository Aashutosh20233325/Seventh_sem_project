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

## Monitoring-overhead reduction and repeat stability check: PASS, no flight — 2026-10-18 22:00–22:04 UTC

- **CPU audit (`top`, 5 s):**

| Process | CPU | Note |
|---|---|---|
| QGroundControl | 134% | llvmpipe UI rendering plus GstVideoWorker threads |
| gz server | 117% | |
| gz GUI | 112% | |
| ros_gz_bridge | 41% | |
| PX4 | 26% | |
| perception node | 26% | |
| recorder | 10% | |

  - Outside a measurement, the only subscriber to the RGB and depth images is `target_perception`.
  - The previous stability check (`measure.py`) added a second full-HD RGB and depth image subscriber, making the bridge serialize every frame twice and decoding it in Python. That load is not present in flight.
- **Change (measurement only; not in the repository):** `measure_light.py` (copied to `logs/`) takes camera rates and the camera-clock simulation rate from `/uav/camera/{rgb,depth}/camera_info`, which carry each frame's sensor stamp, and subscribes to no images. The recorder, perception node, LiDAR, bridge, PX4, GUI, and QGroundControl were untouched.
- **QGroundControl** was kept running and not modified. Closing it would remove the GCS heartbeat, and with `NAV_DLL_ACT=2` that sets `gcs_connection_lost` and fails preflight. Minimising its window would be a manual user action; it was not needed.
- **Six 25 s windows** (`logs/stability_light_w1..6.json`, `logs/stability_check_light.txt`), with the same gates as before. **All six PASSED.**

| Metric | Range across the six windows |
|---|---|
| Sim rate (RGB / depth / PX4) | minimum 0.359 / 0.359 / 0.376; range 0.359–0.446 |
| Status gap max | 1.35–1.81 s |
| Land gap max | 2.61–3.18 s |
| Perception gap max | 0.49–0.65 s |
| RGB / depth frames (camera_info) | 10.8–11.9 Hz wall |
| Perception | 7.0–7.8 Hz |
| LiDAR | 10 Hz |
| CPU: gz server / GUI / QGC / bridge / perception / recorder / PX4 | ~115 / ~115 / 125–165 / ~41 / ~25 / ~10 / ~26% |
| GPU | 35–48%, 563 MiB |

- **Memory risk:** RAM was 3.40–3.48 of 3.91 GB, and swap rose steadily from 1771 to 1991 MiB over the 2.7 min (it was 567–634 MiB at 21:47–21:51). The WSL VM is capped at 4 GB by `.wslconfig`.
- **Outcome:** no arming, takeoff, flight command, or episode. The vehicle stayed disarmed and landed.

## WSL memory investigation and leak finding: stack stopped, nothing applied — 2026-10-18 22:10–22:15 UTC

- **`.wslconfig`:** `C:\Users\HP\.wslconfig` is `memory=4GB, swap=4GB`. It was backed up unchanged to `/home/hp/uav_ros_experiments/wsl_memory_20261018/` and **not edited**.
- **Host RAM:** could not be read, because Windows interop is not registered in this WSL instance. The user must check it on the Windows side.
- **Leak found** (RSS + swap, 60 s sampling): the gz GUI grew **+99 MiB/min** (2832 → 2931 MiB) and the gz server **+37 MiB/min** (1704 → 1741 MiB). VM swap grew +170 MiB/min, reaching 3386 of 4096 MiB, which projected exhaustion within about 4 min. The cause is not established (Mesa D3D12, gz GUI, or other).
- **Action:** to prevent an uncontrolled OOM kill, the simulation stack (not WSL) was stopped through the documented gate.
  - 22:14:14: gate PASS (status 1.45 s, land 1.04 s old; disarmed, landed).
  - All manifest groups exited, and the recorder (PID 60949) was stopped with SIGINT.
  - Memory returned to 1.1 GB RAM used and 0.7 GB swap.
  - QGroundControl (session 58696) is still running.
- **State:** no simulation is running. The vehicle was never armed in run `restart_20261018_oakd3_gpu`. Its evidence is preserved in that run's `logs/`.

## gz memory growth: GPU (D3D12) vs software rendering, no flight — 2026-10-18 22:28–22:46 UTC

- **Setup:** WSL kept at 4 GB RAM and 4 GB swap (no `.wslconfig` change, no `wsl --shutdown`). Evidence and method: `/home/hp/uav_ros_experiments/gz_memory_growth_20261018/README.md`.
- **Tests:** two 300 s visible-GUI tests, each through the gated start and stop. The vehicle stayed disarmed and landed; camera, depth, and LiDAR were unchanged. Samples every 10 s.

| | **A** `restart_20261018_memA_gpu_gui` (D3D12, RTX 3050) | **B** `restart_20261018_memB_sw_gui` (`--software-rendering`, llvmpipe) |
|---|---|---|
| gz server anon+swap | **+47 MiB/min** | **flat (−0.5 MiB/min)** |
| gz GUI anon+swap | **+96 MiB/min** | **flat (+1.0 MiB/min)** |
| Mapping count | server +32/min, GUI +60/min | flat |
| File-backed memory | flat | reclaimed |
| GPU memory | flat 573–581 MiB | 0 |
| End-window sim rate (RGB / depth / PX4) | 0.335 / 0.336 / 0.393 | 0.171 / 0.177 / 0.178 |
| Readiness probe | passed | could not pass (land gaps ~6 s against the 4 s limit); sampled as a recorded no-flight diagnostic. Attempt 1, which did not sample, is preserved. |

- **Conclusion:** the continuous growth is specific to the D3D12 rendering path and occurs in **both** gz processes. The GUI contributes about two thirds. It is anonymous host memory with linearly increasing mappings, not GPU memory, and it has not plateaued. That supports a genuine leak in the D3D12 path or its interaction with ogre2; the exact allocator is unproven.
- **Headless diagnostic:** not needed and not run.
- **Test A margin:** Test A's end-window sim rate (0.335) was below the 0.35 gate under sampler, recorder, and probe load, so the D3D12 margin remains thin.
- **State:** no simulation running; QGroundControl (58699) running. Nothing was armed or flown, and no project code changed.

## Mesa D3D12 leak probe (Gazebo-free), no flight — 2026-10-22 04:49–05:00 UTC

- **Probe:** a standalone off-screen GL loop rendering and reading back 1920x1080 RGB and 640x480 float frames at 10 Hz for 180 s. Each renderer was confirmed from its GL strings and the shared objects mapped into the process.
- **Result:** flat on both renderers. llvmpipe: −0.00 MiB/min, maps 424 → 436. D3D12 on the RTX 3050 (Mesa 25.2.8-0ubuntu0.24.04.2): −0.00 MiB/min, maps 489 → 501. The Gazebo leak is not reproduced by a plain frame readback.
- **Step B** (private Mesa 26.2.5) was not run, because its precondition was not met. No packages, PPA, or WSL settings were changed. The investigation is paused at the user's request.
- **Evidence:** `/home/hp/uav_ros_experiments/mesa_d3d12_leak_20261022/README.md`.

## Fresh no-flight readiness check with the runtime guard: FAILED (speed), stack stopped — 2026-10-22 05:40–05:45 UTC

- **Start:** `master` at `2574773` (runtime guard), clean, with the full offline suite at 244 passed. Before the start, only QGroundControl was running.
- **Run:** `restart_20261022_preflight1` (`--profile sar_oakd`, D3D12/RTX 3050, visible GUI). It used the gated start, recorder, readiness gates, six light 25 s stability windows, a concurrent read-only guard check, and the gated stop. Orchestration and evaluators: `/home/hp/uav_ros_experiments/preflight_guard_20261022/` (`run_preflight.sh`, `evaluate_window.py`, `guard_live_check.py`, run log, `restart_20261022_preflight1_mem.txt`). The evaluator reproduces the 2026-10-18 verdicts (6/6 PASS) and Test A's FAIL.
- **Readiness gates — all PASS:**
  - `verify_isolated_topics` passed on attempt 2, with the gate unchanged: disarmed, nav 4, no failsafe, preflight true, GCS connected, landed/at_rest, 49 sectors.
  - `verify_world_target` passed.
  - `/fmu/in/{offboard_control_mode,trajectory_setpoint,vehicle_command}` had 0 publishers.
  - `ogre2.log` reported `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)`; the server had no llvmpipe threads.
- **Stability check: 0/6 windows PASS.** The camera simulation speed was below the **unchanged 0.35× gate** in every window. Every other window criterion passed.

| Window | Sim rate RGB / depth / PX4 | Status / land / perception gap (s) | Result |
|---|---|---|---|
| 1 | 0.342 / 0.334 / 0.405 | 1.69 / 3.23 / 0.45 | FAIL |
| 2 | 0.347 / 0.344 / 0.404 | 1.64 / 3.13 / 0.40 | FAIL |
| 3 | 0.334 / 0.334 / 0.334 | 1.61 / 3.09 / 0.60 | FAIL |
| 4 | 0.343 / 0.343 / 0.404 | 1.63 / 3.22 / 0.51 | FAIL |
| 5 | 0.343 / 0.343 / 0.406 | 1.57 / 3.10 / 0.60 | FAIL |
| 6 | 0.335 / 0.335 / 0.397 | 1.77 / 3.32 / 0.60 | FAIL |

  RGB/depth ran at 10.0–10.4 Hz wall, perception at 6.5–7.3 Hz, and LiDAR at about 10 Hz. The 2026-10-18 light check measured 0.359–0.395, so that margin did not reproduce.
- **Memory:** MemAvailable fell from **1517 MiB** (30 s after start) to **1036 MiB** at 3.5 min, about −137 MiB/min. Swap stayed flat at 441 MiB.
  - gz server RSS grew 581 → 758 MiB (about +50 MiB/min).
  - gz GUI RSS grew 591 → 910 MiB (about +80 MiB/min).
  - Consequence: start-up, gates, and six windows take about 4.5 min. By then MemAvailable is below the guard's 1.2 GiB start threshold, so `guard_refuse` would trigger even with passing speed.
- **Live guard validation (read-only, `logs/guard_live_check.jsonl`):** `PX4UAVEnv` was not instantiated, so there were no `/fmu/in` publishers and no timer. A subscriptions-only node fed the env's own `position_callback`, `_advance_sim_clock`, and `perception_callback` into a shim, which the runner's `px4_telemetry` adapter and `RuntimeGuard` then read.
  - **Freshness:** both clocks were populated and fresh in all 175 checks. Maximum age was PX4 0.10 s (limit 1.0) and camera 1.02 s (limit 1.5).
  - **Accuracy:** guard rates matched an independent computation from the raw stamps to within 0.0024 (camera) and 0.015 (PX4, a window-edge difference). The guard used the lower clock, the camera.
  - **Memory:** MemAvailable was populated (1323 → 1020 MiB).
  - **Verdicts:** the preflight waited 29 s for a full window, then would have refused with `speed_below_floor` (speed 0.340). All 175 per-step checks returned `speed_below_floor`.
  - **Not exercised live:** the stale, gap, and missing paths (unit-tested only), and in-episode behavior.
- **Stop:** gated stop attempt 1 at 05:45:07. The gate passed (status 0.56 s, land 0.19 s old; disarmed, landed, at rest). Only manifest-owned groups were stopped, and the recorder was stopped with SIGINT. Afterwards no stack processes remained, and 2.7 GB was available.
- **Outcome:** no arming, take-off, flight command, or episode. The live episode is blocked by the speed gate and the memory start threshold while the gz GUI runs on D3D12. No gate was changed. QGroundControl (58699) is still running. Logs: `runs/sar_demo_20261018/restart_20261022_preflight1/logs/`.

## Split-rendering no-flight readiness run: FAILED at the renderer gate (checker bug), stack stopped — 2026-10-23 17:01–17:02 UTC

- **Code:** `master` at `74f783b` (split rendering `b9b4588`). Run `restart_20261023_split1`, `start --profile sar_oakd --split-rendering`. Orchestration is a copy of the 2026-10-22 script, `/home/hp/uav_ros_experiments/preflight_split_20261023/run_preflight_split.sh`; the original is unchanged and an identical copy is kept beside it.
- **Pre-state:** only QGroundControl was running; domain 43 had no nodes, the gz partition was empty, UDP 18890 was free, and 2615 MiB was available.
- **Gates:**
  - `verify_isolated_topics` PASS (attempt 2): disarmed, nav 4, no failsafe, preflight true, GCS connected, landed/at_rest, 49 sectors.
  - `verify_world_target` PASS.
  - `/fmu/in` publishers 0/0/0.
  - **`render-check` FAIL:** it found no gz processes. Cause: gz rewrites its process title, so `/proc/<pid>/cmdline` is one space-joined string padded with NULs (`gz sim --verbose=1 -r -s …`), which the parser (and its tests) did not expect. A checker bug, not an isolation failure.
  - Per the script, the stability windows were skipped; **camera speed was not measured**.
- **Manual renderer verification (read-only /proc):**

| Process | libd3d12 mapped | llvmpipe threads | Environment |
|---|---|---|---|
| gz server, pid 111229 (PX4 group) | 8 | 0 | `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `HEADLESS=1` |
| gz GUI, pid 111285 (launcher-owned `gz_gui`) | 0 | 16 | `GALLIUM_DRIVER=llvmpipe` |

  PX4's log has no "Starting gz gui" line. **The isolation works as designed.**
- **GUI:** 3 `/gui/*` topics; the screenshot `logs/gui_after_start.png` shows the walls, the x500, and the red sphere. PX4 logged "Camera follow offset set to -2.0, -2.0, 2.0" before the launcher's GUI started. The screenshot is a wide default view, so follow was probably not applied (cannot be confirmed on the ground).
- **Memory** (`preflight_split_20261023/restart_20261023_split1_mem.txt`):
  - MemAvailable was 2615 MiB before start, 1195 MiB at +30 s, and **1118 MiB minimum** at +60 s. Swap was flat (285–287 MiB).
  - The GUI on llvmpipe used **~830 MiB RSS** (GPU GUI on 2026-10-22: 591 MiB), roughly flat over ~50 s. The server grew 551 → 557 MiB.
  - **Consequence:** under split rendering, MemAvailable is below the runtime guard's 1.2 GiB start threshold within ~30 s of start, so `guard_refuse` would trigger independently of speed. The threshold is unchanged.
- **Stop:** gated stop attempt 1 PASS (status 0.90 s, land 1.77 s; disarmed, landed, at rest), manifest `stopped_at_utc` 17:02:26. Afterwards there were no stack processes and 2545 MiB was available. QGroundControl (58699) is still running.
- **Outcome:** no arming, take-off, flight command, or episode.

## Split-rendering no-flight readiness run 2: speed gate FAIL (4/6 windows), stack stopped — 2026-10-23 17:31–17:37 UTC

- **Code:** `master` at `8a73737`. Run `restart_20261023_split2`, `start --profile sar_oakd --split-rendering`, after the WSL memory change (the log shows 4915 MiB total, 4096 MiB swap). Orchestration: `/home/hp/uav_ros_experiments/preflight_split_20261023/run_preflight_split.sh`.
- **Gates:** `verify_isolated_topics` PASS, `verify_world_target` PASS, `/fmu/in` publishers 0/0/0 (plain `ros2 topic info`), `render-check` exit 0.
- **Stability: 4/6 windows PASS.** w1 0.341× and w2 0.316× (RGB and depth) failed the unchanged 0.35× gate; w3–w6 passed at 0.363–0.379×. The windows' PX4 term (local-position stamps, slewed by PX4) read 0.364–0.428×.
- **Memory:** MemAvailable ≥ 1809 MiB; swap 0.
- **Stop:** gated stop attempt 1 PASS, manifest `stopped_at_utc` 17:36:47.
- **Outcome:** no arming, take-off, flight command, or episode.

## Passive SITL clock validation (B1/B2, uncommitted): PASS, shutdown incomplete then self-completed — 2026-10-23 19:15–19:25 UTC

- **Code:** `8a73737` plus the uncommitted B1/B2 changes. Run `restart_20261023_clockcheck1`, split rendering. Orchestration and analysis: `/home/hp/uav_ros_experiments/clock_validation_20261023/` (`run_clockcheck.sh`, `clock_recorder.py` (subscriptions only; raw RGB image subscription 200–290 s), `analyze_clock.py`, `analysis_restart_20261023_clockcheck1.json`). **Passive:** no `PX4UAVEnv`, no `/fmu/in` publisher, no arming.
- **Gates:** all PASS (`/fmu/in` 0/0/0 with the plain query; `render-check` exit 0). **Stability 6/6 PASS**, camera 0.388–0.408×.
- **Camera clock (`camera_info`):** 4203 frames, about 30.3 per simulated second, 0 duplicates, 0 backward, maximum gap 36 ms simulated and 0.28 s wall. Every RGB image stamp appeared in `camera_info`. Replaying the stream through the env's camera-clock logic: 0 invalidations, valid from 3.0 s, maximum receipt age 0.11 s.
- **PX4 HRT (`TimesyncStatus.timestamp + estimated_offset`):** 248 samples, 12 duplicates, 0 backward, 108 post-reset burst steps, maximum wall gap 3.05 s. Camera vs HRT drift −0.02 s over 137 s; 25 s rates differed by ≤ 0.0043.
- **Observer load:** subscribing to raw images lowered speed about 6% (0.401× → 0.376×).
- **Resources and state:** MemAvailable ≥ 1664 MiB; swap ≈ 0.4 MiB. Vehicle disarmed, Hold, landed throughout; `/fmu/in` publishers 0.
- **Stop:** attempt 1 passed the telemetry gate but was incomplete: the gz server outlasted the bounded SIGINT/SIGTERM (no SIGKILL). Attempts 2–5 refused (PX4 had exited, no telemetry). The server exited on its own 51–61 s after the stop began. The manifest has no `stopped_at_utc` and was left unmodified.
- **Outcome:** no arming, take-off, flight command, or episode.

## Passive B3 guard validation attempt 1: readiness gate FAIL (`/fmu/in` query `unknown`) — 2026-10-24 08:32–08:35 UTC

- **Code:** `8a73737` plus the uncommitted B1/B2/B3 changes. Run `restart_20261024_guardcheck1`, split rendering. Orchestration: `/home/hp/uav_ros_experiments/guard_validation_20261024/run_guardcheck.sh`.
- **Gates:** `verify_isolated_topics` PASS (attempt 2), `verify_world_target` PASS, `render-check` exit 0. `/fmu/in/offboard_control_mode` returned `publishers=unknown` (the other two 0), so the gate **failed closed**; the observer and stability windows did not run.
- **Diagnosis (read-only):** no ROS 2 daemon existed when the gate ran, so each plain `ros2 topic info` used a fresh 0.5 s discovery (`ros2cli/node/direct.py` `DEFAULT_TIMEOUT = 0.5`). A topic not yet discovered prints "Unknown topic" and no count. This is the likely cause but was not directly confirmed: the gate discarded stderr. The 20 s timeout was not reached (all three queries took about 3.9 s).
- **Stop:** attempt 1 incomplete (gz server), attempts 2–5 refused; the server exited on its own 35–45 s after the stop began. No SIGKILL; no `stopped_at_utc`.
- **Outcome:** no guard observations; no arming, take-off, `/fmu/in` publishing, or episode.

## Passive B3 guard validation attempt 2: PASS — 2026-10-24 08:44–08:53 UTC

- **Code:** as attempt 1. Run `restart_20261024_guardcheck2`. Orchestration: `guard_validation_20261024/guardcheck2/run_guardcheck2.sh` (a copy; only the `/fmu/in` gate changed): `timeout 30 ros2 topic info --no-daemon --spin-time 5 -v`, exit 0, `Publisher count: 0`, `Subscription count` ≥ 1, every attempt logged, at most 3 attempts.
- **`/fmu/in`:** all three topics 0 publishers on attempt 1 (about 6 s each), 2 subscribers each: `passive_telemetry_recorder` (our recorder) and `_CREATED_BY_BARE_DDS_APP_` (PX4's XRCE-DDS reader). The recorder alone would have satisfied "≥ 1 subscriber", so the gate was tightened for attempt 3.
- **Stability 6/6 PASS**, camera 0.380–0.403×.
- **Observer** (`guard_validation_20261024/guard_observer.py`; subscriptions only; the real env callbacks and the real `RuntimeGuard`, plus four shadow guards with synthetic input dropouts from 200 s): full 300 s, no abort. Analysis: `guardcheck2/guard_analysis.json`.
  - Nominal guard: first OK at 27 s (warm-up), then **0 non-OK verdicts in 274 checks**. PX4 HRT vs camera term: maximum difference 0.0067; raw 25 s rates 0.380–0.402× for both. Every verdict was consistent with the 0.35× floor; minimum speed 0.378×.
  - HRT: 198 samples, 8 duplicates, 0 backward, 82 burst steps, maximum wall gap 2.91 s. Position receipts: 5803, maximum gap 0.15 s.
  - Shadow guards (1 s tick resolution): link dropped → `px4_telemetry_stale` at +1 s; camera dropped → `camera_clock_stale` at +2 s; HRT dropped, or re-delivered with identical values → `px4_clock_stale` at +3 s. None recovered.
  - `px4_clock_stalled` never fired: below about 0.4× the 5 s HRT staleness limit fires first. Offline tests cover the stall path.
- **Resources and state:** MemAvailable ≥ 1719 MiB; swap 0.5 → 9.2 MiB; gz server RSS 578 → 888 MiB. `/fmu/in` publishers 0 every second; vehicle disarmed and landed throughout.
- **Stop:** same pattern as above; the server exited 47–58 s after the stop began. No SIGKILL; no `stopped_at_utc`.
- **Outcome:** passive only; no arming, take-off, `/fmu/in` publishing, or episode.

## Passive B3 guard validation attempt 3 (strict `/fmu/in` identity gate): PASS — 2026-10-24 09:14–09:23 UTC

- **Code:** as attempt 1. Run `restart_20261024_guardcheck3`. Orchestration: `guard_validation_20261024/guardcheck3/run_guardcheck3.sh` (sha256 `fe3bc2eb…337c`) with `fmu_in_gate_parse.py`, 30 offline gate tests. A topic passes only with exit 0, one well-formed `Publisher count: 0` and no PUBLISHER endpoint, one well-formed `Subscription count` matching its endpoint records, the expected `px4_msgs` type, and ≥ 1 subscription with node name and namespace `_CREATED_BY_BARE_DDS_APP_` and type hash `INVALID`. The three PX4-side readers must share one DDS participant.
- **`/fmu/in`: PASS.** All three topics on attempt 1 (6.1–6.3 s each): 0 publishers, 2 subscribers; PX4-side reader identified, recorder (`RIHS01_` hash) not counted. Shared participant `01.0f.c6.61.21.54.ca.08.00.00.00.00`, distinct from the recorder's. The endpoint order was reversed relative to attempt 2; the parser does not depend on order.
- **Other gates:** topics PASS (attempt 2), world target PASS, `render-check` exit 0 at start and end. **Stability 6/6 PASS**, camera 0.374–0.411×.
- **Observer:** full 300 s, no abort. Analysis: `guardcheck3/guard_analysis.json`. First OK at 29 s, then **0 non-OK verdicts in 272 checks**; minimum speed 0.365× (all consistent with the 0.35× floor). HRT: 208 samples, 10 duplicates, 0 backward, 89 burst steps, maximum wall gap 3.05 s. Position receipts: maximum gap 0.26 s. Shadow guards: link and camera dropouts failed at +2 s, HRT dropout and repeated HRT at +4 s (`px4_clock_stale`). `px4_clock_stalled` again not triggered live.
- **Resources and state:** MemAvailable ≥ 1716 MiB. Swap grew 26.6 → 82.9 MiB while the observer ran (56 MiB, below its 64 MiB abort limit) and stayed at 84 MiB after the stop; gz server RSS 526 → 882 MiB. `/fmu/in` publishers 0 every second; vehicle disarmed and landed throughout.
- **Stop:** same pattern; the server exited 46–56 s after the stop began. No SIGKILL; no `stopped_at_utc`.
- **Outcome and limits:** passive only; no arming, take-off, `/fmu/in` publishing, or episode. This is evidence about discovery and the guard, **not proof of flight readiness**. Open observations, not demonstrated failures: swap growth, the 35–61 s gz-server exit since clockcheck1 (split2 stopped normally), and gz server memory growth (about 50 MiB/min). The live stall reason remains untriggered (offline coverage only). The B1 step clock, B2 fly-to timeout and B3 in-episode behaviour have not flown.

## Diagnostics between guardcheck3 and the host-memory runs (summary) — 2026-10-24

- **Code:** B1/B2/B3 committed as `837a876`; `close()` ground-confirmation fix as `5df1273`. Orchestration and raw data for each run are in `/home/hp/uav_ros_experiments/<dir>/` (named below).
- **`restart_20261024_action30_1`** (`action_test_20261024/`): one 30-step SITL attempt → `guard_refuse` at speed 0.3496× (floor 0.35×). No take-off.
- **`restart_20261024_runnerload1`** (`runner_load_20261024/`): passive; runner overhead not established.
- **`restart_20261024_earlystart1`** (`early_start_20261024/`): passive; inconclusive, a dip at 410–416 s.
- **`restart_20261024_perf1`** (`perf_measure_20261024/`): readiness FAIL on a gap; the GPU was about 37% busy, not saturated.
- **`restart_20261024_guiab1`** (`runner_gui_ab_20261024/`): readiness FAIL (perception gap 2.05 s), so the GUI on/off comparison was not run.

## Passive host-memory diagnostic 1: readiness FAIL 0/6 (host paging) — 2026-10-24 21:52–21:58 UTC

- **Run:** `restart_20261024_hostmem1`, code `5df1273`, split rendering. Orchestration: `/home/hp/uav_ros_experiments/hostmem_20261024/run_hostmem.sh` (sha256 `0164e6cb…6e53e`): the established gates and windows, plus a 1 s Windows typeperf sampler (Available MBytes, Pages/sec, Page Faults/sec, Paging File % Usage, vmmemWSL and Memory Compression working sets). Analysis: `analyze_hostmem.py` → `hostmem_analysis.json`; 9 offline tests passed. Passive: no runner, no `/fmu/in` publisher, no arming.
- **Readiness:** gates PASS; **windows 0/6** at 0.252–0.310×. Gaps up to 2.71 s (status), 4.69 s (land) and 2.72 s (perception).
- **Windows host:** 7.86 GB RAM, with WSL capped at 5 GB. Already 14,699 pages/s before Gazebo started. During the run: Pages/sec median 1,725 and 90th percentile 17,065; the page file grew 16.9 → 23.6%; Memory Compression used up to 60–88% of a CPU; vmmemWSL working set 0.93–1.42 GB; GPU about 808 MHz / 6.9 W (starved of work).
- **Effects:** the worst speed (0.243–0.257×) and perception gaps of 2.7 s fell in the heaviest paging bursts. PX4 logged `Accel #0 fail: TIMEOUT!`, and preflight was false for about 9 s (at 199–207 s).
- **Resources and state:** WSL MemAvailable ≥ 1995 MiB, swap 13 MiB; vehicle disarmed and landed throughout.
- **Stop:** attempt 1 confirmed landed and disarmed, then was incomplete (gz server, no SIGKILL); attempts 2–5 refused; the server exited on its own.

## Passive host-memory diagnostic 2 (Chrome closed): readiness PASS 6/6, 240 s observation clean — 2026-10-25 09:05–09:16 UTC

- **Run:** `restart_20261025_hostmem2`, the same script (sha256 unchanged) and code. The only deliberate change: the user closed Chrome and its tabs on Windows beforehand (30 s baseline: about 1.5 pages/s, page file 5.5%, 1.9 GB available). Analysis: `hostmem2_analysis.json`.
- **Readiness:** gates PASS; **windows 6/6** at 0.435–0.507×; the 240 s observation had 0 guard failures and ran at 0.489× (60 s chunks 0.47–0.50×). Worst gaps 1.58 s (status), 2.74 s (land), 1.05 s (perception). No PX4 sensor timeouts.
- **Windows host:** Pages/sec median 29 and 90th percentile 2,193; page file flat at about 5.3%; GPU about 1,580 MHz / 21 W. A 15 s burst at 09:13 (up to 18.7k pages/s, Defender at 65–97% of a CPU) dipped speed only to 0.43–0.49×. The lowest speed, 0.428× at 09:09:31, had no paging but a GPU at about 900 MHz (unexplained).
- **WSL memory drift:** MemAvailable fell about 60–70 MiB/min (2,054 → 1,378 MiB), mainly because the gz server grew 577 → 1,125 MiB; it recovered after the stop. The runtime guard's start threshold is 1.2 GiB.
- **Conclusion:** host memory pressure on Windows is the strongest explanation for the slow runs (one run in each condition, so not a controlled proof).
- **Stop:** same pattern as hostmem1; vehicle disarmed and landed throughout.

## First supervised live flight (baseline controller): SUCCESS — 2026-10-25 09:33–09:40 UTC

- **Approval:** the user explicitly approved one supervised baseline flight on 2026-10-25. Code `5df1273` (clean tree), split rendering, QGroundControl running (pid 42280), Chrome closed (host baseline: median 0 pages/s, page file 4.7%).
- **Run:** `restart_20261025_baseline1`. Orchestration: `/home/hp/uav_ros_experiments/baseline_flight_20261025/run_flight.sh` (sha256 `da0c3ee6…b805`), a copy of `run_hostmem.sh`. After 6/6 windows it replaces the 240 s observation with:
  1. stop the passive observer by exact PID;
  2. run the fail-closed `preflight_state.py` check on its last state (no abort record, fresh, disarmed, no failsafe, preflight ok, landed, `/fmu/in` 0);
  3. run `run_target_reaching_episode.py --i-confirm-supervised-flight` once, with no retry.

  Tests: `test_flight_block.py` 14/14; full repo suite 405 passed; an offline stand-in episode returned `success`.
- **Readiness:** gates PASS (topics on attempt 2, world target, strict `/fmu/in`, render-check); **windows 6/6** at 0.497–0.512×. Pre-flight state: disarmed, landed, preflight ok, `/fmu/in` 0, MemAvailable 1679 MiB.
- **Episode (`logs/episode.jsonl`):**
  - Guard preflight OK at 0.493×, 1653 MiB.
  - `reset()` (arm, take-off, fly to start) took 18.8 s; the vehicle held 1.62 m at the start, 4.34 m from the target.
  - The target was first detected at step 2 and seen in 51 of 64 steps; the vehicle reached 0.76 m at step 62.
  - Step 63: **`success`**, no collision, minimum LiDAR 2.01 m, then the return to start (0.04, −0.02) inside that step.
  - 64 steps in 29.2 s wall.
  - `close()` (NAV_LAND, landed-and-disarmed confirmation) took 11.9 s.
  - Runner exit 0, `EPISODE RESULT: success`.
- **Runtime guard during the flight:** never aborted. Its 6 non-OK samples were warm-up (`px4_clock_insufficient_history`) before its preflight; minimum speed 0.478×, minimum MemAvailable 1569 MiB.
- **Stop:** attempt 1 independently confirmed fresh landed, at-rest and disarmed telemetry, then was incomplete (PX4/gz outlasted the bounded stop, no SIGKILL); attempts 2–5 refused (no telemetry); the stack exited on its own ("stack stopped"). No `stopped_at_utc`. QGroundControl left running.
- **Outcome:** the first complete live episode in this setup with the baseline controller. No PPO. Readiness depends on the Windows host having free memory, and WSL MemAvailable drift remains the tightest margin for longer sessions.

## First supervised live PPO flight (E1R 300k seed 2, numpy, safety filter): SUCCESS — 2026-10-25 10:05–10:12 UTC

- **Approval:** the user explicitly asked for a live PPO demo using the numpy policy on 2026-10-25. Code `1fac16b` (clean tree): runner `--policy` path (`18ebe00`) and numpy export (`1fac16b`). Split rendering, QGroundControl running, Chrome closed (host baseline: median 0 pages/s, page file 4.6%, 1.6 GB available).
- **Policy:** `runs/rl/exp20261022_randtarget300k/E1R_s2/final_policy_numpy.npz` (sha256 `fbcacf93…0511`, made read-only), exported from `final_model.zip` (sha256 `06a0b636…278a`). Parity with SB3 ≤ 6e-7.
  - **Why seed 2:** 50/50 stand-in successes on the live world through the exact runner path (numpy and filter), median 51 steps. Seeds 1 and 3 were also 50/50 but took about 146–148 steps. Seed 2 with the filter also scored best in the safety-filter and occlusion evaluations.
- **Run:** `restart_20261025_ppo1`. Orchestration: `/home/hp/uav_ros_experiments/ppo_flight_20261025/run_ppo_flight.sh`, a copy of the flown baseline script. Differences: `--policy` with that file, and its sha256 checked before anything starts. Tests: `test_flight_block.py` 15/15; repo suite 441 passed; an offline dry run of the same command gave `success` at about 66 MiB peak.
- **Readiness:** gates PASS; **windows 6/6** at 0.453–0.518×. Pre-flight state: disarmed, landed, preflight ok, `/fmu/in` 0, 1692 MiB.
- **Episode (`logs/episode.jsonl`):**
  - Policy loaded in numpy format, **torch not loaded**, 1729 MiB available.
  - Guard preflight OK at 0.498×, 1638 MiB.
  - `reset()` took 19.9 s; the start was 4.31 m from the target.
  - The target was first detected at step 2 and seen in 31 of 38 steps.
  - Step 37: **`success`**, no collision, minimum LiDAR 2.18 m, then the return to start (0.04, −0.01).
  - 38 steps in 22.0 s wall; `close()` (land, disarm confirmed) took 12.4 s.
- **Safety filter:** intervened on 10 of 38 steps, all `slow` (speed caps); mean speed 0.92 commanded vs 0.82 applied. LiDAR age at decision: median 0.054 s, maximum 0.181 s (limit 0.5 s).
- **Runtime guard:** never aborted. Its non-OK samples were warm-up only (5 `px4_clock_insufficient_history`, 1 `px4_clock_missing`) before its preflight; minimum speed 0.483×, minimum MemAvailable 1519 MiB.
- **Stop:** attempt 1 confirmed fresh landed, at-rest and disarmed telemetry, then was incomplete (PX4/gz, no SIGKILL); attempts 2–5 refused (no telemetry); "stack stopped". QGroundControl left running.
- **Outcome:** first live PPO episode: 38 steps against the baseline's 64 in the same world. One episode at one fixed start and target is a demonstration, not a reliability measure; the policy has been validated statistically only in the stand-in.

## Stage A: PPO repeatability series, 3 more episodes: 3/3 SUCCESS — 2026-10-25 10:20–10:59 UTC

- **Approval and setup:** the user approved several supervised PPO episodes. Stage A repeats the ppo1 setup exactly: code `1fac16b` (clean tree), the same pinned policy (E1R 300k seed 2, numpy, sha256 `fbcacf93…0511`), the same start and target, and a fresh stack per episode via the unchanged `run_ppo_flight.sh` (sha256 `03297200…7473`).
- **Orchestration:** `/home/hp/uav_ros_experiments/ppo_flight_20261025/`.
  - `run_series.sh`: before each episode it waits for the previous run's samplers, then checks for no stack, QGroundControl running, a clean tree, the script hash, and a 20 s host baseline (median below 500 pages/s, page file below 10%).
  - `check_episode.py`: the stop rules. It stops on a collision, guard refusal or abort, a result other than success, an unexpected filter reason (anything but `slow`/`stop`), torch loaded, no confirmed landing, or the stack not stopped. A readiness failure stops only if it happens twice in a row. Validated on ppo1 and on 10 synthetic failure cases.
- **Series run:**
  - `restart_20261025_ppo2` and `ppo3` flew.
  - Before `ppo4` the host check found about 2,100 pages/s and the series **stopped without starting anything**.
  - Paging settled (about 1 page/s at 10:49), and `ppo4` was run separately with the same checks after the user approved it.

| Run | Windows | Steps | Episode (wall) | Filter slow / stop | Minimum LiDAR | Guard minimum speed / MemAvailable |
|---|---|---|---|---|---|---|
| ppo1 (10:05) | 0.453–0.518× | 38 | 22.0 s | 10 / 0 | 2.18 m | 0.483× / 1519 MiB |
| ppo2 | 0.483–0.533× | 39 | 22.4 s | 10 / 0 | 2.20 m | 0.500× / 1588 MiB |
| ppo3 | 0.483–0.517× | 41 | 23.0 s | 11 / 0 | 2.14 m | 0.502× / 1602 MiB |
| ppo4 | 0.473–0.532× | 38 | 22.6 s | 9 / 0 | 2.19 m | 0.483× / 1446 MiB |

- **All four PPO episodes:** success. Target first detected at step 2–3; maximum LiDAR age 0.10–0.18 s; `reset()` 18.5–19.9 s; `close()` 12.3–13.4 s with landing and disarm confirmed; no guard abort. Every gated stop confirmed landed and disarmed on attempt 1, then followed the known incomplete-then-self-exit pattern (no SIGKILL).
- **Windows paging by phase** (`phase_paging.py`, 1 s host-memory sampler):
  - Flight medians 1–15 pages/s.
  - Startup has short file-load spikes (up to 168k/s) with medians of 31–74.
  - **Shutdown** is the main source: medians 35–2,368 and maxima up to 84k. In ppo4 the page file went 4.8 → 7.5%, with Defender at 29% and Memory Compression at 18% of a CPU, while the WSL VM's resident memory fell from about 1.7 to 1.1 GB.
  - After the stop it settled within 30–60 s in ppo1, ppo2 and ppo4. In ppo3 about 2,200 pages/s lasted about 10 minutes with no matching CPU activity; the cause was not identified.
  - ppo4's Windows samplers were stopped early on request (SIGINT to the exact recorded PIDs at 11:04, about 5 minutes after END). Their raw output is kept; the incomplete last row was excluded from the analysis.
- **Outcome:** 4/4 live PPO successes from the same start and target, with very consistent behaviour; 38–41 steps against the baseline's 64. This shows repeatability, not generalisation (start and target fixed).

## Stage B: start-heading series (3 PPO + 1 baseline): 4/4 SUCCESS — 2026-10-25 11:16–12:01 UTC

- **Approval and code:** the user approved the four flights after the stand-in predictions. Code `4da38f5` (clean tree): `--start-yaw-offset-deg`, honoured identically by both envs. Policy as in Stage A (E1R 300k seed 2, numpy, sha256-pinned).
- **Orchestration:** `/home/hp/uav_ros_experiments/stageb_prep_20261025/`. `run_stageb_flight.sh` (sha256 `d44c21c0…f987`) is a copy of the flown PPO script; it allows offsets 90/180/−90 and modes ppo/baseline, validated before anything starts. `run_series_b.sh` waits for the previous samplers, then at least 180 s after the previous END, then runs the host check. `check_episode.py` adds a reset-options check and requires the start heading reached at `reset_done` within 15° of the request. Tests: 23 offline; dry runs of both modes; full repo suite 469.
- **Stand-in predictions** (`predict_headings.py`, `predictions.json`; 50 episodes per cell, all successes): PPO 47–133 steps vs baseline 72–283. Both search by turning right; +90° (target 45° left) was predicted to be the hardest case.

| Run | Mode | Offset | Start heading reached | Windows | Steps | Episode (wall) | First detection | Filter slow / stop | Minimum LiDAR | Stand-in prediction |
|---|---|---|---|---|---|---|---|---|---|---|
| stageb1 | PPO | +90° | −178.8° | 0.475–0.531× | 36 | 21.2 s | at reset (41° left) | 9 / 0 | 2.01 m | 133 |
| stageb2 | PPO | 180° | −89.3° | 0.514–0.541× | 86 | 34.0 s | step 49 | 10 / 0 | 2.09 m | 114 |
| stageb3 | PPO | −90° | −1.3° | 0.521–0.533× | 62 | 29.0 s | step 26 | 9 / 0 | 2.16 m | 83 |
| stageb4 | baseline | +90° | −178.6° | 0.486–0.507× | 205 | 67.5 s | step 151 | – | 1.79 m | 280 |

- **Start-heading turns:** every 90–180° turn reached its heading within 1.4°; `reset()` took 25–29 s (18.5–20 s without a turn). Guard minimum speed 0.481–0.512×, minimum MemAvailable 1562–1636 MiB, no abort. `close()` took 12.3–13.4 s with landing and disarm confirmed; gated stops as in Stage A (no SIGKILL).
- **Search:** in b2 and b3 (target about 135° off the nose) PPO turned right at full rate (mean turn +1.0) until detection, then approached directly.
- **Field-of-view mismatch:** in b1 the live camera already detected the target 41° to the left at reset, so PPO turned left and flew straight in. The stand-in limits visibility to about 41° (34.5° half-FOV plus the sphere's angular radius), which is why it predicted a full right-turn search. The live edge is marginal: the baseline at the same heading (b4, −178.6°) had no detection at reset and searched right through about 315°.
- **Outcome:** 7/7 live PPO successes in total (4 from the nominal start, 3 turned). Live PPO took fewer steps than the stand-in predicted in every case, and fewer than the live baseline (b1 36 vs b4 205, with the b1 FOV caveat). Still one fixed start point and one fixed target.

## Stage C passive readiness runs (world variants, no flight): 3/3 PASS — 2026-10-25 12:57–13:22 UTC

- **Approval and code:** the user approved three passive runs. Code `36f662c` (branch `stage-c-target-variants`, clean tree): world variants, the `SAR_WORLD_VARIANT` selector, `--world-variant`, and the world-consistency gate.
- **Orchestration:** `/home/hp/uav_ros_experiments/stagec_prep_20261025/`. `run_series_c_passive.sh` runs `run_stagec.sh <run> <variant> passive` (sha256 `4b5287cc…800e`): all readiness gates, six windows, observer stop and pre-flight state check, then **no runner**. `check_episode.py` requires both world gates to have passed for that variant, no episode log, and the stack stopped.
- **Every run, from its logs:**
  - PX4 logged `Starting gazebo with world: …/sim/worlds/variants/<variant>/sar_walls.sdf`.
  - `gz_env.sh` had `PX4_GZ_WORLDS=…/variants/<variant>`.
  - The manifest recorded the variant, file and sha256.
  - The world-target gate passed against the variant file (the live sphere at the moved position).
  - The consistency gate passed.

| Run | Target NED | World sha256 | Windows | Pre-flight state |
|---|---|---|---|---|
| stagec_c2_sw_passive | (−4.5, −2.5) | `44fc2863…` | 6/6, 0.456–0.537× | disarmed, landed, `/fmu/in` 0, 1716 MiB |
| stagec_c5_w_passive | (−1.0, −4.5) | `5991dfdd…` | 6/6, 0.506–0.519× | disarmed, landed, `/fmu/in` 0, 1680 MiB |
| stagec_c1_ne_passive | (2.5, 2.5) | `a81d36a4…` | 6/6, 0.506–0.512× | disarmed, landed, `/fmu/in` 0, 1678 MiB |

- **Host:** before each run median 0–9 pages/s, page file 4.7 %. Stops: stack stopped every time (no SIGKILL). No flight, arming or `/fmu/in` publishing; QGroundControl left running.
- **Also fixed:** the flight-block test harness's observer stand-in (a background `sleep`, which ignores SIGINT in non-interactive bash) made tests hang at random. It now installs its own SIGINT handler as rclpy does. The live observer always stopped within about 1 s (rc 1, rclpy shutdown).

## Stage C flights (target moved, world variants): 4/4 SUCCESS — 2026-10-25 13:41–14:18 UTC

- **Approval and code:** the user approved four flights after the passive runs. Code `8a6fd34` (master, clean tree), default start heading, policy as before (E1R 300k seed 2, numpy, sha256-pinned, LiDAR filter).
- **Orchestration** (`/home/hp/uav_ros_experiments/stagec_prep_20261025/`):
  - Flights 1–2 ran through `run_series_c_flights.sh` and `run_stagec.sh` (full diagnostic samplers).
  - At the user's request (no waiting except when necessary; no recording unless chasing an issue), flights 3–4 ran through `run_series_c_fast.sh` and `run_stagec_lean.sh` (sha256 `dbcb8acd…bc5c`).
    - The lean script is the same script with every diagnostic recorder removed; the gates, windows, WSL memory watchdog, observer, pre-flight check, runner and gated stop are unchanged (28 tests, line-subset test).
    - The fast driver has no fixed gap and no sampler wait; it repeats the host check only while Windows is paging.
  - **Handover:** SIGINT did not stop the old driver, because bash defers SIGINT while its foreground `sleep` runs. The fail-closed handover therefore started nothing. The old driver was then stopped with SIGTERM to its exact PID while it was only waiting (no stack; flight 3 not started). Flight 2's leftover samplers were stopped by exact PID (10/10 statuses).

| Run | Mode | Variant (target NED) | Windows | Target visible at reset | First detection | Steps | Episode (wall) | Filter slow / stop | Minimum LiDAR | Stand-in prediction |
|---|---|---|---|---|---|---|---|---|---|---|
| stagec_c2_sw_ppo | PPO | c2_sw (−4.5, −2.5) | 0.480–0.511× | no | step 21 | 61 | 31.4 s | 0 / 0 | 4.43 m | 81 |
| stagec_c5_w_ppo | PPO | c5_w (−1.0, −4.5) | 0.480–0.510× | no | step 36 | 66 | 34.8 s | 0 / 0 | 4.47 m | 92 |
| stagec_c1_ne_ppo | PPO | c1_ne (2.5, 2.5) | 0.462–0.500× | yes (FOV edge) | step 0 | 29 | 20.1 s | 2 / 0 | 2.41 m | 148 |
| stagec_c2_sw_baseline | baseline | c2_sw (−4.5, −2.5) | 0.460–0.500× | no | step 34 | 101 | 43.2 s | – | 4.45 m | 139 |

- **Checks:** every `check_episode.py` verdict was ok: both world gates passed for the variant, and the runner's `world` record matched it. Guard minimum speed 0.444–0.490×, minimum MemAvailable 1554–1705 MiB, no abort; `close()` 12.4–14.6 s with landing and disarm confirmed; stack stopped every time (no SIGKILL).
- **Timing:** start-to-start fell from about 12.5 min (flights 1–2) to about 7.2 min (flights 3–4).
- **Outcome:** 11/11 live PPO successes in total, now at three target positions besides the original, including two where the target started well out of view (c2_sw, c5_w: search and then approach). PPO took 61 steps vs the baseline's 101 at c2_sw. c1_ne repeats the stageb1 field-of-view effect: the live camera saw the target 45° off the nose at reset.

## Demo kit (tools/demo) live test: 2/2 SUCCESS; remaining test flights cancelled by the user — 2026-10-25 14:36–15:00 UTC

- **Kit:** `80ed7e5` / `f40b56f` on branch `demo-kit`.
  - `tools/demo/run_demo.sh` is a line-for-line port of the flown `run_stagec_lean.sh`, with repo paths, argument checks and the optional start offset.
  - `run_demo_series.sh` adds stop rules via `check_episode.py`; the policy is now in `models/e1r_300k_s2/`.
- **Flights:**
  - `restart_kittest_1_ppo_default_0`: success, 38 steps.
  - `restart_kittest2_1_ppo_default_180`: success, 90 steps.
  - Both landed and disarmed, gated stop with no SIGKILL.
- **Host check removed:** the first series stopped before flight 2. Its pre-run Windows host check reported about 2,200 pages/s, but this was McAfee (`mc-sustainability`, about 9 MB/s) reading files, not memory pressure (Pages Output/sec 0, 2 GB free, page file flat). At the user's request the host check and every inter-run wait were removed (`f40b56f`); simulation speed stays gated by the six stability windows. The waiting driver was stopped with SIGTERM to its exact PID (no stack running).
- **Cancelled flights:** the user then asked for no more flights. The second series driver was stopped by exact PID during its first flight; that flight completed and landed normally. The moved-target and baseline test flights were cancelled (both modes flew earlier the same day through the identical procedure).
- **Report:** `docs/PROJECT_STATE_REPORT.md`, with tag `demo-2026-10-25`.

## 3D step 1: generated rough-terrain worlds, passive laptop load tests (no flight) — 2026-10-25 15:50–16:11 UTC

- **Code:** branch `world-generator`: `23b3bd8` (generator, gen_l1/l2/l3_s1), then `a4a9e97` (terrain mesh visual only).
- **First attempt** (`gentest_1_passive_gen_l3_s1_0`, at `23b3bd8`): the gz server **segfaulted while loading** the world, building the terrain mesh collision in DART's ODE collision detector (`dart::collision::detail::OdeMesh::fillArrays`).
  - PX4 waited for a world that never came; the launcher stopped the start after 30 s without a clock.
  - The gated stop correctly refused (no telemetry; no vehicle had been created). With the user's approval the two leftover owned groups (XRCE agent, PX4) were stopped with the launcher's own `_signal_owned_groups` (verified guards, SIGINT/SIGTERM, no SIGKILL).
- **Fix** (`a4a9e97`): terrain mesh **visual only** (cameras and GPU LiDAR still see it). A thin collision box under each pad, top at 0, plus a safety plane below the lowest terrain. A test forbids mesh collision shapes. Real terrain collision is part of step 2.
- **Re-test** (`run_demo_series.sh gentest2 "passive gen_l3_s1 0" "passive gen_l1_s1 0"`): **2/2 passive PASS**. Both loaded; world-target and consistency gates passed; render-check ok.

| World | Obstacles (tall) | Stability windows | Min MemAvailable | gz server / GUI RSS max | Pre-flight MemAvailable |
|---|---|---|---|---|---|
| gen_l3_s1 (hardest) | 72 (52) | 6/6, 0.455–0.479× | 1250 MiB | 1081 / 1068 MiB | 1355 MiB |
| gen_l1_s1 (easiest) | 23 (17) | 6/6, 0.415–0.480× | 1608 MiB | 930 / 909 MiB | 1719 MiB |
| default world (stagec_c5_w_passive, reference) | 4 walls | 6/6, 0.506–0.521× | 1699 MiB | 849 / 845 MiB | 1680 MiB |

- **Assessment:**
  - **Speed:** about 10 % slower than the default world, still well above the 0.35× floor.
  - **Memory:** the binding constraint. In the hardest world MemAvailable at the pre-flight check was 1355 MiB, about 126 MiB above the runtime guard's 1.2 GiB take-off threshold. The gz GUI (llvmpipe, CPU-rendered) and the server each grew about 220 MiB.
  - **Errors:** the only PX4 errors were two IMU timestamp lines during shutdown.
