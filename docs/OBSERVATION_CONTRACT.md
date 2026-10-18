# Policy observation contract `sar_obs_v1`

**Source of truth:** `sar_observation.py` (`FIELDS`, `ObservationBuilder`). This table is generated from it.
**Dimension:** 65 float32 values. **Gymnasium space:** `Box(sar_observation.LOW, sar_observation.HIGH)`.

## Fields

| index | name | low | high | meaning |
|---|---|---|---|---|
| 0-48 | lidar | 0 | 1 | minimum range per sector / 30 m, 49 equal sectors over +-135 deg |
| 49 | prev_speed | 0 | 1 | previous action[0] (forward speed command, 1 = 1 m/s) |
| 50 | prev_turn | -1 | 1 | previous action[1] (NED yaw-rate command, 1 = +0.5 rad/s = clockwise/right) |
| 51 | vel_forward | -1 | 1 | EKF horizontal velocity along body x / 1.5 m/s, clipped |
| 52 | vel_left | -1 | 1 | EKF horizontal velocity along body y (+left) / 1.5 m/s, clipped |
| 53 | yaw_rate | -1 | 1 | body yaw rate (+left/CCW) / 1 rad/s, clipped |
| 54 | height | 0 | 1 | height above EKF origin (-z NED) / 3 m, clipped |
| 55 | det_now | 0 | 1 | 1 if a fresh perception message reports the target, else 0 |
| 56 | det_bearing_sin | -1 | 1 | sin(measured bearing), 0 when det_now = 0 |
| 57 | det_bearing_cos | -1 | 1 | cos(measured bearing), 0 when det_now = 0 |
| 58 | det_range_valid | 0 | 1 | 1 if the fresh detection includes a valid depth range |
| 59 | det_range | 0 | 1 | measured horizontal range / 10 m, clipped; 0 when invalid |
| 60 | est_valid | 0 | 1 | 1 if a dead-reckoned target estimate exists and has not expired |
| 61 | est_bearing_sin | -1 | 1 | sin(estimate bearing in current body frame), 0 when invalid |
| 62 | est_bearing_cos | -1 | 1 | cos(estimate bearing in current body frame), 0 when invalid |
| 63 | est_range | 0 | 1 | estimate horizontal range / 10 m, clipped; 0 when invalid |
| 64 | est_age | 0 | 1 | time since last range-valid detection / expiry; 1 when none or expired |

## Action (unchanged interface)

The action is `Box([0, -1], [1, 1])`.

| Element | Range | Meaning |
|---|---|---|
| `[0]` speed | 0 to 1 | body-forward speed, 1 = 1 m/s; projected to NED with the EKF heading and suppressed unless `heading_good_for_control` |
| `[1]` turn | −1 to 1 | NED yaw rate, 1 = +0.5 rad/s; **positive is clockwise, a right turn**. The observation's bearings are +left, so turning toward a target at bearing b needs turn ∝ −b. |

## Inputs (onboard only)

| Input | Source | Freshness gate |
|---|---|---|
| LiDAR sectors | `/uav/lidar_sectors` (49 minimum ranges in m), from `observation_publisher.py` (LiDAR scan only) | a new message after the action, at most 1.0 s old |
| EKF position, velocity, heading | `/fmu/out/vehicle_local_position_v1` | at most 1.0 s old and `xy_valid`, `z_valid`, `v_xy_valid`; otherwise the step fails into the safe state |
| Yaw rate | `/fmu/out/vehicle_odometry` `angular_velocity[2]` (FRD), negated to FLU | at most 1.0 s old; otherwise the step fails into the safe state |
| Target perception | `/uav/target_perception` (`target_perception` wire format v1), from camera RGB and depth only | at most 1.5 s host-receipt age (`PX4UAVEnv.PERCEPTION_MAX_AGE`); older is treated as "not detected" |
| Simulation time | PX4 timesync reconstruction (`timestamp + estimated_offset`) | required |
| Previous action | the environment | — |

## Validity rules

- **`det_*`** describe the latest perception message only if it is fresh and reports a detection. Otherwise every `det_*` field is 0.
  - `det_range_valid` = 0 means bearing-only: the target was seen, but no valid depth landed on it, for example invalid or out-of-range depth or no synchronised depth frame.
  - The measured bearing is not motion-compensated; it is the bearing at image time.
  - Messages with an older or equal image stamp are discarded on receipt, so a repeated frame never looks new.
- **`est_*`** come from a dead-reckoned estimate.
  - Each new range-valid detection sets `estimate = EKF position + range · (cos(heading − bearing), sin(heading − bearing))` in local NED.
  - Every step re-projects the estimate into the current body frame.
  - It expires 8 s of simulation time after the last range-valid detection. After expiry, `est_valid` = 0, the bearing and range fields are 0, and `est_age` = 1.
  - Known limitation: the estimate uses the EKF pose at the step that consumes the message, not at image time, so camera latency times speed becomes position error.
- **Bearings** are body FLU, positive to the left, and encoded as (sin, cos) so 0 vs 2π has no discontinuity. In an inactive field both are 0, which no valid angle produces.
- **Reset:** `reset()` clears the estimate before the first observation of an episode.

## Excluded from the observation (privileged)

- Target coordinates, ground-truth distance, ground-truth bearing, and Gazebo poses are excluded. So are local x and y, because with a fixed target they would let a policy memorise the target position.
- Ground truth is used only in `sar_task.py`: the target is loaded from `sim/worlds/sar_walls.sdf`, and the module defines reward, success, and limits. It also feeds `info["privileged"]` for evaluation and debugging.
- These tests enforce the separation:
  - `test_sar_observation.py::test_observation_module_cannot_reach_ground_truth` (imports and signature).
  - `test_px4_uav_env_safety.py::test_changing_ground_truth_target_does_not_change_observation` (identical observation with the target moved, fixed sensor inputs).
  - `test_target_perception.py::test_perception_module_has_no_simulator_or_ground_truth_access`.

## Reward and episode (`sar_task.TaskConfig`, privileged)

- **Reward per step:** −0.1, plus the reduction in horizontal distance to the target centre (m). Add +100 on success (distance ≤ 0.75 m) or −100 on collision.
- **Precedence:** collision wins over success in the same step.
- **Terminated:** on success or collision.
- **Truncated:** at `max_steps` = 600.
- **Collision** is any of: minimum LiDAR sector < 0.25 m, |roll| or |pitch| > 1.2 rad, or a PX4 failsafe.

## Episode lifecycle and control (`px4_uav_env.py`)

1. **`reset()`** waits for fresh landed-and-disarmed telemetry, then takes off. The takeoff gates are unchanged: Offboard stream, preflight, arm ACK, NED z ≤ −0.8 m, and fresh `heading_good_for_control`.
2. **Start pose.** The vehicle then flies to the fixed start pose: NED (0, 0), z = −1.8 m, heading π/2 (east, the spawn heading).
   - Flight uses Offboard position setpoints ("carrot" at most 0.8 m ahead, so roughly ≤ 1 m/s).
   - It must end within 0.3 m horizontal, 0.25 m vertical, 0.25 m/s and 10° yaw.
   - It is bounded by 40 s of simulation time and 120 s wall, and aborts if the vehicle leaves armed OFFBOARD, a failsafe appears, or local position goes stale.
   - Any failure enters the existing safe-state landing.
3. **Each `step()`** applies the action for **0.1 s of simulation time**.
   - The step ends on the first fresh LiDAR sectors after that, within a 2 s wall bound.
   - The simulation clock accumulates `vehicle_local_position` stamp increments of 0–200 ms. Larger jumps are XRCE time-offset resyncs and are ignored. PX4 timesync alone updates only about every 1 s of simulation time.
   - Altitude is held at −1.8 m by a position setpoint on z while x and y follow the velocity command (Offboard `position` flag; per-axis NaN).
4. **Episode end:**
   - On **success** (≤ 0.75 m) or **timeout**, the vehicle flies back to the start pose before the normal NAV_LAND, so it never lands on the target sphere. If that return fails, it lands in place.
   - On **collision**, it lands in place immediately.
   - Landing keeps the committed `e10b92e` behaviour: the zero-velocity heartbeat continues until fresh AUTO_LAND, then setpoints stop, and completion requires fresh landed-and-disarmed telemetry.
5. **World consistency.** `tools/verify_world_target.py` is a read-only gate. It checks that the live Gazebo `target` pose matches `sim/worlds/sar_walls.sdf` within 5 cm before a flight.

**Verified offline only:** the altitude-hold setpoint form, the start-pose flight, and simulation-time stepping are unit-tested. Their PX4 behaviour needs the gated SITL check.
