# Target-reaching task audit (pre-RL)

**Date:** 2026-10-18. **Code audited:** commit `e10b92e`.

**Sources:**
- `px4_uav_env.py`, `observation_publisher.py`, `observation_geometry.py`.
- PX4 `Tools/simulation/gz/models/{x500_lidar_2d,lidar_2d_v2}` and `worlds/walls.sdf`.

**Measurements** (read-only; scripts and outputs are in `/home/hp/uav_ros_experiments/target_reward_audit_20261018/`):
- A live probe of the landed vehicle in `restart_20261018_visible3`.
- The visible3 and smoke2 ULogs and smoke-test logs.

Labels: **[measured]** live or ULog evidence; **[source]** read from code or SDF; **[inference]** reasoned, not measured.

## 1. Target and frames

| Consumer | Target definition | Frame |
|---|---|---|
| Gazebo world | model `target`, static sphere, pose `(3, -3, 0.055)`, radius 0.5 m [source] | world ENU |
| Observation publisher | live pose of model `target` from `/world/walls/pose/info` [source] | world ENU, then a bearing in the LiDAR frame |
| Env reward and success | constants `TARGET_X=-3, TARGET_Y=3` against PX4 `vehicle_local_position` x, y [source] | PX4 local NED |

- **Same target: yes [measured].**
  - The standard ENU→NED mapping (`x_ned = y_enu`, `y_ned = x_enu`) sends Gazebo (3, −3) to NED (−3, 3), which equals the env constants.
  - PX4's local origin is 4–5 cm from the Gazebo world origin (live: Gazebo UAV (−0.027, 0.067) against PX4 (0.111, −0.042) after ENU→NED).
  - Env distance 4.351 m against Gazebo XY distance 4.309 m.
- **Heading offset [measured]:** PX4 `heading` = 1.643 rad, while Gazebo yaw implies 1.574 rad, a constant offset of about **4°** (likely the magnetic model). The env's body-forward velocity uses PX4 heading, while the LiDAR and target bearing use the Gazebo yaw, so the two frames disagree by about 4°.
- **Reproducibility risk [source]:** the `target` model is an *uncommitted local modification* to the shared PX4 checkout's `worlds/walls.sdf` (+26 lines; untracked `walls.sdf.backup`). It is not versioned in this repo. The env constants are hard-coded and nothing checks them against the Gazebo pose at runtime.

## 2. Observation (54 values)

| Index | Raw publisher value | Env-normalized value |
|---|---|---|
| 0–48 | minimum range per 49 equal sectors of the LiDAR's 270° scan (±2.356 rad, 1080 rays, 0.1–30 m, 30 Hz); non-finite → 30 m | `/30`, clipped to [0, 1] |
| 49 | `target_detected` (0/1): target inside the FOV *and* a nearby ray within 0.35 m / 3° of the expected sphere surface | clipped to [0, 1] |
| 50 | measured surface range to the target (m) when detected, else 0 | `/30`, clipped |
| 51 | measured ray angle in the LiDAR frame (rad) when detected, else 0 | `/π`, clipped to [−1, 1] |
| 52–53 | `previous_speed`, `previous_turn`; **always 0** (no update path) | **replaced** by the env's previous action |

**On the ground, the target is observed [measured]:** `target_detected=1`, range 3.790 m, angle −0.806 rad, against a geometric bearing of −0.789 rad.

**In flight, the target is never observed [measured, with source geometry]:**
- Both smoke flights at NED z ≈ −1.63 m reported `target_detected=False` on every step.
- The LiDAR scan plane is about 0.3 m above the model root (include pose +0.26, sensor +0.055), so about 1.9 m at hover. The sphere top is 0.555 m, so the horizontal scan passes over the target. With the current hover altitude, observation slots 49–51 are always 0 during episodes.

**There is no ego-position or target vector in the observation [source].** In flight the policy sees only LiDAR sectors and its own previous action, so it has no observable signal of where the target is. Target reaching is not solvable from the observation unless the policy memorizes a fixed route, and the start state is not fixed (see §5).

Finite and fresh [source, measured]: the publisher drops non-finite or stale (>0.5 s) inputs. The env requires a new observation that is ≤1 s old, with exactly 54 finite values. All 54 values were finite in the live samples.

## 3. Action and control

- **Action [source]:** `Box([0, -1], [1, 1])`. Speed is scaled to at most 1 m/s body-forward (no reverse); turn to ±0.5 rad/s yaw rate. Body-forward speed is projected to NED with the PX4 heading and suppressed unless `heading_good_for_control`. The vertical velocity setpoint is 0; there is **no altitude position hold in the VELOCITY phase**.
- **Rates:**
  - The setpoint timer runs at 20 Hz wall [source].
  - `step()` returns on the next `/uav/observation`, published by a 10 Hz *wall-clock* ROS timer, so one step is about 0.1 s wall [source].
  - Simulation runs at about **0.49× real time** [measured: takeoff ACK to NAV_LAND ACK took 9.35 s sim over 19.06 s wall]. One step is therefore about 0.05 s of vehicle time, and that varies with host load.
- **Response [measured, visible3 ULog]:**
  - The commanded NED velocity was correct: (−0.014, 0.200, 0) for `[0.2, 0]` at heading 1.641.
  - The ten actions spanned only 0.45 s of sim time, and the vehicle reached 0.045 m/s before NAV_LAND. The smoke test validates plumbing, not motion.
- **Safety limits [source]:** speed ≤ 1 m/s, |yaw rate| ≤ 0.5 rad/s, no climb command. There is **no env geofence or altitude bound**. The arena is open to the west and south (walls `box1..box4` only).

## 4. Reward and termination

**Reward per step [source]:** −0.1, plus progress (previous distance − new distance in the PX4 NED plane to (−3, 3)), −100 on `collision`, and +100 when distance ≤ 0.75 m.

- Progress and success use the same target and position source, so distance, reward and success agree [measured: env distance matches Gazebo within the origin offset].
- At 1 m/s and about 0.05 s sim per step, the best-case progress is about +0.05 per step against the −0.1 step cost. Return is therefore dominated by the terminal ±100, and the time-cost weight depends on host speed [inference].
- **`collision` [source]** is any of:
  - the minimum of the 49 sectors below 0.25 m (a proximity threshold);
  - |roll| or |pitch| above 1.2 rad;
  - a PX4 `failsafe`.

  No self-hits were seen: the landed minimum sector was 3.79 m, the target sphere [measured].
- **Termination [source]:** `collision` or `reached`. Truncation happens at `max_steps=1000` (about 100 s wall, about 50 s sim). Every terminal or truncated step calls `_enter_safe_state()`, which lands at the current position; the Gymnasium 5-tuple is preserved.

## 5. Reset behaviour (critical for RL)

`reset()` waits for fresh landed and disarmed telemetry, takes off at the **current** XY position to z = −1.8 m, then starts the episode. It does not reset Gazebo or reposition the vehicle [source].

- **After a success**, the vehicle lands within 0.75 m of the target. Since 0.75 m exceeds the sphere's 0.5 m radius, it may land **on the static sphere** [inference]. The next episode would start already inside the success radius, giving +100 on step 1, which is degenerate.
- **After a proximity "collision"** (< 0.25 m from a wall), it lands next to the wall, and the next episode can trigger an immediate collision [inference].
- **Multi-episode `reset()` has never been flown** [measured: each smoke run did one episode]. After auto-disarm PX4 reverts to OFFBOARD, with `preflight=false` until an Offboard signal is present. `_takeoff()` streams before checking preflight, so this should pass, but it is unverified.

## 6. Proposed changes (need approval; not implemented)

These change task semantics, so each is listed for review.

1. **Make the target observable.** This is required for learning. Options:
   - (a) Recommended: append a state-based target feature from PX4 local position and heading (normalized distance; sin/cos of the target bearing relative to body-forward), making the observation 54 → 57. LiDAR fields are unchanged.
   - (b) Make the world target taller. This edits the shared world, so it is not recommended.
   - (c) Leave the observation unchanged and accept that the task is not learnable.
2. **Deterministic episode start.** After takeoff, fly in Offboard position mode to a fixed start pose (for example NED (0, 0, −1.8)) and hold within a tolerance before the episode begins. This removes the dependence on where the previous episode ended.
3. **Terminal landing site.** On success, either return to the start pose before landing, or land at the current position only once it is outside the sphere radius. This avoids landing on the target.
4. **Step period in sim time.** Make `step()` wait for at least Δt of reconstructed PX4 time (for example 0.1 s, using the existing timesync clock) as well as a fresh observation, so per-step dynamics don't depend on host load.
5. **Altitude hold in VELOCITY.** Send the takeoff z as a position setpoint with x/y velocity. PX4 supports mixed position and velocity setpoints; this needs offline tests and one SITL check.
6. **Consistency guard.** At startup, compare the Gazebo `target` pose (converted to NED) with `TARGET_X/Y`, and fail closed if they disagree by more than 0.2 m.
7. **Non-learning baseline.** Pure pursuit on the existing action interface: speed ∝ distance (capped), turn ∝ heading error to the target. It uses `info` x/y and adds `heading` to `info`, which is non-breaking. Validate first against a kinematic fake node offline; then run one supervised SITL episode once items 2 and 3 exist.
8. **PPO dependencies.** Neither `stable-baselines3` nor `torch` is installed in `.venv`; only `gymnasium` 1.3.0 and numpy 1.26.4 are. Installing them needs network access and your approval.
