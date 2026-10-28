import dataclasses
import math
import os
import threading
import time
import warnings
from collections import deque

import gymnasium as gym
import numpy as np
import rclpy

from gymnasium import spaces
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleAttitude,
    VehicleLandDetected,
    VehicleCommandAck,
    VehicleOdometry,
    TimesyncStatus,
)

import sar_observation
import sar_task
import target_perception


VEHICLE_COMMAND_ACK_TOPIC = "/fmu/out/vehicle_command_ack_v1"
LIDAR_SECTORS_TOPIC = "/uav/lidar_sectors"
# Downward single-beam range (terrain following, 3D step 2, option A): bridged from Gazebo by the launcher under a
# name PX4's gz_bridge does not subscribe to, so PX4's height estimation is unchanged.
AGL_RANGE_TOPIC = os.environ.get("SAR_AGL_RANGE_TOPIC", "/uav/agl_range")
TARGET_PERCEPTION_TOPIC = "/uav/target_perception"
# RGB camera_info header stamps are Gazebo simulation time; they time action steps.
CAMERA_RGB_INFO_TOPIC = os.environ.get("SAR_CAMERA_RGB_INFO_TOPIC", "/uav/camera/rgb/camera_info")


class PX4ControllerNode(Node):

    # PX4's AUTO magnetometer path completes final heading alignment only
    # after rising above 1.5 m HAGL. Hold position above that threshold with
    # margin until fresh heading-good telemetry arrives.
    TAKEOFF_TARGET_Z = -1.8
    # Fly-to-pose ("carrot") setpoints stay within this distance of the vehicle,
    # which bounds PX4's position-controller speed to roughly 1 m/s.
    CARROT_DISTANCE = 0.8
    # Position-message stamp increments larger than this are XRCE time-offset
    # resyncs, not simulation progress, and are not added to the sim clock.
    # This position-stamp clock (sim_clock_us) is not used for timing or by the
    # runtime guard: PX4 slews these DDS stamps toward wall time after each
    # timesync reset, so it runs fast. It is kept only as a diagnostic.
    SIM_CLOCK_MAX_INCREMENT_US = 200_000
    # The camera clock is invalidated (fail closed) by a forward jump larger than
    # this between consecutive RGB frames (30 Hz sensor: 33 ms; frame-skipping
    # perception stamps never exceeded 0.4 s in the recorded runs).
    CAMERA_CLOCK_MAX_FRAME_GAP_US = 500_000

    def __init__(self, env):

        super().__init__("px4_uav_env_controller")

        self.env = env

        # --------------------------------------------------
        # PX4 publishers
        # --------------------------------------------------

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        # --------------------------------------------------
        # PX4 Best-Effort QoS
        # --------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # --------------------------------------------------
        # Subscriptions
        # --------------------------------------------------

        self.obs_sub = self.create_subscription(
            Float32MultiArray,
            LIDAR_SECTORS_TOPIC,
            self.observation_callback,
            10,
        )

        self.perception_sub = self.create_subscription(
            Float64MultiArray,
            TARGET_PERCEPTION_TOPIC,
            self.perception_callback,
            10,
        )

        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos,
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos,
        )

        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            VEHICLE_COMMAND_ACK_TOPIC,
            self.command_ack_callback,
            qos,
        )

        self.timesync_status_sub = self.create_subscription(
            TimesyncStatus,
            "/fmu/out/timesync_status",
            self.timesync_status_callback,
            qos,
        )

        # Best-effort (sensor-data) QoS matches the bridge's camera_info publisher
        # whatever its reliability; the perception node uses the same profile.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            CAMERA_RGB_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self.land_detected_callback,
            qos,
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos,
        )

        self.agl_sub = self.create_subscription(
            LaserScan,
            AGL_RANGE_TOPIC,
            self.agl_callback,
            qos_profile_sensor_data,
        )

        # --------------------------------------------------
        # State
        # --------------------------------------------------

        self.latest_observation = None
        self.observation_counter = 0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.heading_valid = False
        self.position_valid = False
        self.position_received_at = None
        self.observation_received_at = None
        self.yaw_rate = 0.0
        self.odometry_received_at = None
        self.latest_perception = None
        self.perception_received_at = None
        self.px4_sim_time_us = None
        self.px4_sim_time_received_at = None
        # Position-stamp clock: diagnostic only (see SIM_CLOCK_MAX_INCREMENT_US).
        self.sim_clock_us = None
        self._last_position_stamp = None
        # Action-step clock: Gazebo time from RGB camera_info stamps.
        self.camera_clock_us = None
        self.camera_clock_received_at = None
        self.camera_clock_samples = 0
        self.camera_clock_invalid = None   # reason string once invalidated; sticky until reset
        self.hold_z = self.TAKEOFF_TARGET_Z
        # Terrain following (off unless the env enables it): hold TAKEOFF_TARGET_Z relative to the ground height
        # recorded under the start point, from the downward range. See terrain_hold_z().
        self.terrain_following = False
        self.agl_range = None             # m, latest valid downward range (None: no return)
        self.agl_received_at = None       # monotonic receipt time of the latest downward scan
        self.terrain_ref_ground_z = None  # NED z of the ground under the start point
        self._terrain_last_t = None
        self.goal = None              # (x, y, z, yaw) NED for the REPOSITION phase

        self.arming_state = 1
        self.nav_state = 4
        self.system_id = None
        self.failsafe = False
        self.pre_flight_checks_pass = False
        self.latest_arming_reason = None
        self.latest_disarming_reason = None
        self.status_received_at = None
        self.command_ack_counter = 0
        self.command_ack_history = deque(maxlen=32)
        self.landed = None
        self.ground_contact = None
        self.land_detected_received_at = None
        self.landing_complete = False
        # Set once fresh status confirms AUTO_LAND for the current landing
        # request. PX4 routes /fmu/in/trajectory_setpoint onto the same uORB
        # topic as its native landing setpoints, so the LAND stream must stop
        # rather than interleave zero vertical velocity with the descent.
        self.land_setpoints_released = False

        self.roll = 0.0
        self.pitch = 0.0

        # Takeoff / flight phase
        self.phase = "GROUND"
        self.takeoff_setpoint_count = 0
        self.takeoff_stream_started_at = None
        self.takeoff_stream_last_at = None
        self.takeoff_target_x = 0.0
        self.takeoff_target_y = 0.0

        # Current normalized RL action
        self.action = np.zeros(
            2,
            dtype=np.float32,
        )

        # Timer runs continuously so OffboardControlMode
        # and TrajectorySetpoint are continuously streamed.
        self.timer = self.create_timer(
            0.05,  # 20 Hz
            self.control_loop,
        )

        self.condition = threading.Condition()

    # ======================================================
    # ROS callbacks
    # ======================================================

    def observation_callback(self, msg):

        data = np.asarray(
            msg.data,
            dtype=np.float32,
        )

        if (
            len(data) != sar_observation.LIDAR_SECTORS
            or not np.all(np.isfinite(data))
        ):
            return

        with self.condition:

            self.latest_observation = data.copy()
            self.observation_received_at = time.monotonic()

            self.observation_counter += 1

            self.condition.notify_all()

    def position_callback(self, msg):
        with self.condition:
            self.x = float(msg.x)
            self.y = float(msg.y)
            self.z = float(msg.z)
            self.heading = float(msg.heading)
            self.vx = float(msg.vx)
            self.vy = float(msg.vy)
            self.vz = float(getattr(msg, "vz", 0.0))
            self.heading_valid = bool(
                msg.heading_good_for_control and np.isfinite(self.heading)
            )
            self.position_valid = bool(
                msg.xy_valid
                and msg.z_valid
                and msg.v_xy_valid
                and np.all(np.isfinite(
                    [self.x, self.y, self.z, self.heading, self.vx, self.vy]
                ))
            )
            self.position_received_at = time.monotonic()
            self._advance_sim_clock(int(msg.timestamp))
            self.condition.notify_all()

    def _advance_sim_clock(self, stamp_us):
        previous = self._last_position_stamp
        self._last_position_stamp = stamp_us
        if self.sim_clock_us is None:
            self.sim_clock_us = 0
            return
        increment = stamp_us - previous
        if 0 < increment <= self.SIM_CLOCK_MAX_INCREMENT_US:
            self.sim_clock_us += increment

    def camera_info_callback(self, msg):
        stamp = msg.header.stamp
        stamp_us = int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000
        with self.condition:
            self._advance_camera_clock(stamp_us, time.monotonic())
            self.condition.notify_all()

    def _advance_camera_clock(self, stamp_us, received_at):
        """Advance the action clock; never from wall time, and fail closed on any discontinuity.

        Duplicates are ignored (no progress, freshness not refreshed). A backward stamp
        (e.g. a simulation reset) or a forward jump above CAMERA_CLOCK_MAX_FRAME_GAP_US
        invalidates the clock until reset_camera_clock(), which only a new env.reset() calls.
        """
        if self.camera_clock_invalid is not None:
            return
        if self.camera_clock_us is None:
            self.camera_clock_us = stamp_us
            self.camera_clock_received_at = received_at
            self.camera_clock_samples = 1
            return
        increment = stamp_us - self.camera_clock_us
        if increment == 0:
            return
        if increment < 0:
            self.camera_clock_invalid = (
                f"camera stamp moved backwards by {-increment} us (simulation reset?)"
            )
            return
        if increment > self.CAMERA_CLOCK_MAX_FRAME_GAP_US:
            self.camera_clock_invalid = (
                f"camera stamp jumped forward {increment} us between frames"
            )
            return
        self.camera_clock_us = stamp_us
        self.camera_clock_received_at = received_at
        self.camera_clock_samples += 1

    def reset_camera_clock(self):
        """Forget the clock (caller holds the condition); it must re-validate from new frames."""
        self.camera_clock_us = None
        self.camera_clock_received_at = None
        self.camera_clock_samples = 0
        self.camera_clock_invalid = None

    def perception_callback(self, msg):
        decoded = target_perception.decode(msg.data)
        if decoded is None:
            return
        with self.condition:
            previous = self.latest_perception
            # Out-of-order or repeated frames must never look newer than they are.
            if previous is not None and decoded["image_stamp"] <= previous["image_stamp"]:
                return
            self.latest_perception = decoded
            self.perception_received_at = time.monotonic()
            self.condition.notify_all()

    def odometry_callback(self, msg):
        # VehicleOdometry angular velocity is body FRD; the policy uses FLU (+CCW).
        yaw_rate = -float(msg.angular_velocity[2])
        if not math.isfinite(yaw_rate):
            return
        with self.condition:
            self.yaw_rate = yaw_rate
            self.odometry_received_at = time.monotonic()
            self.condition.notify_all()

    def status_callback(self, msg):

        with self.condition:
            self.arming_state = int(msg.arming_state)
            self.nav_state = int(msg.nav_state)
            self.system_id = int(msg.system_id)
            self.failsafe = bool(msg.failsafe)
            self.pre_flight_checks_pass = bool(
                msg.pre_flight_checks_pass
            )
            self.latest_arming_reason = int(msg.latest_arming_reason)
            self.latest_disarming_reason = int(msg.latest_disarming_reason)
            self.status_received_at = time.monotonic()
            if (
                getattr(self, "phase", None) == "LAND"
                and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            ):
                # Latched: later stale or changed status must not restart
                # the competing stream for this landing request.
                self.land_setpoints_released = True
            self._update_landing_complete()
            self.condition.notify_all()

    def command_ack_callback(self, msg):
        with self.condition:
            self.command_ack_counter += 1
            ack = {
                "counter": self.command_ack_counter,
                "command": int(msg.command),
                "result": int(msg.result),
                "result_param1": int(msg.result_param1),
                "result_param2": int(msg.result_param2),
                "received_at": time.monotonic(),
            }
            self.command_ack_history.append(ack)
            self.condition.notify_all()

        if ack["result"] not in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):
            self.get_logger().error(
                "PX4 rejected/failed command "
                f"{ack['command']} with result {ack['result']} "
                f"(result_param1={ack['result_param1']}, "
                f"result_param2={ack['result_param2']})"
            )

    def timesync_status_callback(self, msg):
        # PX4 serializes TimesyncStatus.timestamp with the DDS time offset,
        # while estimated_offset is sent unchanged. Their sum recovers the
        # PX4 HRT timestamp, which follows Gazebo time in lockstep SITL.
        px4_sim_time_us = int(msg.timestamp) + int(msg.estimated_offset)

        with self.condition:
            self.px4_sim_time_us = px4_sim_time_us
            self.px4_sim_time_received_at = time.monotonic()
            self.condition.notify_all()

    def land_detected_callback(self, msg):

        with self.condition:
            self.landed = bool(msg.landed)
            self.ground_contact = bool(getattr(msg, "ground_contact", False))
            self.land_detected_received_at = time.monotonic()
            self._update_landing_complete()
            self.condition.notify_all()

    def _update_landing_complete(self):

        now = time.monotonic()
        status_fresh = (
            self.status_received_at is not None
            and now - self.status_received_at <= 1.0
        )
        landed_fresh = (
            self.land_detected_received_at is not None
            and now - self.land_detected_received_at <= 1.0
        )

        if (
            status_fresh
            and landed_fresh
            and self.landed is True
            and self.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        ):
            self.landing_complete = True
        else:
            self.landing_complete = False

    def agl_callback(self, msg):
        ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        with self.condition:
            self.agl_range = min(ranges) if ranges else None
            self.agl_received_at = time.monotonic()

    AGL_MAX_AGE = 0.5          # s wall: an older downward range is not used
    TERRAIN_MAX_DEVIATION = 1.0  # m: the height target stays within this of TAKEOFF_TARGET_Z
    TERRAIN_MAX_RATE = 0.5     # m/s: largest change of the height target

    def ground_z(self, now):
        """NED z of the ground under the vehicle from a fresh downward range, else None (caller holds condition)."""
        if (self.agl_range is None or self.agl_received_at is None
                or now - self.agl_received_at > self.AGL_MAX_AGE or self.z is None):
            return None
        return self.z + self.agl_range * math.cos(self.roll) * math.cos(self.pitch)

    def terrain_hold_z(self, now):
        """Height target that keeps TAKEOFF_TARGET_Z above the current ground, relative to the reference ground.

        On flat ground the target is exactly TAKEOFF_TARGET_Z. Clamped to +-TERRAIN_MAX_DEVIATION and rate limited to
        TERRAIN_MAX_RATE; without a fresh range or a reference the target is left unchanged (caller holds condition).
        """
        ground = self.ground_z(now)
        dt = 0.0 if self._terrain_last_t is None else min(max(now - self._terrain_last_t, 0.0), 0.2)
        self._terrain_last_t = now
        if ground is None or self.terrain_ref_ground_z is None:
            return self.hold_z
        desired = self.TAKEOFF_TARGET_Z + (ground - self.terrain_ref_ground_z)
        desired = min(max(desired, self.TAKEOFF_TARGET_Z - self.TERRAIN_MAX_DEVIATION),
                      self.TAKEOFF_TARGET_Z + self.TERRAIN_MAX_DEVIATION)
        step = self.TERRAIN_MAX_RATE * dt
        return min(max(desired, self.hold_z - step), self.hold_z + step)

    def attitude_callback(self, msg):

        # PX4 quaternion:
        # q = [w, x, y, z]

        w = float(msg.q[0])
        x = float(msg.q[1])
        y = float(msg.q[2])
        z = float(msg.q[3])

        # Roll
        sin_roll = 2.0 * (
            w * x + y * z
        )

        cos_roll = 1.0 - 2.0 * (
            x * x + y * y
        )

        self.roll = math.atan2(
            sin_roll,
            cos_roll,
        )

        # Pitch
        sin_pitch = 2.0 * (
            w * y - z * x
        )

        sin_pitch = np.clip(
            sin_pitch,
            -1.0,
            1.0,
        )

        self.pitch = math.asin(
            sin_pitch
        )

    # ======================================================
    # PX4 publishing
    # ======================================================

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    def publish_offboard_mode(
        self,
        position_mode,
    ):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = position_mode
        msg.velocity = not position_mode
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    def has_px4_setpoint_subscribers(self):
        """Return whether PX4 has discovered both required input topics."""
        return (
            self.offboard_pub.get_subscription_count() > 0
            and self.setpoint_pub.get_subscription_count() > 0
        )

    def publish_position_setpoint(
        self,
        z_target=-1.0,
        x_target=0.0,
        y_target=0.0,
        yaw=float("nan"),
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [
            x_target,
            y_target,
            z_target,
        ]

        msg.velocity = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.acceleration = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.jerk = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.yaw = float(yaw)
        msg.yawspeed = float("nan")

        self.setpoint_pub.publish(msg)

    def carrot_setpoint(self):
        """Position setpoint toward self.goal, at most CARROT_DISTANCE from the vehicle."""
        gx, gy, gz, gyaw = self.goal
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)
        scale = min(1.0, self.CARROT_DISTANCE / distance) if distance > 1e-6 else 0.0
        return self.x + dx * scale, self.y + dy * scale, gz, gyaw

    def publish_velocity_setpoint(self, hold_z=None):

        speed_action = float(
            self.action[0]
        )

        turn_action = float(
            self.action[1]
        )

        # ----------------------------------------------
        # RL action
        #
        # speed in [0, 1]
        # turn  in [-1, 1]
        # ----------------------------------------------

        MAX_SPEED = 1.0
        MAX_YAW_RATE = 0.5

        forward_speed = (
            speed_action * MAX_SPEED
        )
        if not getattr(self, "heading_valid", False):
            # The policy's forward action is body-relative. Without a valid
            # PX4 heading it cannot be transformed safely into local NED.
            forward_speed = 0.0

        yaw_rate = (
            turn_action * MAX_YAW_RATE
        )

        # ------------------------------------------------
        # Convert body-forward velocity into PX4
        # local NED velocity.
        #
        # PX4 heading is expressed in the local frame.
        # ------------------------------------------------

        vx = (
            forward_speed
            * math.cos(self.heading)
        )

        vy = (
            forward_speed
            * math.sin(self.heading)
        )

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # Position is not controlled in velocity mode.
        if hold_z is None:
            # LAND-phase heartbeat: zero vertical velocity, no position axis.
            msg.position = [float("nan")] * 3
            msg.velocity = [vx, vy, 0.0]
        else:
            # Episode flight: PX4 holds altitude by position; xy by velocity.
            msg.position = [float("nan"), float("nan"), float(hold_z)]
            msg.velocity = [vx, vy, float("nan")]

        msg.acceleration = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.jerk = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        msg.yaw = float("nan")
        msg.yawspeed = yaw_rate

        self.setpoint_pub.publish(msg)

    # ======================================================
    # Continuous control loop
    # ======================================================

    def control_loop(self):
        # Serialize controller output with safe-state transitions so no
        # timer callback can read a stale action after the transition returns.
        with self.condition:
            if self.phase == "TAKEOFF":
                self.publish_offboard_mode(position_mode=True)
                self.publish_position_setpoint(
                    z_target=self.TAKEOFF_TARGET_Z,
                    x_target=self.takeoff_target_x,
                    y_target=self.takeoff_target_y,
                )
                now = time.monotonic()
                if self.takeoff_stream_started_at is None:
                    self.takeoff_stream_started_at = now
                self.takeoff_stream_last_at = now
                self.takeoff_setpoint_count += 1
                self.condition.notify_all()
            elif self.phase == "REPOSITION":
                x, y, z, yaw = self.carrot_setpoint()
                self.publish_offboard_mode(position_mode=True)
                self.publish_position_setpoint(
                    z_target=z, x_target=x, y_target=y, yaw=yaw
                )
            elif self.phase == "VELOCITY":
                # The position flag enables PX4's per-axis position/velocity
                # control: NaN xy position uses the velocity setpoint.
                if getattr(self, "terrain_following", False):
                    self.hold_z = self.terrain_hold_z(time.monotonic())
                self.publish_offboard_mode(position_mode=True)
                self.publish_velocity_setpoint(hold_z=self.hold_z)
            elif self.phase == "LAND":
                # While PX4 still reports OFFBOARD, keep the heartbeat and
                # zero-velocity setpoints; after fresh AUTO_LAND, PX4's native
                # landing setpoints must be the only trajectory input.
                if self.landing_complete or self.land_setpoints_released:
                    return
                self.action[:] = 0.0
                self.publish_offboard_mode(position_mode=False)
                self.publish_velocity_setpoint()

    # ======================================================
    # Commands
    # ======================================================

    def send_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        *,
        require_fresh_status=True,
    ):

        with self.condition:
            target_system = self.system_id
            ack_counter = self.command_ack_counter
            status_received_at = self.status_received_at

        if target_system is None:
            raise RuntimeError(
                "Cannot send PX4 command before VehicleStatus provides the system ID."
            )
        if require_fresh_status and (
            status_received_at is None
            or time.monotonic() - status_received_at > 1.0
        ):
            raise RuntimeError(
                "Cannot send PX4 command using missing or stale VehicleStatus "
                "telemetry (maximum age 1.000 s)."
            )

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = target_system
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)
        return ack_counter

    def request_offboard(self):

        return self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )

    def arm(self):

        return self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )

    def land(self):
        with self.condition:
            self.landing_complete = False
            # A new landing request needs its own fresh AUTO_LAND confirmation.
            self.land_setpoints_released = False
            self.condition.notify_all()
        # Landing is the safety command used precisely when normal flight
        # telemetry may be stale. The target still must be a previously
        # observed PX4 system ID; freshness is enforced for mode/arm commands.
        return self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            require_fresh_status=False,
        )


class PX4UAVEnv(gym.Env):

    """
    Gymnasium wrapper around the PX4/Gazebo UAV.

    Action:
        [speed, turn]

    Observation:
        sar_obs_v1 (65 values; see sar_observation.FIELDS and
        docs/OBSERVATION_CONTRACT.md). It is built only from LiDAR sectors,
        the previous action, PX4 EKF estimates, and sensor-derived target
        perception. Simulator ground truth (sar_task) is used only for
        reward, success, and the ``info["privileged"]`` debugging values.
    """

    metadata = {
        "render_modes": []
    }

    # The preserved SITL trace took ~24 s from NAV_LAND to fresh disarmed
    # status; this remains bounded while avoiding a normal close-timeout race.
    LAND_WAIT_TIMEOUT = 30.0
    # Stuck landing (mines, 2026-10-28): after a landing off the solid pads PX4 reported ground contact but never
    # "landed", and stayed armed. While close() waits, a vehicle resting this long (ground contact, armed, not landed,
    # |vz| and horizontal speed small) gets an alert and a forced disarm, at most STUCK_LANDING_ATTEMPTS times.
    STUCK_LANDING_S = 15.0
    STUCK_LANDING_ATTEMPTS = 3
    TAKEOFF_STREAM_TIMEOUT = 3.0
    TAKEOFF_STREAM_MIN_DURATION = 1.0
    TAKEOFF_ACK_TIMEOUT = 3.0
    TAKEOFF_MODE_TIMEOUT = 5.0
    TAKEOFF_TIMEOUT = 15.0
    TAKEOFF_WALL_TIMEOUT = 45.0
    TAKEOFF_TELEMETRY_MAX_AGE = 1.0
    # Before this env has ever commanded flight, safe-state entry first waits
    # up to this long for fresh disarmed + landed telemetry instead of sending
    # NAV_LAND. Replaying recorded SITL status/land receipt times through the
    # same check confirmed the ground within 3.2 s in every case.
    GROUND_CONFIRM_TIMEOUT = 10.0
    # Keep decisions tied to <=1 s-old status, but allow one replacement
    # VehicleStatus sample at the measured 1.71 s worst interval in the
    # isolated, competing SITL runs, with a small bounded margin.
    TAKEOFF_STATUS_UPDATE_TIMEOUT = 2.0
    TAKEOFF_CLOCK_STALL_TIMEOUT = 5.0
    OBSERVATION_MAX_AGE = 1.0
    # Wall-clock receipt age after which a perception message is not current.
    # The RGB camera measured ~2 Hz wall under software rendering.
    PERCEPTION_MAX_AGE = 1.5
    # Action clock (RGB camera_info Gazebo stamps). Wall-clock age after which the
    # last new frame no longer counts as a running clock: same bound as
    # OBSERVATION_MAX_AGE (~11 frames at 0.35x, ~6 at 0.2x).
    CAMERA_CLOCK_MAX_AGE = 1.0
    # reset() requires this many increasing frames, received after the reset
    # began, within this wall time before it may take off (never arms without).
    CAMERA_CLOCK_MIN_SAMPLES = 2
    CAMERA_CLOCK_WAIT_TIMEOUT = 3.0
    TARGET_MEMORY_EXPIRY = 8.0          # s of simulation time
    # Repeatable episode start (PX4 local NED; the spawn heading is east).
    START_X = 0.0
    START_Y = 0.0
    START_YAW = math.pi / 2
    REPOSITION_TOLERANCE = 0.3          # m horizontal
    BESIDE_TARGET_M = 1.1               # land_beside_target: landing point this far from the target centre (on the
                                        # target's 3 x 3 m pad, the only solid ground there besides the start pad)
    REPOSITION_Z_TOLERANCE = 0.25       # m
    REPOSITION_YAW_TOLERANCE = math.radians(10.0)
    REPOSITION_SPEED_TOLERANCE = 0.25   # m/s
    REPOSITION_SIM_TIMEOUT = 40.0       # s of simulation time
    REPOSITION_WALL_TIMEOUT = 120.0     # s wall ceiling
    STEP_DURATION = 0.1                 # s of simulation time per action

    def __init__(self, terrain_following=False, max_steps=None, land_beside_target=False):

        super().__init__()
        self.terrain_following = bool(terrain_following)
        # Mines: after success, land beside the target (deliver and stay) instead of the straight-line return to the
        # start, which would cross solid rock; a timeout lands in place.
        self.land_beside_target = bool(land_beside_target)

        # --------------------------------------------------
        # Same action interface as old environment
        # --------------------------------------------------

        self.action_space = spaces.Box(
            low=np.array(
                [0.0, -1.0],
                dtype=np.float32,
            ),
            high=np.array(
                [1.0, 1.0],
                dtype=np.float32,
            ),
            dtype=np.float32,
        )

        # --------------------------------------------------
        # sar_obs_v1 observation; privileged task ground truth
        # --------------------------------------------------

        self.observation_space = spaces.Box(
            low=sar_observation.LOW,
            high=sar_observation.HIGH,
            dtype=np.float32,
        )
        self.task_config = sar_task.TaskConfig()
        # The versioned world SDF is the single source of target ground truth.
        self.target = sar_task.load_target()
        self.builder = sar_observation.ObservationBuilder(
            memory_expiry_s=self.TARGET_MEMORY_EXPIRY,
            perception_max_age_s=self.PERCEPTION_MAX_AGE,
        )

        # Episode step limit: the task default (600 steps = 60 s of simulation time) unless the caller passes one
        # (the runner's --max-steps, e.g. 2000 for explorer episodes).
        self.max_steps = self.task_config.max_steps if max_steps is None else int(max_steps)
        self.current_step = 0
        self._episode_active = False
        # Set (never cleared) when this env first enters a flight phase.
        self._flight_commanded = False
        self._landing_pending = False
        self._landing_request_in_progress = False
        self._closed = False
        self._close_waiting = False
        self._close_lock = threading.Lock()
        self._landing_cleanup_thread = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32,
        )

        # --------------------------------------------------
        # ROS
        # --------------------------------------------------

        if not rclpy.ok():
            rclpy.init()

        self.node = PX4ControllerNode(self)
        self.node.terrain_following = self.terrain_following

        self.spin_thread = threading.Thread(
            target=self._spin,
            daemon=True,
        )

        self.spin_thread.start()

    def _spin(self):

        rclpy.spin(self.node)

    # ======================================================
    # Observation conversion
    # ======================================================

    def _convert_observation(self, raw):
        """Build the sar_obs_v1 policy observation from raw LiDAR sectors and fresh onboard state."""
        raw = np.asarray(raw, dtype=np.float64)
        node = self.node
        with node.condition:
            now = time.monotonic()
            position_fresh = (
                node.position_received_at is not None
                and now - node.position_received_at <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            odometry_fresh = (
                node.odometry_received_at is not None
                and now - node.odometry_received_at <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            if not position_fresh or not node.position_valid:
                raise ValueError("Observation needs fresh, valid PX4 local position.")
            if not odometry_fresh:
                raise ValueError("Observation needs fresh PX4 odometry (yaw rate).")
            clock_problem = self._camera_clock_problem(now)
            if clock_problem is not None:
                raise ValueError(f"Observation needs a valid simulation clock: {clock_problem}.")
            ego = sar_observation.EgoState(
                x=node.x, y=node.y, z=self._observation_z(node, now), vx=node.vx, vy=node.vy,
                heading=node.heading, yaw_rate=node.yaw_rate,
            )
            perception = node.latest_perception
            perception_age = (
                None if node.perception_received_at is None
                else now - node.perception_received_at
            )
            now_s = node.camera_clock_us / 1_000_000
        return self.builder.build(
            raw, self.previous_action, ego, perception, perception_age, now_s
        )

    def _camera_clock_problem(self, now, min_samples=1):
        """None if the action clock is valid and running; otherwise why not (caller holds the condition)."""
        node = self.node
        if node.camera_clock_invalid is not None:
            return f"invalid ({node.camera_clock_invalid})"
        if node.camera_clock_us is None or node.camera_clock_received_at is None:
            return "unavailable (no RGB camera_info received)"
        if node.camera_clock_samples < min_samples:
            return f"not yet validated ({node.camera_clock_samples} of {min_samples} frames)"
        age = now - node.camera_clock_received_at
        if age > self.CAMERA_CLOCK_MAX_AGE:
            return f"stalled (no new camera frame for {age:.2f} s)"
        return None

    def _wait_for_camera_clock(self):
        """Before take-off: require a running camera clock re-validated since reset(); never arm without it."""
        deadline = time.monotonic() + self.CAMERA_CLOCK_WAIT_TIMEOUT
        with self.node.condition:
            while True:
                now = time.monotonic()
                problem = self._camera_clock_problem(now, self.CAMERA_CLOCK_MIN_SAMPLES)
                if problem is None:
                    return
                if self.node.camera_clock_invalid is not None or now >= deadline:
                    raise RuntimeError(f"Cannot take off: simulation clock {problem}.")
                self.node.condition.wait(timeout=min(0.1, deadline - now))

    def _privileged_info(self):
        """Simulator ground truth for evaluation and debugging only; never part of the observation."""
        return {
            "distance_to_target": self._distance_to_target(),
            "target_ned": (self.target.x_ned, self.target.y_ned),
            "x": self.node.x,
            "y": self.node.y,
            "z": self.node.z,
            "heading": self.node.heading,
        }

    def observation_age(self):
        """Wall seconds since the LiDAR sectors last returned by reset()/step() were received, or None if
        unknown (none returned yet in this episode). Used by the runner's LiDAR safety filter."""
        received = getattr(self, "last_observation_received_at", None)
        return None if received is None else time.monotonic() - received

    # ======================================================
    # Wait for observation
    # ======================================================

    def _wait_for_observation(
        self,
        timeout=2.0,
        previous_counter=None,
        step_applied_at=None,
    ):

        start = time.monotonic()
        step_us = int(self.STEP_DURATION * 1_000_000)
        anchor_us = None
        if step_applied_at is not None:
            self._step_anchor_us = None          # visible for diagnostics and tests

        with self.node.condition:

            while True:

                # An action step (step_applied_at = monotonic time the action was
                # applied) is timed in camera-clock (Gazebo) time from an anchor: the
                # first valid frame received after the action. Earlier progress never
                # counts. If several frames arrive before this re-check, the anchor is
                # the latest of them, which can only lengthen the step. A stalled clock
                # never completes the step, so the wall timeout below ends the wait.
                sim_ready = step_applied_at is None
                if step_applied_at is not None:
                    if self.node.camera_clock_invalid is not None:
                        raise RuntimeError(
                            f"Simulation clock invalid during the action step: {self.node.camera_clock_invalid}."
                        )
                    clock_ok = self._camera_clock_problem(time.monotonic()) is None
                    if (
                        anchor_us is None
                        and clock_ok
                        and self.node.camera_clock_received_at > step_applied_at
                    ):
                        anchor_us = self._step_anchor_us = self.node.camera_clock_us
                    sim_ready = (
                        anchor_us is not None
                        and clock_ok
                        and self.node.camera_clock_us >= anchor_us + step_us
                    )
                if sim_ready and (
                    previous_counter is None
                    or self.node.observation_counter
                    > previous_counter
                ):

                    if (
                        self.node.latest_observation
                        is not None
                        and self.node.observation_received_at is not None
                        and time.monotonic()
                        - self.node.observation_received_at
                        <= self.OBSERVATION_MAX_AGE
                    ):

                        if step_applied_at is not None:
                            self.last_step_clock = (anchor_us, self.node.camera_clock_us)
                        # Receipt time of exactly these sectors: a newer message may replace
                        # latest_observation before the caller acts (see observation_age()).
                        self.last_observation_received_at = self.node.observation_received_at
                        return (
                            self.node.latest_observation.copy()
                        )

                remaining = (
                    timeout
                    - (
                        time.monotonic()
                        - start
                    )
                )

                if remaining <= 0:
                    raise TimeoutError(
                        "No fresh LiDAR sectors after the action's simulation-time duration."
                    )

                self.node.condition.wait(
                    timeout=remaining
                )

    # ======================================================
    # Takeoff
    # ======================================================

    def _wait_for_takeoff_setpoint_stream(self):
        deadline = time.monotonic() + self.TAKEOFF_STREAM_TIMEOUT
        node = self.node

        with node.condition:
            while True:
                started_at = node.takeoff_stream_started_at
                last_at = node.takeoff_stream_last_at
                stream_ready = (
                    node.has_px4_setpoint_subscribers()
                    and started_at is not None
                    and last_at is not None
                    and node.takeoff_setpoint_count >= 3
                    and last_at - started_at
                    >= self.TAKEOFF_STREAM_MIN_DURATION
                    and time.monotonic() - last_at <= 0.5
                )
                if stream_ready:
                    return

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        "PX4 Offboard setpoint stream was not observed "
                        "continuously with PX4 topic subscriptions before "
                        "the mode request."
                    )
                node.condition.wait(timeout=remaining)

    def _wait_for_command_ack(self, command, after_counter):
        deadline = time.monotonic() + self.TAKEOFF_ACK_TIMEOUT
        accepted = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        in_progress = VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS
        node = self.node

        with node.condition:
            while True:
                for ack in node.command_ack_history:
                    if (
                        ack["counter"] > after_counter
                        and ack["command"] == command
                    ):
                        if ack["result"] == accepted:
                            return
                        if ack["result"] != in_progress:
                            raise RuntimeError(
                                f"PX4 rejected command {command}: "
                                f"result={ack['result']}, "
                                f"result_param1={ack['result_param1']}, "
                                f"result_param2={ack['result_param2']}."
                            )

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Timed out waiting for VehicleCommandAck for "
                        f"command {command}."
                    )
                node.condition.wait(timeout=remaining)

    def _wait_for_offboard_mode(self):
        deadline = time.monotonic() + self.TAKEOFF_MODE_TIMEOUT
        node = self.node
        max_age = self.TAKEOFF_TELEMETRY_MAX_AGE
        offboard_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD

        with node.condition:
            while True:
                now = time.monotonic()
                status_fresh = (
                    node.status_received_at is not None
                    and now - node.status_received_at <= max_age
                )
                if status_fresh and node.nav_state == offboard_state:
                    return

                remaining = deadline - now
                if remaining <= 0:
                    state = "stale/missing" if not status_fresh else str(node.nav_state)
                    raise TimeoutError(
                        "PX4 acknowledged the Offboard request but did not "
                        f"confirm fresh Offboard mode (nav_state={state})."
                    )
                node.condition.wait(timeout=remaining)

    def _require_fresh_preflight(self):
        node = self.node
        with node.condition:
            now = time.monotonic()
            status_fresh = (
                node.status_received_at is not None
                and now - node.status_received_at
                <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            if not status_fresh:
                raise RuntimeError(
                    "Cannot arm: VehicleStatus telemetry is missing or stale."
                )
            if not node.pre_flight_checks_pass:
                raise RuntimeError(
                    "Cannot arm: PX4 pre-flight checks are failing "
                    f"(latest_arming_reason={node.latest_arming_reason}, "
                    f"latest_disarming_reason={node.latest_disarming_reason})."
                )

    def _wait_for_fresh_command_status(self, timeout=None):
        """Wait outside callbacks for fresh status before a command is sent."""
        node = self.node
        if timeout is None:
            timeout = self.TAKEOFF_STATUS_UPDATE_TIMEOUT
        deadline = time.monotonic() + timeout
        with node.condition:
            while True:
                now = time.monotonic()
                status_fresh = (
                    node.status_received_at is not None
                    and now - node.status_received_at
                    <= self.TAKEOFF_TELEMETRY_MAX_AGE
                )
                if status_fresh:
                    return
                remaining = deadline - now
                if remaining <= 0:
                    raise TimeoutError(
                        "Cannot send PX4 command: VehicleStatus did not provide "
                        "a fresh update within "
                        f"{timeout:.3f} s."
                    )
                node.condition.wait(timeout=remaining)

    def _wait_for_airborne_state(self):
        node = self.node
        max_age = self.TAKEOFF_TELEMETRY_MAX_AGE
        clock_stall_timeout = self.TAKEOFF_CLOCK_STALL_TIMEOUT
        armed_state = VehicleStatus.ARMING_STATE_ARMED
        offboard_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        takeoff_start_px4_time = None
        last_px4_time = None
        wall_start_at = time.monotonic()
        last_clock_progress_at = time.monotonic()
        timeout_px4_us = int(self.TAKEOFF_TIMEOUT * 1_000_000)

        with node.condition:
            while True:
                now = time.monotonic()
                status_fresh = (
                    node.status_received_at is not None
                    and now - node.status_received_at <= max_age
                )
                status_age = (
                    now - wall_start_at
                    if node.status_received_at is None
                    else now - node.status_received_at
                )
                position_fresh = (
                    node.position_received_at is not None
                    and now - node.position_received_at <= max_age
                )
                clock_fresh = (
                    node.px4_sim_time_received_at is not None
                    and now - node.px4_sim_time_received_at
                    <= clock_stall_timeout
                )

                if not position_fresh or not node.position_valid:
                    raise TimeoutError(
                        "PX4 takeoff stopped: local-position telemetry is "
                        "stale, missing, or invalid."
                    )
                px4_sim_time = node.px4_sim_time_us
                if px4_sim_time is not None and (
                    last_px4_time is None or px4_sim_time > last_px4_time
                ):
                    last_px4_time = px4_sim_time
                    last_clock_progress_at = now
                elif (
                    px4_sim_time is not None
                    and last_px4_time is not None
                    and px4_sim_time < last_px4_time
                ):
                    raise RuntimeError(
                        "PX4 simulation clock moved backwards during takeoff."
                    )

                if takeoff_start_px4_time is None and px4_sim_time is not None:
                    takeoff_start_px4_time = px4_sim_time

                if now - last_clock_progress_at >= clock_stall_timeout:
                    raise TimeoutError(
                        "PX4 takeoff stopped: simulation clock did not "
                        "advance before the clock-stall timeout."
                    )

                if status_fresh and not node.pre_flight_checks_pass:
                    raise RuntimeError(
                        "PX4 pre-flight checks failed during takeoff "
                        f"(latest_arming_reason={node.latest_arming_reason}, "
                        f"latest_disarming_reason={node.latest_disarming_reason})."
                    )

                if now - wall_start_at >= self.TAKEOFF_WALL_TIMEOUT:
                    raise TimeoutError(
                        "PX4 takeoff wall-clock limit reached while waiting "
                        f"for the airborne state (wall elapsed="
                        f"{now - wall_start_at:.3f} s, "
                        f"PX4 elapsed={0.0 if takeoff_start_px4_time is None or px4_sim_time is None else (px4_sim_time - takeoff_start_px4_time) / 1_000_000:.3f} s, "
                        f"NED z={node.z:.3f} m, "
                        f"heading_good_for_control={node.heading_valid})."
                    )

                if (
                    clock_fresh
                    and status_fresh
                    and px4_sim_time is not None
                    and node.arming_state == armed_state
                    and node.nav_state == offboard_state
                    and node.z <= -0.8
                    and node.heading_valid
                ):
                    return

                px4_sim_elapsed_us = (
                    0
                    if takeoff_start_px4_time is None or px4_sim_time is None
                    else px4_sim_time - takeoff_start_px4_time
                )
                if (
                    takeoff_start_px4_time is not None
                    and clock_fresh
                    and px4_sim_elapsed_us >= timeout_px4_us
                ):
                    unmet = []
                    if node.arming_state != armed_state:
                        unmet.append(f"armed state (arming_state={node.arming_state})")
                    if node.nav_state != offboard_state:
                        unmet.append(f"Offboard mode (nav_state={node.nav_state})")
                    if node.z > -0.8:
                        unmet.append(f"altitude (NED z={node.z:.3f}, need <= -0.8 m)")
                    if not node.heading_valid:
                        unmet.append("final heading alignment (heading_good_for_control=false)")
                    raise TimeoutError(
                        "PX4 simulation-time takeoff limit reached while "
                        "waiting for: "
                        + ", ".join(unmet)
                        + f" (PX4 elapsed={px4_sim_elapsed_us / 1_000_000:.3f} s)."
                    )

                if (
                    not status_fresh
                    and status_age >= self.TAKEOFF_STATUS_UPDATE_TIMEOUT
                ):
                    raise TimeoutError(
                        "PX4 takeoff stopped: VehicleStatus did not provide "
                        "a fresh update within "
                        f"{self.TAKEOFF_STATUS_UPDATE_TIMEOUT:.3f} s."
                    )

                stall_remaining = clock_stall_timeout - (
                    now - last_clock_progress_at
                )
                if stall_remaining <= 0:
                    raise TimeoutError(
                        "PX4 takeoff stopped: simulation clock telemetry "
                        "was missing, stale, or did not advance before the "
                        "clock-stall timeout."
                    )
                status_wait_remaining = (
                    self.TAKEOFF_STATUS_UPDATE_TIMEOUT - status_age
                    if not status_fresh
                    else 0.1
                )
                node.condition.wait(
                    timeout=min(0.1, stall_remaining, status_wait_remaining)
                )

    def _takeoff(self):
        with self.node.condition:
            now = time.monotonic()
            position_fresh = (
                self.node.position_received_at is not None
                and now - self.node.position_received_at
                <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            if not position_fresh or not self.node.position_valid:
                raise RuntimeError(
                    "Cannot take off: PX4 local-position telemetry is "
                    "missing, stale, or invalid."
                )
            self.node.takeoff_target_x = self.node.x
            self.node.takeoff_target_y = self.node.y
            self._flight_commanded = True
            self.node.phase = "TAKEOFF"
            self.node.land_setpoints_released = False
            self.node.takeoff_setpoint_count = 0
            self.node.takeoff_stream_started_at = None
            self.node.takeoff_stream_last_at = None
            self.node.condition.notify_all()

        # PX4 must receive a sustained Offboard stream before the mode request.
        self._wait_for_takeoff_setpoint_stream()
        self._wait_for_fresh_command_status()

        mode_command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        ack_counter = self.node.request_offboard()
        self._wait_for_command_ack(mode_command, ack_counter)
        self._wait_for_offboard_mode()

        # Do not ask PX4 to arm unless its current checks explicitly pass.
        self._require_fresh_preflight()

        arm_command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        ack_counter = self.node.arm()
        self._wait_for_command_ack(arm_command, ack_counter)
        self._wait_for_airborne_state()

        # Repeatable episode start: fly (speed-limited) to the start pose.
        self._fly_to_start()

        self._enter_velocity_phase()

    def _enter_velocity_phase(self):
        """Start the policy phase at TAKEOFF_TARGET_Z. With terrain following, first record the ground under the
        (flat) start point as the reference; without a fresh downward range it raises before the phase starts, so
        the caller's safe state lands."""
        with self.node.condition:
            self.node.hold_z = self.node.TAKEOFF_TARGET_Z
            if getattr(self, "terrain_following", False):
                ground = self.node.ground_z(time.monotonic())
                if ground is None:
                    raise RuntimeError("Terrain following needs a fresh downward range at the start point.")
                self.node.terrain_ref_ground_z = ground
                self.node._terrain_last_t = None
            self.node.phase = "VELOCITY"
            self.node.condition.notify_all()

    @staticmethod
    def _observation_z(node, now):
        """z for the policy's height input: with terrain following, relative to the current ground (so it equals
        -height above ground as on the start pad); otherwise the EKF z. Caller holds the condition."""
        if getattr(node, "terrain_following", False) and getattr(node, "terrain_ref_ground_z", None) is not None:
            ground = node.ground_z(now)
            if ground is not None:
                return node.z - (ground - node.terrain_ref_ground_z)
        return node.z

    def _fly_to_start(self):
        yaw = getattr(self, "_start_yaw", self.START_YAW)      # set by reset(options); nominal otherwise
        self._fly_to(self.START_X, self.START_Y, self.node.TAKEOFF_TARGET_Z, yaw)

    def _fly_to(self, x, y, z, yaw):
        """Offboard position flight to a pose; bounded in simulation and wall time.

        Requires fresh armed OFFBOARD status and valid local position throughout. Raises on any
        failure so callers fall back to the existing safe-state landing.

        Simulation time is the PX4 HRT clock (as in takeoff), which updates about once per
        simulated second. Elapsed time is anchored on the first HRT sample received after this
        call starts, so it never exceeds the true elapsed time (the timeout cannot fire early; it
        can fire up to ~1 simulated second late). No HRT progress for TAKEOFF_CLOCK_STALL_TIMEOUT
        or a backward HRT stops the fly-to; the wall ceiling still bounds the whole call.
        """
        node = self.node
        max_age = self.TAKEOFF_TELEMETRY_MAX_AGE
        started_at = time.monotonic()
        wall_deadline = started_at + self.REPOSITION_WALL_TIMEOUT
        anchor_px4_time = None
        last_px4_time = None
        last_clock_progress_at = started_at
        with node.condition:
            node.goal = (float(x), float(y), float(z), float(yaw))
            self._flight_commanded = True
            node.phase = "REPOSITION"
            node.condition.notify_all()
            while True:
                now = time.monotonic()
                position_fresh = (
                    node.position_received_at is not None
                    and now - node.position_received_at <= max_age
                    and node.position_valid
                )
                if not position_fresh:
                    raise TimeoutError("Fly-to stopped: local position stale or invalid.")
                status_fresh = (
                    node.status_received_at is not None
                    and now - node.status_received_at <= self.TAKEOFF_STATUS_UPDATE_TIMEOUT
                )
                if status_fresh and (
                    node.arming_state != VehicleStatus.ARMING_STATE_ARMED
                    or node.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    or node.failsafe
                ):
                    raise RuntimeError(
                        "Fly-to stopped: vehicle left armed OFFBOARD "
                        f"(arming_state={node.arming_state}, nav_state={node.nav_state}, "
                        f"failsafe={node.failsafe})."
                    )
                yaw_error = abs(math.atan2(math.sin(node.heading - yaw), math.cos(node.heading - yaw)))
                arrived = (
                    math.hypot(x - node.x, y - node.y) <= self.REPOSITION_TOLERANCE
                    and abs(z - node.z) <= self.REPOSITION_Z_TOLERANCE
                    and math.hypot(node.vx, node.vy) <= self.REPOSITION_SPEED_TOLERANCE
                    and yaw_error <= self.REPOSITION_YAW_TOLERANCE
                    and node.heading_valid
                )
                if arrived:
                    return
                px4_time = node.px4_sim_time_us
                received_at = node.px4_sim_time_received_at
                if px4_time is not None and received_at is not None and received_at > started_at:
                    if last_px4_time is not None and px4_time < last_px4_time:
                        raise RuntimeError("PX4 simulation clock moved backwards during fly-to.")
                    if last_px4_time is None or px4_time > last_px4_time:
                        last_px4_time = px4_time
                        last_clock_progress_at = now
                    if anchor_px4_time is None:
                        anchor_px4_time = px4_time
                if now - last_clock_progress_at >= self.TAKEOFF_CLOCK_STALL_TIMEOUT:
                    raise TimeoutError(
                        "Fly-to stopped: PX4 simulation clock did not advance before "
                        "the clock-stall timeout."
                    )
                sim_elapsed = (
                    0.0 if anchor_px4_time is None
                    else (last_px4_time - anchor_px4_time) / 1_000_000
                )
                if sim_elapsed >= self.REPOSITION_SIM_TIMEOUT or now >= wall_deadline:
                    raise TimeoutError(
                        f"Fly-to ({x:.2f}, {y:.2f}) not reached: distance "
                        f"{math.hypot(x - node.x, y - node.y):.2f} m, yaw error "
                        f"{math.degrees(yaw_error):.1f} deg, sim {sim_elapsed:.1f} s."
                    )
                node.condition.wait(timeout=0.1)

    def _wait_until_landed(
        self,
        timeout=20.0,
        max_age=1.0,
    ):

        deadline = time.monotonic() + timeout

        with self.node.condition:
            while True:
                now = time.monotonic()

                landed_fresh = (
                    self.node.land_detected_received_at is not None
                    and now - self.node.land_detected_received_at <= max_age
                )
                status_fresh = (
                    self.node.status_received_at is not None
                    and now - self.node.status_received_at <= max_age
                )

                if (
                    landed_fresh
                    and status_fresh
                    and self.node.landed is True
                    and self.node.arming_state
                    == VehicleStatus.ARMING_STATE_DISARMED
                ):
                    self._landing_pending = False
                    self.node.landing_complete = True
                    return

                remaining = deadline - now
                if remaining <= 0:
                    raise TimeoutError(
                        "Timed out waiting for fresh PX4 landed and disarmed state."
                    )

                self.node.condition.wait(timeout=remaining)

    # ======================================================
    # Reward helpers
    # ======================================================

    def _distance_to_target(self):
        return sar_task.horizontal_distance(self.node.x, self.node.y, self.target)

    def _collision_detected(
        self,
        raw_observation,
    ):

        # 49 raw LiDAR distances
        min_lidar = float(
            np.min(
                raw_observation[:49]
            )
        )

        # Conservative collision threshold
        lidar_collision = (
            min_lidar < 0.25
        )

        # Large attitude excursion
        attitude_collision = (
            abs(self.node.roll) > 1.2
            or abs(self.node.pitch) > 1.2
        )

        return (
            lidar_collision
            or attitude_collision
            or self.node.failsafe
        )

    def _return_to_start_then_land(self):
        """Leave the target area before landing so the vehicle never lands on the target.

        Any failure during the return (stale telemetry, mode change, timeout) falls back to the
        unchanged safe-state landing at the current position.
        """
        with self.node.condition:
            self._episode_active = False
            self.node.action[:] = 0.0
        try:
            self._fly_to_start()
        except Exception as exc:
            warnings.warn(
                f"Return to start failed; landing in place instead: {exc}",
                RuntimeWarning,
                stacklevel=2,
            )
        self._enter_safe_state()

    def _land_beside_target(self):
        """Hop to BESIDE_TARGET_M from the target on the side the vehicle came from (just flown, so clear of rock),
        facing the target, then the safe-state landing. Any failure lands in place."""
        with self.node.condition:
            self._episode_active = False
            self.node.action[:] = 0.0
            x, y = self.node.x, self.node.y
        tx, ty = self.target.x_ned, self.target.y_ned
        d = math.hypot(x - tx, y - ty)
        ux, uy = ((x - tx) / d, (y - ty) / d) if d > 1e-3 else (1.0, 0.0)
        px, py = tx + ux * self.BESIDE_TARGET_M, ty + uy * self.BESIDE_TARGET_M
        try:
            self._fly_to(px, py, self.node.TAKEOFF_TARGET_Z, math.atan2(ty - py, tx - px))
        except Exception as exc:
            warnings.warn(
                f"Hop beside the target failed; landing in place instead: {exc}",
                RuntimeWarning,
                stacklevel=2,
            )
        self._enter_safe_state()

    def _confirm_ground_before_flight(self, timeout):
        """Before any flight command: bounded wait for fresh disarmed + landed telemetry.

        On success the phase becomes GROUND under the same lock and True is
        returned. Fresh armed status, a flight command issued meanwhile, or the
        timeout returns False so the caller falls through to the existing
        safe-state behavior.
        """
        deadline = time.monotonic() + timeout
        with self.node.condition:
            while True:
                if getattr(self, "_flight_commanded", True):
                    return False
                now = time.monotonic()
                status_fresh = (
                    self.node.status_received_at is not None
                    and now - self.node.status_received_at
                    <= self.TAKEOFF_TELEMETRY_MAX_AGE
                )
                if (
                    status_fresh
                    and self.node.arming_state
                    != VehicleStatus.ARMING_STATE_DISARMED
                ):
                    return False
                landed_fresh = (
                    self.node.land_detected_received_at is not None
                    and now - self.node.land_detected_received_at
                    <= self.TAKEOFF_TELEMETRY_MAX_AGE
                )
                if status_fresh and landed_fresh and self.node.landed is True:
                    self.node.phase = "GROUND"
                    self.node.condition.notify_all()
                    return True
                remaining = deadline - now
                if remaining <= 0:
                    return False
                self.node.condition.wait(timeout=min(remaining, 0.25))

    def _enter_safe_state(self):

        # Do not issue a landing command when fresh telemetry already says
        # the vehicle is disarmed. In particular, reset() can fail before
        # arming; NAV_LAND in that state can leave PX4 in AUTO_LAND and make
        # the next reset wait indefinitely for a landed indication.
        with self.node.condition:
            self._episode_active = False
            self.node.action[:] = 0.0
            status_fresh = (
                self.node.status_received_at is not None
                and time.monotonic() - self.node.status_received_at
                <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            is_disarmed = (
                status_fresh
                and self.node.arming_state
                == VehicleStatus.ARMING_STATE_DISARMED
            )
            landed_fresh = (
                self.node.land_detected_received_at is not None
                and time.monotonic() - self.node.land_detected_received_at
                <= self.TAKEOFF_TELEMETRY_MAX_AGE
            )
            confirmed_ground = (
                is_disarmed and landed_fresh and self.node.landed is True
            )
            # An env built without __init__ counts as having commanded
            # flight, which keeps the existing safe-state path.
            never_commanded = not getattr(self, "_flight_commanded", True)

        # If this env never commanded flight, PX4's slow status/land-detected
        # publication (often > 1 s apart in SITL) must not turn an
        # unconfirmed-but-grounded vehicle into a NAV_LAND request. Never wait
        # on the spin thread: it delivers the telemetry being waited for.
        if (
            not confirmed_ground
            and never_commanded
            and threading.current_thread()
            is not getattr(self, "spin_thread", None)
            and self._confirm_ground_before_flight(
                self.GROUND_CONFIRM_TIMEOUT
            )
        ):
            return

        if confirmed_ground:
            with self.node.condition:
                self.node.phase = "GROUND"
                self.node.condition.notify_all()
            return

        # For armed or unknown/stale state, LAND streams zero horizontal
        # velocity while PX4 performs its normal landing sequence.
        with self.node.condition:
            self.node.phase = "LAND"

            # Track an in-flight publication separately from a successful
            # send. A publisher exception must leave a later safe-state call
            # able to retry the landing request.
            request_land = (
                not is_disarmed
                and not self._landing_pending
                and not self._landing_request_in_progress
            )
            if request_land:
                self._landing_request_in_progress = True
            self.node.condition.notify_all()

        if request_land:
            try:
                self.node.land()
            except Exception:
                with self.node.condition:
                    self._landing_request_in_progress = False
                    self.node.condition.notify_all()
                raise
            with self.node.condition:
                self._landing_request_in_progress = False
                self._landing_pending = True
                self.node.condition.notify_all()

    # ======================================================
    # Gymnasium reset
    # ======================================================

    def reset(
        self,
        *,
        seed=None,
        options=None,
    ):

        with self._close_lock:
            if self._closed or self._close_waiting:
                raise RuntimeError(
                    "Cannot reset while environment shutdown is in progress."
                )

        super().reset(seed=seed)
        # Validated before anything is commanded: a bad request never takes off.
        offset = sar_task.start_yaw_offset(options)
        self._start_yaw = sar_task.wrap_angle(self.START_YAW + offset)

        self._episode_active = False
        self.current_step = 0
        self.last_observation_received_at = None

        self.previous_action[:] = 0.0
        with self.node.condition:
            self.node.action[:] = 0.0
            # A new episode never inherits the previous clock (including a sticky
            # invalidation): it must re-validate from frames received from now on.
            self.node.reset_camera_clock()

        # For the FIRST integration test we assume a
        # clean / landed PX4 simulation.
        #
        # We do not automatically recover a crashed
        # vehicle here.

        try:
            self._wait_until_landed(
                timeout=self.LAND_WAIT_TIMEOUT
            )

            # Refuse before arming unless the action clock is running (no fallback).
            self._wait_for_camera_clock()

            self._takeoff()

            self.builder.reset()
            raw_observation = (
                self._wait_for_observation()
            )
            observation = self._convert_observation(
                raw_observation
            )

            info = {
                "observation_version": sar_observation.VERSION,
                "target_detected": bool(observation[sar_observation.SLICES["det_now"]][0]),
                "privileged": self._privileged_info(),
            }

            self._episode_active = True

            return observation, info
        except BaseException:
            try:
                self._enter_safe_state()
            except BaseException:
                pass
            raise

    # ======================================================
    # Gymnasium step
    # ======================================================

    def step(self, action):

        if not self._episode_active:
            raise RuntimeError(
                "Episode is not active. Call reset() and wait for it to succeed before step()."
            )

        action = np.asarray(
            action,
            dtype=np.float32,
        )

        action = np.clip(
            action,
            self.action_space.low,
            self.action_space.high,
        )

        old_distance = (
            self._distance_to_target()
        )

        try:
            # Send action to PX4 controller; the step lasts STEP_DURATION of
            # simulation (camera-clock) time after the first camera frame received
            # once the action is applied, and ends on the first fresh LiDAR sectors
            # after that. Without a valid running clock the step is not timed at
            # all: refuse before applying the action (safe state below).
            with self.node.condition:
                clock_problem = self._camera_clock_problem(time.monotonic())
                if clock_problem is not None:
                    raise RuntimeError(f"Cannot time the action step: simulation clock {clock_problem}.")
                self.node.action[:] = action
                previous_counter = self.node.observation_counter
                self._step_applied_at = time.monotonic()

            return self._step_after_action(
                action,
                old_distance,
                previous_counter,
            )
        except BaseException:
            try:
                self._enter_safe_state()
            except BaseException:
                pass
            raise

    def _step_after_action(
        self,
        action,
        old_distance,
        previous_counter,
    ):

        # Wait for a fresh observation after the action's simulation-time duration.
        raw_observation = (
            self._wait_for_observation(
                timeout=2.0,
                previous_counter=previous_counter,
                step_applied_at=getattr(self, "_step_applied_at", None),
            )
        )

        self.current_step += 1

        new_distance = self._distance_to_target()
        collision = self._collision_detected(raw_observation)
        outcome = sar_task.step_outcome(
            old_distance,
            new_distance,
            collision,
            self.current_step,
            dataclasses.replace(self.task_config, max_steps=self.max_steps),
        )

        self.previous_action = action.copy()
        observation = self._convert_observation(raw_observation)

        if getattr(self, "land_beside_target", False) and outcome.reason == "success":
            self._land_beside_target()
        elif getattr(self, "land_beside_target", False) and outcome.reason == "timeout":
            self._enter_safe_state()                     # no straight-line return through rock: land where we are
        elif outcome.reason in ("success", "timeout"):
            self._return_to_start_then_land()
        elif outcome.terminated or outcome.truncated:
            # Collision (proximity, attitude, or failsafe): land where we are.
            self._enter_safe_state()

        info = {
            "observation_version": sar_observation.VERSION,
            "reason": outcome.reason,
            "collision": collision,
            "reached_target": outcome.reason == "success",
            "min_lidar": float(np.min(np.asarray(raw_observation)[:sar_observation.LIDAR_SECTORS])),
            "target_detected": bool(observation[sar_observation.SLICES["det_now"]][0]),
            "target_estimate_valid": bool(observation[sar_observation.SLICES["est_valid"]][0]),
            "roll": self.node.roll,
            "pitch": self.node.pitch,
            "step": self.current_step,
            "privileged": self._privileged_info(),
        }

        return (
            observation,
            float(outcome.reward),
            outcome.terminated,
            outcome.truncated,
            info,
        )

    # ======================================================
    # Render
    # ======================================================

    def render(self):
        pass

    # ======================================================
    # Close
    # ======================================================

    def _cleanup_ros(self):

        node = getattr(self, "node", None)
        spin_thread = getattr(self, "spin_thread", None)

        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                warnings.warn(
                    f"Could not destroy the PX4 ROS node during close: {exc}",
                    RuntimeWarning,
                    stacklevel=2,
                )

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            warnings.warn(
                f"Could not shut down rclpy during close: {exc}",
                RuntimeWarning,
                stacklevel=2,
            )

        if spin_thread is not None and spin_thread.is_alive():
            if threading.current_thread() is spin_thread:
                return
            try:
                spin_thread.join(timeout=1.0)
            except Exception as exc:
                warnings.warn(
                    f"Could not join the ROS spin thread during close: {exc}",
                    RuntimeWarning,
                    stacklevel=2,
                )

    def _finish_close_after_landing(self):

        # Each wait is bounded, while this non-daemon monitor keeps ROS alive
        # until fresh landed-and-disarmed telemetry arrives.
        stuck = {"since": None, "attempts": 0, "last": None}
        while True:
            try:
                self._wait_until_landed(timeout=5.0)
                break
            except TimeoutError:
                self._check_stuck_landing(stuck)
                continue
            except Exception as exc:
                warnings.warn(
                    "Landing telemetry is still unavailable during close; "
                    f"ROS control remains active: {exc}",
                    RuntimeWarning,
                    stacklevel=2,
                )
                time.sleep(1.0)

        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._close_waiting = False

        self._cleanup_ros()

    def _check_stuck_landing(self, stuck, now=None):
        """Alert and force-disarm a vehicle that rests on something without PX4 declaring it landed (see
        STUCK_LANDING_S). `stuck` carries the state between calls; returns True when a disarm was sent."""
        node = self.node
        now = time.monotonic() if now is None else now
        with node.condition:
            resting = (
                node.ground_contact is True
                and node.landed is not True
                and node.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and abs(getattr(node, "vz", 0.0)) < 0.15
                and math.hypot(node.vx, node.vy) < 0.2
            )
        if not resting:
            stuck["since"] = None
            return False
        if stuck["since"] is None:
            stuck["since"] = now
            return False
        due = stuck["last"] is None or now - stuck["last"] >= self.STUCK_LANDING_S
        if now - stuck["since"] < self.STUCK_LANDING_S or not due or stuck["attempts"] >= self.STUCK_LANDING_ATTEMPTS:
            return False
        stuck["attempts"] += 1
        stuck["last"] = now
        warnings.warn(
            f"STUCK LANDING: resting with ground contact for {now - stuck['since']:.0f} s, armed, not landed "
            f"(off the solid pads?); sending forced disarm {stuck['attempts']}/{self.STUCK_LANDING_ATTEMPTS}",
            RuntimeWarning,
            stacklevel=2,
        )
        try:
            node.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0, require_fresh_status=False
            )
        except Exception as exc:
            warnings.warn(f"STUCK LANDING: forced disarm could not be sent: {exc}", RuntimeWarning, stacklevel=2)
            return False
        return True

    def _schedule_cleanup_after_landing(self):

        with self._close_lock:
            thread = self._landing_cleanup_thread
            if thread is not None and thread.is_alive():
                return
            thread = threading.Thread(
                target=self._finish_close_after_landing,
                name="px4_landing_cleanup",
                daemon=False,
            )
            self._landing_cleanup_thread = thread
            thread.start()

    def close(self):

        with self._close_lock:
            if self._closed or self._close_waiting:
                return
            self._close_waiting = True

        node = getattr(self, "node", None)
        spin_thread = getattr(self, "spin_thread", None)

        if node is None:
            with self._close_lock:
                self._closed = True
                self._close_waiting = False
            self._cleanup_ros()
            return

        wait_for_landing = node is not None
        if node is not None:
            try:
                self._enter_safe_state()
            except Exception as exc:
                warnings.warn(
                    f"Could not fully enter the PX4 safe state during close: {exc}",
                    RuntimeWarning,
                    stacklevel=2,
                )

            if threading.current_thread() is spin_thread:
                warnings.warn(
                    "close() was called from the ROS spin thread; "
                    "landing confirmation will continue in a background monitor.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                wait_for_landing = False
            else:
                try:
                    self._wait_until_landed(
                        timeout=self.LAND_WAIT_TIMEOUT
                    )
                except TimeoutError as exc:
                    warnings.warn(
                        "Landing was not confirmed before close timed out; "
                        f"ROS control remains active: {exc}",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    wait_for_landing = False
                except Exception as exc:
                    warnings.warn(
                        "Landing could not be confirmed during close; "
                        f"ROS control remains active: {exc}",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    wait_for_landing = False

        if not wait_for_landing:
            self._schedule_cleanup_after_landing()
            return

        with self._close_lock:
            self._closed = True
            self._close_waiting = False

        self._cleanup_ros()
