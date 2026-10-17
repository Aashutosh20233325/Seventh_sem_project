import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


class PX4ActionTest(Node):

    def __init__(self):
        super().__init__("px4_action_test")

        # Publishers
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

        # PX4 outputs use Best Effort QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Actual UAV local position
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos,
        )

        self.current_z = None

        # We only switch after actual altitude is reached
        self.airborne = False

        # Require several consecutive altitude measurements
        # below the threshold to avoid switching due to noise.
        self.airborne_count = 0

        self.counter = 0
        self.last_phase = ""

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

    # --------------------------------------------------
    # PX4 local position callback
    # --------------------------------------------------

    def position_callback(self, msg):

        self.current_z = msg.z

        # PX4 NED:
        # z = 0     -> local origin
        # z < 0     -> above origin
        #
        # Wait until actual UAV altitude is at least
        # about 0.8 m above the local origin.

        if not self.airborne and self.current_z <= -0.8:

            self.airborne_count += 1

            # Require ~0.5 sec of stable measurements
            if self.airborne_count >= 5:

                self.airborne = True

                self.get_logger().info(
                    f"ACTUAL ALTITUDE REACHED: "
                    f"z={self.current_z:.2f} m"
                )

                self.get_logger().info(
                    "Switching to VELOCITY CONTROL"
                )

        else:

            if not self.airborne:
                self.airborne_count = 0

    # --------------------------------------------------
    # Timestamp
    # --------------------------------------------------

    def timestamp(self):

        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # --------------------------------------------------
    # OffboardControlMode
    # --------------------------------------------------

    def publish_offboard_mode(self, position_mode):

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

    # --------------------------------------------------
    # Takeoff setpoint
    # --------------------------------------------------

    def publish_takeoff_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # PX4 NED:
        # -2 m = approximately 2 m above local origin
        msg.position = [
            0.0,
            0.0,
            -2.0,
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

        msg.yaw = float("nan")
        msg.yawspeed = float("nan")

        self.setpoint_pub.publish(msg)

    # --------------------------------------------------
    # Velocity setpoint
    # --------------------------------------------------

    def publish_velocity_setpoint(self):

        # This is our RL-style action.
        speed = 1.0
        turn = 0.0

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        # Position controller disabled
        msg.position = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

        # PX4 NED/world-frame velocity
        msg.velocity = [
            speed,
            0.0,
            0.0,
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

        msg.yaw = float("nan")

        # Turn -> yaw rate
        msg.yawspeed = turn

        self.setpoint_pub.publish(msg)

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------

    def control_loop(self):

        # ----------------------------------------------
        # PHASE 1: TAKEOFF
        # ----------------------------------------------

        if not self.airborne:

            if self.last_phase != "TAKEOFF":

                self.get_logger().info(
                    "PHASE 1: POSITION TAKEOFF"
                )

                self.last_phase = "TAKEOFF"

            self.publish_offboard_mode(
                position_mode=True
            )

            self.publish_takeoff_setpoint()

        # ----------------------------------------------
        # PHASE 2: VELOCITY CONTROL
        # ----------------------------------------------

        else:

            if self.last_phase != "VELOCITY":

                self.get_logger().info(
                    "PHASE 2: VELOCITY CONTROL "
                    "speed=1.0 turn=0.0"
                )

                self.last_phase = "VELOCITY"

            self.publish_offboard_mode(
                position_mode=False
            )

            self.publish_velocity_setpoint()

        # ----------------------------------------------
        # OFFBOARD
        # ----------------------------------------------

        if self.counter == 15:

            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )

            self.get_logger().info(
                "Requested OFFBOARD"
            )

        # ----------------------------------------------
        # ARM
        # ----------------------------------------------

        if self.counter == 20:

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )

            self.get_logger().info(
                "Requested ARM"
            )

        self.counter += 1

    # --------------------------------------------------
    # VehicleCommand helper
    # --------------------------------------------------

    def send_command(
        self,
        command,
        param1,
        param2=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)


def main():

    rclpy.init()

    node = PX4ActionTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
