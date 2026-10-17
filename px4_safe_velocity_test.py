import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


class SafeVelocityTest(Node):

    def __init__(self):
        super().__init__("safe_velocity_test")

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

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos,
        )

        self.z = None
        self.airborne = False
        self.airborne_count = 0
        self.velocity_start = None

        self.counter = 0

        self.timer = self.create_timer(
            0.1,
            self.control_loop,
        )

    def timestamp(self):
        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    def position_callback(self, msg):

        self.z = msg.z

        if not self.airborne and self.z <= -0.8:

            self.airborne_count += 1

            if self.airborne_count >= 5:

                self.airborne = True
                self.velocity_start = time.monotonic()

                self.get_logger().info(
                    f"TAKEOFF CONFIRMED: z={self.z:.2f}"
                )

                self.get_logger().info(
                    "STARTING SAFE VELOCITY TEST: "
                    "vx=-0.3 m/s"
                )

        elif not self.airborne:

            self.airborne_count = 0

    def publish_offboard(self, position_mode):

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

    def publish_position(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

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

    def publish_velocity(self):

        elapsed = time.monotonic() - self.velocity_start

        # Move only 0.3 m/s in negative X direction
        # for 2 seconds.
        if elapsed < 2.0:
            speed = -0.3
        else:
            speed = 0.0

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]

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
        msg.yawspeed = 0.0

        self.setpoint_pub.publish(msg)

        if abs(elapsed - 2.0) < 0.05:
            self.get_logger().info(
                "Velocity test complete -> holding position"
            )

    def control_loop(self):

        if not self.airborne:

            self.publish_offboard(
                position_mode=True
            )

            self.publish_position()

        else:

            self.publish_offboard(
                position_mode=False
            )

            self.publish_velocity()

        if self.counter == 15:

            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0,
            )

            self.get_logger().info(
                "Requested OFFBOARD"
            )

        if self.counter == 20:

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
            )

            self.get_logger().info(
                "Requested ARM"
            )

        self.counter += 1

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

    node = SafeVelocityTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
