import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

import gz.transport13
from gz.msgs10.pose_v_pb2 import Pose_V


LIDAR_TOPIC = "/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan"
POSE_TOPIC = "/world/walls/pose/info"

OUTPUT_TOPIC = "/uav/observation"

NUM_SECTORS = 49


class ObservationPublisher(Node):

    def __init__(self):
        super().__init__("observation_publisher")

        self.uav_x = None
        self.uav_y = None

        self.target_x = None
        self.target_y = None

        self.scan = None

        # --------------------------------------------------
        # PX4 QoS
        # --------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos
        )

        self.create_subscription(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback,
            10
        )

        # --------------------------------------------------
        # Gazebo target position
        # --------------------------------------------------

        self.gz_node = gz.transport13.Node()

        self.gz_node.subscribe(
            Pose_V,
            POSE_TOPIC,
            self.pose_callback
        )

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------

        self.publisher = self.create_publisher(
            Float32MultiArray,
            OUTPUT_TOPIC,
            10
        )

        # --------------------------------------------------
        # Previous action
        #
        # Initially zero.
        # Later the RL controller will update these.
        # --------------------------------------------------

        self.previous_speed = 0.0
        self.previous_turn = 0.0

        self.timer = self.create_timer(
            0.1,
            self.publish_observation
        )

        print("54-D observation publisher started")
        print(f"Publishing on: {OUTPUT_TOPIC}")

    # ======================================================
    # PX4 UAV position
    # ======================================================

    def position_callback(self, msg):

        self.uav_x = msg.x
        self.uav_y = msg.y

    # ======================================================
    # LiDAR
    # ======================================================

    def lidar_callback(self, msg):

        self.scan = msg

    # ======================================================
    # Gazebo target
    # ======================================================

    def pose_callback(self, msg):

        for pose in msg.pose:

            if pose.name == "target":

                self.target_x = pose.position.x
                self.target_y = pose.position.y

                break

    # ======================================================
    # Convert 1080 LiDAR rays -> 49 sectors
    # ======================================================

    def get_49_lidar(self):

        if self.scan is None:
            return None

        ranges = self.scan.ranges

        sector_width = (
            (self.scan.angle_max - self.scan.angle_min)
            / NUM_SECTORS
        )

        lidar_49 = []

        for sector in range(NUM_SECTORS):

            start_angle = (
                self.scan.angle_min
                + sector * sector_width
            )

            end_angle = (
                start_angle
                + sector_width
            )

            start_index = max(
                0,
                int(
                    (start_angle - self.scan.angle_min)
                    / self.scan.angle_increment
                )
            )

            end_index = min(
                len(ranges),
                int(
                    (end_angle - self.scan.angle_min)
                    / self.scan.angle_increment
                ) + 1
            )

            sector_ranges = []

            for i in range(start_index, end_index):

                r = ranges[i]

                if not math.isfinite(r):
                    r = self.scan.range_max

                r = max(
                    self.scan.range_min,
                    min(r, self.scan.range_max)
                )

                sector_ranges.append(r)

            if sector_ranges:
                lidar_49.append(min(sector_ranges))
            else:
                lidar_49.append(self.scan.range_max)

        return lidar_49

    # ======================================================
    # Target detection
    # ======================================================

    def get_target_values(self):

        if (
            self.uav_x is None
            or self.uav_y is None
            or self.target_x is None
            or self.target_y is None
            or self.scan is None
        ):
            return 0.0, 0.0, 0.0

        dx = self.target_x - self.uav_x
        dy = self.target_y - self.uav_y

        target_distance = math.sqrt(
            dx * dx + dy * dy
        )

        target_angle = math.atan2(dy, dx)

        # --------------------------------------------------
        # Check whether target lies inside LiDAR FOV
        # --------------------------------------------------

        if not (
            self.scan.angle_min
            <= target_angle
            <= self.scan.angle_max
        ):
            return 0.0, 0.0, 0.0

        # --------------------------------------------------
        # Find LiDAR ray corresponding to target
        # --------------------------------------------------

        center_index = round(
            (
                target_angle
                - self.scan.angle_min
            )
            / self.scan.angle_increment
        )

        search_radius = 5

        start = max(
            0,
            center_index - search_radius
        )

        end = min(
            len(self.scan.ranges),
            center_index + search_radius + 1
        )

        best_range = float("inf")
        best_index = None

        for i in range(start, end):

            r = self.scan.ranges[i]

            if not math.isfinite(r):
                continue

            if (
                r < self.scan.range_min
                or r > self.scan.range_max
            ):
                continue

            if r < best_range:

                best_range = r
                best_index = i

        if best_index is None:
            return 0.0, 0.0, 0.0

        measured_angle = (
            self.scan.angle_min
            + best_index * self.scan.angle_increment
        )

        # Our target sphere radius = 0.5 m.
        expected_surface_distance = (
            target_distance - 0.5
        )

        distance_error = abs(
            best_range
            - expected_surface_distance
        )

        angle_error = abs(
            measured_angle
            - target_angle
        )

        # --------------------------------------------------
        # Target considered visible
        # --------------------------------------------------

        if (
            distance_error < 0.35
            and angle_error < math.radians(3)
        ):

            return (
                1.0,
                best_range,
                measured_angle
            )

        return 0.0, 0.0, 0.0

    # ======================================================
    # Publish 54-dimensional observation
    # ======================================================

    def publish_observation(self):

        lidar_49 = self.get_49_lidar()

        if lidar_49 is None:
            return

        target_detected, target_distance, target_angle = (
            self.get_target_values()
        )

        observation = (
            lidar_49
            + [
                target_detected,
                target_distance,
                target_angle,
                self.previous_speed,
                self.previous_turn
            ]
        )

        # --------------------------------------------------
        # Safety check
        # --------------------------------------------------

        if len(observation) != 54:

            self.get_logger().error(
                f"Wrong observation size: {len(observation)}"
            )

            return

        msg = Float32MultiArray()

        msg.data = observation

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = ObservationPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()


