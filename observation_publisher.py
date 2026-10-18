import math
import os
import threading
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

import gz.transport13
from gz.msgs10.pose_v_pb2 import Pose_V

from observation_geometry import (
    quaternion_yaw,
    required_inputs_fresh,
    target_bearing_in_sensor_frame,
)


UAV_MODEL_NAME = os.environ.get("SAR_UAV_MODEL_NAME", "x500_lidar_2d_0")
LIDAR_TOPIC = os.environ.get(
    "SAR_LIDAR_TOPIC",
    f"/world/walls/model/{UAV_MODEL_NAME}/link/link/sensor/lidar_2d_v2/scan",
)
POSE_TOPIC = "/world/walls/pose/info"

OUTPUT_TOPIC = "/uav/observation"

NUM_SECTORS = 49
TARGET_MODEL_NAME = "target"
# The model SDF gives the fixed lidar joint and sensor a zero yaw offset.
LIDAR_YAW_OFFSET = 0.0
INPUT_MAX_AGE = 0.5


class ObservationPublisher(Node):

    def __init__(self):
        super().__init__("observation_publisher")

        self.uav_x = None
        self.uav_y = None

        self.target_x = None
        self.target_y = None

        self.scan = None

        self._input_lock = threading.Lock()
        self.uav_yaw = None
        self.scan_valid = False
        self.pose_valid = False
        self.position_valid = False
        self.scan_received_at = None
        self.pose_received_at = None
        self.position_received_at = None

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
        now = time.monotonic()
        try:
            valid = (
                bool(msg.xy_valid)
                and math.isfinite(float(msg.x))
                and math.isfinite(float(msg.y))
            )
        except (AttributeError, TypeError, ValueError):
            valid = False

        with self._input_lock:
            self.position_received_at = now
            self.position_valid = valid

    # ======================================================
    # LiDAR
    # ======================================================

    def lidar_callback(self, msg):
        now = time.monotonic()
        try:
            scan_parameters = (
                float(msg.angle_min),
                float(msg.angle_max),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
            )
            valid = (
                len(msg.ranges) > 0
                and all(math.isfinite(value) for value in scan_parameters)
                and scan_parameters[1] > scan_parameters[0]
                and scan_parameters[2] > 0.0
                and scan_parameters[3] >= 0.0
                and scan_parameters[4] >= scan_parameters[3]
            )
        except (AttributeError, TypeError, ValueError):
            valid = False

        with self._input_lock:
            self.scan = msg
            self.scan_valid = valid
            self.scan_received_at = now

    # ======================================================
    # Gazebo target
    # ======================================================

    def pose_callback(self, msg):
        now = time.monotonic()
        uav_pose = None
        target_pose = None
        for pose in msg.pose:
            if pose.name == UAV_MODEL_NAME:
                uav_pose = pose
            elif pose.name == TARGET_MODEL_NAME:
                target_pose = pose

        valid = uav_pose is not None and target_pose is not None
        if valid:
            try:
                uav_x = float(uav_pose.position.x)
                uav_y = float(uav_pose.position.y)
                uav_yaw = quaternion_yaw(
                    uav_pose.orientation.x,
                    uav_pose.orientation.y,
                    uav_pose.orientation.z,
                    uav_pose.orientation.w,
                )
                target_x = float(target_pose.position.x)
                target_y = float(target_pose.position.y)
                valid = all(math.isfinite(value) for value in (
                    uav_x, uav_y, uav_yaw, target_x, target_y
                ))
            except (AttributeError, TypeError, ValueError):
                valid = False

        with self._input_lock:
            self.pose_received_at = now
            self.pose_valid = valid
            if valid:
                self.uav_x = uav_x
                self.uav_y = uav_y
                self.uav_yaw = uav_yaw
                self.target_x = target_x
                self.target_y = target_y

    # ======================================================
    # Convert 1080 LiDAR rays -> 49 sectors
    # ======================================================

    def get_49_lidar(self, scan=None):

        scan = self.scan if scan is None else scan
        if scan is None:
            return None

        ranges = scan.ranges

        sector_width = (
            (scan.angle_max - scan.angle_min)
            / NUM_SECTORS
        )

        lidar_49 = []

        for sector in range(NUM_SECTORS):

            start_angle = (
                scan.angle_min
                + sector * sector_width
            )

            end_angle = (
                start_angle
                + sector_width
            )

            start_index = max(
                0,
                int(
                    (start_angle - scan.angle_min)
                    / scan.angle_increment
                )
            )

            end_index = min(
                len(ranges),
                int(
                    (end_angle - scan.angle_min)
                    / scan.angle_increment
                ) + 1
            )

            sector_ranges = []

            for i in range(start_index, end_index):

                r = ranges[i]

                if not math.isfinite(r):
                    r = scan.range_max

                r = max(
                    scan.range_min,
                    min(r, scan.range_max)
                )

                sector_ranges.append(r)

            if sector_ranges:
                lidar_49.append(min(sector_ranges))
            else:
                lidar_49.append(scan.range_max)

        return lidar_49

    # ======================================================
    # Target detection
    # ======================================================

    def get_target_values(self, scan=None, pose=None):

        scan = self.scan if scan is None else scan
        if pose is None:
            pose = (
                self.uav_x,
                self.uav_y,
                self.uav_yaw,
                self.target_x,
                self.target_y,
            )

        if (
            scan is None
            or any(value is None for value in pose)
        ):
            return 0.0, 0.0, 0.0

        uav_x, uav_y, uav_yaw, target_x, target_y = pose
        dx = target_x - uav_x
        dy = target_y - uav_y

        target_distance = math.sqrt(
            dx * dx + dy * dy
        )

        target_angle = target_bearing_in_sensor_frame(
            uav_x,
            uav_y,
            uav_yaw,
            target_x,
            target_y,
            sensor_yaw=LIDAR_YAW_OFFSET,
        )

        # --------------------------------------------------
        # Check whether target lies inside LiDAR FOV
        # --------------------------------------------------

        if not (
            scan.angle_min
            <= target_angle
            <= scan.angle_max
        ):
            return 0.0, 0.0, 0.0

        # --------------------------------------------------
        # Find LiDAR ray corresponding to target
        # --------------------------------------------------

        center_index = round(
            (
                target_angle
                - scan.angle_min
            )
            / scan.angle_increment
        )

        search_radius = 5

        start = max(
            0,
            center_index - search_radius
        )

        end = min(
            len(scan.ranges),
            center_index + search_radius + 1
        )

        best_range = float("inf")
        best_index = None

        for i in range(start, end):

            r = scan.ranges[i]

            if not math.isfinite(r):
                continue

            if (
                r < scan.range_min
                or r > scan.range_max
            ):
                continue

            if r < best_range:

                best_range = r
                best_index = i

        if best_index is None:
            return 0.0, 0.0, 0.0

        measured_angle = (
            scan.angle_min
            + best_index * scan.angle_increment
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
        now = time.monotonic()
        with self._input_lock:
            inputs_fresh = required_inputs_fresh(
                now,
                self.scan_received_at,
                self.pose_received_at,
                self.position_received_at,
                self.scan_valid,
                self.pose_valid,
                self.position_valid,
                max_age=INPUT_MAX_AGE,
            )
            scan = self.scan
            pose = (
                self.uav_x,
                self.uav_y,
                self.uav_yaw,
                self.target_x,
                self.target_y,
            )

        if not inputs_fresh:
            return

        lidar_49 = self.get_49_lidar(scan)

        if lidar_49 is None:
            return

        target_detected, target_distance, target_angle = (
            self.get_target_values(scan, pose)
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

        if len(observation) != 54 or not all(
            math.isfinite(float(value)) for value in observation
        ):

            self.get_logger().error(
                f"Wrong observation size: {len(observation)}"
            )

            return

        msg = Float32MultiArray()

        msg.data = [float(value) for value in observation]

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


