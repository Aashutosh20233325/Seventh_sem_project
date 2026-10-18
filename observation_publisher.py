"""Publish 49 horizontal-LiDAR obstacle sectors (metres) on /uav/lidar_sectors.

LiDAR-only by design: this node reads no Gazebo poses and no target coordinates. Target information for
the policy comes from target_perception_node (camera), and ground truth stays in sar_task.
"""

import math
import os
import threading
import time

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


UAV_MODEL_NAME = os.environ.get("SAR_UAV_MODEL_NAME", "x500_lidar_2d_0")
LIDAR_TOPIC = os.environ.get(
    "SAR_LIDAR_TOPIC",
    f"/world/walls/model/{UAV_MODEL_NAME}/link/link/sensor/lidar_2d_v2/scan",
)

OUTPUT_TOPIC = "/uav/lidar_sectors"

NUM_SECTORS = 49
INPUT_MAX_AGE = 0.5


class ObservationPublisher(Node):

    def __init__(self):
        super().__init__("lidar_sector_publisher")

        self.scan = None
        self._input_lock = threading.Lock()
        self.scan_valid = False
        self.scan_received_at = None

        self.create_subscription(
            LaserScan,
            LIDAR_TOPIC,
            self.lidar_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            OUTPUT_TOPIC,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_observation
        )

        print("49-sector LiDAR publisher started")
        print(f"Publishing on: {OUTPUT_TOPIC}")

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
    # Publish 49 sectors
    # ======================================================

    def publish_observation(self):
        now = time.monotonic()
        with self._input_lock:
            fresh = (
                self.scan_valid
                and self.scan_received_at is not None
                and 0.0 <= now - self.scan_received_at <= INPUT_MAX_AGE
            )
            scan = self.scan

        if not fresh:
            return

        sectors = self.get_49_lidar(scan)

        if sectors is None or len(sectors) != NUM_SECTORS or not all(
            math.isfinite(float(value)) for value in sectors
        ):
            self.get_logger().error("Invalid LiDAR sectors; not published")
            return

        msg = Float32MultiArray()
        msg.data = [float(value) for value in sectors]
        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = ObservationPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
