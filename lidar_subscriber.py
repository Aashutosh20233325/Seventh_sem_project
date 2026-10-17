import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


TOPIC = (
    "/world/walls/model/x500_lidar_2d_0/link/link/"
    "sensor/lidar_2d_v2/scan"
)


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")

        self.subscription = self.create_subscription(
            LaserScan,
            TOPIC,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Listening to LiDAR scan topic...")

    def scan_callback(self, msg):
        # Replace invalid or infinite readings with the sensor's max range.
        ranges = [
            r if math.isfinite(r) else msg.range_max
            for r in msg.ranges
        ]

        # Divide the scan into 12 angular sectors.
        sector_size = max(1, len(ranges) // 12)
        sector_minima = []

        for i in range(0, len(ranges), sector_size):
            sector = ranges[i:i + sector_size]
            sector_minima.append(min(sector))

        self.get_logger().info(
            f"rays={len(msg.ranges)}, "
            f"min_distance={min(ranges):.2f} m, "
            f"12-sector minima={[round(x, 2) for x in sector_minima]}"
        )


def main():
    rclpy.init()
    node = LidarSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
