import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LiDAR49Publisher(Node):

    def __init__(self):
        super().__init__('lidar_49_publisher')

        self.num_sectors = 49

        self.subscription = self.create_subscription(
            LaserScan,
            '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/uav/lidar_49',
            10
        )

        self.get_logger().info(
            '49-sector LiDAR publisher started'
        )

    def scan_callback(self, msg):

        sector_ranges = [
            msg.range_max
        ] * self.num_sectors

        sector_width = (
            (msg.angle_max - msg.angle_min)
            / self.num_sectors
        )

        for i, distance in enumerate(msg.ranges):

            # Ignore invalid readings
            if not math.isfinite(distance):
                distance = msg.range_max

            # Clamp to valid LiDAR range
            distance = max(
                msg.range_min,
                min(distance, msg.range_max)
            )

            # Angle of this LiDAR ray
            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # Convert angle to sector index
            sector = int(
                (angle - msg.angle_min)
                / sector_width
            )

            # Protect against the final boundary
            if sector >= self.num_sectors:
                sector = self.num_sectors - 1

            # Minimum distance in this sector
            if distance < sector_ranges[sector]:
                sector_ranges[sector] = distance

        output = Float32MultiArray()
        output.data = sector_ranges

        self.publisher.publish(output)


def main():

    rclpy.init()

    node = LiDAR49Publisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
