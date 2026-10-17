import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


SCAN_TOPIC = (
    "/world/walls/model/x500_lidar_2d_0/"
    "link/link/sensor/lidar_2d_v2/scan"
)
OBS_TOPIC = "/uav/lidar_observation"
NUM_SECTORS = 12


class LidarObservationPublisher(Node):
    def __init__(self):
        super().__init__("lidar_observation_publisher")

        self.publisher = self.create_publisher(
            Float32MultiArray, OBS_TOPIC, 10
        )
        self.subscription = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, 10
        )

        self.get_logger().info(f"Listening to: {SCAN_TOPIC}")
        self.get_logger().info(f"Publishing observations to: {OBS_TOPIC}")

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        if not ranges:
            return

        # Replace invalid readings with the sensor's maximum range,
        # and clamp all valid readings to the sensor range.
        cleaned = []
        for distance in ranges:
            if not math.isfinite(distance):
                distance = msg.range_max

            distance = max(msg.range_min, min(distance, msg.range_max))
            cleaned.append(distance)

        # Divide the scan into 12 equal sectors and take
        # the nearest distance in each sector.
        observations = []
        n = len(cleaned)

        for i in range(NUM_SECTORS):
            start = i * n // NUM_SECTORS
            end = (i + 1) * n // NUM_SECTORS
            sector = cleaned[start:end]
            observations.append(min(sector))

        output = Float32MultiArray()
        output.data = [float(x) for x in observations]
        self.publisher.publish(output)

        self.get_logger().info(
            "12-sector distances (m): "
            + str([round(x, 2) for x in observations])
        )


def main():
    rclpy.init()
    node = LidarObservationPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
