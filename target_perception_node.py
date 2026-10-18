"""ROS 2 wrapper for target_perception: OAK-D Lite RGB + depth -> /uav/target_perception.

Subscribes only to camera images and camera_info. It does not subscribe to Gazebo poses or any
ground truth.

Publishes std_msgs/Float64MultiArray in target_perception.MESSAGE_FIELDS order, one message per processed
RGB frame (including "no target" results), so consumers can distinguish an empty view from a silent camera.
Stamps are the camera header stamps (simulation seconds).
"""

import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import target_perception as tp

RGB_TOPIC = os.environ.get("SAR_CAMERA_RGB_TOPIC", "/uav/camera/rgb/image")
RGB_INFO_TOPIC = os.environ.get("SAR_CAMERA_RGB_INFO_TOPIC", "/uav/camera/rgb/camera_info")
DEPTH_TOPIC = os.environ.get("SAR_CAMERA_DEPTH_TOPIC", "/uav/camera/depth/image")
DEPTH_INFO_TOPIC = os.environ.get("SAR_CAMERA_DEPTH_INFO_TOPIC", "/uav/camera/depth/camera_info")
OUTPUT_TOPIC = "/uav/target_perception"
MAX_SYNC_DT = 0.1         # s of sensor time between paired RGB and depth frames
PROCESS_PERIOD = 0.05     # s wall; each RGB frame is processed at most once


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_to_array(msg):
    """sensor_msgs/Image -> numpy (RGB uint8 HxWx3 or float32 depth HxW); None for unsupported data."""
    if msg.encoding in ("rgb8", "bgr8"):
        if msg.step < msg.width * 3 or len(msg.data) < msg.step * msg.height:
            return None
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a[..., ::-1] if msg.encoding == "bgr8" else a
    if msg.encoding == "32FC1":
        if msg.step < msg.width * 4 or len(msg.data) < msg.step * msg.height:
            return None
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, : msg.width]
    return None


def pair_depth(rgb_stamp, depth_frames, max_dt=MAX_SYNC_DT):
    """Return the buffered (stamp, depth) closest to the RGB stamp within max_dt, else None."""
    best = min(depth_frames, key=lambda f: abs(f[0] - rgb_stamp), default=None)
    if best is None or abs(best[0] - rgb_stamp) > max_dt:
        return None
    return best


def camera_matrix(info_msg):
    if info_msg is None:
        return None
    K = np.array(info_msg.k, float).reshape(3, 3)
    return K if np.all(np.isfinite(K)) and K[0, 0] > 0 and K[1, 1] > 0 else None


class TargetPerceptionNode(Node):
    def __init__(self):
        super().__init__("target_perception")
        self._lock = threading.Lock()
        self._rgb = None                  # newest unprocessed RGB message
        self._depth = []                  # recent (stamp, array) depth frames
        self._rgb_info = None
        self._depth_info = None
        self._last_processed_stamp = -math.inf
        self._stats = {"rgb": 0, "depth": 0, "processed": 0, "detected": 0, "ms": 0.0}
        self._stats_started = time.monotonic()
        self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, RGB_INFO_TOPIC, self._on_rgb_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC, self._on_depth_info, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Float64MultiArray, OUTPUT_TOPIC, 10)
        self.create_timer(PROCESS_PERIOD, self._process)
        self.create_timer(10.0, self._log_stats)

    def _on_rgb(self, msg):
        with self._lock:
            self._rgb = msg
            self._stats["rgb"] += 1

    def _on_depth(self, msg):
        array = image_to_array(msg)
        if array is None:
            return
        with self._lock:
            self._depth = (self._depth + [(stamp_seconds(msg), array)])[-6:]
            self._stats["depth"] += 1

    def _on_rgb_info(self, msg):
        with self._lock:
            self._rgb_info = msg

    def _on_depth_info(self, msg):
        with self._lock:
            self._depth_info = msg

    def _process(self):
        with self._lock:
            msg, self._rgb = self._rgb, None
            depth_frames = list(self._depth)
            K_rgb, K_depth = camera_matrix(self._rgb_info), camera_matrix(self._depth_info)
        if msg is None:
            return
        stamp = stamp_seconds(msg)
        if stamp <= self._last_processed_stamp:
            return                        # never republish an old frame as new
        self._last_processed_stamp = stamp
        started = time.perf_counter()
        rgb = image_to_array(msg)
        paired = pair_depth(stamp, depth_frames)
        if rgb is None or rgb.ndim != 3 or K_rgb is None:
            det = tp.Detection(False, tp.STATUS_INPUT_INVALID)
        else:
            det = tp.detect(rgb, None if paired is None else paired[1], K_rgb, K_depth)
        elapsed = (time.perf_counter() - started) * 1000.0
        out = Float64MultiArray()
        out.layout.dim = [MultiArrayDimension(label="sar_target_perception_v1", size=len(tp.MESSAGE_FIELDS),
                                              stride=len(tp.MESSAGE_FIELDS))]
        out.data = tp.encode(det, stamp, math.nan if paired is None else paired[0], elapsed)
        self.publisher.publish(out)
        self._stats["processed"] += 1
        self._stats["detected"] += int(det.detected)
        self._stats["ms"] += elapsed

    def _log_stats(self):
        now = time.monotonic()
        dt = max(now - self._stats_started, 1e-6)
        s = self._stats
        self.get_logger().info(
            f"rgb {s['rgb'] / dt:.2f} Hz, depth {s['depth'] / dt:.2f} Hz, processed {s['processed'] / dt:.2f} Hz, "
            f"detected {s['detected']}, mean processing {s['ms'] / max(s['processed'], 1):.1f} ms"
        )
        self._stats = {"rgb": 0, "depth": 0, "processed": 0, "detected": 0, "ms": 0.0}
        self._stats_started = now


def main():
    rclpy.init()
    node = TargetPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
