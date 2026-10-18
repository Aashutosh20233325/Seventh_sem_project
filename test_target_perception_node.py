import math
import threading
import time

import numpy as np
import pytest

pytest.importorskip("rclpy")
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

import synthetic_scene as ss  # noqa: E402
import target_perception as tp  # noqa: E402
import target_perception_node as node_mod  # noqa: E402


def image_msg(array, stamp, encoding):
    msg = Image()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    msg.height, msg.width = array.shape[:2]
    msg.encoding = encoding
    msg.step = array.strides[0]
    msg.data = array.tobytes()
    return msg


def info_msg(K):
    msg = CameraInfo()
    msg.k = [float(v) for v in np.asarray(K).ravel()]
    return msg


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def make_node():
    node = node_mod.TargetPerceptionNode.__new__(node_mod.TargetPerceptionNode)
    node._lock = threading.Lock()
    node._rgb, node._depth = None, []
    node._rgb_info = node._depth_info = None
    node._last_processed_stamp = -math.inf
    node._stats = {"rgb": 0, "depth": 0, "processed": 0, "detected": 0, "ms": 0.0}
    node._stats_started = time.monotonic()
    node.publisher = Publisher()
    return node


def scene(distance=4.31):
    uav = np.array([3.0 - distance, -3.0, 1.615])
    return ss.render(uav, 0.0)


def test_image_conversion_handles_rgb_bgr_depth_and_rejects_others():
    rgb, depth = scene()
    assert np.array_equal(node_mod.image_to_array(image_msg(rgb, 1.0, "rgb8")), rgb)
    bgr = np.ascontiguousarray(rgb[..., ::-1])
    assert np.array_equal(node_mod.image_to_array(image_msg(bgr, 1.0, "bgr8")), rgb)
    assert np.array_equal(node_mod.image_to_array(image_msg(depth, 1.0, "32FC1")), depth, equal_nan=True)
    assert node_mod.image_to_array(image_msg(rgb, 1.0, "mono8")) is None
    truncated = image_msg(rgb, 1.0, "rgb8")
    truncated.data = truncated.data[:100]
    assert node_mod.image_to_array(truncated) is None


def test_depth_pairing_uses_nearest_frame_within_window():
    frames = [(10.00, "a"), (10.08, "b"), (10.30, "c")]
    assert node_mod.pair_depth(10.07, frames)[1] == "b"
    assert node_mod.pair_depth(10.50, frames) is None
    assert node_mod.pair_depth(10.0, []) is None


def test_processed_frame_publishes_detection_with_sensor_stamps():
    node = make_node()
    rgb, depth = scene()
    node._rgb_info, node._depth_info = info_msg(ss.K_RGB), info_msg(ss.K_DEPTH)
    node._on_depth(image_msg(depth, 20.00, "32FC1"))
    node._on_rgb(image_msg(rgb, 20.02, "rgb8"))

    node._process()

    assert len(node.publisher.messages) == 1
    out = node.publisher.messages[0]
    assert out.layout.dim[0].label == "sar_target_perception_v1"
    msg = tp.decode(out.data)
    assert msg["detected"] and msg["range_valid"]
    assert msg["image_stamp"] == pytest.approx(20.02) and msg["depth_stamp"] == pytest.approx(20.00)
    assert msg["horizontal_range"] == pytest.approx(4.31, rel=0.02)


def test_same_frame_is_never_republished_and_old_frames_are_dropped():
    node = make_node()
    rgb, depth = scene()
    node._rgb_info, node._depth_info = info_msg(ss.K_RGB), info_msg(ss.K_DEPTH)
    node._on_rgb(image_msg(rgb, 30.0, "rgb8"))
    node._process()
    node._process()                                  # no new frame
    node._on_rgb(image_msg(rgb, 29.5, "rgb8"))       # out-of-order older frame
    node._process()
    assert len(node.publisher.messages) == 1


def test_missing_depth_or_camera_info_is_explicit():
    node = make_node()
    rgb, _ = scene()
    node._on_rgb(image_msg(rgb, 40.0, "rgb8"))
    node._process()                                  # no camera_info yet
    first = tp.decode(node.publisher.messages[-1].data)
    assert not first["detected"] and first["status"] == tp.STATUS_INPUT_INVALID

    node._rgb_info = info_msg(ss.K_RGB)
    node._on_rgb(image_msg(rgb, 41.0, "rgb8"))
    node._process()                                  # RGB but no depth frame
    second = tp.decode(node.publisher.messages[-1].data)
    assert second["detected"] and not second["range_valid"]
    assert second["status"] == tp.STATUS_DEPTH_INVALID and math.isnan(second["depth_stamp"])


def test_node_subscribes_only_to_camera_topics():
    source = open(node_mod.__file__).read()
    assert "pose/info" not in source and "gz." not in source and "px4_msgs" not in source
