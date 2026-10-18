import importlib.util
import math
import sys
import threading
import time
import types
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def publisher_module(monkeypatch):
    """Load the publisher with ROS/Gazebo interfaces shimmed out."""
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = type("Node", (), {})
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: None
    rclpy_qos.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
    rclpy_qos.DurabilityPolicy = SimpleNamespace(VOLATILE=1)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    px4_msgs = types.ModuleType("px4_msgs")
    px4_msgs_msg = types.ModuleType("px4_msgs.msg")
    px4_msgs_msg.VehicleLocalPosition = type("VehicleLocalPosition", (), {})

    gz = types.ModuleType("gz")
    gz.__path__ = []
    gz_transport = types.ModuleType("gz.transport13")
    gz_transport.Node = object
    gz_msgs10 = types.ModuleType("gz.msgs10")
    gz_msgs10.__path__ = []
    gz_pose = types.ModuleType("gz.msgs10.pose_v_pb2")
    gz_pose.Pose_V = type("Pose_V", (), {})

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "px4_msgs": px4_msgs,
        "px4_msgs.msg": px4_msgs_msg,
        "gz": gz,
        "gz.transport13": gz_transport,
        "gz.msgs10": gz_msgs10,
        "gz.msgs10.pose_v_pb2": gz_pose,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    source = Path(__file__).with_name("observation_publisher.py")
    spec = importlib.util.spec_from_file_location(
        "observation_publisher_subject", source
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_publisher(module, scan, received_ago):
    publisher = module.ObservationPublisher.__new__(module.ObservationPublisher)
    publisher._input_lock = threading.Lock()
    publisher.scan = scan
    publisher.scan_valid = True
    publisher.scan_received_at = time.monotonic() - received_ago
    publisher.publisher = Publisher()
    publisher.get_logger = lambda: SimpleNamespace(error=lambda _msg: None)
    return publisher


def scan_with(ranges):
    return SimpleNamespace(
        angle_min=-2.356195, angle_max=2.356195,
        angle_increment=4.71239 / (len(ranges) - 1),
        range_min=0.1, range_max=30.0, ranges=ranges,
    )


def test_publishes_49_finite_sectors_with_non_finite_rays_as_max_range(publisher_module):
    ranges = [float("inf")] * 1080
    ranges[540] = 2.5
    ranges[0] = float("nan")
    publisher = make_publisher(publisher_module, scan_with(ranges), 0.05)
    publisher_module.Float32MultiArray = lambda: SimpleNamespace(data=None)

    publisher.publish_observation()

    (msg,) = publisher.publisher.messages
    assert len(msg.data) == 49 and all(math.isfinite(v) for v in msg.data)
    assert msg.data[24] == pytest.approx(2.5)
    assert msg.data[0] == pytest.approx(30.0)


def test_publisher_does_not_republish_when_scan_is_stale(publisher_module):
    publisher = make_publisher(publisher_module, scan_with([5.0] * 1080), 1.0)

    publisher.publish_observation()

    assert publisher.publisher.messages == []


def test_lidar_publisher_has_no_ground_truth_access():
    source = Path(__file__).with_name("observation_publisher.py").read_text()
    for forbidden in ("gz.", "pose/info", "Pose_V", "target_x", "TARGET"):
        assert forbidden not in source


def test_standin_lidar_binning_matches_publisher(publisher_module):
    import random
    import sar_standin_env

    rng = random.Random(7)
    ranges = [rng.choice([float("inf"), float("nan"), rng.uniform(0.05, 40.0)]) for _ in range(1080)]
    scan = scan_with(ranges)
    publisher = publisher_module.ObservationPublisher.__new__(publisher_module.ObservationPublisher)

    expected = publisher.get_49_lidar(scan)
    actual = sar_standin_env.bin_sectors(
        ranges, scan.angle_min, scan.angle_max, scan.angle_increment, scan.range_min, scan.range_max)

    assert actual == pytest.approx(expected)
