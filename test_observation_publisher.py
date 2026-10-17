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


def test_publisher_target_detection_uses_sensor_relative_angle(publisher_module):
    publisher = publisher_module.ObservationPublisher.__new__(
        publisher_module.ObservationPublisher
    )
    scan = SimpleNamespace(
        angle_min=-math.pi / 2,
        angle_max=math.pi / 2,
        angle_increment=math.pi / 180,
        range_min=0.1,
        range_max=30.0,
        ranges=[float("inf")] * 181,
    )
    # Vehicle faces +Y in Gazebo ENU; target is directly forward.
    scan.ranges[90] = 4.5
    pose = (0.0, 0.0, math.pi / 2, 0.0, 5.0)

    assert publisher.get_target_values(scan, pose) == pytest.approx(
        (1.0, 4.5, 0.0)
    )


def test_pose_callback_tracks_gazebo_uav_and_target(publisher_module):
    publisher = publisher_module.ObservationPublisher.__new__(
        publisher_module.ObservationPublisher
    )
    publisher._input_lock = threading.Lock()
    yaw = math.pi / 3
    uav = SimpleNamespace(
        name="x500_lidar_2d_0",
        position=SimpleNamespace(x=1.0, y=2.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2),
            w=math.cos(yaw / 2),
        ),
    )
    target = SimpleNamespace(
        name="target",
        position=SimpleNamespace(x=4.0, y=-2.0),
    )

    publisher.pose_callback(SimpleNamespace(pose=[target, uav]))

    assert publisher.pose_valid
    assert publisher.uav_x == 1.0
    assert publisher.uav_y == 2.0
    assert publisher.uav_yaw == pytest.approx(yaw)
    assert publisher.target_x == 4.0
    assert publisher.target_y == -2.0


def test_publisher_does_not_republish_when_a_source_is_stale(publisher_module):
    publisher = publisher_module.ObservationPublisher.__new__(
        publisher_module.ObservationPublisher
    )
    publisher._input_lock = threading.Lock()
    now = time.monotonic()
    publisher.scan_received_at = now - 1.0
    publisher.pose_received_at = now
    publisher.position_received_at = now
    publisher.scan_valid = True
    publisher.pose_valid = True
    publisher.position_valid = True
    publisher.scan = object()
    publisher.uav_x = publisher.uav_y = publisher.uav_yaw = 0.0
    publisher.target_x = publisher.target_y = 1.0
    publisher.publisher = Publisher()

    publisher.publish_observation()

    assert publisher.publisher.messages == []
