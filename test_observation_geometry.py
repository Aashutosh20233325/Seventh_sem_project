import math

import pytest

from observation_geometry import (
    quaternion_yaw,
    required_inputs_fresh,
    target_bearing_in_sensor_frame,
)


def test_quaternion_yaw_recovers_gazebo_model_yaw():
    yaw = math.pi / 2.0
    assert quaternion_yaw(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)


@pytest.mark.parametrize(
    ("yaw", "target_xy", "expected"),
    [
        (0.0, (1.0, 0.0), 0.0),
        (math.pi / 2.0, (1.0, 0.0), -math.pi / 2.0),
        (math.pi / 2.0, (0.0, 1.0), 0.0),
        (0.0, (0.0, 1.0), math.pi / 2.0),
    ],
)
def test_target_bearing_is_relative_to_sensor_forward(yaw, target_xy, expected):
    result = target_bearing_in_sensor_frame(
        0.0, 0.0, yaw, target_xy[0], target_xy[1]
    )
    assert result == pytest.approx(expected)


def test_sensor_yaw_offset_is_removed_from_world_bearing():
    result = target_bearing_in_sensor_frame(
        0.0, 0.0, math.pi / 4, 1.0, 1.0, sensor_yaw=math.pi / 4
    )
    assert result == pytest.approx(-math.pi / 4)


def test_required_inputs_fresh_accepts_valid_recent_samples():
    assert required_inputs_fresh(
        now=10.5,
        scan_received_at=10.1,
        pose_received_at=10.2,
        position_received_at=10.0,
        scan_valid=True,
        pose_valid=True,
        position_valid=True,
    )


@pytest.mark.parametrize(
    "overrides",
    [
        {"scan_received_at": None},
        {"scan_received_at": 9.49},
        {"pose_received_at": 9.49},
        {"position_received_at": 9.49},
        {"scan_valid": False},
        {"pose_valid": False},
        {"position_valid": False},
        {"position_received_at": 10.6},
    ],
)
def test_required_inputs_fresh_rejects_missing_stale_or_invalid(overrides):
    inputs = {
        "now": 10.0,
        "scan_received_at": 9.8,
        "pose_received_at": 9.8,
        "position_received_at": 9.8,
        "scan_valid": True,
        "pose_valid": True,
        "position_valid": True,
    }
    inputs.update(overrides)
    assert not required_inputs_fresh(**inputs)


def test_quaternion_yaw_rejects_invalid_quaternion():
    with pytest.raises(ValueError):
        quaternion_yaw(0.0, 0.0, 0.0, 0.0)
