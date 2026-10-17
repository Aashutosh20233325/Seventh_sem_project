"""Pure helpers for the Gazebo-backed observation publisher.

This module deliberately has no ROS or Gazebo imports so its frame math and
freshness gate can be tested offline.
"""

import math


def wrap_angle(angle):
    """Wrap an angle in radians to [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(x, y, z, w):
    """Return yaw from a quaternion, rejecting malformed orientations."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain only finite values")

    qx, qy, qz, qw = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("quaternion must have nonzero length")

    qx, qy, qz, qw = (value / norm for value in values)
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def target_bearing_in_sensor_frame(
    uav_x,
    uav_y,
    uav_yaw,
    target_x,
    target_y,
    sensor_yaw=0.0,
):
    """Compute target bearing in the 2D LiDAR sensor's angular frame.

    The inputs are Gazebo world XY positions and the Gazebo model yaw. The
    sensor offset is its yaw relative to the UAV model; the configured lidar
    joint and sensor both have zero yaw, so the publisher passes zero.
    """
    values = tuple(
        float(value)
        for value in (uav_x, uav_y, uav_yaw, target_x, target_y, sensor_yaw)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("positions and angles must be finite")

    uav_x, uav_y, uav_yaw, target_x, target_y, sensor_yaw = values
    world_bearing = math.atan2(target_y - uav_y, target_x - uav_x)
    return wrap_angle(world_bearing - uav_yaw - sensor_yaw)


def required_inputs_fresh(
    now,
    scan_received_at,
    pose_received_at,
    position_received_at,
    scan_valid,
    pose_valid,
    position_valid,
    max_age=0.5,
):
    """Whether all required publisher inputs are valid and recently received."""
    if not (scan_valid and pose_valid and position_valid):
        return False

    now = float(now)
    max_age = float(max_age)
    if not math.isfinite(now) or not math.isfinite(max_age) or max_age < 0.0:
        return False

    for received_at in (
        scan_received_at,
        pose_received_at,
        position_received_at,
    ):
        if received_at is None:
            return False
        age = now - float(received_at)
        if not math.isfinite(age) or age < 0.0 or age > max_age:
            return False
    return True
