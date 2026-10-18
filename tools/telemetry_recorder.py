"""Passive PX4 telemetry recorder (subscribe-only; publishes nothing).

Usage (ROS Jazzy + px4_ros2_ws sourced, ROS_DOMAIN_ID set):
    .venv/bin/python tools/telemetry_recorder.py <output.jsonl> [max_seconds]

The output file is opened exclusively ("x"), so an existing log is never overwritten.

Each JSONL line carries three separate clocks:
  wall_utc  - host receipt time (UTC)
  mono      - host monotonic receipt time (seconds)
  msg       - full message fields; msg["timestamp"] is the PX4-published
              timestamp, which the XRCE bridge offsets toward epoch time and
              re-steps on timesync, so it is NOT a simulation-rate clock.
TimesyncStatus additionally records px4_hrt_us = timestamp + estimated_offset,
the reconstruction used by px4_uav_env.py.

The /fmu/in/* subscriptions observe the environment's own Offboard stream so
its start/stop can be correlated with PX4 nav_state transitions.
"""

import datetime
import json
import math
import sys
import time

import rclpy
from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TimesyncStatus,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Float64MultiArray

TOPICS = [
    ("status", VehicleStatus, "/fmu/out/vehicle_status_v4"),
    ("land", VehicleLandDetected, "/fmu/out/vehicle_land_detected"),
    ("position", VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1"),
    ("ack", VehicleCommandAck, "/fmu/out/vehicle_command_ack_v1"),
    ("timesync", TimesyncStatus, "/fmu/out/timesync_status"),
    ("failsafe_flags", FailsafeFlags, "/fmu/out/failsafe_flags"),
    ("in_offboard_mode", OffboardControlMode, "/fmu/in/offboard_control_mode"),
    ("in_setpoint", TrajectorySetpoint, "/fmu/in/trajectory_setpoint"),
    ("in_command", VehicleCommand, "/fmu/in/vehicle_command"),
    ("lidar_sectors", Float32MultiArray, "/uav/lidar_sectors"),
    ("target_perception", Float64MultiArray, "/uav/target_perception"),
]


def plain(value):
    """Convert ROS/NumPy field values into JSON-safe Python values."""
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, (bool, int, float, str)) or value is None:
        return value
    return str(value)


class Recorder(Node):
    def __init__(self, out):
        super().__init__("passive_telemetry_recorder")
        self.out = out
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )
        for event, msg_type, topic in TOPICS:
            self.create_subscription(
                msg_type, topic, lambda m, e=event: self.record(e, m), qos
            )

    def record(self, event, msg):
        fields = {
            name: plain(getattr(msg, name))
            for name in msg.get_fields_and_field_types()
        }
        line = {
            "event": event,
            "wall_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "mono": time.monotonic(),
            "msg": fields,
        }
        if event == "timesync":
            line["px4_hrt_us"] = int(msg.timestamp) + int(msg.estimated_offset)
        self.out.write(json.dumps(line) + "\n")
        self.out.flush()


def main():
    path = sys.argv[1]
    max_seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 4 * 3600.0
    rclpy.init()
    with open(path, "x", encoding="utf-8") as out:
        node = Recorder(out)
        deadline = time.monotonic() + max_seconds
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
        except KeyboardInterrupt:
            pass
        finally:
            out.flush()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
