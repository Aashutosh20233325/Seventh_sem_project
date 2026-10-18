#!/usr/bin/env python3
"""Read-only bounded validation of isolated PX4, LiDAR, and observation data."""

from __future__ import annotations

import argparse
import math
import os
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

LIDAR_TOPIC = os.environ.get(
    "SAR_LIDAR_TOPIC",
    "/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan",
)


class TopicProbe(Node):
    def __init__(self) -> None:
        super().__init__("sar_isolated_read_only_probe")
        self.received: dict[str, list[tuple[float, object]]] = defaultdict(list)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            lambda msg: self._record("status", msg),
            px4_qos,
        )
        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            lambda msg: self._record("land", msg),
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            lambda msg: self._record("position", msg),
            px4_qos,
        )
        self.create_subscription(
            LaserScan,
            LIDAR_TOPIC,
            lambda msg: self._record("scan", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            "/uav/lidar_sectors",
            lambda msg: self._record("observation", msg),
            10,
        )

    def _record(self, key: str, msg: object) -> None:
        self.received[key].append((time.monotonic(), msg))


def _finite(values: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def validate(
    probe: TopicProbe, started: float, expected_system_id: int
) -> list[str]:
    errors: list[str] = []
    now = time.monotonic()
    latest = {key: values[-1] for key, values in probe.received.items() if values}
    required = {"status", "land", "position", "scan", "observation"}
    missing = required - latest.keys()
    if missing:
        errors.append(f"missing fresh topics: {sorted(missing)}")
        return errors

    for key, (received_at, _msg) in latest.items():
        max_age = 4.0 if key == "land" else 2.0 if key == "status" else 1.0
        if now - received_at > max_age:
            errors.append(f"{key} sample stale by {now - received_at:.3f}s")

    status = latest["status"][1]
    land = latest["land"][1]
    position = latest["position"][1]
    scan = latest["scan"][1]
    observation = latest["observation"][1]

    if status.system_id != expected_system_id:
        errors.append(
            f"PX4 system ID mismatch: got {status.system_id}, "
            f"expected {expected_system_id}"
        )
    if status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
        errors.append(f"PX4 is not disarmed (arming_state={status.arming_state})")
    if not land.landed or not land.at_rest:
        errors.append(f"PX4 is not confirmed landed/at-rest: {land.landed}/{land.at_rest}")
    if status.failsafe:
        errors.append("PX4 reports failsafe=true")
    if not position.xy_valid or not position.z_valid:
        errors.append("PX4 local position is invalid")
    elif not _finite([position.x, position.y, position.z]):
        errors.append("PX4 local position contains a non-finite value")
    scan_metadata_valid = (
        len(scan.ranges) >= 49
        and math.isfinite(float(scan.angle_min))
        and math.isfinite(float(scan.angle_max))
        and math.isfinite(float(scan.angle_increment))
        and math.isfinite(float(scan.range_min))
        and math.isfinite(float(scan.range_max))
        and scan.angle_max > scan.angle_min
        and scan.angle_increment > 0.0
        and scan.range_max >= scan.range_min
        and all(not math.isnan(float(value)) for value in scan.ranges)
    )
    if not scan_metadata_valid:
        errors.append("LiDAR scan metadata is invalid or contains NaN beams")
    if len(observation.data) != 49 or not _finite(list(observation.data)):
        errors.append(
            f"LiDAR sectors must contain 49 finite values; got {len(observation.data)}"
        )

    minimum_counts = {"status": 2, "land": 2, "position": 2, "scan": 3, "observation": 3}
    for key, minimum in minimum_counts.items():
        count = sum(1 for received_at, _ in probe.received[key] if received_at >= started)
        if count < minimum:
            errors.append(f"only {count} {key} samples in bounded probe, need {minimum}")

    print(
        "PX4 state: "
        f"system_id={status.system_id}, armed_state={status.arming_state}, "
        f"nav_state={status.nav_state}, failsafe={status.failsafe}, "
        f"preflight={status.pre_flight_checks_pass}, "
        f"gcs_connection_lost={status.gcs_connection_lost}"
    )
    print(
        f"Land detection: landed={land.landed}, at_rest={land.at_rest}, "
        f"ground_contact={land.ground_contact}"
    )
    print(
        f"Local NED position: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f}); "
        f"valid={position.xy_valid and position.z_valid}, "
        f"heading_good_for_control={position.heading_good_for_control}, "
        f"dist_bottom_valid={position.dist_bottom_valid}"
    )
    print(
        f"LiDAR ranges: {len(scan.ranges)} (finite={sum(math.isfinite(float(v)) for v in scan.ranges)}); "
        f"observation values: {len(observation.data)}"
    )
    print("Sample counts: " + ", ".join(f"{k}={len(v)}" for k, v in probe.received.items()))
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--expected-system-id", type=int, default=2)
    args = parser.parse_args()
    if args.duration < 5.0 or args.duration > 15.0:
        parser.error("duration must be between 5 and 15 seconds")

    rclpy.init()
    node = TopicProbe()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        errors = validate(node, started, args.expected_system_id)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if errors:
        for error in errors:
            print(f"FAIL: {error}")
        return 1
    print("PASS: isolated telemetry, LiDAR, and observation samples are fresh and valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
