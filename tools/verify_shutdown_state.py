#!/usr/bin/env python3
"""Read-only landed/disarmed gate for stopping an isolated SITL process group."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLandDetected, VehicleStatus


class ShutdownProbe(Node):
    def __init__(self) -> None:
        super().__init__("sar_isolated_shutdown_read_only_probe")
        self.status: list[tuple[float, VehicleStatus]] = []
        self.land: list[tuple[float, VehicleLandDetected]] = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self._status, qos)
        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self._land,
            qos,
        )

    def control_publishers(self) -> dict[str, list[str]]:
        topics = (
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/vehicle_command",
        )
        return {
            topic: [info.node_name for info in self.get_publishers_info_by_topic(topic)]
            for topic in topics
        }

    def _status(self, msg: VehicleStatus) -> None:
        self.status.append((time.monotonic(), msg))

    def _land(self, msg: VehicleLandDetected) -> None:
        self.land.append((time.monotonic(), msg))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--expected-system-id", type=int, default=2)
    args = parser.parse_args()
    if args.duration < 3.0 or args.duration > 15.0:
        parser.error("duration must be between 3 and 15 seconds")

    rclpy.init()
    node = ShutdownProbe()
    started = time.monotonic()
    active_publishers = {}
    try:
        deadline = started + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        active_publishers = {
            topic: names
            for topic, names in node.control_publishers().items()
            if names
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()

    now = time.monotonic()
    if len(node.status) < 2 or len(node.land) < 2:
        print(f"REFUSE: insufficient telemetry samples: status={len(node.status)}, land={len(node.land)}")
        return 1
    status_at, status = node.status[-1]
    land_at, land = node.land[-1]
    print(
        f"vehicle_status age={now-status_at:.3f}s system_id={status.system_id} "
        f"arming_state={status.arming_state} "
        f"nav_state={status.nav_state} failsafe={status.failsafe}"
    )
    print(
        f"land_detected age={now-land_at:.3f}s landed={land.landed} "
        f"at_rest={land.at_rest} ground_contact={land.ground_contact}"
    )
    if now - status_at > 2.0 or now - land_at > 4.0:
        print("REFUSE: telemetry is stale")
        return 1
    if status.system_id != args.expected_system_id:
        print(
            f"REFUSE: PX4 system_id={status.system_id}, "
            f"expected {args.expected_system_id}"
        )
        return 1
    if status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
        print("REFUSE: PX4 is armed")
        return 1
    if not land.landed or not land.at_rest:
        print("REFUSE: PX4 has not freshly confirmed landed and at-rest")
        return 1
    if active_publishers:
        print(f"REFUSE: control publishers are still active: {active_publishers}")
        return 1
    print("PASS: fresh landed, at-rest, disarmed telemetry")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
