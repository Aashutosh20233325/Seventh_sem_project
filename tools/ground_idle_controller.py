#!/usr/bin/env python3
"""Run the PX4 controller node in GROUND without publishing control topics."""

from __future__ import annotations

import time
import sys
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from px4_uav_env import PX4ControllerNode


def main() -> int:
    if not rclpy.ok():
        rclpy.init()

    node = PX4ControllerNode(None)
    node.phase = "GROUND"

    def report_readiness() -> None:
        now = time.monotonic()
        status_age = (
            None
            if node.status_received_at is None
            else now - node.status_received_at
        )
        print(
            "GROUND idle controller: "
            f"system_id={node.system_id}, armed_state={node.arming_state}, "
            f"nav_state={node.nav_state}, failsafe={node.failsafe}, "
            f"preflight={node.pre_flight_checks_pass}, "
            f"status_age_s={status_age}, landed={node.landed}, "
            f"position_valid={node.position_valid}, "
            f"heading_valid={node.heading_valid}, "
            "offboard_subscribers="
            f"{node.offboard_pub.get_subscription_count()}, "
            "setpoint_subscribers="
            f"{node.setpoint_pub.get_subscription_count()}; "
            "no control messages are published in GROUND",
            flush=True,
        )

    node.create_timer(5.0, report_readiness)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
