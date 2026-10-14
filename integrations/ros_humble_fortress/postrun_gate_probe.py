"""Check that the live command gate is forwarding only zero after a run."""
from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("full_track_postrun_gate_probe")
        self.samples: list[tuple[float, float]] = []
        self.create_subscription(
            Twist, "/cmd_vel_supervised", self.on_command, 10)

    def on_command(self, msg: Twist) -> None:
        self.samples.append((float(msg.linear.x), float(msg.angular.z)))


def main() -> int:
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 5.0
    try:
        while rclpy.ok() and len(node.samples) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = (len(node.samples) >= 5 and all(
            abs(linear) < 1e-6 and abs(angular) < 1e-6
            for linear, angular in node.samples))
        print(json.dumps({"safety_gate_passed": passed,
                          "zero_command_samples": node.samples[:10]}))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
