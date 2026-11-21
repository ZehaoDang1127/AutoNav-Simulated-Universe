"""Fail-closed velocity gate between Nav2 and the Gazebo dynamics adapter.

The adapter must read /cmd_vel_supervised. This process is the sole publisher
of that topic; losing the Run heartbeat, input commands, or the referee makes
it publish zero using a steady-clock timer, including when /clock stalls.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass
class GateState:
    heartbeat_timeout_s: float = 0.35
    command_timeout_s: float = 0.40
    enabled: bool = False
    failed: bool = False
    last_heartbeat_s: float = -math.inf
    last_command_s: float = -math.inf
    linear_mps: float = 0.0
    angular_radps: float = 0.0
    interlock: bool = True

    def set_interlock(self, permitted: bool, now_s: float) -> None:
        self.interlock = permitted
        if not permitted:
            self.set_enable(False, now_s)

    def set_enable(self, enabled: bool, now_s: float) -> None:
        if self.failed or not enabled or not self.interlock:
            self.enabled = False
            self.linear_mps = 0.0
            self.angular_radps = 0.0
            self.last_command_s = -math.inf
            return
        self.enabled = True
        self.last_heartbeat_s = now_s

    def set_command(self, linear_mps: float, angular_radps: float,
                    now_s: float) -> None:
        if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
            self.failed = True
            return
        if self.enabled and not self.failed:
            self.linear_mps = linear_mps
            self.angular_radps = angular_radps
            self.last_command_s = now_s

    def set_referee_failure(self, failed: bool) -> None:
        if failed:
            self.failed = True
            self.enabled = False

    def output(self, now_s: float) -> tuple[float, float]:
        if (self.failed or not self.enabled or not self.interlock
                or now_s - self.last_heartbeat_s > self.heartbeat_timeout_s):
            self.enabled = False
            self.linear_mps = 0.0
            self.angular_radps = 0.0
            self.last_command_s = -math.inf
            return 0.0, 0.0
        if now_s - self.last_command_s > self.command_timeout_s:
            return 0.0, 0.0
        return self.linear_mps, self.angular_radps


def main() -> None:
    import os
    from pathlib import Path
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool

    class CommandGate(Node):
        def __init__(self) -> None:
            super().__init__("full_track_command_gate")
            self.state = GateState()
            self.desktop_lease = None
            self.auto_pub = None
            if os.environ.get("DESKTOP_CONTROL") == "true":
                from autonav_universe.full_track import ControlLease
                self.desktop_lease = ControlLease()
                self.state.interlock = False
                self.control_path = Path(os.environ["RUN_DIR"]) / "desktop_control.json"
                self.auto_pub = self.create_publisher(Bool, "/autonomous_mode", 1)
            self.output_pub = self.create_publisher(
                Twist, "/cmd_vel_supervised", 10)
            self.create_subscription(
                Twist, "/cmd_vel",
                lambda msg: self.state.set_command(
                    float(msg.linear.x), float(msg.angular.z), time.monotonic()),
                10)
            self.create_subscription(
                Bool, "/full_track/enable",
                lambda msg: self.state.set_enable(
                    bool(msg.data), time.monotonic()), 10)
            self.create_subscription(
                Bool, "/igvc_sim/fail",
                lambda msg: self.state.set_referee_failure(bool(msg.data)), 10)
            self.create_timer(
                0.05, self.tick,
                clock=Clock(clock_type=ClockType.STEADY_TIME))

        def tick(self) -> None:
            if self.desktop_lease is not None:
                from autonav_universe.full_track import read_json
                now = time.monotonic()
                desired = self.desktop_lease.update(read_json(self.control_path), now)
                self.state.set_interlock(desired == "running", now)
            linear, angular = self.state.output(time.monotonic())
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self.output_pub.publish(msg)
            if self.auto_pub is not None:
                self.auto_pub.publish(Bool(data=self.state.enabled and not self.state.failed))

    rclpy.init()
    node = CommandGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Try to make shutdown explicit as well as relying on the downstream
        # adapter's command timeout.
        node.state.set_enable(False, time.monotonic())
        if rclpy.ok():
            for _ in range(3):
                node.tick()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
