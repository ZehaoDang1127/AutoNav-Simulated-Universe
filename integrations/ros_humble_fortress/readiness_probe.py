"""Wall-clock bounded readiness check for the Humble/Fortress smoke stack.

Run in a sourced ROS workspace. This is also the preflight contract for the
future Run supervisor; it does not enable autonomous motion.
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix, PointCloud2
from tf2_ros import Buffer, TransformListener

from autonav_interfaces.action import NavigateToWaypoint


def seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ReadinessProbe(Node):
    def __init__(self, line_mode: str, require_imu: bool = False,
                 require_gps_odom: bool = False) -> None:
        super().__init__("full_track_readiness_probe")
        self.line_mode = line_mode
        self.clock_count = 0
        self.clock_s = 0.0
        self.last_clock_wall = 0.0
        self.samples: dict[str, tuple[float, float]] = {}
        self.last_gated_command: tuple[float, float, float] | None = None
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock, "/clock", self.on_clock, sensor_qos)
        topics = {
            "/scan_fullframe": LaserScan,
            "/scan_pca_obstacle_volume_points": PointCloud2,
            "/gps_fix": NavSatFix,
            "/local_ekf/odom": Odometry,
            "/map_padded": OccupancyGrid,
            "/model/shogi/odometry": Odometry,
        }
        if line_mode == "camera":
            topics["/zed/zed_node/rgb/color/rect/image"] = Image
        if require_imu:
            topics["/imu/data"] = Imu
        if require_gps_odom:
            topics["/odometry/gps"] = Odometry
            topics["/global_ekf/odom"] = Odometry
        for topic, message_type in topics.items():
            self.create_subscription(
                message_type, topic,
                lambda msg, name=topic: self.on_sample(name, msg),
                map_qos if message_type is OccupancyGrid else sensor_qos)
        self.required_topics = tuple(topics)
        self.create_subscription(
            Twist, "/cmd_vel_supervised", self.on_gated_command, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.action = ActionClient(self, NavigateToWaypoint, "/navigate_to_waypoint")
        self.lifecycle = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in ("planner_server", "controller_server", "bt_navigator")
        }
        self.lifecycle_futures = {}
        self.lifecycle_states = {}

    def on_clock(self, msg: Clock) -> None:
        value = seconds(msg.clock)
        if value > self.clock_s:
            self.clock_count += 1
            self.last_clock_wall = time.monotonic()
        self.clock_s = value

    def on_sample(self, topic: str, msg) -> None:
        self.samples[topic] = (seconds(msg.header.stamp), time.monotonic())

    def on_gated_command(self, msg: Twist) -> None:
        self.last_gated_command = (
            float(msg.linear.x), float(msg.angular.z), time.monotonic())

    def check_lifecycle(self) -> None:
        for name, client in self.lifecycle.items():
            pending = self.lifecycle_futures.get(name)
            if pending is not None and pending[0].done():
                try:
                    self.lifecycle_states[name] = pending[0].result().current_state.label
                except Exception as exc:  # Preserve the diagnostic on timeout.
                    self.lifecycle_states[name] = f"error: {exc}"
                del self.lifecycle_futures[name]
            elif pending is not None and time.monotonic() - pending[1] > 3.0:
                pending[0].cancel()
                del self.lifecycle_futures[name]
            elif pending is None and client.service_is_ready():
                self.lifecycle_futures[name] = (client.call_async(GetState.Request()),
                                                time.monotonic())

    def missing(self, stale_sim_s: float, stale_wall_s: float) -> list[str]:
        missing = []
        now_wall = time.monotonic()
        if self.clock_count < 2 or now_wall - self.last_clock_wall > stale_wall_s:
            missing.append("advancing /clock")
        for topic in self.required_topics:
            sample = self.samples.get(topic)
            if sample is None:
                missing.append(topic)
            elif (self.clock_s - sample[0] > stale_sim_s
                  or now_wall - sample[1] > stale_wall_s):
                missing.append(f"fresh {topic}")
        if self.count_publishers("/cmd_vel_supervised") != 1:
            missing.append("exactly one command gate publisher")
        if self.count_subscribers("/cmd_vel_supervised") < 1:
            missing.append("dynamics adapter subscribed to command gate")
        if (self.last_gated_command is None
                or now_wall - self.last_gated_command[2] > stale_wall_s):
            missing.append("fresh gated zero command")
        elif (abs(self.last_gated_command[0]) > 1e-6
              or abs(self.last_gated_command[1]) > 1e-6):
            missing.append("gated command zero before Run")
        for parent, child in (("map", "odom"), ("odom", "base_link"),
                              ("base_link", "nav_center")):
            if not self.tf_buffer.can_transform(parent, child, rclpy.time.Time()):
                missing.append(f"TF {parent}->{child}")
        for name in self.lifecycle:
            if self.lifecycle_states.get(name) != "active":
                missing.append(f"{name} active")
        if not self.action.server_is_ready():
            missing.append("/navigate_to_waypoint action")
        return missing


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--line-mode", choices=("camera", "lidar"), default="lidar")
    parser.add_argument("--timeout-wall-s", type=float, default=90.0)
    parser.add_argument("--stale-sim-s", type=float, default=5.0)
    parser.add_argument("--stale-wall-s", type=float, default=5.0)
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument("--require-gps-odom", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = ReadinessProbe(args.line_mode, args.require_imu, args.require_gps_odom)
    deadline = time.monotonic() + args.timeout_wall_s
    try:
        missing = ["startup"]
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_lifecycle()
            missing = node.missing(args.stale_sim_s, args.stale_wall_s)
            if not missing:
                break
        result = {
            "ready": not missing,
            "imu_required": args.require_imu,
            "gps_odom_required": args.require_gps_odom,
            "clock_s": round(node.clock_s, 3),
            "lifecycle_states": node.lifecycle_states,
            "missing": missing,
        }
        print(json.dumps(result, indent=2))
        return 0 if result["ready"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
