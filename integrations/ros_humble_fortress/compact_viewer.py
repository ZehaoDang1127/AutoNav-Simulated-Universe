#!/usr/bin/env python3
"""Read-only live course viewer for the compact ROS/Gazebo diagnostic run."""

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class ViewerState(Node):
    def __init__(self, course_config: Path):
        super().__init__("compact_course_viewer")
        with course_config.open(encoding="utf-8") as stream:
            course = yaml.safe_load(stream)
        self.lock = threading.Lock()
        self.data = {
            "course": {key: course.get(key) for key in (
                "course_id", "course_geometry", "line_breaks", "start", "finish", "centerline", "internal_lines",
                "dashed_lines", "obstacles", "ramps", "mission_waypoints")},
            "pose": None,
            "trail": [],
            "plan": [],
            "plan_frame": "map",
            "goal": None,
            "goal_update": None,
            "nav_goal": None,
            "score": None,
            "run_result": None,
            "map_to_world": None,
            "odom_to_world": None,
            "updated_wall_s": None,
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Odometry, "/igvc_sim/ground_truth_odom", self.on_odom, 10)
        self.create_subscription(String, "/igvc_sim/score", self.on_score, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(PoseStamped, "/goal_update", self.on_goal_update, 10)
        self.create_subscription(PoseStamped, "/nav_goal", self.on_nav_goal, 10)
        self.create_subscription(NavPath, "/plan", self.on_plan, 10)
        self.create_timer(0.2, self.update_map_alignment)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = {"x": p.x, "y": p.y, "yaw": yaw,
                "speed": math.hypot(msg.twist.twist.linear.x,
                                    msg.twist.twist.linear.y)}
        with self.lock:
            trail = self.data["trail"]
            if not trail or math.hypot(p.x - trail[-1][0], p.y - trail[-1][1]) >= 0.08:
                trail.append([round(p.x, 3), round(p.y, 3)])
            self.data["pose"] = pose
            self.data["updated_wall_s"] = time.time()

    def on_score(self, msg: String) -> None:
        try:
            score = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.data["score"] = score

    def on_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ("map", "odom"):
            return
        with self.lock:
            self.data["goal"] = [msg.pose.position.x, msg.pose.position.y]
            self.data["goal_frame"] = msg.header.frame_id

    def on_goal_update(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ("map", "odom"):
            return
        with self.lock:
            self.data["goal_update"] = [msg.pose.position.x, msg.pose.position.y]
            self.data["goal_update_frame"] = msg.header.frame_id

    def on_nav_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ("map", "odom"):
            return
        with self.lock:
            self.data["nav_goal"] = [msg.pose.position.x, msg.pose.position.y]
            self.data["nav_goal_frame"] = msg.header.frame_id

    def on_plan(self, msg: NavPath) -> None:
        points = msg.poses
        stride = max(1, len(points) // 250)
        with self.lock:
            self.data["plan_frame"] = msg.header.frame_id
            self.data["plan"] = [[p.pose.position.x, p.pose.position.y]
                                 for p in points[::stride]]

    def update_map_alignment(self) -> None:
        with self.lock:
            truth = self.data["pose"]
        if truth is None:
            return
        for frame in ("map", "odom"):
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame, "base_link", Time())
            except TransformException:
                continue
            position = transform.transform.translation
            q = transform.transform.rotation
            frame_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            angle = truth["yaw"] - frame_yaw
            c, s = math.cos(angle), math.sin(angle)
            translation_x = truth["x"] - c * position.x + s * position.y
            translation_y = truth["y"] - s * position.x - c * position.y
            with self.lock:
                self.data[frame + "_to_world"] = [translation_x, translation_y, angle]

    def snapshot(self) -> bytes:
        with self.lock:
            return json.dumps(self.data, separators=(",", ":")).encode("utf-8")


def serve(node: ViewerState, port: int, html_path: Path) -> ThreadingHTTPServer:
    page = html_path.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/":
                body, mime = page, "text/html; charset=utf-8"
            elif self.path == "/state":
                body, mime = node.snapshot(), "application/json; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args) -> None:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-config", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--replay-output", type=Path)
    parser.add_argument("--result-file", type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = ViewerState(args.course_config)
    server = serve(node, args.port, Path(__file__).with_name("compact_viewer.html"))
    print(f"Compact course viewer listening on port {args.port}", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        server.shutdown()
        if args.result_file and args.result_file.exists():
            try:
                result = json.loads(args.result_file.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                result = None
            with node.lock:
                node.data["run_result"] = result
        if args.replay_output:
            snapshot = node.snapshot().decode("utf-8").replace("<", "\\u003c")
            page = Path(__file__).with_name("compact_viewer.html").read_text(
                encoding="utf-8")
            page = page.replace("window.__AUTONAV_EMBEDDED_STATE__ = null;",
                                "window.__AUTONAV_EMBEDDED_STATE__ = " + snapshot + ";")
            args.replay_output.write_text(page, encoding="utf-8")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
