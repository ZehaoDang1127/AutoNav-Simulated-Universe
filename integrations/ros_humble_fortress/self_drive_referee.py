"""Start the original IGVC referee only after ground-truth pose stabilizes."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from igvc_competition_sim.course_monitor import IgvcCourseMonitor
from autonav_universe.full_track import write_json
from autonav_universe.referee_startup import StartupPoseGuard


class PoseWait(Node):
    def __init__(self, x, y):
        super().__init__('self_drive_start_pose_guard')
        self.guard = StartupPoseGuard(x, y)
        self.stable = False
        self.create_subscription(Odometry, '/igvc_sim/ground_truth_odom', self.observe, 20)

    def observe(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.stable = self.guard.update(p.x, p.y, math.hypot(v.x, v.y), time.monotonic())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--course-config', required=True)
    parser.add_argument('--ready-file', type=Path, required=True)
    args = parser.parse_args()
    course = json.loads(Path(args.course_config).read_text(encoding='utf-8'))
    start = course['start']
    rclpy.init(args=['self_drive_referee', '--ros-args', '-p', 'use_sim_time:=true',
                     '-p', 'course_config:='+args.course_config, '-p', 'fallback_odom_topic:=""'])
    node = PoseWait(start['x_m'], start['y_m'])
    try:
        deadline = time.monotonic() + 30
        while rclpy.ok() and not node.stable and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not node.stable:
            raise RuntimeError('ground-truth pose did not stabilize at the track start')
        node.destroy_node()
        node = IgvcCourseMonitor()
        # The controller must not start until the monitor has consumed fresh
        # primary odometry and established its initial scoring pose.
        deadline = time.monotonic() + 10
        while rclpy.ok() and node.primary_odom_sample_count < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.primary_odom_sample_count < 3 or node.failures:
            raise RuntimeError('course referee did not establish a clean initial pose')
        node._publish_score()
        write_json(args.ready_file, {'ready': True, 'primary_odom_samples': node.primary_odom_sample_count})
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
