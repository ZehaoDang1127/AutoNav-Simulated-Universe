#!/usr/bin/env python3
"""Record which compact-run costmap feed marks the planner's start cell."""

import json
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


TOPICS = {
    "global": "/global_costmap/costmap",
    "local": "/local_costmap/costmap",
    "camera_lines": "/line_costmap",
    "lidar_lines": "/lidar_line_costmap",
    "padded_map": "/map_padded",
}


class CostmapProbe(Node):
    def __init__(self):
        super().__init__("compact_costmap_probe")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.grids = {}
        self.truth = None
        for name, topic in TOPICS.items():
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, key=name: self.grids.__setitem__(key, msg), 10)
        self.create_subscription(
            Odometry, "/igvc_sim/ground_truth_odom", self.on_truth, 10)
        self.create_timer(2.0, self.report)

    def on_truth(self, msg):
        self.truth = [round(msg.pose.pose.position.x, 3),
                      round(msg.pose.pose.position.y, 3),
                      round(math.hypot(msg.twist.twist.linear.x,
                                       msg.twist.twist.linear.y), 3)]

    def cell(self, grid):
        frame = grid.header.frame_id
        if not frame or grid.info.resolution <= 0:
            return None
        try:
            tf = self.buffer.lookup_transform(frame, "nav_center", Time())
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        ix = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
        iy = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= ix < grid.info.width and 0 <= iy < grid.info.height):
            return {"in_grid": False}
        index = iy * grid.info.width + ix
        return {"value": int(grid.data[index]),
                "x": round(x, 3), "y": round(y, 3),
                "cell": [ix, iy]}

    def report(self):
        values = {name: self.cell(grid) for name, grid in self.grids.items()}
        print(json.dumps({"wall_s": round(time.time(), 2),
                          "truth_x_y_speed": self.truth,
                          "start_cell": values}, sort_keys=True), flush=True)


def main():
    rclpy.init()
    node = CostmapProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
