"""Build a rolling semantic divider mask exclusively from live LiDAR returns."""
from __future__ import annotations
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException
from autonav_universe.self_drive_divider import rasterize_observations


def rotate_point(q, x, y, z):
    # General quaternion rotation, including a tilted lidar frame.
    tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
    return (x+q.w*tx+q.y*tz-q.z*ty,
            y+q.w*ty+q.z*tx-q.x*tz,
            z+q.w*tz+q.x*ty-q.y*tx)


class DividerCostmap(Node):
    def __init__(self):
        super().__init__('self_drive_divider_costmap')
        self.template = None
        self.hits = {}
        self.last_cloud_wall = -math.inf
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_map, 10)
        self.create_subscription(PointCloud2, '/self_drive/divider_observations', self.on_cloud, qos_profile_sensor_data)
        self.publisher = self.create_publisher(OccupancyGrid, '/self_drive/divider_costmap', 10)
        self.create_timer(.2, self.publish_map)

    def on_map(self, msg):
        self.template = msg

    def on_cloud(self, msg):
        if not self.template or not msg.header.frame_id:
            return
        try:
            transform = self.tf.lookup_transform(self.template.header.frame_id, msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
        except TransformException:
            return
        q, t = transform.transform.rotation, transform.transform.translation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if not math.isfinite(norm) or abs(norm-1) > .01:
            return
        now = time.monotonic()
        for point in point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            x, y, _ = rotate_point(q, float(point[0]), float(point[1]), float(point[2]))
            x, y = x+t.x, y+t.y
            if math.isfinite(x) and math.isfinite(y):
                self.hits[(round(x/.1), round(y/.1))] = (x, y, now)
        self.last_cloud_wall = now

    def publish_map(self):
        now = time.monotonic()
        if not self.template or now-self.last_cloud_wall > 1.0:
            return  # A stopped sensor must not look like a fresh empty mask.
        self.hits = {key: point for key, point in self.hits.items() if now-point[2] <= 4.0}
        source = self.template
        origin = source.info.origin
        q = origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        points = [(x,y) for x,y,_ in self.hits.values()]
        result = OccupancyGrid()
        result.header = copy.deepcopy(source.header)
        result.info = copy.deepcopy(source.info)
        result.data = list(rasterize_observations(points, source.info.width, source.info.height,
                                                 source.info.resolution,
                                                 (origin.position.x, origin.position.y, yaw)))
        self.publisher.publish(result)


def main():
    rclpy.init()
    node = DividerCostmap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
