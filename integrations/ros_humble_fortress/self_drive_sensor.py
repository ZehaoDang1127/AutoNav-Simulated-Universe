"""ROS simulated LiDAR with separately classified crossable paint returns.

The SICK model cannot classify identical white paint. This simulation assumes
that crossable paint has a distinct detectable material response. The label is
applied only to actual ray returns and never supplies the course geometry to
navigation. Hard boundary/obstacle returns retain their original semantics.
"""
from __future__ import annotations
import math
import sys
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from igvc_competition_sim.sensor_harness import IgvcSensorHarness, FLOOR_RSSI
from autonav_universe.self_drive_divider import classified_divider


class ClassifiedSensorHarness(IgvcSensorHarness):
    def __init__(self):
        super().__init__()
        if not any(getattr(t, 'crossable', False) for t in self.course.tapes):
            raise RuntimeError('crossable tape support is absent from the ROS workspace')
        self.divider_pub = self.create_publisher(PointCloud2, '/self_drive/divider_observations', qos_profile_sensor_data)
        self._divider_local = threading.local()

    def _append_cloud_point(self, out, world_x, world_y, z_above_ground,
                            intensity, reflector, layer, pose=None):
        hits = getattr(self._divider_local, 'hits', None)
        if hits is not None and reflector and abs(z_above_ground) < 1e-6:
            matches = [t for t in self._tape_index.get((math.floor(world_x), math.floor(world_y)), ())
                       if self._point_segment_distance(world_x, world_y, t.start, t.end) <= t.width_m * .5]
            if classified_divider(matches):
                # Same ray, same occlusion and pose as the lidar cloud. Mask the
                # hard-reflector channel so the hard-line detector never turns
                # classified crossable paint into a lethal boundary.
                super()._append_cloud_point(hits, world_x, world_y, z_above_ground,
                                            intensity, True, layer, pose)
                return super()._append_cloud_point(out, world_x, world_y, z_above_ground,
                                                   FLOOR_RSSI, False, layer, pose)
        return super()._append_cloud_point(out, world_x, world_y, z_above_ground,
                                           intensity, reflector, layer, pose)

    def _build_cloud_points(self, pose=None):
        self._divider_local.hits = []
        try:
            result = super()._build_cloud_points(pose)
            stamp = getattr(self._divider_local, 'stamp', None)
            if stamp is not None:
                self.divider_pub.publish(self._make_cloud(stamp, self._divider_local.hits))
            return result
        finally:
            del self._divider_local.hits

    def _sensor_pose_sample(self):
        pose, stamp = super()._sensor_pose_sample()
        self._divider_local.stamp = stamp
        return pose, stamp


def main():
    rclpy.init(args=sys.argv)
    node = ClassifiedSensorHarness()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
