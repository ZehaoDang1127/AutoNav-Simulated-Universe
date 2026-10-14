"""LaserScan projection must use the sensor mount, including its near blind area."""
import ast
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class LidarScanOriginTests(unittest.TestCase):
    def test_scan_range_and_visibility_use_the_forward_sensor_mount(self):
        launch = Path('/ws/install/igvc_competition_sim/share/igvc_competition_sim/launch/igvc_competition.launch.py')
        frames = []
        for node in ast.walk(ast.parse(launch.read_text())):
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    if isinstance(key, ast.Constant) and key.value == 'target_frame':
                        frames.append(ast.literal_eval(value))
        self.assertEqual(frames, ['lidar_planar', 'lidar_planar'])
        rclpy.init()
        node = rclpy.create_node('lidar_origin_regression')
        publisher = node.create_publisher(PointCloud2, '/lidar_origin/cloud', qos_profile_sensor_data)
        observed = []
        node.create_subscription(LaserScan, '/lidar_origin/scan', observed.append, qos_profile_sensor_data)
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'lidar_planar'
        transform.transform.translation.x = .6598
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(['ros2','run','pointcloud_to_laserscan','pointcloud_to_laserscan_node',
                '--ros-args','-r','cloud_in:=/lidar_origin/cloud','-r','scan:=/lidar_origin/scan',
                '-p','target_frame:=lidar_planar','-p','min_height:=-0.1','-p','max_height:=1.5',
                '-p','angle_min:=-1.2217','-p','angle_max:=1.2217','-p','angle_increment:=0.0087',
                '-p','range_min:=0.3','-p','range_max:=25.0','-p','use_inf:=true'],
                stdout=log,stderr=log,start_new_session=True)
            try:
                deadline=time.monotonic()+8
                while not observed and time.monotonic()<deadline:
                    header=Header(stamp=node.get_clock().now().to_msg(),frame_id='base_link')
                    # The near point is in the body's front sector but behind
                    # the sensor. Only the point two metres ahead is visible.
                    publisher.publish(create_cloud_xyz32(header,[(.3,.3,.3),(2.6598,.5,.3)]))
                    rclpy.spin_once(node,timeout_sec=.05)
                    time.sleep(.03)
                log.seek(0)
                self.assertTrue(observed,log.read())
                scan=observed[-1]
                self.assertEqual(scan.header.frame_id,'lidar_planar')
                finite=[r for r in scan.ranges if math.isfinite(r)]
                self.assertEqual(len(finite),1,'A point behind the mount was projected into the visible scan')
                self.assertAlmostEqual(finite[0],math.hypot(2.,.5),places=5)
            finally:
                if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__=='__main__':
    unittest.main()
