"""Verify gyro-only measurements and reject slip-biased wheel yaw in the EKF."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from igvc_competition_sim.sensor_harness import IgvcSensorHarness


class SensorEkfTests(unittest.TestCase):
    def test_global_ekf_corrects_translation_slip_with_gps(self):
        rclpy.init()
        node = rclpy.create_node('gps_ekf_regression')
        velocity_pub = node.create_publisher(Odometry, '/gps_regression/velocity', 10)
        gps_pub = node.create_publisher(Odometry, '/gps_regression/gps', 10)
        observed = []
        subscription = node.create_subscription(Odometry, '/gps_regression/output', observed.append, 10)
        config = '/ws/install/igvc_competition_sim/share/igvc_competition_sim/config/global_sensor_ekf.yaml'
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                '-r', '__node:=global_ekf', '--params-file', config,
                '-p', 'use_sim_time:=false', '-p', 'publish_tf:=false',
                '-r', '/local_ekf/odom:=/gps_regression/velocity',
                '-r', '/odometry/gps:=/gps_regression/gps',
                '-r', 'odometry/filtered:=/gps_regression/output',
            ], stdout=log, stderr=log, start_new_session=True)
            try:
                ready = time.monotonic() + 8.0
                while time.monotonic() < ready and not (
                        velocity_pub.get_subscription_count() and gps_pub.get_subscription_count()):
                    rclpy.spin_once(node, timeout_sec=.05)
                log.seek(0)
                self.assertGreater(gps_pub.get_subscription_count(), 0, log.read())
                start = time.monotonic()
                while time.monotonic() - start < 2.2:
                    elapsed = time.monotonic() - start
                    velocity = Odometry()
                    velocity.header.stamp = node.get_clock().now().to_msg()
                    velocity.header.frame_id = 'map'
                    velocity.child_frame_id = 'base_link'
                    velocity.pose.pose.orientation.w = 1.0
                    velocity.pose.covariance[35] = .001
                    velocity.twist.twist.linear.x = .3  # 3x true translation.
                    velocity.twist.covariance[0] = velocity.twist.covariance[7] = .01
                    velocity.twist.covariance[35] = .001
                    gps = Odometry()
                    gps.header = velocity.header
                    gps.child_frame_id = 'base_link'
                    gps.pose.pose.orientation.w = 1.0
                    gps.pose.pose.position.x = .1 * elapsed
                    gps.pose.covariance[0] = gps.pose.covariance[7] = .01
                    velocity_pub.publish(velocity)
                    gps_pub.publish(gps)
                    rclpy.spin_once(node, timeout_sec=.02)
                    time.sleep(.01)
                self.assertTrue(observed, 'Global EKF did not publish')
                self.assertAlmostEqual(observed[-1].pose.pose.position.x, .22, delta=.08)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()

    def test_gyro_wraps_yaw_without_exposing_absolute_pose(self):
        messages = []
        stub = SimpleNamespace(
            _imu_lock=threading.RLock(),
            _imu_physics_yaw=-math.pi + .1,
            _imu_physics_stamp=Time(sec=2),
            _imu_previous_yaw=math.pi - .1,
            _imu_previous_s=1.0,
            heading=123.0,  # Dead-reckoned wheel yaw must not feed this gyro.
            imu_pub=SimpleNamespace(publish=messages.append))
        with patch('igvc_competition_sim.sensor_harness.random.gauss', return_value=0.0):
            IgvcSensorHarness._publish_imu(stub, Time(sec=2))
        message = messages[0]
        self.assertAlmostEqual(message.angular_velocity.z, .2)
        self.assertEqual(message.orientation_covariance[0], -1.0)
        self.assertEqual(message.linear_acceleration_covariance[0], -1.0)
        self.assertEqual(message.orientation.w, 1.0)

    def test_gyro_uses_new_physics_samples_without_timer_rate_spikes(self):
        messages = []
        stub = SimpleNamespace(
            _imu_lock=threading.RLock(),
            _imu_physics_yaw=.02, _imu_physics_stamp=Time(sec=1, nanosec=100000000),
            _imu_previous_yaw=0.0, _imu_previous_s=1.0,
            imu_pub=SimpleNamespace(publish=messages.append))
        with patch('igvc_competition_sim.sensor_harness.random.gauss', return_value=0.0):
            IgvcSensorHarness._publish_imu(stub, Time(sec=1, nanosec=120000000))
            self.assertAlmostEqual(messages[-1].angular_velocity.z, .2)
            self.assertEqual(messages[-1].header.stamp.nanosec, 100000000)
            IgvcSensorHarness._publish_imu(stub, Time(sec=1, nanosec=140000000))
            self.assertEqual(len(messages), 1, 'Held physics pose generated a spurious zero rate')
            IgvcSensorHarness._publish_imu(stub, Time(sec=1, nanosec=250000000))
            self.assertEqual(messages[-1].angular_velocity.z, 0.0)
            self.assertAlmostEqual(stub._imu_previous_s, 1.25)

    def test_gyro_resumption_keeps_turn_after_stationary_heartbeats(self):
        messages = []
        stub = SimpleNamespace(
            _imu_lock=threading.RLock(),
            _imu_physics_yaw=0.0, _imu_physics_stamp=Time(sec=1),
            _imu_previous_yaw=0.0, _imu_previous_s=1.0,
            imu_pub=SimpleNamespace(publish=messages.append))
        with patch('igvc_competition_sim.sensor_harness.random.gauss', return_value=0.0):
            IgvcSensorHarness._publish_imu(stub, Time(sec=3))
            self.assertEqual(messages[-1].angular_velocity.z, 0.0)
            stub._imu_physics_yaw = .01
            stub._imu_physics_stamp = Time(sec=3, nanosec=20000000)
            IgvcSensorHarness._publish_imu(stub, stub._imu_physics_stamp)
        self.assertAlmostEqual(messages[-1].angular_velocity.z, .5)
        self.assertAlmostEqual(messages[-1].angular_velocity.z * .02, .01)

    def test_physics_callback_publishes_gyro_between_odom_timer_ticks(self):
        messages = []
        stub = SimpleNamespace(
            robot_model_name='robot', _imu_lock=threading.RLock(),
            _imu_physics_yaw=None, _imu_physics_stamp=None,
            _imu_previous_yaw=None, _imu_previous_s=None,
            imu_pub=SimpleNamespace(publish=messages.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=1, nanosec=20000000))),
            get_logger=lambda: SimpleNamespace(info=lambda *args, **kwargs: None))
        stub._publish_imu = lambda stamp: IgvcSensorHarness._publish_imu(stub, stamp)
        with patch('igvc_competition_sim.sensor_harness.random.gauss', return_value=0.0):
            for nanosec, yaw in ((0, 0.0), (20000000, .01)):
                transform = TransformStamped()
                transform.child_frame_id = 'robot'
                transform.header.stamp = Time(sec=1, nanosec=nanosec)
                transform.transform.rotation.z = math.sin(yaw/2)
                transform.transform.rotation.w = math.cos(yaw/2)
                IgvcSensorHarness._true_pose_callback(
                    stub, TFMessage(transforms=[transform]))
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[-1].header.stamp.nanosec, 20000000)
        self.assertAlmostEqual(messages[-1].angular_velocity.z, .5)
        self.assertEqual(messages[-1].orientation_covariance[0], -1.0)

    def test_gyro_waits_for_a_physics_sample(self):
        messages = []
        stub = SimpleNamespace(_imu_lock=threading.RLock(), _imu_physics_yaw=None,
                               imu_pub=SimpleNamespace(publish=messages.append))
        IgvcSensorHarness._publish_imu(stub, Time(sec=1))
        self.assertEqual(messages, [])

    def test_local_ekf_follows_gyro_when_wheel_yaw_slips(self):
        rclpy.init()
        node = rclpy.create_node('sensor_ekf_regression')
        wheel_pub = node.create_publisher(Odometry, '/ekf_regression/wheel', 10)
        imu_pub = node.create_publisher(Imu, '/ekf_regression/imu', 10)
        observed = []
        subscription = node.create_subscription(Odometry, '/ekf_regression/output', observed.append, 10)
        config = Path('/ws/install/igvc_competition_sim/share/igvc_competition_sim/config/local_sensor_ekf.yaml')
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args',
                '-r', '__node:=ekf_node', '--params-file', str(config),
                '-p', 'use_sim_time:=false',
                '-r', '/odom:=/ekf_regression/wheel',
                '-r', '/imu/data:=/ekf_regression/imu',
                '-r', 'odometry/filtered:=/ekf_regression/output',
                '-r', '/tf:=/ekf_regression/tf',
            ], stdout=log, stderr=log, start_new_session=True)
            try:
                ready = time.monotonic() + 8.0
                while time.monotonic() < ready and not (
                        wheel_pub.get_subscription_count() and imu_pub.get_subscription_count()):
                    rclpy.spin_once(node, timeout_sec=.05)
                log.seek(0)
                startup_log = log.read()
                self.assertGreater(wheel_pub.get_subscription_count(), 0, startup_log)
                self.assertGreater(imu_pub.get_subscription_count(), 0, startup_log)
                start = time.monotonic()
                while time.monotonic() - start < 2.2:
                    elapsed = time.monotonic() - start
                    wheel = Odometry()
                    wheel.header.stamp = node.get_clock().now().to_msg()
                    wheel.header.frame_id = 'odom'
                    wheel.child_frame_id = 'base_link'
                    # Wheel yaw/rate are 4x too large; only vx/vy are fused.
                    wheel.pose.pose.orientation.z = math.sin(.8*elapsed/2)
                    wheel.pose.pose.orientation.w = math.cos(.8*elapsed/2)
                    wheel.twist.twist.linear.x = .1
                    wheel.twist.twist.angular.z = .8
                    wheel.twist.covariance[0] = wheel.twist.covariance[7] = .01
                    wheel.twist.covariance[35] = .04
                    imu = Imu()
                    imu.header.stamp = wheel.header.stamp
                    imu.header.frame_id = 'base_link'
                    imu.orientation_covariance[0] = -1.0
                    imu.angular_velocity.z = .2
                    imu.angular_velocity_covariance[8] = .0001
                    imu.linear_acceleration_covariance[0] = -1.0
                    wheel_pub.publish(wheel)
                    imu_pub.publish(imu)
                    rclpy.spin_once(node, timeout_sec=.02)
                    time.sleep(.01)
                self.assertTrue(observed, 'EKF did not publish odometry')
                pose = observed[-1].pose.pose
                q = pose.orientation
                yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
                self.assertGreater(yaw, .30)
                self.assertLess(yaw, .60, 'EKF used slip-biased wheel yaw')
                self.assertGreater(pose.position.x, .14)
                self.assertLess(pose.position.x, .29)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
