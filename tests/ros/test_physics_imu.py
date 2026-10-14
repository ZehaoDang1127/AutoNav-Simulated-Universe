"""Check native angular rate through turn, stop, and resume in Fortress."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from types import SimpleNamespace
import unittest
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from igvc_competition_sim.physics_imu import (
    PhysicsGyro, add_imu_system, add_imu_to_description)


class PhysicsImuTests(unittest.TestCase):
    def test_sensor_addition_preserves_robot_and_course_geometry(self):
        source = Path('/ws/install/bringup/share/bringup/description/shogi.urdf').read_text()
        original = ET.fromstring(source)
        updated = ET.fromstring(add_imu_to_description(source))
        for tag in ('link', 'joint'):
            self.assertEqual([ET.tostring(p) for p in original.findall(tag)],
                             [ET.tostring(p) for p in updated.findall(tag)])
        sensor = updated.find("gazebo[@reference='base_link']/sensor[@type='imu']")
        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.findtext('update_rate'), '50')
        world = Path('/host/runs/full-track-diagnostic/world.sdf').read_text()
        before, after = ET.fromstring(world), ET.fromstring(add_imu_system(world))
        self.assertEqual([ET.tostring(p) for p in before.findall('world/model')],
                         [ET.tostring(p) for p in after.findall('world/model')])
        self.assertEqual(add_imu_system(add_imu_system(world)), add_imu_system(world))

    def test_adapter_publishes_only_angular_rate_in_the_body_frame(self):
        observed = []
        source = Imu()
        source.header.stamp.sec = 5
        source.orientation.z = .6
        source.angular_velocity.z = .5
        source.linear_acceleration.x = 2.0
        PhysicsGyro.callback(SimpleNamespace(publisher=SimpleNamespace(publish=observed.append)), source)
        message = observed[0]
        self.assertEqual(message.header.frame_id, 'base_link')
        self.assertEqual(message.header.stamp.sec, 5)
        self.assertEqual(message.angular_velocity.z, .5)
        self.assertEqual(message.orientation_covariance[0], -1.0)
        self.assertEqual(message.orientation.z, 0.0)
        self.assertEqual(message.linear_acceleration_covariance[0], -1.0)
        self.assertEqual(message.linear_acceleration.x, 0.0)

    def test_native_rate_is_continuous_when_rotation_stops_and_resumes(self):
        sensor = ET.fromstring(add_imu_to_description(
            '<robot name="test"><link name="base_link"/></robot>')).find('gazebo/sensor')
        world = '''<sdf version="1.8"><world name="imu_regression">
        <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
        <gravity>0 0 0</gravity>
        <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
        <model name="rotor">
          <link name="base"><inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial></link>
          <link name="spin"><inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial>SENSOR</link>
          <joint name="anchor" type="fixed"><parent>world</parent><child>base</child></joint>
          <joint name="rotation" type="revolute"><parent>base</parent><child>spin</child><axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>
          <plugin filename="ignition-gazebo-joint-controller-system" name="ignition::gazebo::systems::JointController"><joint_name>rotation</joint_name><initial_velocity>.5</initial_velocity><topic>/imu_test/velocity</topic></plugin>
        </model></world></sdf>'''.replace('SENSOR', ET.tostring(sensor, encoding='unicode'))
        env = dict(os.environ, IGN_PARTITION='imu-regression-'+uuid.uuid4().hex)
        rclpy.init()
        node = PhysicsGyro()
        messages = []
        node.create_subscription(Imu, '/imu/data', messages.append, qos_profile_sensor_data)
        processes = []
        with tempfile.TemporaryDirectory(prefix='physics-imu-') as directory:
            path = Path(directory)/'world.sdf'
            path.write_text(add_imu_system(world))
            with tempfile.TemporaryFile(mode='w+') as log:
                try:
                    processes.append(subprocess.Popen(['ign','gazebo','-s','-r',str(path)],
                        env=env, stdout=log, stderr=log, start_new_session=True))
                    processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                        '/igvc_sim/physics_imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                        env=env, stdout=log, stderr=log, start_new_session=True))
                    deadline = time.monotonic()+12
                    while len(messages)<25 and time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.05)
                    log.seek(0)
                    self.assertGreaterEqual(len(messages),25,log.read())
                    def collect(duration=1):
                        messages.clear()
                        end=time.monotonic()+duration
                        while time.monotonic()<end:
                            rclpy.spin_once(node,timeout_sec=.02)
                        return list(messages)
                    for target in (.5,0.,.5):
                        subprocess.run(['ign','topic','-t','/imu_test/velocity','-m','ignition.msgs.Double',
                                        '-p',f'data: {target}'],env=env,check=True,timeout=5,
                                       stdout=subprocess.DEVNULL,stderr=log)
                        collect(.3)
                        samples=collect()
                        self.assertGreater(len(samples),30,'Native gyro feed became sparse')
                        self.assertAlmostEqual(sum(m.angular_velocity.z for m in samples)/len(samples),target,delta=.02)
                        stamps=[m.header.stamp.sec+1e-9*m.header.stamp.nanosec for m in samples]
                        self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.061)
                finally:
                    for process in processes:
                        if process.poll() is None:
                            os.killpg(process.pid,signal.SIGINT)
                    for process in processes:
                        try: process.wait(timeout=4)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGKILL)
                            process.wait()
                    node.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
