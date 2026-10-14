"""The synthetic sensor pose must retain its physical acquisition timestamp."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
import xml.etree.ElementTree as ET
import rclpy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from igvc_competition_sim.physics_imu import add_physical_pose_to_description


class PhysicsPoseTests(unittest.TestCase):
    def test_publisher_preserves_physical_geometry(self):
        source=Path('/ws/install/bringup/share/bringup/description/shogi.urdf').read_text()
        original=ET.fromstring(source)
        text=add_physical_pose_to_description(source,'shogi')
        updated=ET.fromstring(text)
        for tag in ('link','joint'):
            self.assertEqual([ET.tostring(p) for p in original.findall(tag)],
                             [ET.tostring(p) for p in updated.findall(tag)])
        self.assertEqual(add_physical_pose_to_description(text,'shogi'),text)
        plugin=updated.find("gazebo/plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
        self.assertEqual(plugin.findtext('tf_topic'),'/igvc_sim/physics_pose')
        self.assertEqual(plugin.findtext('robot_base_frame'),'shogi')

    def test_native_world_pose_arrives_with_nonzero_acquisition_stamps(self):
        text=add_physical_pose_to_description('<robot name="pose_robot"><link name="base_link"/></robot>',
                                              'pose_robot')
        plugin=ET.tostring(ET.fromstring(text).find('gazebo/plugin'),encoding='unicode')
        world='''<sdf version="1.8"><world name="pose_regression">
        <physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>
        <gravity>0 0 0</gravity>
        <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
        <model name="pose_robot"><pose>3 4 0 0 0 .3</pose><static>true</static>
        <link name="base_link"><inertial><mass>1</mass><inertia><ixx>.1</ixx><iyy>.1</iyy><izz>.1</izz></inertia></inertial></link>
        PLUGIN</model></world></sdf>'''.replace('PLUGIN',plugin)
        env=dict(os.environ,IGN_PARTITION='pose-regression-'+uuid.uuid4().hex)
        rclpy.init()
        node=rclpy.create_node('physics_pose_regression')
        poses=[]
        odometry=[]
        node.create_subscription(TFMessage,'/igvc_sim/physics_pose',poses.append,10)
        node.create_subscription(Odometry,'/igvc_sim/physics_odom',odometry.append,10)
        processes=[]
        try:
            with tempfile.TemporaryDirectory(prefix='physics-pose-') as directory:
                path=Path(directory)/'world.sdf'
                path.write_text(world)
                with tempfile.TemporaryFile(mode='w+') as log:
                    processes.append(subprocess.Popen(['ign','gazebo','-s','-r',str(path)],
                        env=env,stdout=log,stderr=log,start_new_session=True))
                    processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                        '/igvc_sim/physics_pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/igvc_sim/physics_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                        env=env,stdout=log,stderr=log,start_new_session=True))
                    deadline=time.monotonic()+12
                    while min(len(poses),len(odometry))<25 and time.monotonic()<deadline:
                        rclpy.spin_once(node,timeout_sec=.02)
                    log.seek(0)
                    self.assertGreaterEqual(len(poses),25,log.read())
                    self.assertGreaterEqual(len(odometry),25)
                    stamps=[]
                    for msg in poses:
                        self.assertEqual(len(msg.transforms),1)
                        tf=msg.transforms[0]
                        self.assertEqual(tf.child_frame_id,'pose_robot')
                        stamps.append(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
                        self.assertAlmostEqual(tf.transform.translation.x,3.,places=5)
                        self.assertAlmostEqual(tf.transform.translation.y,4.,places=5)
                        self.assertAlmostEqual(tf.transform.rotation.z,math.sin(.15),places=5)
                    self.assertGreater(min(stamps),0.)
                    self.assertGreater(stamps[-1]-stamps[0],.3)
                    self.assertLess(max(b-a for a,b in zip(stamps,stamps[1:])),.061)
                    self.assertTrue(all(m.header.stamp.sec+m.header.stamp.nanosec*1e-9>0 for m in odometry))
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    unittest.main()
