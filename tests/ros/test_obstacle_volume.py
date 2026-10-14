"""Observed circular surfaces must exclude their solid interiors too."""
import math
import unittest
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from igvc_competition_sim.obstacle_volume import round_obstacle_volumes, ObstacleVolume

class ObstacleVolumeTests(unittest.TestCase):
    def arc(self,center,radius):
        angles=np.linspace(2.0,4.3,24)
        return np.c_[center[0]+radius*np.cos(angles),center[1]+radius*np.sin(angles),
                     np.full(len(angles),.3)]

    def test_unseen_side_and_center_are_solid_from_observed_arc(self):
        source=self.arc((2.,1.),.3831)
        filled,fits=round_obstacle_volumes(source)
        self.assertEqual(len(fits),1)
        self.assertAlmostEqual(fits[0]['radius'],.3831,places=6)
        self.assertLess(np.linalg.norm(filled[:,:2]-[2.,1.],axis=1).min(),.03)
        self.assertLess(np.linalg.norm(filled[:,:2]-[2.35,1.],axis=1).min(),.03)
        for p in source:self.assertLess(np.linalg.norm(filled-p,axis=1).min(),1e-6)
        self.assertLessEqual(np.linalg.norm(filled[:,:2]-[2.,1.],axis=1).max(),.4082)

    def test_clusters_rotated_sensor_and_repeated_vertical_layers(self):
        points=np.r_[self.arc((2,1),.3),self.arc((4,-1),.45)]
        rotation=np.array([[math.cos(.9),-math.sin(.9)],[math.sin(.9),math.cos(.9)]])
        points[:,:2]=points[:,:2]@rotation.T+[-1,2]
        upper=points.copy();upper[:,2]=.8
        filled,fits=round_obstacle_volumes(np.r_[points,upper])
        self.assertEqual(len(fits),2)
        self.assertTrue(np.allclose(sorted(f['radius'] for f in fits),[.3,.45]))
        self.assertGreater(len(filled),len(points)*2)

    def test_sparse_straight_and_non_round_returns_do_not_invent_volume(self):
        for points in [[],[[1,2,.3]],self.arc((2,1),.4)[:4],
                       np.c_[np.linspace(1,2,20),np.ones(20),np.full(20,.3)],
                       np.c_[2+.8*np.cos(np.linspace(2,4,25)),
                             1+.2*np.sin(np.linspace(2,4,25)),np.full(25,.3)]]:
            filled,fits=round_obstacle_volumes(points)
            self.assertFalse(fits)
            self.assertEqual(len(filled),len(points))

    def test_runtime_preserves_acquisition_header_and_sensor_visibility(self):
        rclpy.init()
        volume=ObstacleVolume()
        probe=rclpy.create_node('obstacle_volume_regression')
        publisher=probe.create_publisher(PointCloud2,'/scan_pca_filtered_points',qos_profile_sensor_data)
        observed=[]
        probe.create_subscription(PointCloud2,'/scan_pca_obstacle_volume_points',observed.append,qos_profile_sensor_data)
        broadcaster=StaticTransformBroadcaster(probe)
        mount=TransformStamped();mount.header.frame_id='base_link';mount.child_frame_id='lidar_footprint'
        mount.transform.translation.x=.6598;mount.transform.translation.z=.20568
        mount.transform.rotation.x=1.;mount.transform.rotation.w=0.
        planar=TransformStamped();planar.header.frame_id='base_link';planar.child_frame_id='lidar_planar'
        planar.transform.translation.x=.6598;planar.transform.rotation.w=1.
        broadcaster.sendTransform([mount,planar])
        header=Header(stamp=probe.get_clock().now().to_msg(),frame_id='lidar_footprint')
        points=np.r_[self.arc((2,1),.3831),self.arc((-2,1),.3)]
        try:
            deadline=time.monotonic()+5
            while not observed and time.monotonic()<deadline:
                publisher.publish(point_cloud2.create_cloud_xyz32(header,points.tolist()))
                rclpy.spin_once(volume,timeout_sec=.02)
                rclpy.spin_once(probe,timeout_sec=.02)
            self.assertTrue(observed,'No filled obstacle cloud published')
            self.assertEqual(observed[-1].header.stamp,header.stamp)
            self.assertEqual(observed[-1].header.frame_id,'lidar_planar')
            output=np.array([tuple(p) for p in point_cloud2.read_points(observed[-1],field_names=('x','y','z'))])
            self.assertGreater(len(output),len(points))
            self.assertTrue((output[:,0]>0).all(),'Out-of-sector points entered the volume map')
            self.assertTrue((output[:,2]==0).all(),'Inverted sensor height bypassed ground projection')
        finally:
            probe.destroy_node();volume.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
