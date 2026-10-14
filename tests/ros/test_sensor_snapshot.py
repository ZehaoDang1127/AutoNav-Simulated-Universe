"""Physical pose callbacks must not distort an in-progress synthetic scan."""
import math
import threading
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch
from builtin_interfaces.msg import Time
from igvc_competition_sim.sensor_harness import IgvcSensorHarness


def harness_stub():
    robot=SimpleNamespace(lidar_x_from_base_link_m=.6598,
                          lidar_z_from_base_link_m=.2057,
                          base_link_height_above_ground_m=.113)
    stub=SimpleNamespace(base_x=1., base_y=2., heading=.3, robot=robot,
                         _imu_lock=threading.RLock(), _imu_physics_stamp=Time(sec=4),
                         _true_pose_live=lambda:True)
    for name in ('_sensor_pose_sample','_lidar_origin_world','_world_to_base',
                 '_base_to_lidar','_append_cloud_point','_ground_truth_obstacle_points',
                 '_build_cloud_points','_publish_sensor_frame'):
        setattr(stub,name,MethodType(getattr(IgvcSensorHarness,name),stub))
    stub._cloud_tuple=IgvcSensorHarness._cloud_tuple
    return stub


class SnapshotTests(unittest.TestCase):
    def test_cone_cloud_retains_pose_while_physical_callbacks_advance(self):
        stub=harness_stub()
        pose,stamp=stub._sensor_pose_sample()
        points=[SimpleNamespace(x=3.,y=2.,z=.2,layer=0),
                SimpleNamespace(x=3.,y=2.2,z=.2,layer=1)]
        def advancing_feed(observation_pose):
            self.assertEqual(observation_pose,pose)
            for point in points:
                stub.base_x+=10
                stub.base_y-=10
                stub.heading+=1
                yield point
        stub._raycast_obstacle_hits=advancing_feed
        cloud=stub._ground_truth_obstacle_points(pose)
        for point,actual in zip(points,cloud):
            expected=stub._base_to_lidar(stub._world_to_base(point.x,point.y,point.z,pose))
            for v,w in zip(actual[:3],expected):
                self.assertAlmostEqual(v,w)
        self.assertEqual(stamp.sec,4)

    def test_full_cloud_and_pca_share_acquisition_pose_and_timestamp(self):
        stub=harness_stub()
        pose,stamp=stub._sensor_pose_sample()
        observed=[]
        def full(observation_pose):
            observed.append(observation_pose)
            stub.base_x=100
            stub.heading=2
            stub._imu_physics_stamp=Time(sec=8)
            return ['full']
        def pca(observation_pose):
            observed.append(observation_pose)
            return ['pca']
        published=[]
        stub.publish_full_lidar_cloud=True
        stub._build_cloud_points=full
        stub._ground_truth_obstacle_points=pca
        stub._make_cloud=lambda stamp,points:(stamp.sec,points)
        stub.cloud_pub=SimpleNamespace(publish=published.append)
        stub.pca_gt_pub=SimpleNamespace(publish=published.append)
        stub._publish_scan_fullframe=lambda stamp,points:published.append((stamp.sec,points))
        stub._publish_sensor_frame()
        self.assertEqual(observed,[pose,pose])
        self.assertEqual(published,[(4,['full']),(4,['full']),(4,['pca'])])

    def test_floor_returns_use_the_same_pose_after_ray_casting(self):
        stub=harness_stub()
        pose,_=stub._sensor_pose_sample()
        def moving_feed(observation_pose):
            stub.base_x=100
            stub.base_y=-200
            stub.heading=2
            return []
        stub._raycast_obstacle_hits=moving_feed
        stub._point_on_tape=lambda x,y:False
        with patch.multiple('igvc_competition_sim.sensor_harness',
                            LIDAR_AZIMUTH_MIN_RAD=0.,LIDAR_AZIMUTH_MAX_RAD=0.,
                            MULTISCAN_LAYERS=1,LIDAR_ELEVATION_MIN_RAD=-.1,
                            LIDAR_ELEVATION_MAX_RAD=-.1):
            cloud=stub._build_cloud_points(pose)
        self.assertEqual(len(cloud),1)
        self.assertAlmostEqual(cloud[0][1],0.,places=6)
        self.assertAlmostEqual(cloud[0][0],.3187 / math.tan(.1),places=6)


if __name__=='__main__':
    unittest.main()
