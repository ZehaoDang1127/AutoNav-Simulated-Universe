"""Exercise the installed handler's LOCAL and GPS routes with real ROS messages."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.task import Future
from robot_localization.srv import FromLL
from autonav_interfaces.action import NavigateToWaypoint
from gps_waypoint_handler.gps_handler_node import GpsHandlerNode
from tf2_ros import LookupException


class GoalDispatchTests(unittest.TestCase):
    def make_node(self, goal_type):
        node = GpsHandlerNode.__new__(GpsHandlerNode)
        node._lock = threading.RLock()
        node._active = NS(
            handle=object(), goal_type=goal_type, goal_world_xy=(10.0, 0.0),
            final_yaw=None, last_published_goal_map=None, last_published_goal_world=None,
            last_published_t_s=None, last_published_theta=None, last_goal_pose_t_s=0.0,
            nav2_goal_sent=False, nav2_goal_accepted=None, nav2_goal_handle=None,
            nav2_done_status=None, nav2_cancel_requested=False,
        )
        node._publisher_disabled = False
        node._map_frame = 'map'
        node._odom_frame = 'odom'
        node._tf_timeout_s = 0.1
        node._ekf = NS(pos_xy=(0.0, 0.0), theta=0.0, theta_std_rad=0.01)
        node._smoothed_candidate = None
        node._last_odom_xy = (0.0, 0.0)
        node._bootstrap_done = True
        node._use_navsat_projection = False
        node._now_s = Mock(return_value=10.0)
        node.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: Time(sec=10))))
        node._nav2_client = Mock()
        node._nav2_client.server_is_ready.return_value = True
        node._goal_pub = Mock()  # Tripwire for the diagnosed duplicate topic dispatch.
        node._goal_update_pub = Mock()
        node._marker_pub = Mock()
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        node._tf_buffer = Mock()
        node._tf_buffer.lookup_transform.return_value = transform
        return node

    def test_projected_gps_target_stays_fixed_when_odometry_heading_drifts(self):
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_GPS)
        node._use_navsat_projection = True
        node._active.goal_lat_lon = (37.2303, -80.42463579)
        node._active.projected_goal_map = None
        node._active.projected_goal_future = None
        node._navsat_client = Mock()
        node._navsat_client.service_is_ready.return_value = False
        node._publisher_tick()
        node._nav2_client.send_goal_async.assert_not_called()
        future = Future()
        node._navsat_client.service_is_ready.return_value = True
        node._navsat_client.call_async.return_value = future
        node._publisher_tick()
        node._nav2_client.send_goal_async.assert_not_called()
        response = FromLL.Response()
        response.map_point.x, response.map_point.y = 32.359, -.075
        future.set_result(response)
        node._publisher_tick()
        goal = node._nav2_client.send_goal_async.call_args.args[0].pose.pose.position
        self.assertAlmostEqual(goal.x, 32.359)
        self.assertAlmostEqual(goal.y, -.075)
        node._ekf.theta = 1.0
        node._last_odom_xy = (100.0, -10.0)
        node._smoothed_candidate = (36.084, 9.216)
        node._publisher_tick()
        updated = node._goal_update_pub.publish.call_args.args[0].pose.position
        self.assertAlmostEqual(updated.x, 32.359)
        self.assertAlmostEqual(updated.y, -.075)
        node._nav2_client.send_goal_async.assert_called_once()
        node._navsat_client.call_async.assert_called_once()
        self.assertEqual(node._active.goal_world_xy, (10.0, 0.0))

    def test_late_gps_projection_does_not_dispatch_a_replaced_goal(self):
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_GPS)
        replaced = node._active
        node._active = object()
        node._publish_projected_gps_goal(replaced)
        node._nav2_client.send_goal_async.assert_not_called()

    def test_local_and_gps_dispatch_once_then_update(self):
        for goal_type in (NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL,
                          NavigateToWaypoint.Goal.GOAL_TYPE_GPS):
            with self.subTest(goal_type=goal_type):
                node = self.make_node(goal_type)
                node._publisher_tick()
                node._nav2_client.send_goal_async.assert_called_once()
                node._goal_pub.publish.assert_not_called()
                node._now_s.return_value = 11.0
                node._publisher_tick()
                node._nav2_client.send_goal_async.assert_called_once()
                node._goal_update_pub.publish.assert_called_once()
                node._goal_pub.publish.assert_not_called()
                # A genuine Nav2 failure may retry once through the same action.
                node._reset_nav2_for_redispatch(node._active)
                node._now_s.return_value = 12.0
                node._publisher_tick()
                self.assertEqual(node._nav2_client.send_goal_async.call_count, 2)
                node._goal_pub.publish.assert_not_called()

    def test_projected_arrival_uses_antenna_and_shared_map_datum(self):
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_GPS)
        node._use_navsat_projection = True
        node._active.projected_goal_map = (10.0, 0.0)
        # The independent first-fix estimator has a different origin.
        node._ekf.pos_xy = (25.0, 0.0)
        transform = TransformStamped()
        transform.transform.translation.x = 8.9
        node._tf_buffer.lookup_transform.return_value = transform
        self.assertAlmostEqual(node._goal_distance_m(node._active), 1.1)
        self.assertEqual(node._tf_buffer.lookup_transform.call_args.args[1], 'gps_footprint')
        transform.transform.translation.x = 9.1
        self.assertAlmostEqual(node._goal_distance_m(node._active), .9)

    def test_projected_arrival_waits_for_projection_and_antenna_tf(self):
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_GPS)
        node._use_navsat_projection = True
        node._active.projected_goal_map = None
        self.assertIsNone(node._goal_distance_m(node._active))
        node._active.projected_goal_map = (10.0, 0.0)
        node._tf_buffer.lookup_transform.side_effect = LookupException('missing antenna TF')
        self.assertIsNone(node._goal_distance_m(node._active))

    def test_native_gps_arrival_rotates_antenna_offset(self):
        import math
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_GPS)
        node._ekf.pos_xy = (9.1, 0.0)
        node._gps_link_offset_xy = (-.2122, 0.0)
        node._last_odom_yaw = 0.0
        self.assertAlmostEqual(node._goal_distance_m(node._active), 1.1122)
        node._last_odom_yaw = math.pi
        self.assertAlmostEqual(node._goal_distance_m(node._active), .6878)
        node._gps_link_offset_xy = None
        self.assertIsNone(node._goal_distance_m(node._active))

    def test_local_arrival_keeps_base_reference(self):
        node = self.make_node(NavigateToWaypoint.Goal.GOAL_TYPE_LOCAL)
        transform = TransformStamped()
        transform.transform.translation.x = 9.1
        node._tf_buffer.lookup_transform.return_value = transform
        self.assertAlmostEqual(node._goal_distance_m(node._active), .9)


if __name__ == '__main__':
    unittest.main()
