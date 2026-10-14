"""An overlapping finish circle cannot stop strict scoring early."""
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from nav_msgs.msg import Odometry
from igvc_competition_sim.course_monitor import IgvcCourseMonitor


class OrderedFinishTests(unittest.TestCase):
    def monitor(self):
        monitor = IgvcCourseMonitor.__new__(IgvcCourseMonitor)
        waypoint = NS(kind='gps', x_m=0.0, y_m=0.0, radius_m=1.0)
        monitor.course = NS(finish=(0.0, 0.0, 1.25), mission_waypoints=[waypoint]*5)
        monitor.robot = NS(gps_x_from_base_link_m=-.2122, gps_y_from_base_link_m=-.0001)
        monitor.next_waypoint_index = 4
        monitor.waypoints_reached = [True]*4 + [False]
        monitor.finish_armed = True
        monitor.finish_reached = False
        monitor.finish_reentry_margin_m = .5
        monitor.robot_sweep_radius_m = .9
        monitor.sample_spacing_m = .02
        monitor.primary_odom_seen = True
        monitor.primary_odom_sample_count = 0
        monitor.odom_sample_count = 0
        monitor.odom_source = 'primary'
        monitor.last_pose = None
        monitor.last_time_s = None
        monitor.max_speed_mps = 0.0
        monitor.distance_m = 0.0
        monitor.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=0))))
        monitor._check_course_contact_swept = Mock()
        monitor._update_speed_checks = Mock()
        return monitor

    def test_finish_overlap_keeps_waypoint_contact_and_speed_checks_active(self):
        monitor = self.monitor()
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.position.x = -1.1
        monitor._odom_callback(odom)
        self.assertFalse(monitor.finish_reached)
        self.assertEqual(monitor.next_waypoint_index, 4)
        # Still inside the finish; the GPS antenna now reaches the final marker.
        odom.pose.pose.position.x = -.7
        monitor._odom_callback(odom)
        self.assertTrue(monitor.finish_reached)
        self.assertEqual(monitor.next_waypoint_index, 5)
        self.assertEqual(monitor.waypoints_reached, [True]*5)
        self.assertEqual(monitor._check_course_contact_swept.call_count, 2)
        self.assertEqual(monitor._update_speed_checks.call_count, 2)

    def test_finish_does_not_skip_missing_ordered_waypoint(self):
        monitor = self.monitor()
        monitor.next_waypoint_index = 0
        monitor.course.mission_waypoints[0] = NS(kind='gps', x_m=10.0, y_m=0.0, radius_m=1.0)
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        monitor._odom_callback(odom)
        self.assertFalse(monitor.finish_reached)
        self.assertEqual(monitor.next_waypoint_index, 0)


if __name__ == '__main__':
    unittest.main()
