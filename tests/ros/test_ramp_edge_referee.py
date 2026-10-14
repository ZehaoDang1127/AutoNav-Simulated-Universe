"""Strict ramp-edge contact must not flag a robot wholly beside the deck."""
import math
from types import SimpleNamespace
import unittest
from igvc_competition_sim.course_monitor import IgvcCourseMonitor


class RampEdgeRefereeTests(unittest.TestCase):
    def monitor(self, rotated=False):
        monitor = IgvcCourseMonitor.__new__(IgvcCourseMonitor)
        monitor.robot = SimpleNamespace(base_link_to_nav_center_m=.225,
            physical_half_length_m=.545, physical_half_width_m=.41, footprint_padding_m=.05)
        ramp = SimpleNamespace(name='Ramp', start_x_m=-2.5693, end_x_m=1.3898,
            center_x_m=-.5897 if rotated else 0., center_y_m=3.6169 if rotated else 0.,
            yaw_rad=1.7357 if rotated else 0., run_length_m=3.9591, width_m=3.2157)
        monitor.course = SimpleNamespace(tapes=[], obstacles=[], ramps=[ramp])
        monitor.ramp_monitor_lateral_margin_m = 1.
        monitor.robot_sweep_radius_m = math.hypot(.77+.05, .41+.05)
        monitor.sample_spacing_m = .02
        monitor.failures = []
        monitor._fail = lambda reason, **details: monitor.failures.append((reason, details))
        return monitor

    def test_recorded_robot_beside_rotated_ramp_is_clear(self):
        monitor = self.monitor(rotated=True)
        monitor._check_course_contact(2.713, 3.189, 1.889)
        self.assertEqual(monitor.failures, [], 'A body one metre outside the deck was called a departure')

    def test_deck_and_longitudinal_open_ends_are_clear(self):
        monitor = self.monitor()
        for pose in [(0., 0., 0.), (-2.1, 0., 0.), (2.1, 0., 0.), (0., 3., 0.)]:
            monitor._check_course_contact(*pose)
        self.assertEqual(monitor.failures, [])

    def test_both_actual_padded_body_edge_crossings_fail(self):
        for side in [-1, 1]:
            monitor = self.monitor()
            monitor._check_course_contact(0., side*1.2, 0.)
            self.assertTrue(monitor.failures)
            self.assertEqual(monitor.failures[0][0], 'ramp_edge_departure:Ramp')

    def test_swept_departure_detects_edge_between_clear_endpoints(self):
        monitor = self.monitor()
        monitor._check_course_contact_swept((0., 0., 0.), (0., 3., 0.))
        self.assertTrue(monitor.failures, 'Swept ramp departure missed an edge between samples')

    def test_rotated_corner_contact_fails(self):
        monitor = self.monitor(rotated=True)
        ramp = monitor.course.ramps[0]
        # Place the body near the east side in ramp coordinates.
        local_x, local_y = 0., 1.2
        x = ramp.center_x_m + local_x*math.cos(ramp.yaw_rad)-local_y*math.sin(ramp.yaw_rad)
        y = ramp.center_y_m + local_x*math.sin(ramp.yaw_rad)+local_y*math.cos(ramp.yaw_rad)
        monitor._check_course_contact(x, y, ramp.yaw_rad)
        self.assertTrue(monitor.failures)


if __name__ == '__main__':
    unittest.main()
