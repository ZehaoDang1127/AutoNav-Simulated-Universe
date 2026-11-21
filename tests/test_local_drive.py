"""Local controller tests, independent of ROS installation."""
import math
import random
import unittest
from unittest.mock import patch

import autonav_universe.local_drive as local_drive
from autonav_universe.local_drive import (LocalDrivePlanner, LocalGrid, ManeuverPlanner,
                                         divider_crossings, integrated_clearance_cost,
                                         integrated_heading_deviation, unknown_traversal_cost)


def grid(obstacles=(), unknown=False, resolution=.1):
    width = height = 100
    origin = -5.0
    cells = []
    for iy in range(height):
        for ix in range(width):
            x = origin+(ix+.5)*resolution
            y = origin+(iy+.5)*resolution
            value = -1 if unknown else 0
            if any(math.hypot(x-ox, y-oy) <= radius for ox, oy, radius in obstacles):
                value = 100
            cells.append(value)
    return LocalGrid(width, height, resolution, origin, origin, 0.0, tuple(cells))


def divider_grid(resolution=.1):
    width = height = round(10/resolution)
    origin = -5.0
    cells = tuple(100 if abs(origin+(iy+.5)*resolution) < .11 else 0
                  for iy in range(height) for _ in range(width))
    return LocalGrid(width, height, resolution, origin, origin, 0.0, cells)


class LocalDriveTests(unittest.TestCase):
    def setUp(self):
        self.planner = LocalDrivePlanner()

    def test_clear_map_drives_forward(self):
        plan = self.planner.plan(grid(), 0, 0, 0)
        self.assertEqual(plan.reason, 'ok')
        self.assertGreater(plan.linear, 0)
        self.assertEqual(plan.angular, 0)

    def test_heading_cost_distinguishes_excursion_from_straight_path(self):
        straight = [0.0] * 5
        excursion = [0.0, .2, .4, .2, 0.0]
        # Both have the same start and finish heading, but the second path
        # swings away from the observed lane direction in between.
        self.assertEqual(integrated_heading_deviation(straight, 0.0, .1), 0.0)
        self.assertGreater(integrated_heading_deviation(excursion, 0.0, .1), 0.0)
        self.assertEqual(integrated_heading_deviation(excursion, None, .1), 0.0)

    def test_small_heading_error_has_fine_safe_correction(self):
        course = grid()
        plan = self.planner.plan(course, 0, 0, .08, guidance_yaw=0.0)
        self.assertEqual(plan.reason, 'ok')
        self.assertLessEqual(abs(plan.angular), .12)
        self.assertIsNotNone(self.planner._footprint_cost(course, 0, 0, .08))

    def test_clearance_cost_depends_on_distance_not_sample_rate(self):
        # The same metre at either sampling rate or driving speed incurs the
        # same cost, matching the previous per-metre geometric penalty.
        self.assertAlmostEqual(integrated_clearance_cost([.5]*10, .1), 1.5)
        self.assertAlmostEqual(integrated_clearance_cost([.5]*20, .05), 1.5)

    def test_observed_corridor_direction_guides_turn_without_map_truth(self):
        course = grid()
        left = self.planner.plan(course, 0, 0, 0, reference_yaw=.55, guidance_yaw=.55)
        right = self.planner.plan(course, 0, 0, 0, reference_yaw=-.55, guidance_yaw=-.55)
        self.assertEqual((left.reason, right.reason), ('ok', 'ok'))
        self.assertGreater(left.angular, 0)
        self.assertLess(right.angular, 0)

    def test_touch_return_is_not_a_change_and_partial_entry_is_provisional(self):
        divider = divider_grid()
        self.assertEqual(divider_crossings([(0, -1), (0, 0), (1, -1)], divider), 0)
        self.assertEqual(divider_crossings([(0, -1), (0, 0)], divider), 1)

    def test_crossing_penalty_is_once_per_observed_strip(self):
        for resolution in (.05, .1, .2):
            divider = divider_grid(resolution)
            self.assertEqual(divider_crossings([(0, -1), (0, 1)], divider), 1)
            dense = [(0, -.9+i*.05) for i in range(37)]
            self.assertEqual(divider_crossings(dense, divider), 1)
            self.assertEqual(divider_crossings([(0, -1), (0, 1), (0, -1)], divider), 2)

    def test_penalty_prefers_staying_in_lane_when_both_routes_safe(self):
        course, divider = grid(((2, -.4, .3),)), divider_grid()
        no_penalty = LocalDrivePlanner(0).plan(course, 0, -.35, 0, divider)
        costly = LocalDrivePlanner(1).plan(course, 0, -.35, 0, divider)
        self.assertEqual((no_penalty.reason, costly.reason), ('ok', 'ok'))
        self.assertEqual(no_penalty.divider_crossings, 1)
        self.assertEqual(costly.divider_crossings, 0)
        self.assertGreater(no_penalty.angular, 0)
        self.assertLess(costly.angular, 0)

    def test_high_penalty_still_permits_required_safe_crossing(self):
        course = grid(((2, -.4, .3),))
        cells = tuple(100 if course.origin_y+(i//course.width+.5)*course.resolution < -1.3 else value
                      for i, value in enumerate(course.cells))
        course = LocalGrid(course.width, course.height, course.resolution, course.origin_x,
                           course.origin_y, course.origin_yaw, cells)
        plan = LocalDrivePlanner(1).plan(course, 0, -.35, 0, divider_grid())
        self.assertEqual(plan.reason, 'ok')
        self.assertEqual(plan.divider_crossings, 1)
        self.assertEqual(plan.divider_cost_m, 1)
        self.assertGreater(plan.angular, 0)
        heading = 0.0
        for index, (x, y) in enumerate(plan.points):
            angular = plan.angular if index < len(plan.points)//2 else plan.second_angular
            heading += angular*self.planner.dt
            self.assertIsNotNone(self.planner._footprint_cost(course, x, y, heading))

    def test_obstacle_deflects_search(self):
        clear = self.planner.plan(grid(), 0, 0, 0)
        blocked_straight = self.planner.plan(grid(((2.0, 0, .3),)), 0, 0, 0)
        self.assertEqual(blocked_straight.reason, 'ok')
        self.assertEqual(clear.angular, 0)
        self.assertNotEqual(blocked_straight.angular, 0)

    def test_unknown_uses_previous_endpoint_averaged_multiplier(self):
        # Prior v2 charges 0.15 m per metre if both ends are unknown, half
        # that when only one end is unknown, and nothing for known space.
        self.assertAlmostEqual(unknown_traversal_cost(True, True, 1), .15)
        self.assertAlmostEqual(unknown_traversal_cost(True, False, 1), .075)
        self.assertEqual(unknown_traversal_cost(False, False, 1), 0)
        unknown = grid(unknown=True)
        before = unknown.cells
        self.assertEqual(self.planner.plan(unknown, 0, 0, 0).reason, 'ok')
        self.assertEqual(unknown.cells, before)  # unknown is never rewritten as observed free

    def test_full_footprint_and_out_of_map_remain_hard(self):
        self.assertEqual(self.planner.plan(grid(), 4.8, 0, 0).reason, 'footprint_blocked_or_unknown')
        # The obstacle lies beside the base frame but inside the actual body.
        # A semantic divider mask must never override a lethal obstacle.
        result = self.planner.plan(grid(((.6, .35, .12),)), 0, 0, 0, divider_grid())
        self.assertEqual(result.reason, 'footprint_blocked_or_unknown')
        self.assertEqual(result.linear, 0)

    def test_stops_when_forward_is_blocked(self):
        result = self.planner.plan(grid(((1.5, 0, .5),)), 0, 0, 0)
        self.assertEqual(result.reason, 'no_safe_forward_trajectory')
        self.assertEqual((result.linear, result.angular), (0, 0))

    def test_invalid_grid_fails_closed(self):
        invalid = LocalGrid(3, 3, .1, 0, 0, 0, ())
        self.assertEqual(self.planner.plan(invalid, 0, 0, 0).reason, 'invalid_input')

    @unittest.skipIf(local_drive.np is None, 'NumPy not installed')
    def test_vectorized_footprint_matches_fallback(self):
        rng = random.Random(2)
        values = tuple(100 if rng.random() < .002 else rng.randrange(65)
                       for _ in range(80*80))
        course = LocalGrid(80, 80, .1, -4, -4, .15, values)
        for _ in range(100):
            x, y = rng.uniform(-3.8, 3.8), rng.uniform(-3.8, 3.8)
            heading = rng.uniform(-math.pi, math.pi)
            vector = self.planner._footprint_cost(course, x, y, heading)
            with patch.object(local_drive, 'np', None):
                fallback = self.planner._footprint_cost(course, x, y, heading)
            self.assertEqual(vector, fallback)

    def test_replans_past_tire_without_turning_back(self):
        # Sample a straight section of the two-lane track with a tire in
        # the starting lane. Replan and integrate the commanded velocities,
        # checking actual progress rather than one selected steering command.
        resolution, width, height = .2, 100, 50
        origin_x, origin_y = -2.0, -5.0
        cells = []
        for iy in range(height):
            y = origin_y + (iy + .5) * resolution
            for ix in range(width):
                x = origin_x + (ix + .5) * resolution
                clearance = min(math.hypot(x - 6.5, y + 1.2) - .45, 2.4 - abs(y))
                cells.append(100 if clearance <= 0 else int(max(0, 1-clearance)*70))
        course = LocalGrid(width, height, resolution, origin_x, origin_y, 0, tuple(cells))
        controller = ManeuverPlanner()
        x, y, heading = 0.0, -1.2, 0.0
        for step in range(150):
            plan = controller.plan(course, x, y, heading, step*.2)
            self.assertEqual(plan.reason, 'ok', (x, y, heading))
            mid = heading + plan.angular*.1
            x += plan.linear*.2*math.cos(mid)
            y += plan.linear*.2*math.sin(mid)
            heading += plan.angular*.2
            self.assertLess(abs(heading), math.pi/2, (x, y, heading))
            self.assertIsNotNone(self.planner._footprint_cost(course, x, y, heading))
            if x > 8:
                break
        self.assertGreater(x, 8, (x, y, heading))

    def test_maneuver_rechecks_new_obstacle(self):
        controller = ManeuverPlanner()
        initial = controller.plan(grid(((2.0, 0, .3),)), 0, 0, 0, 0)
        self.assertEqual(initial.reason, 'ok')
        self.assertNotEqual(initial.angular, 0)
        # An obstacle appearing within the robot body revokes the old turn.
        next_plan = controller.plan(grid(((.6, .35, .12),)), 0, 0, 0, .2)
        self.assertEqual(next_plan.reason, 'footprint_blocked_or_unknown')
        self.assertEqual(next_plan.linear, 0)

    def test_active_maneuver_reports_remaining_observed_crossing(self):
        course = grid(((2, -.4, .3),))
        cells = tuple(100 if course.origin_y+(i//course.width+.5)*course.resolution < -1.3 else value
                      for i, value in enumerate(course.cells))
        course = LocalGrid(course.width, course.height, course.resolution, course.origin_x,
                           course.origin_y, course.origin_yaw, cells)
        controller = ManeuverPlanner(1)
        divider = divider_grid()
        first = controller.plan(course, 0, -.35, 0, 0, divider)
        self.assertEqual(first.reason, 'ok')
        self.assertEqual((first.divider_crossings, first.divider_cost_m), (1, 1))
        heading = first.angular*.2
        x = first.linear*.2*math.cos(heading/2)
        y = -.35 + first.linear*.2*math.sin(heading/2)
        continued = controller.plan(course, x, y, heading, .2, divider)
        self.assertEqual(continued.reason, 'ok')
        self.assertEqual(continued.angular, first.angular)
        self.assertEqual((continued.divider_crossings, continued.divider_cost_m), (1, 1))


if __name__ == '__main__':
    unittest.main()
