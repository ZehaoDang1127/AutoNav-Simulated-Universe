"""Regression tests for the captured stalled local costmap."""
import base64
import json
import math
from pathlib import Path
import unittest
import zlib

from autonav_universe.local_drive import LocalGrid, ManeuverPlanner


def captured_grid(name='stalled_local_costmap.json'):
    raw = json.loads((Path(__file__).parent / 'fixtures' / name).read_text())
    def read(item):
        cells = tuple(value-1 for value in zlib.decompress(base64.b64decode(item['data_zlib_base64'])))
        return LocalGrid(item['width'], item['height'], item['resolution_m'], *item['origin'], cells)
    return read(raw['costmap']), read(raw['divider_costmap']), tuple(raw['robot_pose'])


class LocalRecoveryTests(unittest.TestCase):
    def test_inner_border_stall_turns_back_to_local_travel_direction(self):
        grid, divider, (x, y, yaw) = captured_grid('inner_border_costmap.json')
        start_x = x
        planner = ManeuverPlanner(.25)
        # The prior travel direction is odometry-relative; no course geometry
        # or goal is supplied to the controller.
        planner.direction_anchor = (x-.5, y-1.0, .002)
        linear = angular = 0.0
        initial = planner.plan(grid, x, y, yaw, 0, divider, linear_speed_mps=0,
                               angular_speed_radps=0, lateral_speed_mps=0)
        self.assertEqual(initial.linear, 0)
        self.assertLess(initial.angular, 0)
        self.assertEqual(planner.recovery['phase'], 'turn_back')
        for step in range(1, 130):
            plan = planner.plan(grid, x, y, yaw, step*.2, divider,
                                linear_speed_mps=linear, angular_speed_radps=angular,
                                lateral_speed_mps=0)
            self.assertIn(plan.reason, ('ok', 'recovery_settling'), (step, x, y, yaw))
            linear, angular = plan.linear, plan.angular
            mid = yaw+angular*.1
            x += linear*.2*math.cos(mid)
            y += linear*.2*math.sin(mid)
            yaw += angular*.2
            self.assertIsNotNone(planner.search._footprint_cost(grid, x, y, yaw))
            if x-start_x > .7:
                break
        self.assertGreater(x-start_x, .7)

    def test_inner_border_turn_stops_if_new_obstacle_appears(self):
        grid, divider, (x, y, yaw) = captured_grid('inner_border_costmap.json')
        planner = ManeuverPlanner(.25)
        planner.direction_anchor = (x-.5, y-1.0, .002)
        initial = planner.plan(grid, x, y, yaw, 0, divider, linear_speed_mps=0,
                               angular_speed_radps=0, lateral_speed_mps=0)
        self.assertLess(initial.angular, 0)
        gx, gy = grid.local(x, y)
        cells = list(grid.cells)
        cells[math.floor(gy)*grid.width+math.floor(gx)] = 100
        changed = LocalGrid(grid.width, grid.height, grid.resolution, grid.origin_x,
                            grid.origin_y, grid.origin_yaw, tuple(cells))
        result = planner.plan(changed, x, y, yaw, .2, divider, linear_speed_mps=0,
                              angular_speed_radps=-.25, lateral_speed_mps=0)
        self.assertEqual(result.reason, 'recovery_unavailable')
        self.assertEqual((result.linear, result.angular), (0, 0))

    def test_captured_stall_performs_bounded_safe_lane_shift(self):
        grid, divider, (x, y, yaw) = captured_grid()
        start_x = x
        planner = ManeuverPlanner(.25)
        linear = angular = 0.0
        self.assertEqual(planner.search.plan(grid, x, y, yaw, divider).reason, 'no_safe_forward_trajectory')
        for step in range(155):
            plan = planner.plan(grid, x, y, yaw, step*.2, divider,
                                linear_speed_mps=linear, angular_speed_radps=angular,
                                lateral_speed_mps=0.0)
            self.assertIn(plan.reason, ('ok', 'recovery_settling'), (step, x, y, yaw))
            self.assertIsInstance(plan.linear, float)
            self.assertIsInstance(plan.angular, float)
            linear, angular = plan.linear, plan.angular
            mid = yaw+angular*.1
            x += linear*.2*math.cos(mid)
            y += linear*.2*math.sin(mid)
            yaw += angular*.2
            self.assertIsNotNone(planner.search._footprint_cost(grid, x, y, yaw))
            if x-start_x > .7:
                break
        self.assertGreater(x-start_x, .7)

    def test_recovery_requires_current_stationary_twist(self):
        grid, divider, pose = captured_grid()
        for linear, angular, lateral in ((None, None, None), (.2, 0, 0), (0, .2, 0), (0, 0, .2)):
            planner = ManeuverPlanner(.25)
            plan = planner.plan(grid, *pose, 0, divider, linear_speed_mps=linear,
                                angular_speed_radps=angular, lateral_speed_mps=lateral)
            self.assertEqual(plan.linear, 0)
            self.assertEqual(plan.angular, 0)
            self.assertIsNone(planner.recovery)

    def test_recovery_aborts_on_unexpected_translation_during_turn(self):
        grid, divider, pose = captured_grid()
        planner = ManeuverPlanner(.25)
        initial = planner.plan(grid, *pose, 0, divider, linear_speed_mps=0,
                               angular_speed_radps=0, lateral_speed_mps=0)
        self.assertEqual(initial.linear, 0)
        self.assertNotEqual(initial.angular, 0)
        stopped = planner.plan(grid, *pose, .2, divider, linear_speed_mps=.08,
                               angular_speed_radps=.25, lateral_speed_mps=0)
        self.assertEqual(stopped.reason, 'recovery_unavailable')
        self.assertEqual((stopped.linear, stopped.angular), (0, 0))
        self.assertIsNone(planner.recovery)

    def test_recovery_stops_if_new_hard_obstacle_appears(self):
        grid, divider, (x, y, yaw) = captured_grid()
        planner = ManeuverPlanner(.25)
        initial = planner.plan(grid, x, y, yaw, 0, divider, linear_speed_mps=0,
                               angular_speed_radps=0, lateral_speed_mps=0)
        self.assertEqual(initial.reason, 'ok')
        gx, gy = grid.local(x, y)
        cells = list(grid.cells)
        cells[math.floor(gy)*grid.width+math.floor(gx)] = 100
        changed = LocalGrid(grid.width, grid.height, grid.resolution, grid.origin_x,
                            grid.origin_y, grid.origin_yaw, tuple(cells))
        stopped = planner.plan(changed, x, y, yaw, .2, divider, linear_speed_mps=0,
                               angular_speed_radps=0, lateral_speed_mps=0)
        self.assertEqual(stopped.reason, 'recovery_unavailable')
        self.assertEqual((stopped.linear, stopped.angular), (0, 0))

    def test_normal_forward_arc_cannot_keep_turning_back(self):
        grid, divider, (x, y, yaw) = captured_grid()
        planner = ManeuverPlanner(.25)
        # An all-clear local map makes the angular bound, not collision,
        # the reason this 2.16 rad forward arc is rejected.
        clear = LocalGrid(grid.width, grid.height, grid.resolution, grid.origin_x,
                          grid.origin_y, grid.origin_yaw, (0,)*len(grid.cells))
        plan = planner.search.validate_maneuver(clear, x, y, yaw, .22, .54, .54,
                                               2.0, 2.0, divider, reference_yaw=yaw)
        self.assertEqual(plan.reason, 'heading_limit')

    def test_direction_anchor_advances_only_with_forward_progress(self):
        grid, divider, (x, y, yaw) = captured_grid()
        planner = ManeuverPlanner(.25)
        clear = LocalGrid(grid.width, grid.height, grid.resolution, grid.origin_x,
                          grid.origin_y, grid.origin_yaw, (0,)*len(grid.cells))
        planner.plan(clear, x, y, yaw, 0, divider)
        first = planner.direction_anchor
        # Lateral motion alone cannot rotate the chosen travel direction.
        planner.plan(clear, x, y+1.6, yaw+.4, .2, divider)
        self.assertEqual(planner.direction_anchor, first)
        # Forward progress permits the reference to follow a gradual bend.
        planner.plan(clear, x+1.6, y, yaw+.4, .4, divider)
        self.assertNotEqual(planner.direction_anchor, first)


if __name__ == '__main__':
    unittest.main()
