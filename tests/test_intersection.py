"""Junction recognition and the stop / straight / left route decision."""
import importlib.util
import math
from pathlib import Path
import unittest

from autonav_universe.corridor_tangent import estimate_corridor_tangent
from autonav_universe.intersection import (GUIDANCE_MAX_DEVIATION_RAD, IntersectionManager,
                                           left_turn_poses, observe_junction,
                                           parse_turn_command, trim_to_stop)
from autonav_universe.lane_reference import LaneReferencePlanner
from autonav_universe.local_drive import LocalGrid

ROOT = Path(__file__).resolve().parents[1]
RES = .1
SIZE = 80          # 8 m rolling window, as in the self-drive Nav2 profile


def window(cx, cy, value):
    """Grid centred on (cx, cy); ``value(x, y)`` gives the cell value."""
    ox, oy = cx-SIZE*RES/2, cy-SIZE*RES/2
    cells = tuple(value(ox+(ix+.5)*RES, oy+(iy+.5)*RES) for iy in range(SIZE) for ix in range(SIZE))
    return LocalGrid(SIZE, SIZE, RES, ox, oy, 0.0, cells)


def near(x, y, a, b, half=.06):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx-ax, by-ay
    t = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy)))
    return math.hypot(x-ax-t*dx, y-ay-t*dy) <= half


# The generated T-intersection: road y 2.0..6.8, branch x 12.0..16.8 to the
# north, eastbound stop line at x = 11.4 across the south lane.
T_TAPE = (((0.6, 2.0), (30.0, 2.0)), ((0.6, 6.8), (12.0, 6.8)), ((12.0, 6.8), (12.0, 20.0)),
          ((30.0, 6.8), (16.8, 6.8)), ((16.8, 6.8), (16.8, 20.0)))
T_PAINT = (((11.4, 2.05), (11.4, 4.4)), ((0.9, 4.4), (11.4, 4.4)), ((17.4, 4.4), (30.0, 4.4)),
           ((14.4, 7.4), (14.4, 19.7)))
STRAIGHT_TAPE = (((-20.0, 2.0), (40.0, 2.0)), ((-20.0, 6.8), (40.0, 6.8)))


def scene(x, y, tape=T_TAPE, paint=T_PAINT, dashed=False, extra_lethal=None):
    def lethal(px, py):
        return any(near(px, py, a, b) for a, b in tape) or bool(extra_lethal and extra_lethal(px, py))

    def paint_at(px, py):
        for a, b in paint:
            if near(px, py, a, b):
                along = px if a[1] == b[1] else py
                return not dashed or (along % 1.2) < .75
        return False

    costmap = window(x, y, lambda px, py: 100 if lethal(px, py) else 0)
    boundary = window(x, y, lambda px, py: 100 if any(near(px, py, a, b) for a, b in tape) else 0)
    divider = window(x, y, lambda px, py: 100 if paint_at(px, py) else 0)
    return costmap, divider, boundary


class PatternTests(unittest.TestCase):
    def test_turn_commands(self):
        self.assertEqual(parse_turn_command(' Left '), 'left')
        self.assertEqual(parse_turn_command('STRAIGHT'), 'straight')
        for value in ('right', '', None, 3):
            self.assertIsNone(parse_turn_command(value))

    def test_t_junction_pattern_ahead(self):
        # The 8 m window shows >= 1.6 m of the gap once within ~2 m of it.
        costmap, divider, boundary = scene(10.0, 3.2)
        obs = observe_junction(costmap, divider, 10.0, 3.2, 0.0, boundary=boundary)
        self.assertAlmostEqual(obs.right_edge_m, -1.2, delta=.1)
        self.assertAlmostEqual(obs.left_edge_m, 3.6, delta=.1)
        # Near edge of the line 1.4 m ahead, to one 0.1 m cell.
        self.assertAlmostEqual(obs.stop_line_m, 1.35, delta=.1)
        self.assertIsNotNone(obs.left)
        self.assertAlmostEqual(obs.left.start_m, 2.0, delta=.25)   # x = 12.0
        self.assertIsNone(obs.right)
        self.assertTrue(obs.straight_open)

    def test_plain_road_and_divider_are_not_a_junction(self):
        paint = (((-20.0, 4.4), (40.0, 4.4)),)
        costmap, divider, boundary = scene(9.0, 3.2, STRAIGHT_TAPE, paint, dashed=True)
        obs = observe_junction(costmap, divider, 9.0, 3.2, 0.0, boundary=boundary)
        self.assertFalse(obs.is_junction)
        self.assertIsNone(obs.stop_line_m)
        self.assertTrue(obs.straight_open)

    def test_left_curve_is_not_a_left_branch(self):
        # Both edges bend left (radius 5 m inner, 9.8 m outer) from x = 10.
        def arc(cx, cy, r, count=40):
            pts = [(cx+r*math.sin(math.pi/2*k/count), cy-r*math.cos(math.pi/2*k/count))
                   for k in range(count+1)]
            return tuple(zip(pts, pts[1:]))
        tape = (((-20.0, 2.0), (10.0, 2.0)), ((-20.0, 6.8), (10.0, 6.8)))
        tape += arc(10.0, 11.8, 9.8) + arc(10.0, 11.8, 5.0)
        costmap, divider, boundary = scene(9.0, 3.2, tape, ())
        obs = observe_junction(costmap, divider, 9.0, 3.2, 0.0, boundary=boundary)
        self.assertIsNone(obs.left)

    def test_obstacle_beside_lane_is_not_a_road_edge(self):
        tire = lambda px, py: math.hypot(px-10.0, py-1.6) <= .75
        costmap, divider, boundary = scene(9.0, 3.2, STRAIGHT_TAPE, (), extra_lethal=tire)
        obs = observe_junction(costmap, divider, 9.0, 3.2, 0.0, boundary=boundary)
        self.assertFalse(obs.is_junction)

    def test_dead_end_blocks_straight(self):
        tape = STRAIGHT_TAPE + (((12.0, 2.0), (12.0, 6.8)),)
        costmap, divider, boundary = scene(9.0, 3.2, tape, ())
        self.assertFalse(observe_junction(costmap, divider, 9.0, 3.2, 0.0, boundary=boundary).straight_open)

    def test_stop_line_is_not_taken_for_the_lane_direction(self):
        # Gazebo 2026-11-18: the divider fit locked onto the stop line and the
        # robot pivoted 80 degrees at it. A few degrees of odometry heading
        # error is enough to pass the fit's perpendicular-line check.
        _, divider, boundary = scene(10.2, 3.2, paint=(((11.3, 2.05), (11.55, 4.4)),))
        loose = estimate_corridor_tangent(divider, 10.2, 3.2, 0.0)
        self.assertIsNotNone(loose)
        self.assertGreater(abs(loose.yaw_rad), 1.0)
        gated = estimate_corridor_tangent(divider, 10.2, 3.2, 0.0, boundary,
                                          max_deviation_rad=GUIDANCE_MAX_DEVIATION_RAD)
        self.assertEqual(gated.source, 'boundary')
        self.assertLess(abs(gated.yaw_rad), .05)

    def test_trim_and_turn_geometry(self):
        poses = tuple((i*.1, 0.0, 0.0) for i in range(30))
        self.assertEqual(len(trim_to_stop(poses, (1.0, 0.0), 0.0)), 11)
        turn, arc_end = left_turn_poses(10.0, 3.2, 0.0, 5.4, 3.0)
        self.assertAlmostEqual(turn[arc_end][0], 15.4, places=6)
        self.assertAlmostEqual(turn[arc_end][1], 6.2, places=6)
        self.assertAlmostEqual(turn[arc_end][2], math.pi/2, places=6)
        self.assertAlmostEqual(turn[-1][1], 6.2+3.0, places=6)   # default 3 m exit leg
        for a, b in zip(turn, turn[1:]):
            self.assertLess(math.hypot(b[0]-a[0], b[1]-a[1]), .06)


class ManagerTests(unittest.TestCase):
    """Drive the state machine at fixed poses on the T-intersection grids."""

    def step(self, manager, now, x, speed=.5, yaw=0.0):
        costmap, divider, boundary = scene(x, 3.2)
        return manager.update(now, costmap, divider, x, 3.2, yaw, speed, 0.0, boundary)

    def stop_at_line(self, manager):
        directive = None
        for i, x in enumerate((8.6, 8.8, 9.0)):
            directive = self.step(manager, i*.2, x)
        self.assertEqual(directive.mode, 'stop_at')
        # Padded nose 0.25 m before the line: base_link at 11.4-0.87-0.25.
        self.assertAlmostEqual(directive.stop_point[0], 10.28, delta=.1)
        directive = self.step(manager, 1.0, 10.2, speed=.02)
        self.assertEqual((directive.mode, manager.state), ('hold', 'stopped'))
        return directive

    def test_default_is_stop_then_straight(self):
        manager = IntersectionManager(LaneReferencePlanner())
        self.stop_at_line(manager)
        self.assertEqual(self.step(manager, 2.5, 10.2, speed=0).mode, 'hold')
        directive = self.step(manager, 3.1, 10.2, speed=0)
        # A straight line through the box at the robot's lane position.
        self.assertEqual(directive.mode, 'follow')
        self.assertEqual(manager.route, 'straight')
        self.assertEqual(manager.route_note, 'default')
        self.assertTrue(all(abs(py-3.2) < 1e-9 and abs(h) < 1e-9 for _, py, h in directive.poses))
        self.assertGreater(directive.poses[-1][0], 16.8+1.0)

    def test_left_request_turns_into_branch_right_lane(self):
        manager = IntersectionManager(LaneReferencePlanner())
        self.assertTrue(manager.set_request('left', 'topic'))
        self.stop_at_line(manager)
        directive = self.step(manager, 3.1, 10.2, speed=0)
        self.assertEqual(directive.mode, 'follow')
        self.assertEqual(manager.route, 'left')
        end = directive.poses[-1]
        # Ends heading north in the branch's east (outbound) lane.
        self.assertAlmostEqual(end[2], math.pi/2, places=6)
        self.assertGreater(end[0], 14.4+.5)
        self.assertLess(end[0], 16.8-.5)
        self.assertAlmostEqual(manager.expected_yaw(10.2, 3.2), 0.0, places=6)

    def test_request_changes_until_the_robot_leaves_the_line(self):
        manager = IntersectionManager(LaneReferencePlanner())
        self.stop_at_line(manager)
        manager.set_request('left', 'desktop')
        self.assertEqual(self.step(manager, 3.1, 10.2, speed=0).mode, 'follow')
        self.assertFalse(manager.set_request('right'))
        self.assertEqual(manager.request, 'left')

    def test_left_without_branch_goes_straight(self):
        manager = IntersectionManager(LaneReferencePlanner())
        manager.set_request('left')
        tape = STRAIGHT_TAPE
        paint = (((11.4, 2.05), (11.4, 4.4)), ((-20.0, 4.4), (11.4, 4.4)))
        for i, x in enumerate((8.6, 8.8, 9.0, 10.2, 10.2, 10.2)):
            costmap, divider, boundary = scene(x, 3.2, tape, paint)
            directive = manager.update(i*.2 if i < 4 else 1.0+i, costmap, divider, x, 3.2, 0.0,
                                       .5 if i < 3 else 0.0, 0.0, boundary)
        self.assertEqual(directive.mode, 'follow')
        self.assertEqual((manager.route, manager.route_note), ('straight', 'left_unavailable'))

    def test_left_is_taken_when_far_edge_is_not_visible(self):
        # The far edge is often beyond the LiDAR line detector: unknown cells
        # neither confirm nor refute the branch, so the request is honoured.
        manager = IntersectionManager(LaneReferencePlanner())
        manager.set_request('left')
        for i, x in enumerate((8.6, 8.8, 9.0, 10.2, 10.2, 10.2)):
            costmap, divider, boundary = scene(x, 3.2)
            cells = tuple(-1 if costmap.origin_y+(k//SIZE+.5)*RES > 5.6 else v
                          for k, v in enumerate(costmap.cells))
            costmap = LocalGrid(SIZE, SIZE, RES, costmap.origin_x, costmap.origin_y, 0.0, cells)
            directive = manager.update(i*.2 if i < 4 else 1.0+i, costmap, divider, x, 3.2, 0.0,
                                       .5 if i < 3 else 0.0, 0.0, boundary)
        self.assertEqual(directive.mode, 'follow')
        self.assertEqual((manager.route, manager.route_note), ('left', 'requested_branch_unconfirmed'))
        self.assertEqual(manager.options()['left'], 'unknown')

    def test_no_stop_line_means_no_action(self):
        manager = IntersectionManager(LaneReferencePlanner())
        manager.set_request('left')
        paint = tuple(p for p in T_PAINT if p[0][0] != 11.4 or p[1][0] != 11.4)
        for i, x in enumerate((8.0, 8.5, 9.0, 9.5, 10.0, 10.5)):
            costmap, divider, boundary = scene(x, 3.2, T_TAPE, paint)
            directive = manager.update(i*.2, costmap, divider, x, 3.2, 0.0, .5, 0.0, boundary)
            self.assertEqual(directive.mode, 'normal')
        self.assertTrue(manager.last_obs.left)


_spec = importlib.util.spec_from_file_location('simulate_intersection', ROOT/'tools/simulate_intersection.py')
_sim = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_sim)


class ClosedLoopTests(unittest.TestCase):
    """Offline closed loop (rendered grids, pure-pursuit stand-in for MPPI)."""

    @classmethod
    def setUpClass(cls):
        cls.runs = {turn: _sim.run(turn) for turn in ('straight', 'left')}
        cls.runs['closed'] = _sim.run('left', layout='closed_branch', max_time_s=90)

    def check_common(self, run):
        self.assertEqual(run['tape_contacts'], 0)
        stop = run['stopped']
        self.assertIsNotNone(stop)
        # Physical nose stops short of the 11.4 m stop line, not far before it.
        self.assertLess(stop['front_x'], run['stop_line_x'])
        self.assertGreater(stop['front_x'], run['stop_line_x']-.8)
        self.assertGreaterEqual(run['stopped_state_s'], 2.0)

    def test_default_goes_straight(self):
        run = self.runs['straight']
        self.check_common(run)
        self.assertEqual((run['route'], run['exit']), ('straight', 'straight'))
        self.assertTrue(all(2.0+.41 < y < 4.4-.41 for _, y in run['trail']))
        # No sideways drift through the junction or after it (Gazebo: 0.8 m).
        ys = [y for x, y in run['trail'] if x >= 10.5]
        self.assertLess(max(ys)-min(ys), .15)

    def test_left_request_turns_left(self):
        run = self.runs['left']
        self.check_common(run)
        self.assertEqual((run['route'], run['exit']), ('left', 'left'))
        self.assertTrue(14.4 < run['final_pose'][0] < 16.8)

    def test_left_request_without_a_branch_goes_straight(self):
        run = self.runs['closed']
        self.check_common(run)
        self.assertEqual((run['route'], run['exit']), ('straight', 'straight'))


if __name__ == '__main__':
    unittest.main()
