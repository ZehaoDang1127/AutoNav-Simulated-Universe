"""Lane reference paths that self-drive hands to Nav2's MPPI controller."""
import math
import unittest

from autonav_universe.lane_reference import LaneReferencePlanner, ReferencePath
from autonav_universe.local_drive import LocalGrid

RESOLUTION = .1
ORIGIN = -5.0
SIZE = 100


def grid(lethal=lambda x, y: False, unknown=lambda x, y: False):
    cells = []
    for iy in range(SIZE):
        for ix in range(SIZE):
            x, y = ORIGIN+(ix+.5)*RESOLUTION, ORIGIN+(iy+.5)*RESOLUTION
            cells.append(100 if lethal(x, y) else -1 if unknown(x, y) else 0)
    return LocalGrid(SIZE, SIZE, RESOLUTION, ORIGIN, ORIGIN, 0.0, tuple(cells))


def disc(cx, cy, r):
    return lambda x, y: math.hypot(x-cx, y-cy) <= r


def body_clearance(pose, cx, cy, r, planner=LaneReferencePlanner):
    """Distance from a padded body rectangle at ``pose`` to a disc's edge."""
    x, y, heading = pose
    dx, dy = cx-x, cy-y
    bx = math.cos(heading)*dx+math.sin(heading)*dy
    by = -math.sin(heading)*dx+math.cos(heading)*dy
    qx = max(planner.rear-bx, 0, bx-planner.front)
    qy = max(abs(by)-planner.half_width, 0)
    return math.hypot(qx, qy)-r


def two_lanes(cone=None, dashed=False):
    """Tape at y=-1.2 and y=3.6, divider paint at y=1.2, robot lane at y=0."""
    def lethal(x, y):
        return abs(y+1.2) < .06 or abs(y-3.6) < .06 or bool(cone and disc(*cone)(x, y))
    def paint(x, y):
        return abs(y-1.2) < .06 and (not dashed or (x+5) % 1.2 < .75)
    divider = tuple(100 if paint(ORIGIN+(ix+.5)*RESOLUTION, ORIGIN+(iy+.5)*RESOLUTION) else 0
                    for iy in range(SIZE) for ix in range(SIZE))
    return grid(lethal), LocalGrid(SIZE, SIZE, RESOLUTION, ORIGIN, ORIGIN, 0.0, divider)


class LaneReferenceTests(unittest.TestCase):
    def setUp(self):
        self.planner = LaneReferencePlanner(.25)

    def test_open_map_gives_straight_dense_path_from_robot(self):
        path = self.planner.plan(grid(), 0, 0, 0, 0.0)
        self.assertEqual(path.reason, 'ok')
        self.assertTrue(path)
        self.assertEqual(path.poses[0], (0, 0, 0))
        self.assertAlmostEqual(path.length_m, self.planner.horizon_m)
        self.assertEqual(path.pivot_rad, 0.0)
        for (ax, ay, _), (bx, by, _) in zip(path.poses, path.poses[1:]):
            # Nav2 critics index path points; match the reference's 5 cm Smac paths.
            self.assertLessEqual(math.hypot(bx-ax, by-ay), .051)
        self.assertLess(max(abs(p[1]) for p in path.poses), .05)
        self.assertLess(max(abs(p[2]) for p in path.poses), .05)

    def test_path_follows_observed_corridor_direction(self):
        left = self.planner.plan(grid(), 0, 0, 0, .5)
        right = LaneReferencePlanner().plan(grid(), 0, 0, 0, -.5)
        self.assertGreater(left.poses[-1][2], .3)
        self.assertLess(right.poses[-1][2], -.3)

    def test_obstacle_bypass_keeps_padded_body_clear(self):
        cone = (2.2, 0.0, .5)
        path = self.planner.plan(grid(disc(*cone)), 0, 0, 0, 0.0)
        self.assertEqual(path.reason, 'ok')
        self.assertGreater(max(abs(p[1]) for p in path.poses), .8)
        self.assertGreater(min(body_clearance(p, *cone) for p in path.poses), 0)

    def test_obstacle_at_horizon_starts_gentle_swerve_now(self):
        # The straight body meets this cone ~2.5 m out, inside the 3.2 m
        # horizon. The search must keep early swerves alive instead of
        # returning a partial straight path that ends at the cone.
        cone = (4.0, 0.0, .6)
        path = self.planner.plan(grid(disc(*cone)), 0, 0, 0, 0.0)
        self.assertEqual(path.reason, 'ok')
        self.assertAlmostEqual(path.length_m, self.planner.horizon_m)
        early = max(abs(p[1]) for p in path.poses if p[0] <= 1.2)
        self.assertGreater(early, .1)
        self.assertLess(max(abs(p[2]) for p in path.poses), math.radians(45))

    def test_lane_change_pays_crossing_only_when_needed(self):
        costmap, divider = two_lanes()
        clear = self.planner.plan(costmap, 0, 0, 0, 0.0, divider)
        self.assertEqual(clear.divider_crossings, 0)
        self.assertLess(max(p[1] for p in clear.poses), .3)
        costmap, divider = two_lanes(cone=(2.6, 0.0, .75))
        bypass = self.planner.plan(costmap, 0, 0, 0, 0.0, divider)
        self.assertEqual(bypass.reason, 'ok')
        self.assertEqual(bypass.divider_crossings, 1)
        self.assertGreater(max(p[1] for p in bypass.poses), 1.2)

    def test_touch_and_return_is_not_a_lane_change(self):
        _, divider = two_lanes(dashed=True)
        planner = self.planner
        paint = planner.paint_field(divider)
        normal = (0.0, 1.0)

        def count(points):
            state, total = planner._start_band_state(paint, *points[0]), 0
            for (ax, ay), (bx, by) in zip(points, points[1:]):
                delta, state = planner._paint_entries(paint, ax, ay, bx, by, state, normal)
                total += delta
            return total
        # Paint at y=1.2: brush the band from below and return; cross it;
        # end inside it (an unfinished change stays charged).
        self.assertEqual(count([(0, 0), (1, 1.05), (2, 0.4)]), 0)
        self.assertEqual(count([(0, 0), (1, 1.05), (3, 2.0)]), 1)
        self.assertEqual(count([(0, 0), (2, 1.2)]), 1)
        self.assertEqual(count([(0, 0), (1, 2.4), (2, 0)]), 2)
        # A shallow crossing along the wavy band edge is still one change.
        self.assertEqual(count([(0, 0.6), (3, 1.0), (6, 1.4), (9, 1.8)]), 1)

    def test_dash_gaps_do_not_hide_a_lane_change(self):
        costmap, divider = two_lanes(cone=(2.6, 0.0, .75), dashed=True)
        for cost in (.25, 1.0):
            path = LaneReferencePlanner(cost).plan(costmap, 0, 0, 0, 0.0, divider)
            self.assertEqual(path.reason, 'ok')
            self.assertEqual(path.divider_crossings, 1)

    def test_previous_path_breaks_symmetric_tie(self):
        cone = (2.4, 0.0, .5)
        costmap = grid(disc(*cone))
        for side in (1, -1):
            previous = tuple((x/10, side*min(1.5, x/10), 0.0) for x in range(0, 33))
            path = LaneReferencePlanner().plan(costmap, 0, 0, 0, 0.0, previous=previous)
            self.assertEqual(path.reason, 'ok')
            self.assertGreater(side*sum(p[1] for p in path.poses), 0)

    def test_newly_blocked_previous_path_does_not_delay_avoidance(self):
        # The last path ran straight; a cone is now observed on it. Attraction
        # to that path would postpone the swerve until the cone is close.
        costmap = grid(disc(2.6, 0.0, .5))
        straight = tuple((x/20, 0.0, 0.0) for x in range(0, 65))
        fresh = LaneReferencePlanner().plan(costmap, 0, 0, 0, 0.0)
        remembered = LaneReferencePlanner().plan(costmap, 0, 0, 0, 0.0, previous=straight)
        self.assertEqual(remembered.poses, fresh.poses)
        early = [abs(p[1]) for p in remembered.poses if 1.0 <= p[0] <= 1.2]
        self.assertGreater(max(early), .15)

    def test_pivot_when_lane_turns_beside_a_wall(self):
        wall = lambda x, y: 1.3 <= x <= 1.5
        path = self.planner.plan(grid(wall), 0, 0, 0, math.pi/2)
        self.assertEqual(path.reason, 'ok')
        self.assertGreater(path.pivot_rad, .3)
        self.assertGreater(path.poses[-1][1], 1.0)

    def test_unknown_space_is_traversable(self):
        path = self.planner.plan(grid(unknown=lambda x, y: x > .5), 0, 0, 0, 0.0)
        self.assertEqual(path.reason, 'ok')
        self.assertGreater(path.length_m, 2.0)

    def test_blocked_start_and_enclosure_stop(self):
        self.assertEqual(self.planner.plan(grid(disc(.5, 0, .2)), 0, 0, 0, 0.0).reason,
                         'footprint_blocked_or_unknown')
        box = lambda x, y: max(abs(x-.225), abs(y)) >= 1.1
        enclosed = self.planner.plan(grid(box), 0, 0, 0, 0.0)
        self.assertFalse(enclosed)
        self.assertEqual(enclosed.reason, 'no_reference_path')

    def test_invalid_inputs_are_refused(self):
        self.assertEqual(self.planner.plan(grid(), 0, 0, 0, None).reason, 'invalid_input')
        self.assertEqual(self.planner.plan(grid(), math.nan, 0, 0, 0.0).reason, 'invalid_input')
        self.assertFalse(ReferencePath())
        with self.assertRaises(ValueError):
            LaneReferencePlanner(-1)


if __name__ == '__main__':
    unittest.main()
