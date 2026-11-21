import math
import unittest
from dataclasses import replace

from autonav_universe.api import Observation, Pose2, ReferenceNavigation
from autonav_universe.algorithms import FixedPipelinePlanner
from autonav_universe.lane_costmap import LaneAwareCostmap, LaneCostConfig
from autonav_universe.lane_demo import demo_data, make_demo
from autonav_universe.runner import EpisodeRunner
from autonav_universe.scenarios import load_scenario


class LaneCostmapTests(unittest.TestCase):
    def corridor(self, resolution=0.25, penalty=0.25):
        grid = LaneAwareCostmap(resolution_m=resolution,
                               config=LaneCostConfig(lane_change_cost_m=penalty, clearance_weight=0))
        for ix in range(-8, 25):
            for iy in range(-8, 9):
                grid.cells[ix, iy] = "free"
                grid.lane_ids[ix, iy] = "left" if ix * resolution < 2 else "right"
        return grid

    def test_one_crossing_small_default_and_resolution_independence(self):
        for resolution in (0.1, 0.25, 0.5):
            grid = self.corridor(resolution)
            result = grid.plan_with_costs((1, 0), (3, 0), 0)
            self.assertEqual(result.status, "ok")
            self.assertEqual(result.lane_changes, 1)
            self.assertEqual(result.lane_change_cost_m, 0.25)
            self.assertAlmostEqual(result.length_m, 2)

    def test_unclassified_strip_does_not_erase_lane_history(self):
        grid = self.corridor()
        grid.lane_ids = {cell: lane for cell, lane in grid.lane_ids.items() if cell[0] != 8}
        result = grid.plan_with_costs((1, 0), (3, 0), 0)
        self.assertEqual(result.lane_changes, 1)
        self.assertEqual(result.lane_change_cost_m, 0.25)

    def test_same_lane_and_missing_semantics_have_no_change_cost(self):
        grid = self.corridor()
        self.assertEqual(grid.plan_with_costs((1, -1), (1, 1), 0).lane_changes, 0)
        grid.lane_ids.clear()
        self.assertEqual(grid.plan_with_costs((1, 0), (3, 0), 0).lane_changes, 0)

    def test_required_tire_bypass_is_allowed_even_with_large_penalty(self):
        for penalty in (0, 0.25, 30):
            grid, start, goal, _ = make_demo("sketch", penalty)
            plan = grid.plan_with_costs(start, goal, 0.28)
            self.assertEqual(plan.status, "ok")
            self.assertEqual(plan.lane_changes, 1)
            self.assertEqual(plan.lane_change_cost_m, penalty)
            self.assertTrue(all(grid.segment_is_clear(a, b, 0.28)
                                for a, b in zip(plan.points, plan.points[1:])))

    def test_penalty_changes_optional_detour_choice(self):
        _, low = demo_data("optional", 0)
        _, high = demo_data("optional", 10)
        self.assertEqual(low["plan"]["lane_changes"], 2)
        self.assertEqual(high["plan"]["lane_changes"], 0)
        self.assertGreater(high["plan"]["length_m"], low["plan"]["length_m"])

    def test_hard_obstacles_steep_and_boundaries_win_over_lane_ids(self):
        for label in ("obstacle", "steep", "boundary"):
            grid = self.corridor()
            grid.cells[4, 0] = label
            self.assertEqual(grid.plan_with_costs((1, 0), (3, 0), 0).status, "start_blocked")
            self.assertEqual(grid.plan_with_costs((3, 0), (1, 0), 0).status, "goal_blocked")

    def test_diagonal_and_subcell_connectors_cannot_clip_obstacles(self):
        grid = LaneAwareCostmap(resolution_m=1)
        grid.cells[1, 0] = "obstacle"
        self.assertFalse(grid.segment_is_clear((0, 0), (1, 1), 0))
        self.assertFalse(grid.segment_is_clear((0, 0.6), (2, 0.6), 0.2))
        self.assertTrue(grid.segment_is_clear((0, 0.8), (2, 0.8), 0.2))
        self.assertEqual(grid.plan_with_costs((0.49, 0), (3, 0), 0.02).status, "start_blocked")

    def test_sealed_boundary_cannot_be_bought_with_soft_cost(self):
        grid = LaneAwareCostmap(resolution_m=0.5)
        for ix in range(-2, 3):
            for iy in range(-2, 3):
                if abs(ix) == 2 or abs(iy) == 2:
                    grid.cells[ix, iy] = "boundary"
        plan = grid.plan_with_costs((0, 0), (2, 0), 0)
        self.assertEqual(plan.status, "no_path")
        self.assertEqual(plan.points, ())
        self.assertTrue(math.isinf(plan.total_cost_m))

    def test_search_expands_for_a_detour_beyond_initial_window(self):
        # These walls require the second and third search windows respectively.
        for wall_half_length in (4, 7):
            with self.subTest(wall_half_length=wall_half_length):
                grid = LaneAwareCostmap(resolution_m=0.5,
                                        config=LaneCostConfig(clearance_weight=0))
                cells = int(wall_half_length / grid.resolution_m)
                for iy in range(-cells, cells + 1):
                    grid.cells[0, iy] = "obstacle"
                plan = grid.plan_with_costs((-1, 0), (1, 0), 0)
                self.assertEqual(plan.status, "ok")
                self.assertTrue(any(abs(y) > wall_half_length for _, y in plan.points))
                self.assertTrue(all(grid.segment_is_clear(a, b, 0)
                                    for a, b in zip(plan.points, plan.points[1:])))

    def test_cost_breakdown_and_exact_endpoints(self):
        grid = LaneAwareCostmap(resolution_m=0.25, config=LaneCostConfig(clearance_weight=0))
        plan = grid.plan_with_costs((0.03, 0.02), (1.03, 0.02), 0)
        self.assertEqual(plan.points[0], (0.03, 0.02))
        self.assertEqual(plan.points[-1], (1.03, 0.02))
        self.assertAlmostEqual(plan.unknown_cost_m, plan.length_m * 0.15)
        self.assertAlmostEqual(plan.total_cost_m, plan.length_m * 1.15)

    def test_clearance_field_decays_and_is_not_added_per_obstacle(self):
        grid = LaneAwareCostmap(resolution_m=0.1)
        grid.cells[0, 0] = "obstacle"
        soft = grid.clearance_field(0.2)
        self.assertGreater(soft[3, 0], soft[6, 0])
        self.assertNotIn((12, 0), soft)
        grid.cells[0, 1] = "obstacle"
        self.assertEqual(grid.clearance_field(0.2)[6, 0], soft[6, 0])

    def test_invalid_costs_and_coordinates_are_rejected(self):
        for key in ("lane_change_cost_m", "clearance_weight", "clearance_distance_m", "unknown_multiplier"):
            for value in (-1, math.inf, math.nan):
                with self.assertRaises(ValueError):
                    LaneCostConfig(**{key: value})
        for value in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                LaneAwareCostmap(resolution_m=value)
        with self.assertRaises(ValueError):
            LaneAwareCostmap().plan((math.nan, 0), (1, 0))

    def test_reference_planner_consumes_lane_observations_and_stops_when_blocked(self):
        runner = EpisodeRunner(load_scenario("planner-straight"),
                               "autonav_universe.algorithms:FixedPipelinePlanner")
        context = replace(runner._context(), parameters=(("costmap_version", "2"),))
        planner = FixedPipelinePlanner()
        planner.reset(context)
        grid = self.corridor()
        goal = replace(context.goals[0], x_m=3, y_m=0)
        reference = ReferenceNavigation("lane-costmap-v2", Pose2(1, 0, 0, "odom"), goal,
                                        grid.resolution_m,
                                        tuple((*cell, label) for cell, label in grid.cells.items()),
                                        0, 0, tuple((*cell, lane) for cell, lane in grid.lane_ids.items()))
        observation = Observation(context.episode_id, 0, 0, 0, 0, (), reference)
        command = planner.step(observation)
        self.assertEqual(dict(command.diagnostics)["lane_changes"], "1")
        blocked = replace(reference, costmap_cells=reference.costmap_cells + ((4, 0, "obstacle"),))
        command = planner.step(replace(observation, time_s=1, reference=blocked))
        self.assertEqual((command.linear_mps, command.angular_radps), (0, 0))
        self.assertEqual(dict(command.diagnostics)["plan_status"], "start_blocked")


if __name__ == "__main__":
    unittest.main()
