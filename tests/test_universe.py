import math
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from autonav_universe.api import NavigationCommand
from autonav_universe.cli import _suite
from autonav_universe.dynamics import RobotProfile, RobotState
from autonav_universe.perception import ReferenceCostmap, pca_grade_percent
from autonav_universe.runner import EpisodeRunner, footprint_goal_fraction
from autonav_universe.scenarios import Circle, Scenario, load_scenario
from tools.audit_course_export import audit


class UniverseTests(unittest.TestCase):
    def test_local_goal_rotates_from_world_to_odom(self):
        scenario = Scenario("rotate", 20, 20, (5, 5, math.pi/2), ((5, 9),))
        goal = scenario.public_goals()[0]
        self.assertAlmostEqual(goal.x_m, 4)
        self.assertAlmostEqual(goal.y_m, 0)

    def test_spawn_and_goal_validation(self):
        profile = RobotProfile()
        with self.assertRaisesRegex(ValueError, "spawn"):
            Scenario("bad", 10, 10, (0.05, 5, 0), ((8, 5),)).validate(profile)
        with self.assertRaisesRegex(ValueError, "goal inside obstacle"):
            Scenario("bad", 10, 10, (2, 5, 0), ((8, 5),),
                     obstacles=(Circle(8, 5, 1),)).validate(profile)

    def test_packaged_scenarios_match_repository(self):
        root = Path(__file__).resolve().parents[1]
        packaged = root / "src" / "autonav_universe" / "data" / "scenarios"
        for path in (root / "scenarios").glob("*.json"):
            self.assertEqual(path.read_bytes(), (packaged / path.name).read_bytes())
        packaged_suite = root / "src" / "autonav_universe" / "data" / "benchmarks"
        for path in (root / "benchmarks").glob("*.json"):
            self.assertEqual(path.read_bytes(), (packaged_suite / path.name).read_bytes())

    def test_course_metadata_audit_checks_units_and_properties(self):
        identity = [[1, 0, 0, 0], [0, 1, 0, 0],
                    [0, 0, 1, 0], [0, 0, 0, 1]]
        data = {"schema_version": 1, "unit_system": "METRIC", "scale_length": 1.0,
                "objects": [{"name": "ramp", "matrix_world": identity,
                             "bounds_m": {"min": [0, 0, 0], "max": [1, 1, 1]},
                             "custom_properties": {"grade": 15}}]}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "metadata.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertTrue(audit(path)["valid_geometry_metadata"])
            self.assertEqual(audit(path)["custom_property_keys"], {"grade": 1})
            data["scale_length"] = 0.01
            path.write_text(json.dumps(data), encoding="utf-8")
            self.assertFalse(audit(path)["valid_geometry_metadata"])

    def test_invalid_suite_scenario_is_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "suite.json"
            path.write_text(json.dumps({"schema_version": 1,
                                        "scenarios": ["straight", "absent-case"],
                                        "seeds": [1]}), encoding="utf-8")
            scenarios, seeds, invalid = _suite(str(path))
            self.assertEqual([scenario.name for scenario in scenarios], ["straight"])
            self.assertEqual(seeds, [1])
            self.assertEqual(invalid[0]["scenario"], "absent-case")

    def test_footprint_goal_area_rule(self):
        profile = RobotProfile()
        state = RobotState(5, 5, 0)
        self.assertEqual(footprint_goal_fraction(state, profile, (5, 5), 1), 1)
        self.assertEqual(footprint_goal_fraction(state, profile, (8, 5), 1), 0)

    def test_command_expiry_brakes(self):
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:Forward")
        runner.command = NavigationCommand(1, 0, 0.1)
        runner.time_s = 0.1
        runner._controller()
        self.assertEqual(runner.applied, (0, 0))

    def test_same_seed_repeats_truth(self):
        scenario = load_scenario("straight")
        first = EpisodeRunner(scenario, "tests.plugins:Forward", seed=4)
        second = EpisodeRunner(scenario, "tests.plugins:Forward", seed=4)
        self.assertEqual(first.run().outcome, "success")
        self.assertEqual(second.run().outcome, "success")
        self.assertEqual(first.truth, second.truth)
        commands = [event for event in first.events if event["type"] == "command"]
        self.assertEqual(commands[0]["time_s"], 0)
        self.assertAlmostEqual(commands[1]["time_s"], 0.05)

    def test_sensor_latency_preserves_acquisition_and_delivery(self):
        scenario = Scenario("latency", 10, 10, (2, 5, 0), ((8, 5),),
                            sensors=("odometry",), odom_latency_s=0.07,
                            max_time_s=0.2)
        runner = EpisodeRunner(scenario, "tests.plugins:Forward", capture_observations=True)
        runner.run()
        self.assertEqual(runner.observations[0]["samples"], ())
        delayed = next(s for record in runner.observations for s in record["samples"]
                       if s["kind"] == "odometry")
        self.assertAlmostEqual(delayed["acquired_s"], 0)
        self.assertAlmostEqual(delayed["delivered_s"], 0.07)

    def test_sensor_queue_is_bounded_and_reports_drops(self):
        scenario = Scenario("queue", 10, 10, (2, 5, 0), ((8, 5),),
                            sensors=("gps",), gps_latency_s=100)
        runner = EpisodeRunner(scenario, "tests.plugins:Forward")
        for index in range(260):
            runner.time_s = index*0.1
            runner._sample("gps")
        self.assertEqual(len(runner._delivery_queue), 256)
        self.assertEqual(runner.sensor_queue_drops, 4)

    def test_sensor_frame_pca_and_costmap(self):
        low = [(i*0.1, j*0.1, 0.15*i*0.1-0.3)
               for i in range(1, 21) for j in range(-5, 6)]
        high = [(i*0.1, j*0.1, 0.30*i*0.1-0.3)
                for i in range(1, 21) for j in range(-5, 6)]
        self.assertAlmostEqual(pca_grade_percent(low), 15, places=5)
        self.assertAlmostEqual(pca_grade_percent(high), 30, places=5)
        costmap = ReferenceCostmap(20)
        costmap.update(tuple(low), 0, 0, 0)
        self.assertNotIn("steep", costmap.cells.values())
        costmap = ReferenceCostmap(20)
        costmap.update(tuple(high), 0, 0, 0)
        self.assertIn("steep", costmap.cells.values())

    def test_reference_ramp_traversal_and_forbidden_bypass(self):
        allowed = EpisodeRunner(load_scenario("ramp"),
                                "autonav_universe.algorithms:ReferenceNavigator").run()
        forbidden = EpisodeRunner(load_scenario("ramp-forbidden"),
                                  "autonav_universe.algorithms:ReferenceNavigator").run()
        self.assertEqual(allowed.outcome, "success")
        self.assertEqual(allowed.ramp_traversals, 1)
        self.assertEqual(forbidden.outcome, "success")
        self.assertEqual(forbidden.ramp_traversals, 0)

    def test_contact_prevents_clean_arrival(self):
        scenario = Scenario("blocked", 12, 8, (2, 4, 0), ((9, 4),),
                            obstacles=(Circle(5, 4, 0.5),))
        runner = EpisodeRunner(scenario, "tests.plugins:Forward")
        result = runner.run()
        self.assertEqual(result.outcome, "collision")
        self.assertEqual(result.contacts, 1)
        self.assertIsNotNone(result.first_contact_time_s)
        self.assertLessEqual(result.min_clearance_m, 0)

    def test_invalid_command_is_recorded(self):
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:Invalid")
        self.assertEqual(runner.run().outcome, "invalid_action")
        self.assertIn("invalid_action", [event["type"] for event in runner.events])
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:BadDiagnostics")
        self.assertEqual(runner.run().outcome, "invalid_action")

    def test_plugin_exception_isolated(self):
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:Crash")
        self.assertEqual(runner.run().outcome, "algorithm_error")

    def test_plugin_hang_is_bounded(self):
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:Hang",
                               algorithm_timeout_s=0.2)
        self.assertEqual(runner.run().outcome, "algorithm_timeout")

    def test_deadline_discards_late_actions(self):
        scenario = Scenario("short", 10, 10, (2, 5, 0), ((8, 5),),
                            max_time_s=0.25)
        runner = EpisodeRunner(scenario, "tests.plugins:Slow", timing_mode="deadline",
                               action_budget_s=0.03, algorithm_timeout_s=0.5)
        result = runner.run()
        self.assertEqual(result.outcome, "timeout")
        self.assertGreaterEqual(result.deadline_overruns, 1)
        self.assertEqual(result.commands, 0)
        self.assertIn("late_action_discarded", [event["type"] for event in runner.events])

    def test_manifest_rejects_incompatible_api_and_sensors(self):
        runner = EpisodeRunner(load_scenario("straight"), "tests.plugins:WrongVersion")
        self.assertEqual(runner.run().outcome, "algorithm_error")
        runner = EpisodeRunner(load_scenario("gps-blackout"), "tests.plugins:NeedsLidar")
        self.assertEqual(runner.run().outcome, "algorithm_error")

    def test_gps_only_does_not_expose_heading_or_lidar(self):
        runner = EpisodeRunner(load_scenario("gps-blackout"), "tests.plugins:Forward",
                               capture_observations=True)
        runner.run()
        first = runner.observations[0]["samples"]
        self.assertEqual({s["kind"] for s in first}, {"odometry", "gps"})
        odom = next(s for s in first if s["kind"] == "odometry")
        gps = next(s for s in first if s["kind"] == "gps")
        self.assertEqual(len(odom["data"]), 2)
        self.assertEqual(set(gps["data"]), {"latitude_deg", "longitude_deg"})

    def test_planner_track_receives_only_fixed_sensor_derived_reference(self):
        scenario = replace(load_scenario("planner-straight"),
                           odom_latency_s=0.1, lidar_latency_s=0.1,
                           max_time_s=0.3)
        runner = EpisodeRunner(scenario,
                               "autonav_universe.algorithms:FixedPipelinePlanner",
                               capture_observations=True)
        self.assertEqual(runner._context().capabilities, ("reference-navigation",))
        runner.run()
        first = runner.observations[0]
        self.assertEqual(first["samples"], ())
        self.assertIsNone(first["reference"]["pose"])
        self.assertEqual(first["reference"]["costmap_cells"], ())
        delivered = next(record for record in runner.observations
                         if record["reference"]["pose"] is not None)
        self.assertEqual(delivered["samples"], ())
        self.assertEqual(delivered["reference"]["pose"]["frame"], "odom")
        self.assertTrue(delivered["reference"]["costmap_cells"])
        self.assertEqual(delivered["reference"]["pipeline_version"], "pca-costmap-v1")
        self.assertTrue(runner.perception_ms)

    def test_gps_waypoints_lidar_ramp_and_outage_share_one_run(self):
        scenario = load_scenario("gps-combined")
        runner = EpisodeRunner(scenario, "autonav_universe.algorithms:GPSLidarNavigator",
                               capture_observations=True)
        goals = runner._context().goals
        self.assertTrue(all(goal.frame == "gps" and goal.x_m is None and goal.y_m is None
                            for goal in goals))
        result = runner.run()
        self.assertEqual(result.outcome, "success")
        self.assertEqual(result.goals_reached, 2)
        self.assertEqual(result.ramp_traversals, 1)
        near_obstacle = min(runner.truth, key=lambda frame: abs(frame["x_m"]-12))
        self.assertGreater(abs(near_obstacle["y_m"]-5), 1.0)
        self.assertTrue(any(not sample["valid"] for observation in runner.observations
                            for sample in observation["samples"] if sample["kind"] == "gps"))


if __name__ == "__main__":
    unittest.main()
