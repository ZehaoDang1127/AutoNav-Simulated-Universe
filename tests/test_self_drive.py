"""End-to-end checks for the waypoint-free track simulation."""

import math
import unittest

from autonav_universe.self_drive import SelfDriveSimulation


class SelfDriveTests(unittest.TestCase):
    def test_default_track_completes_a_safe_lap_and_avoids_obstacles(self):
        sim = SelfDriveSimulation()
        self.assertTrue(sim.is_safe(sim.x, sim.y))
        previous = (sim.x, sim.y)
        for _ in range(1600):
            sim.step(0.1)
            # Check the swept centre, not just sampled endpoints. is_safe
            # includes the simulation's conservative robot footprint.
            for fraction in (0.25, 0.5, 0.75, 1.0):
                x = previous[0] + fraction * (sim.x - previous[0])
                y = previous[1] + fraction * (sim.y - previous[1])
                self.assertTrue(sim.is_safe(x, y), (sim.time_s, x, y))
            previous = (sim.x, sim.y)
            self.assertEqual(sim.collision_count, 0)
            self.assertEqual(sim.status, "running")
            if sim.laps >= 1:
                break
        self.assertGreaterEqual(sim.laps, 1, (sim.time_s, sim.distance_m, sim.status))
        self.assertGreater(sim.distance_m, 20)
        self.assertGreaterEqual(sim.lane_changes, 1)

    def test_blocked_track_stops_without_crossing_a_boundary(self):
        # This disc spans the road ahead while leaving the start clear.
        sim = SelfDriveSimulation(obstacles=((5.6, 2.0, 1.6),))
        self.assertTrue(sim.is_safe(sim.x, sim.y))
        for _ in range(400):
            sim.step(0.1)
            self.assertTrue(sim.is_safe(sim.x, sim.y))
            self.assertEqual(sim.collision_count, 0)
            if sim.status == "blocked":
                break
        self.assertEqual(sim.status, "blocked")
        stopped = (sim.x, sim.y)
        for _ in range(20):
            sim.step(0.1)
        self.assertLess(math.dist(stopped, (sim.x, sim.y)), 1e-6)

    def test_reset_restarts_a_waypoint_free_run(self):
        sim = SelfDriveSimulation(obstacles=())
        start = (sim.x, sim.y, sim.yaw)
        for _ in range(30):
            sim.step(0.1)
        self.assertGreater(sim.distance_m, 0)
        sim.reset()
        self.assertEqual((sim.x, sim.y, sim.yaw), start)
        self.assertEqual((sim.time_s, sim.distance_m, sim.laps, sim.lane_changes), (0, 0, 0, 0))
        self.assertEqual(sim.status, "running")

    def test_clear_track_does_not_need_a_lane_change(self):
        sim = SelfDriveSimulation(obstacles=())
        for _ in range(1000):
            sim.step(0.1)
            if sim.laps:
                break
        self.assertEqual(sim.laps, 1)
        self.assertEqual(sim.lane_changes, 0)
        self.assertEqual(sim.collision_count, 0)


if __name__ == "__main__":
    unittest.main()
