"""Tests for observed-only crossable paint processing."""
import math
import unittest
from types import SimpleNamespace
from autonav_universe.self_drive_divider import classified_divider, rasterize_observations


class DividerObservationTests(unittest.TestCase):
    def test_boundary_wins_at_overlaps(self):
        soft, hard = SimpleNamespace(crossable=True), SimpleNamespace(crossable=False)
        self.assertTrue(classified_divider([soft]))
        self.assertFalse(classified_divider([soft, hard]))
        self.assertFalse(classified_divider([]))

    def test_only_observed_points_and_short_gaps_are_marked(self):
        mask = rasterize_observations([(1.05, 1.05), (1.65, 1.05), (3.05, 1.05)],
                                      50, 30, .1, (0, 0, 0), bridge_m=.8)
        self.assertEqual(mask[10*50+13], 100)  # Short gap interpolated.
        self.assertEqual(mask[10*50+23], 0)  # Large gap stays unobserved.
        self.assertFalse(any(rasterize_observations([], 10, 10, .1, (0,0,0))))

    def test_rotated_origin_is_respected(self):
        mask = rasterize_observations([(0, 1)], 20, 20, .1, (0, 0, math.pi/2))
        self.assertEqual(mask[10], 100)


if __name__ == '__main__':
    unittest.main()
