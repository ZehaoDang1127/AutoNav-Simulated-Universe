"""Checks for evaluation-only straightness measurements."""
import importlib.util
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('self_drive_straightness', ROOT/'tools/analyze_self_drive_straightness.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StraightnessTests(unittest.TestCase):
    def test_fitted_straight_line_has_no_waviness_or_extra_distance(self):
        points = [(i*.1, 2+.3*i*.1) for i in range(101)]
        metrics = module.straightness(points, 'x', 0, 10)
        self.assertAlmostEqual(metrics['lateral_residual_rms_m'], 0.0, places=10)
        self.assertAlmostEqual(metrics['extra_distance_percent'], 0.0, places=10)
        self.assertAlmostEqual(metrics['lateral_end_to_end_m'], 3.0)

    def test_waviness_is_measured_and_segments_are_not_joined(self):
        wave = [(i*.1, .2*math.sin(i*.1*math.pi)) for i in range(101)]
        path = [(0, 20), (1, 20), (2, 20), (3, 20)] + [(50, 50)] + wave
        metrics = module.straightness(path, 'x', 0, 10)
        self.assertEqual(metrics['samples'], 101)
        self.assertGreater(metrics['lateral_residual_rms_m'], .1)
        self.assertGreater(metrics['extra_distance_percent'], 1)

    def test_invalid_or_degenerate_regions_fail_clearly(self):
        with self.assertRaises(ValueError):
            module.straightness([(0, 1), (0, 2), (0, 3)], 'x', -1, 1)
        with self.assertRaises(ValueError):
            module.straightness([(0, 1)], 'y', 2, 3)


if __name__ == '__main__':
    unittest.main()
