"""The accelerated synthetic tape lookup must preserve exact sensor geometry."""
import math
import random
import unittest
from types import SimpleNamespace

from igvc_competition_sim.course import load_course
from igvc_competition_sim.sensor_harness import IgvcSensorHarness
from igvc_competition_sim.tape_index import build_tape_index


class TapeIndexTests(unittest.TestCase):
    def check_queries(self, tapes, queries):
        index = build_tape_index(tapes)
        distance = IgvcSensorHarness._point_segment_distance
        for x, y in queries:
            expected = any(distance(x, y, tape.start, tape.end) <= tape.width_m / 2
                           for tape in tapes)
            actual = any(distance(x, y, tape.start, tape.end) <= tape.width_m / 2
                         for tape in index.get((math.floor(x), math.floor(y)), ()))
            self.assertEqual(actual, expected, (x, y))

    def test_negative_cells_endpoints_and_degenerate_segments(self):
        tapes = [
            SimpleNamespace(start=(-2.0, -1.0), end=(3.0, 4.0), width_m=0.2),
            SimpleNamespace(start=(-1.0, -1.0), end=(-1.0, -1.0), width_m=0.2),
        ]
        queries = [(-1.1, -1.0), (-0.9, -1.0), (-1.0, -1.1),
                   (-1.0, -0.9), (-2.0, -1.0), (3.0, 4.0)]
        rng = random.Random(61)
        queries += [(rng.uniform(-3, 4), rng.uniform(-2, 5)) for _ in range(500)]
        self.check_queries(tapes, queries)

    def test_full_course_matches_exhaustive_tape_lookup(self):
        course = load_course('/host/runs/full-track-diagnostic/course.yaml')
        rng = random.Random(62)
        queries = [(rng.uniform(-10, 40), rng.uniform(-10, 30)) for _ in range(1000)]
        for tape in course.tapes:
            queries.extend([tape.start, tape.end,
                            ((tape.start[0] + tape.end[0]) / 2,
                             (tape.start[1] + tape.end[1]) / 2)])
        self.check_queries(course.tapes, queries)


if __name__ == '__main__':
    unittest.main()
