"""Course geometry must not predeclare unseen regions free of obstacles."""
from types import SimpleNamespace
import unittest

from builtin_interfaces.msg import Time
from igvc_competition_sim.sensor_harness import IgvcSensorHarness


class MappingCanvasTests(unittest.TestCase):
    def test_canvas_preserves_unknown_with_correct_geometry(self):
        published = []
        stamp = Time(sec=10, nanosec=0)
        stub = SimpleNamespace(
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
            _map_bounds=lambda resolution: (-1.0, -2.0, 1.0, 1.0),
            map_pub=SimpleNamespace(publish=published.append))
        IgvcSensorHarness._publish_map(stub)
        self.assertEqual(len(published), 1)
        message = published[0]
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual((message.info.width, message.info.height), (40, 60))
        self.assertEqual(message.info.resolution, .05)
        self.assertEqual((message.info.origin.position.x, message.info.origin.position.y), (-1, -2))
        self.assertEqual(len(message.data), 2400)
        self.assertEqual(set(message.data), {-1})


if __name__ == '__main__':
    unittest.main()
