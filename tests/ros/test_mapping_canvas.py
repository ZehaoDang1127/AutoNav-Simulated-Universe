"""Course geometry must not predeclare unseen regions free of obstacles."""
from types import SimpleNamespace
import threading
import time
import unittest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Time
from igvc_competition_sim.sensor_harness import IgvcSensorHarness


class MappingCanvasTests(unittest.TestCase):
    def test_busy_raycaster_cannot_starve_map_timer(self):
        started, release, map_during_scan = threading.Event(), threading.Event(), threading.Event()

        class BusyHarness(IgvcSensorHarness):
            def _publish_sensor_frame(self):
                started.set()
                release.wait(5)

            def _publish_map(self):
                if started.is_set() and not release.is_set():
                    map_during_scan.set()

        rclpy.init()
        node = BusyHarness()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        try:
            self.assertTrue(started.wait(2), 'Ray-casting callback did not start')
            self.assertTrue(map_during_scan.wait(2.5), 'Map publication was starved by a busy scan')
        finally:
            release.set()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            node.destroy_node()
            rclpy.shutdown()

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
