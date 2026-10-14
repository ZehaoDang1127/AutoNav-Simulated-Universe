"""The preflight map must survive late discovery and still obey freshness."""
import importlib.util
from pathlib import Path
import time
import unittest

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

source_file = Path(__file__).resolve().parents[2] / 'integrations/ros_humble_fortress/readiness_probe.py'
spec = importlib.util.spec_from_file_location('readiness_probe', source_file)
readiness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(readiness)


class ReadinessMapTests(unittest.TestCase):
    def test_late_reader_receives_large_cached_map_and_rejects_stale_stamp(self):
        rclpy.init()
        source = rclpy.create_node('latched_map_readiness_source')
        probe = None
        try:
            publisher = source.create_publisher(OccupancyGrid, '/map_padded',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            grid = OccupancyGrid()
            grid.header.stamp = source.get_clock().now().to_msg()
            grid.header.frame_id = 'map'
            grid.info.width, grid.info.height, grid.info.resolution = 880, 920, .05
            grid.data = [-1] * (880*920)
            publisher.publish(grid)
            probe = readiness.ReadinessProbe('lidar')
            deadline = time.monotonic()+5
            while time.monotonic()<deadline and '/map_padded' not in probe.samples:
                rclpy.spin_once(probe, timeout_sec=.05)
            self.assertIn('/map_padded', probe.samples)
            stamp = readiness.seconds(grid.header.stamp)
            self.assertAlmostEqual(probe.samples['/map_padded'][0], stamp)
            probe.clock_s = stamp+4
            self.assertIn('fresh /map_padded', probe.missing(3, 3))
        finally:
            if probe is not None:
                probe.destroy_node()
            source.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
