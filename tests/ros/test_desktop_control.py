"""Finish/Stop races must retain the final command and let scoring finish.

These call the real control callback without creating a ROS node, publishing
topics, or starting Gazebo. Run in the sourced pinned workspace for its imports.
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from autonav_universe.full_track import ControlLease, MissionClock, write_json

spec = importlib.util.spec_from_file_location("desktop_mission", ROOT /
    "integrations/ros_humble_fortress/desktop_mission.py")
desktop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(desktop)


class DesktopControlTests(unittest.TestCase):
    def exercise(self, desired, paused, finishing):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            write_json(folder / "desktop_control.json", {"heartbeat": 1, "desired": desired})
            physics, reports, enables = [], [], []
            node = SimpleNamespace(
                folder=folder, lease=ControlLease(), active_clock=MissionClock(time.monotonic(), paused),
                paused=paused, finishing=finishing, control_failure=None, referee_failed=False,
                mission_active=not finishing, deadline_wall=time.monotonic()+900,
                watchdog_grace=float("inf"), last_sensor={},
                set_physics_pause=physics.append,
                report=lambda state, reason=None: reports.append((state, reason)),
                _publish_enable=lambda: enables.append(True))
            desktop.DesktopMission.control_tick(node)
            self.assertTrue(enables)
            return node, physics, reports

    def test_finish_unfreezes_physics_with_navigation_already_disabled(self):
        node, physics, reports = self.exercise("paused", paused=True, finishing=True)
        self.assertEqual(physics, [False])
        self.assertFalse(node.paused)
        self.assertFalse(node.mission_active)
        self.assertNotIn(("Running", None), reports)

    def test_a_late_pause_cannot_freeze_final_score_publication(self):
        node, physics, reports = self.exercise("paused", paused=False, finishing=True)
        self.assertFalse(node.paused)
        self.assertEqual(physics, [])

    def test_final_control_read_preserves_operator_stop_after_an_action_abort(self):
        node, physics, reports = self.exercise("stopped", paused=True, finishing=True)
        self.assertEqual(node.control_failure, "operator_stop")
        self.assertTrue(node.referee_failed)
        self.assertEqual(physics, [False])
        self.assertEqual(reports[-1], ("Stopped", "operator_stop"))


if __name__ == "__main__":
    unittest.main()
