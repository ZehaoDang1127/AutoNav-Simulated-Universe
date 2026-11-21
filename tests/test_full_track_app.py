"""Desktop control must stop on lost ownership and preserve fresh run state."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from autonav_universe.full_track import (ControlLease, FullTrackSession, MissionClock,
                                         load_bundle, read_json, write_json)


def make_bundle(root):
    repo = Path(root)
    source = repo / "Mock_Course_Asset/Course.blend"
    source.parent.mkdir()
    source.write_bytes(b"test course source")
    bundle = repo / "runs/bundle"
    bundle.mkdir(parents=True)
    (bundle / "course.yaml").write_text(
        'course_id: test\nstart: {x_m: 0, y_m: 0, yaw_rad: 0}\n'
        'mission_waypoints: [{label: "1", x_m: 4, y_m: 1, radius_m: 1}]\n')
    (bundle / "world.sdf").write_text('<sdf version="1.9"><world name="test"/></sdf>')
    manifest = {"course_id": "test", "geometry_check": {"passed": True},
                "semantic_gaps": ["test asset is diagnostic"]}
    for path, key in ((source, "source_asset_sha256"),
                      (bundle / "course.yaml", "course_yaml_sha256"),
                      (bundle / "world.sdf", "world_sdf_sha256")):
        manifest[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    write_json(bundle / "course_manifest.json", manifest)
    return repo, bundle


class LeaseTests(unittest.TestCase):
    def test_supervisor_io_failure_is_not_mislabeled_as_operator_stop(self):
        lease = ControlLease()
        self.assertEqual(lease.update({"heartbeat": 1, "desired": "stopped",
                                      "reason": "desktop_heartbeat_write_failed"}, 1), "stopped")
        self.assertEqual(lease.reason, "desktop_heartbeat_write_failed")

    def test_atomic_heartbeat_retries_transient_windows_file_sharing(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "control.json"
            replace = os.replace
            calls = []

            def briefly_locked(source, destination):
                calls.append(1)
                if len(calls) < 3:
                    raise PermissionError("File is being read by Docker Desktop")
                return replace(source, destination)
            with patch("autonav_universe.full_track.os.replace", side_effect=briefly_locked):
                write_json(target, {"heartbeat": 1, "desired": "running"})
            self.assertEqual(read_json(target)["heartbeat"], 1)
            self.assertEqual(len(calls), 3)

    def test_pausing_does_not_lose_ownership_and_stop_cannot_resume(self):
        lease = ControlLease()
        self.assertEqual(lease.update({"heartbeat": 1, "desired": "running"}, 0), "running")
        self.assertEqual(lease.update({"heartbeat": 2, "desired": "paused"}, 1), "paused")
        self.assertEqual(lease.update({"heartbeat": 3, "desired": "running"}, 2), "running")
        self.assertEqual(lease.update({"heartbeat": 4, "desired": "stopped"}, 2.1), "stopped")
        self.assertEqual(lease.update({"heartbeat": 5, "desired": "running"}, 2.2), "stopped")
        self.assertEqual(lease.reason, "operator_stop")

    def test_stale_repeated_or_corrupt_file_cannot_keep_motion_enabled(self):
        for invalid in ({}, {"heartbeat": 1, "desired": "running"},
                        {"heartbeat": 2, "desired": "invalid"}):
            with self.subTest(invalid=invalid):
                lease = ControlLease()
                lease.update({"heartbeat": 1, "desired": "running"}, 10)
                self.assertEqual(lease.update(invalid, 12), "stopped")
                self.assertEqual(lease.reason, "desktop_heartbeat_lost")
                self.assertEqual(lease.update({"heartbeat": 100, "desired": "running"}, 13), "stopped")

    def test_pause_excludes_wall_time_from_mission_budget(self):
        clock = MissionClock(100)
        self.assertEqual(clock.update(110, True), 110)
        self.assertEqual(clock.update(200, True), 110)
        self.assertEqual(clock.update(250, False), 110)
        self.assertEqual(clock.update(260, False), 120)
        self.assertEqual(clock.paused_s, 140)


@unittest.skipUnless(importlib.util.find_spec("yaml"), "Full course GUI extra is optional")
class SessionTests(unittest.TestCase):
    def test_bundle_detects_source_and_world_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)
            self.assertEqual(load_bundle(repo, bundle)[0]["course_id"], "test")
            (bundle / "world.sdf").write_text("changed")
            with self.assertRaisesRegex(ValueError, "mismatched"):
                load_bundle(repo, bundle)

    def test_fresh_container_success_and_scoped_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)

            class FakeSession(FullTrackSession):
                calls = []
                inspections = 0

                def docker(self, *args, **kwargs):
                    self.calls.append(args)
                    if args[0] == "info":
                        return "linux"
                    if args[0] == "run":
                        write_json(self.run_dir / "result.json", {"mission_passed": True})
                    if args[0] == "inspect":
                        self.inspections += 1
                        return json.dumps([{"State": {"Running": self.inspections == 1},
                                            "NetworkSettings": {"Ports": {"8765/tcp": [{"HostPort": "40000"}]}}}])
                    return ""

            session = FakeSession(repo, bundle)
            snapshots = []
            with patch("autonav_universe.full_track.urlopen", side_effect=OSError("not connected")):
                session.run(snapshots.append)
            self.assertTrue(session.done.is_set())
            self.assertEqual(snapshots[-1]["state"], "Completed")
            self.assertEqual(read_json(session.run_dir / "desktop_host_result.json")["state"], "Completed")
            launch = next(call for call in session.calls if call[0] == "run")
            self.assertIn("DESKTOP_CONTROL=true", launch)
            self.assertIn("127.0.0.1::8765", launch)
            self.assertIn(("rm", "-f", session.name), session.calls)
            self.assertEqual(read_json(session.run_dir / "desktop_control.json")["desired"], "stopped")
            self.assertNotEqual(FullTrackSession(repo, bundle).run_dir, session.run_dir)

    def test_docker_failure_is_visible_and_does_not_remove_unowned_containers(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)
            session = FullTrackSession(repo, bundle)
            snapshots = []
            with patch.object(session, "docker", side_effect=RuntimeError("Linux engine unavailable")) as docker:
                session.run(snapshots.append)
            self.assertEqual(snapshots[-1]["state"], "Failed")
            self.assertIn("Linux engine unavailable", snapshots[-1]["message"])
            self.assertEqual(docker.call_count, 1)
            self.assertTrue(session.done.is_set())

    def test_early_container_exit_reports_logs_and_preserves_them(self):
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)
            session = FullTrackSession(repo, bundle)
            calls = []

            def docker(*args, **kwargs):
                calls.append(args)
                if args[0] == "info":
                    return "linux"
                if args[0] == "inspect":
                    return json.dumps([{"State": {"Running": False, "ExitCode": 2},
                                        "NetworkSettings": {"Ports": {}}}])
                if args[0] == "logs":
                    return "launcher: invalid shell option"
                return ""

            snapshots = []
            with patch.object(session, "docker", side_effect=docker):
                session.run(snapshots.append)
            result = read_json(session.run_dir / "desktop_host_result.json")
            self.assertEqual(result["state"], "Failed")
            self.assertIn("exit 2", result["reason"])
            self.assertIn("launcher: invalid shell option", snapshots[-1]["message"])
            self.assertEqual((session.run_dir / "container.log").read_text(),
                             "launcher: invalid shell option")
            self.assertIn(("rm", "-f", session.name), calls)

    def test_docker_logs_include_container_stderr(self):
        from subprocess import CompletedProcess
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)
            session = FullTrackSession(repo, bundle)
            result = CompletedProcess([], 0, stdout="normal output\n", stderr="fatal error\n")
            with patch("autonav_universe.full_track.subprocess.run", return_value=result):
                self.assertEqual(session.docker("logs", "container"), "normal output\nfatal error")

    def test_client_timeout_cleans_up_a_container_only_with_our_ownership_label(self):
        import subprocess
        for owned in (False, True):
            with self.subTest(owned=owned), tempfile.TemporaryDirectory() as directory:
                repo, bundle = make_bundle(directory)
                session = FullTrackSession(repo, bundle)
                calls = []

                def docker(*args, **kwargs):
                    calls.append(args)
                    if args[0] == "info":
                        return "linux"
                    if args[0] == "run":
                        raise subprocess.TimeoutExpired("docker run", 45)
                    if args[0] == "inspect":
                        return json.dumps([{"Config": {"Labels": {"autonav.desktop-session":
                                                                  session.name if owned else "someone-else"}}}])
                    return ""
                with patch.object(session, "docker", side_effect=docker):
                    session.run(lambda value: None)
                self.assertEqual(("rm", "-f", session.name) in calls, owned)
                self.assertTrue(session.done.is_set())

    def test_desktop_stop_cannot_be_reported_as_course_success(self):
        from autonav_universe.cli import PACKAGE_ROOT
        import subprocess
        import sys
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            documents = {"score": {"failed": False, "all_waypoints_reached": True, "finish_reached": True},
                         "gate": {"safety_gate_passed": True},
                         "status": {"state": "Stopped", "reason": "operator_stop"}}
            for name, value in documents.items():
                write_json(folder / name, value)
            (folder / "mission").write_text("")
            process = subprocess.run([
                sys.executable, str(PACKAGE_ROOT / "integrations/ros_humble_fortress/run_summary.py"),
                "--score", str(folder / "score"), "--gate", str(folder / "gate"),
                "--mission-log", str(folder / "mission"), "--mission-exit", "0",
                "--desktop-status", str(folder / "status")], capture_output=True, text=True)
            self.assertEqual(process.returncode, 1)
            result = json.loads(process.stdout)
            self.assertEqual(result["reason"], "operator_stop")
            self.assertFalse(result["mission_passed"])


class AppRoutingTests(unittest.TestCase):
    def test_default_app_opens_full_course_and_python_is_explicit(self):
        from autonav_universe.cli import main
        with patch("autonav_universe.full_track_ui.launch_full_track_app") as full:
            self.assertEqual(main(["app"]), 0)
            full.assert_called_once_with(None, None)
        with patch("autonav_universe.ui.launch_app") as python:
            self.assertEqual(main(["app", "--backend", "python"]), 0)
            python.assert_called_once_with()


@unittest.skipUnless(importlib.util.find_spec("PyQt5") and importlib.util.find_spec("yaml"),
                     "Qt workspace extras are optional")
class QtWorkspaceTests(unittest.TestCase):
    def test_preview_and_pause_resume_buttons_follow_supervisor_acknowledgements(self):
        from types import SimpleNamespace
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        if os.name == "nt":
            os.environ.setdefault("QT_QPA_FONTDIR", "C:/Windows/Fonts")
        from autonav_universe.full_track_ui import _classes
        _, widgets, _, Window = _classes()
        app = widgets.QApplication.instance() or widgets.QApplication([])
        with tempfile.TemporaryDirectory() as directory:
            repo, bundle = make_bundle(directory)
            window = Window(repo, bundle)
            try:
                self.assertEqual(window.canvas.course["course_id"], "test")
                self.assertTrue(window.run_button.isEnabled())
                window.session = FullTrackSession(repo, bundle)
                window.worker = SimpleNamespace(is_alive=lambda: True)
                window.on_snapshot({"state": "Running"})
                self.assertTrue(window.pause_button.isEnabled())
                window.pause_button.click()
                self.assertEqual(window.session.desired, "paused")
                self.assertFalse(window.pause_button.isEnabled())
                window.on_snapshot({"state": "Paused"})
                self.assertEqual(window.pause_button.text(), "Resume")
                self.assertTrue(window.pause_button.isEnabled())
                window.pause_button.click()
                self.assertEqual(window.session.desired, "running")
                window.on_snapshot({"state": "Running"})
                window.stop_button.click()
                self.assertEqual(window.session.desired, "stopped")
                self.assertFalse(window.pause_button.isEnabled())
            finally:
                window.worker = None
                window.close()


if __name__ == "__main__":
    unittest.main()
