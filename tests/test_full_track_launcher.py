"""Keep full-course input pairing and startup failure reports trustworthy."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from integrations.ros_humble_fortress.validate_course_bundle import validate


ROOT = Path(__file__).resolve().parents[1]


class FullTrackLauncherTests(unittest.TestCase):
    @unittest.skipUnless(os.name == 'posix', 'Full runner executes in the Linux container')
    def test_full_run_keeps_a_snapshot_when_source_script_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            host = Path(directory)
            bundle = host / 'bundle'
            bundle.mkdir()
            bundle, source = self.bundle(bundle)
            assets = host / 'Mock_Course_Asset'
            assets.mkdir()
            shutil.copyfile(source, assets / 'Course.blend')
            scripts = host / 'integrations/ros_humble_fortress'
            scripts.mkdir(parents=True)
            shutil.copyfile(ROOT / 'integrations/ros_humble_fortress/validate_course_bundle.py',
                            scripts / 'validate_course_bundle.py')
            runner = scripts / 'run_compact_smoke.sh'
            runner.write_text('echo snapshot_ready\nsleep 0.3\necho snapshot_done\n')
            (scripts / 'full_track_dynamics.yaml').write_text('{}\n')
            robot = host / 'integrations/robot'
            robot.mkdir()
            (robot / 'full_track_baseline.yaml').write_text('{}\n')
            (robot / 'full_track_bt.xml').write_text('<root/>\n')
            output = host / 'output'
            process = subprocess.Popen([
                'bash', str(ROOT / 'integrations/ros_humble_fortress/run_full_track.sh')],
                env={**os.environ, 'HOST_REPO': str(host),
                     'COURSE_BUNDLE': str(bundle), 'RUN_DIR': str(output)},
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                while True:
                    line = process.stdout.readline()
                    if line.strip() == 'snapshot_ready' or not line:
                        break
                self.assertEqual(line.strip(), 'snapshot_ready')
                runner.write_text('echo "broken source script\n')
                stdout, stderr = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, stderr)
                self.assertIn('snapshot_done', stdout)
                self.assertIn('snapshot_done', (output / 'runner.sh').read_text())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_humble_recovery_waits_are_nonnegative_integer_seconds(self):
        tree = ET.parse(ROOT / 'integrations/robot/full_track_bt.xml')
        waits = tree.findall('.//Wait')
        self.assertTrue(waits)
        for wait in waits:
            duration = wait.attrib['wait_duration']
            self.assertTrue(duration.isdigit(),
                            f'Humble truncates fractional wait duration {duration!r}')
            self.assertGreaterEqual(int(duration), 0)
            if wait.attrib.get('server_name') not in ('wait_plan', 'wait_controller'):
                self.assertGreater(int(duration), 0)

    def bundle(self, directory):
        bundle = Path(directory)
        source = bundle / 'Course.blend'
        source.write_bytes(b'course asset')
        (bundle / 'course.yaml').write_text('course_id: test\n', encoding='utf-8')
        (bundle / 'world.sdf').write_text('<sdf version="1.9"><world name="test"/></sdf>', encoding='utf-8')
        manifest = {'course_id': 'test', 'geometry_check': {'passed': True}}
        for path, key in ((source, 'source_asset_sha256'),
                          (bundle / 'course.yaml', 'course_yaml_sha256'),
                          (bundle / 'world.sdf', 'world_sdf_sha256')):
            manifest[key] = hashlib.sha256(path.read_bytes()).hexdigest()
        (bundle / 'course_manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
        return bundle, source

    def test_matched_bundle_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle, source = self.bundle(directory)
            self.assertEqual(validate(bundle, source)['course_id'], 'test')

    def test_mismatched_world_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle, source = self.bundle(directory)
            (bundle / 'world.sdf').write_text('<sdf version="1.9"><world name="other"/></sdf>', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'mismatched course bundle'):
                validate(bundle, source)

    def test_stale_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bundle, source = self.bundle(directory)
            source.write_bytes(b'changed asset')
            with self.assertRaisesRegex(ValueError, 'source_asset_sha256'):
                validate(bundle, source)

    def test_startup_failure_cannot_report_mission_success(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            documents = {
                'score': {'failed': False, 'all_waypoints_reached': True, 'finish_reached': True},
                'gate': {'safety_gate_passed': True},
                'readiness': {'ready': False, 'missing': ['fresh /scan_fullframe']},
            }
            for name, value in documents.items():
                (directory / name).write_text(json.dumps(value), encoding='utf-8')
            (directory / 'mission').write_text('', encoding='utf-8')
            process = subprocess.run([
                sys.executable, str(ROOT / 'integrations/ros_humble_fortress/run_summary.py'),
                '--score', str(directory / 'score'), '--gate', str(directory / 'gate'),
                '--readiness', str(directory / 'readiness'),
                '--mission-log', str(directory / 'mission'), '--mission-exit', '0',
            ], capture_output=True, text=True)
            result = json.loads(process.stdout)
            self.assertEqual(process.returncode, 1)
            self.assertFalse(result['mission_passed'])
            self.assertEqual(result['reason'], 'startup_readiness_failed')
            self.assertTrue(result['safety_gate_passed'])


if __name__ == '__main__':
    unittest.main()
