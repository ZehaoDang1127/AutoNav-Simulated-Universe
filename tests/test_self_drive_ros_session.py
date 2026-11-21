"""Host-side checks for the ROS track launch and local costmap configuration."""
import importlib.util
import hashlib
import io
import json
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from autonav_universe.self_drive_ros import SelfDriveRosSession, reserve_ros_domain, referee_failure

ROOT = Path(__file__).resolve().parents[1]


def write_test_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


class SelfDriveSessionTests(unittest.TestCase):
    def test_completed_lap_stops_and_reports_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory)
            events = []
            calls = []
            def docker(*args, **kwargs):
                calls.append(args)
                if args[0] == 'info':
                    return 'linux'
                if args[0] == 'inspect':
                    first = sum(call[0] == 'inspect' for call in calls) == 1
                    detail = {'NetworkSettings': {'Ports': {'8765/tcp': [{'HostPort': '12345'}]}},
                              'State': {'Running': first, 'ExitCode': 0}}
                    return json.dumps([detail])
                return ''
            payload = json.dumps({'score': {'failed': False, 'finish_reached': True}}).encode()
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.full_track.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.self_drive_ros.urlopen', side_effect=lambda *a, **k: io.BytesIO(payload)), \
                 patch.object(session, 'docker', side_effect=docker):
                session.run(events.append)
            self.assertEqual(events[-1]['state'], 'Completed')
            self.assertTrue(session.done.is_set())
            self.assertEqual(session.desired, 'stopped')

    def test_lane_change_penalty_is_validated_and_sent_to_container(self):
        for value in (-.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                SelfDriveRosSession(lane_change_cost_m=value)
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory, lane_change_cost_m=.5)
            calls = []
            def docker(*args, **kwargs):
                calls.append(args)
                if args[0] == 'info':
                    return 'linux'
                if args[0] == 'run':
                    raise RuntimeError('test stop after container arguments')
                return ''
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.full_track.write_json', side_effect=write_test_json), \
                 patch.object(session, 'docker', side_effect=docker):
                session.run(lambda _: None)
            run = next(args for args in calls if args[0] == 'run')
            self.assertIn('SELF_DRIVE_LANE_CHANGE_COST_M=0.5', run)
            metadata = json.loads((session.run_dir/'desktop_session.json').read_text())
            self.assertEqual(metadata['lane_change_cost_m'], .5)

    def test_intersection_layout_and_turn_command_reach_container(self):
        with self.assertRaises(ValueError):
            SelfDriveRosSession(track_layout='figure8')
        with self.assertRaises(ValueError):
            SelfDriveRosSession(turn_command='right')
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory, track_layout='intersection', turn_command='left')
            calls = []
            def docker(*args, **kwargs):
                calls.append(args)
                if args[0] == 'info':
                    return 'linux'
                if args[0] == 'run':
                    raise RuntimeError('test stop after container arguments')
                return ''
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.full_track.write_json', side_effect=write_test_json), \
                 patch.object(session, 'docker', side_effect=docker):
                session.run(lambda _: None)
            run = next(args for args in calls if args[0] == 'run')
            self.assertIn('SELF_DRIVE_TRACK=intersection', run)
            self.assertIn('SELF_DRIVE_TURN_COMMAND=left', run)

    def test_turn_command_rides_the_desktop_heartbeat(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory, track_layout='intersection')
            session.run_dir.mkdir(parents=True)
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json):
                session.set_turn_command('LEFT')
                control = json.loads((session.run_dir/'desktop_control.json').read_text())
                self.assertEqual(control['turn_command'], 'left')
                with self.assertRaises(ValueError):
                    session.set_turn_command('right')
                self.assertEqual(session.turn_command, 'left')

    def test_left_exit_completes_intersection_route(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory, track_layout='intersection')
            (session.run_dir/'track').mkdir(parents=True)
            write_test_json(session.run_dir/'track'/'track.json',
                            {'exits': {'left': {'x_m': 15.6, 'y_m': 17.0, 'radius_m': 1.25}}})
            self.assertEqual(session._exit_reached({'x': 15.5, 'y': 16.5}), 'left')
            self.assertIsNone(session._exit_reached({'x': 15.5, 'y': 12.0}))
            loop = SelfDriveRosSession(repo=directory)
            self.assertIsNone(loop._exit_reached({'x': 15.5, 'y': 16.5}))

    def test_referee_failure_reports_reason_instead_of_driving(self):
        self.assertIsNone(referee_failure({'failed': False}))
        score = {'failed': True, 'failure_details': [{'reason': 'tape_crossing'}]}
        self.assertEqual(referee_failure(score), 'ROS safety monitor: tape_crossing')

    def test_concurrent_sessions_reserve_distinct_ros_domains(self):
        with patch('autonav_universe.self_drive_ros.secrets.randbelow', return_value=0):
            first, lock1 = reserve_ros_domain()
            try:
                second, lock2 = reserve_ros_domain()
                try:
                    self.assertNotEqual(first, second)
                    self.assertNotEqual(first, 0)
                    self.assertNotEqual(second, 0)
                finally:
                    lock2.close()
            finally:
                lock1.close()

    def test_stop_before_start_never_launches_a_container(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory)
            session.request('stopped')
            events = []
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.full_track.write_json', side_effect=write_test_json), \
                 patch.object(session, 'docker', side_effect=AssertionError('Docker should not run')):
                session.run(events.append)
            self.assertTrue(session.done.is_set())
            self.assertEqual(events[-1]['state'], 'Stopped')

    def test_docker_failure_is_reported_and_session_closes(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SelfDriveRosSession(repo=directory)
            events = []
            with patch('autonav_universe.self_drive_ros.write_json', side_effect=write_test_json), \
                 patch('autonav_universe.full_track.write_json', side_effect=write_test_json), \
                 patch.object(session, 'docker', side_effect=RuntimeError('Docker unavailable')):
                session.run(events.append)
            self.assertEqual(events[-1]['state'], 'Failed')
            self.assertIn('Docker unavailable', events[-1]['message'])
            self.assertTrue(session.done.is_set())

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'requires PyYAML')
    def test_nav2_profile_uses_full_local_sensor_grid(self):
        import yaml
        spec = importlib.util.spec_from_file_location('self_drive_config',
            ROOT / 'integrations/ros_humble_fortress/prepare_self_drive_nav2.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'nav2.yaml'
            module.make_config(ROOT / 'integrations/robot/full_track_baseline.yaml', output)
            config = yaml.safe_load(output.read_text(encoding='utf-8'))
            base = yaml.safe_load((ROOT / 'integrations/robot/full_track_baseline.yaml').read_text(encoding='utf-8'))
            local = config['local_costmap']['local_costmap']['ros__parameters']
            base_local = base['local_costmap']['local_costmap']['ros__parameters']
            self.assertNotIn('global_memory_layer', local['plugins'])
            self.assertIn('lidar_line_layer', local['plugins'])
            self.assertIn('obstacle_layer', local['plugins'])
            self.assertTrue(local['always_send_full_costmap'])
            self.assertIn('0.77', local['footprint'])
            self.assertIn('0.41', local['footprint'])
            # Restore ROS v2 line clearance and baseline obstacle inflation,
            # without changing occupancy marking or the padded footprint.
            self.assertEqual(local['footprint_padding'], 0.10)
            self.assertEqual(local['footprint'], base_local['footprint'])
            expected_obstacle = copy.deepcopy(base_local['obstacle_layer'])
            expected_obstacle['mark_scan']['obstacle_max_range'] = 4.5
            self.assertEqual(local['obstacle_layer'], expected_obstacle)
            self.assertTrue(local['obstacle_layer']['mark_scan']['marking'])
            self.assertEqual(local['inflation_layer'], base_local['inflation_layer'])
            self.assertEqual(local['inflation_layer']['inflation_radius'], 1.15)
            self.assertEqual(local['inflation_layer']['cost_scaling_factor'], 3.2)
            self.assertEqual((local['width'], local['height'], local['resolution']), (8, 8, 0.05))
            self.assertTrue(local['track_unknown_space'])
            for name in ('line_layer', 'lidar_line_layer'):
                self.assertTrue(local[name]['quadratic_clearance'])
                self.assertEqual(local[name]['clearance_reach_m'], 0.8)
                self.assertTrue(local[name]['preserve_unknown_space'])
                self.assertEqual(local[name]['inscribed_radius'], base_local[name]['inscribed_radius'])
                self.assertEqual(local[name]['inflation_radius'], base_local[name]['inflation_radius'])
                self.assertEqual(local[name]['cost_scaling_factor'], base_local[name]['cost_scaling_factor'])
                self.assertTrue(local[name]['enabled'])

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'requires PyYAML')
    def test_nav2_profile_uses_reference_mppi_controller(self):
        import yaml
        spec = importlib.util.spec_from_file_location('self_drive_config',
            ROOT / 'integrations/ros_humble_fortress/prepare_self_drive_nav2.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'nav2.yaml'
            module.make_config(ROOT / 'integrations/robot/full_track_baseline.yaml', output)
            controller = yaml.safe_load(output.read_text(encoding='utf-8'))['controller_server']['ros__parameters']
        follow = controller['FollowPath']
        self.assertEqual(follow['plugin'], 'nav2_mppi_controller::MPPIController')
        self.assertEqual(controller['controller_plugins'], ['FollowPath'])
        self.assertEqual(controller['controller_frequency'], 20.0)
        self.assertEqual(controller['odom_topic'], '/local_ekf/odom')
        self.assertIn('general_goal_checker', controller['goal_checker_plugins'])
        # Everything except the documented simulator overrides is the robot's.
        expected = copy.deepcopy(module.REFERENCE_MPPI)
        expected['vx_max'] = 0.50
        expected['CostCritic']['consider_footprint'] = True
        self.assertEqual(follow, expected)
        self.assertEqual(follow['motion_model'], 'DiffDrive')
        self.assertEqual(follow['critics'], ['ConstraintCritic', 'CostCritic', 'GoalCritic',
                                             'GoalAngleCritic', 'PathAlignCritic',
                                             'PathFollowCritic', 'PathAngleCritic',
                                             'PreferForwardCritic'])
        # The supervisor addresses the same controller and goal checker.
        node = (ROOT / 'integrations/ros_humble_fortress/self_drive_controller.py').read_text(encoding='utf-8')
        self.assertIn("CONTROLLER_ID = 'FollowPath'", node)
        self.assertIn("GOAL_CHECKER_ID = 'general_goal_checker'", node)
        self.assertNotIn('ManeuverPlanner', node)

    @unittest.skipUnless(importlib.util.find_spec('yaml'), 'requires PyYAML')
    def test_self_drive_requires_matching_ros_v2_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            manifest = folder / 'workspace.json'
            output = folder / 'nav2.yaml'
            script = ROOT / 'integrations/ros_humble_fortress/prepare_self_drive_nav2.py'
            command = [sys.executable, str(script), '--base',
                       str(ROOT / 'integrations/robot/full_track_baseline.yaml'),
                       '--workspace-manifest', str(manifest), '--output', str(output)]
            manifest.write_text(json.dumps({'input_sha256': {}}), encoding='utf-8')
            stale = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(stale.returncode, 0)
            self.assertIn('run setup.cmd', stale.stderr)
            self.assertFalse(output.exists())
            required = ('integrations/robot/classified_lane_costmap.patch',
                        'integrations/autonav_sim/crossable_dashed_tape.patch')
            hashes = {'patch:' + path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
                      for path in required}
            manifest.write_text(json.dumps({'input_sha256': hashes}), encoding='utf-8')
            good = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(good.returncode, 0, good.stderr)
            self.assertTrue(output.is_file())


if __name__ == '__main__':
    unittest.main()
