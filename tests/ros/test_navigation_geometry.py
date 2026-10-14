"""Drive-frame planning must preserve the physical body's offset geometry."""
import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


class NavigationGeometryTests(unittest.TestCase):
    def test_controller_planner_and_safety_gate_share_the_drive_body(self):
        root = Path('/host')
        profile = yaml.safe_load(Path('/ws/src/autonav_sim/igvc_competition_sim/profiles/shogi/profile.yaml').read_text())
        geometry = profile['geometry']
        offset = geometry['base_link_to_nav_center_m']
        half_length = geometry['physical_half_length_m']
        half_width = geometry['physical_half_width_m']
        expected = [[offset+half_length, half_width], [offset+half_length, -half_width],
                    [offset-half_length, -half_width], [offset-half_length, half_width]]
        params = yaml.safe_load((root / 'integrations/robot/full_track_baseline.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            costmap = params[name][name]['ros__parameters']
            self.assertEqual(costmap['robot_base_frame'], 'base_link')
            self.assertEqual(costmap['global_frame'], 'odom')
            points = ast.literal_eval(costmap['footprint'])
            for point, physical in zip(points, expected):
                for actual, desired in zip(point, physical):
                    self.assertAlmostEqual(actual, desired)
            padding = costmap['footprint_padding']
            self.assertGreaterEqual(padding, geometry['footprint_padding_m'])
            radius = max(math.hypot(x+math.copysign(padding, x-offset),
                                    y+math.copysign(padding, y)) for x, y in points)
            self.assertGreaterEqual(costmap['inflation_layer']['inflation_radius'], radius)
            self.assertLess(2*(half_width+padding), 1.524)
        self.assertEqual(params['bt_navigator']['ros__parameters']['robot_base_frame'], 'base_link')
        self.assertEqual(params['bt_navigator']['ros__parameters']['global_frame'], 'odom')
        self.assertEqual(params['controller_server']['ros__parameters']['odom_topic'],
                         params['bt_navigator']['ros__parameters']['odom_topic'])
        self.assertEqual(params['controller_server']['ros__parameters']['odom_topic'], '/local_ekf/odom')
        self.assertEqual(params['behavior_server']['ros__parameters']['robot_base_frame'], 'base_link')
        self.assertFalse(params['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['footprint_clearing_enabled'])
        self.assertFalse(params['global_costmap']['global_costmap']['ros__parameters']['far_obstacle_layer']['footprint_clearing_enabled'])
        for name,layer in [('local_costmap','obstacle_layer'),('global_costmap','far_obstacle_layer')]:
            marking=params[name][name]['ros__parameters'][layer]['mark_scan']
            self.assertEqual(marking['topic'],'/scan_pca_obstacle_volume_points')
            self.assertEqual(marking['data_type'],'PointCloud2')
        execution_padding = params['local_costmap']['local_costmap']['ros__parameters']['footprint_padding']
        planning_padding = params['global_costmap']['global_costmap']['ros__parameters']['footprint_padding']
        self.assertGreaterEqual(planning_padding, execution_padding)
        self.assertAlmostEqual(planning_padding,execution_padding)
        tree = ET.parse(root / 'integrations/robot/full_track_bt.xml')
        gates = tree.findall('.//PathFootprintSafe')
        self.assertEqual(len(gates), 3)
        for gate in gates:
            self.assertEqual(gate.attrib['robot_base_frame'], 'base_link')
            self.assertEqual(gate.attrib['global_frame'], 'odom')
            self.assertEqual(ast.literal_eval(gate.attrib['footprint']), points)
            self.assertEqual(float(gate.attrib['footprint_padding']), execution_padding)

    def test_active_guard_covers_controller_rollout_without_truncating_progress(self):
        root = Path('/host')
        params = yaml.safe_load((root / 'integrations/robot/full_track_baseline.yaml').read_text())
        controller = params['controller_server']['ros__parameters']['FollowPath']
        rollout = controller['time_steps'] * controller['model_dt'] * controller['vx_max']
        self.assertGreater(controller['prune_distance'], rollout)
        self.assertGreater(controller['prune_distance'], controller['PathFollowCritic']['threshold_to_consider'])
        self.assertFalse(controller['enforce_path_inversion'])
        self.assertTrue(controller['PathAngleCritic']['consider_lattice_rotations'])
        self.assertAlmostEqual(controller['PathAngleCritic']['lattice_rotation_min_angle'],0.30)
        self.assertTrue(params['planner_server']['ros__parameters']['GridBased']['allow_goal_heading_change'])
        self.assertGreaterEqual(params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],
                                2*math.pi)
        gates = ET.parse(root / 'integrations/robot/full_track_bt.xml').findall('.//PathFootprintSafe')
        self.assertNotIn('validation_distance_m', gates[0].attrib)
        for gate in gates[1:]:
            self.assertGreaterEqual(float(gate.attrib['validation_distance_m']), rollout)

    def test_controller_receives_global_hazards_without_occupied_feedback(self):
        root = Path('/host')
        params = yaml.safe_load((root / 'integrations/robot/full_track_baseline.yaml').read_text())
        local = params['local_costmap']['local_costmap']['ros__parameters']
        global_map = params['global_costmap']['global_costmap']['ros__parameters']
        memory = local['global_memory_layer']
        self.assertIn('global_memory_layer', local['plugins'])
        self.assertLess(local['plugins'].index('global_memory_layer'), local['plugins'].index('inflation_layer'))
        self.assertEqual(memory['source_topic'], '/global_costmap/costmap')
        self.assertEqual(memory['min_occupied_value_to_mirror'], 100)
        self.assertTrue(memory['allow_decrease'])
        self.assertFalse(memory['decrease_only_in_front'])
        self.assertFalse(memory['retain_source_history'])
        for name in global_map['plugins']:
            self.assertNotEqual(global_map[name].get('source_topic'), '/local_costmap/costmap')
        exploration = global_map['exploration_layer']
        self.assertEqual(exploration['source_topic'], '/map_padded')
        self.assertFalse(exploration['mirror_free_space'])
        self.assertEqual(exploration['unknown_space_cost'], 200)

    def test_prediction_uses_the_calibrated_vehicle_response(self):
        root = Path('/host')
        params = yaml.safe_load((root / 'integrations/robot/full_track_baseline.yaml').read_text())
        controller = params['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(controller['motion_model'], 'CalibratedDiffDrive')
        smoother=params['velocity_smoother']['ros__parameters']
        self.assertEqual(controller['command_feedback_topic'],'/cmd_vel_supervised')
        self.assertEqual(controller['command_smoothing_frequency'],smoother['smoothing_frequency'])
        dynamics = yaml.safe_load((root / 'integrations/ros_humble_fortress/full_track_dynamics.yaml').read_text())['dynamics_model']
        for name, value in controller['CalibratedDiffDrive'].items():
            self.assertEqual(value, dynamics[name])
        self.assertEqual(controller['vx_max'], dynamics['max_linear_speed_mps'])
        self.assertEqual(controller['wz_max'], dynamics['max_angular_speed_radps'])
        smoother = params['velocity_smoother']['ros__parameters']
        self.assertEqual(-smoother['max_decel'][0], dynamics['max_linear_acceleration_mps2'])
        self.assertEqual(-smoother['max_decel'][2], dynamics['max_angular_acceleration_radps2'])


if __name__ == '__main__':
    unittest.main()
