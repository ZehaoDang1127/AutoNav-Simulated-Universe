"""Build the self-drive profile: ROS v2 clearance costs and reference MPPI.

The local costmap keeps the earlier ROS v2 clearance penalties. The
controller is the AutoNav_25-26 robot's MPPI configuration, which
self_drive_controller.py drives through controller_server's FollowPath.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import yaml


REQUIRED_PATCHES = ('integrations/robot/classified_lane_costmap.patch',
                    'integrations/autonav_sim/crossable_dashed_tape.patch')

# controller_server.FollowPath from KazakhStallion/AutoNav_25-26
# isaac_ros-dev/src/slam/config/nav2_paramsv2.yaml at commit
# 00875b391b9ea897681893ec063c99a3d9d32835 (main, 2026-07-05).
REFERENCE_CONTROLLER_FREQUENCY = 20.0
REFERENCE_MPPI = {
    'plugin': 'nav2_mppi_controller::MPPIController',
    'time_steps': 56, 'model_dt': 0.05, 'batch_size': 1200,
    'vx_std': 0.35, 'vy_std': 0.01, 'wz_std': 0.70,
    'vx_max': 1.00, 'vx_min': 0.04, 'vy_max': 0.0, 'wz_max': 1.0,
    'iteration_count': 1, 'prune_distance': 1.2, 'transform_tolerance': 0.5,
    'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'DiffDrive',
    'visualize': False, 'retry_attempt_limit': 5, 'reset_period': 1.0,
    'regenerate_noises': False,
    'TrajectoryVisualizer': {'trajectory_step': 8, 'time_step': 4},
    'critics': ['ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
                'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic', 'PreferForwardCritic'],
    'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
    'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0,
                   'threshold_to_consider': 1.2},
    'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0,
                        'threshold_to_consider': 0.5},
    'PreferForwardCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 10.0,
                            'threshold_to_consider': 0.5},
    'CostCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 1.6,
                   'critical_cost': 300.0, 'consider_footprint': False,
                   'collision_cost': 1000000.0, 'near_goal_distance': 0.6,
                   'inflation_layer_name': 'inflation_layer'},
    'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 12.0,
                        'max_path_occupancy_ratio': 0.12, 'trajectory_point_step': 4,
                        'threshold_to_consider': 0.5, 'offset_from_furthest': 5,
                        'use_path_orientations': True},
    'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0,
                         'offset_from_furthest': 4, 'threshold_to_consider': 1.2},
    'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0,
                        'offset_from_furthest': 4, 'threshold_to_consider': 0.5,
                        'max_angle_to_furthest': 1.0, 'forward_preference': True},
}
# The only departures from the robot profile, each forced by the simulator:
SIM_MPPI_OVERRIDES = {
    # The simulated drive saturates at 0.50 m/s (full_track_dynamics.yaml);
    # predicting the robot's 1.0 m/s would make every rollout optimistic.
    'vx_max': 0.50,
    # The robot profile has no footprint, so its 0.1 m default radius about
    # nav_center is all CostCritic can test. This local costmap defines the
    # real asymmetric body about the rear axle; without footprint checks the
    # 0.87 m padded nose could cross tape while the axle cell is still clear.
    ('CostCritic', 'consider_footprint'): True,
}


def reference_controller():
    """Return the reference FollowPath block with the simulator overrides."""
    follow = copy.deepcopy(REFERENCE_MPPI)
    for key, value in SIM_MPPI_OVERRIDES.items():
        if isinstance(key, tuple):
            follow[key[0]][key[1]] = value
        else:
            follow[key] = value
    return follow


def validate_workspace(manifest_path: Path, repo: Path):
    """Reject stale binaries that cannot enforce quadratic or crossable tape."""
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    hashes = manifest.get('input_sha256', {})
    for relative in REQUIRED_PATCHES:
        expected = hashlib.sha256((repo / relative).read_bytes()).hexdigest()
        if hashes.get('patch:' + relative) != expected:
            raise ValueError('ROS lane cost map or referee build is missing or stale; run setup.cmd')


class _ParameterDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def make_config(source: Path, output: Path):
    config = yaml.safe_load(source.read_text(encoding='utf-8'))
    local = config['local_costmap']['local_costmap']['ros__parameters']
    local['plugins'] = [name for name in local['plugins'] if name != 'global_memory_layer']
    local.pop('global_memory_layer', None)
    # Match the earlier ROS v2 profile: retain baseline obstacle inflation
    # (1.15 m / 3.2) and use the v2 quadratic clearance reach on both line
    # layers. The detector supplies classified divider observations separately.
    for name in ('line_layer', 'lidar_line_layer'):
        local[name]['quadratic_clearance'] = True
        local[name]['clearance_reach_m'] = 0.8
        local[name]['lane_semantics_frame'] = 'map'
        local[name]['lane_classification_tolerance_m'] = 0.12
        # The line layer otherwise defaults all its untouched cells to free
        # and updateWithMax overwrites unknown in the master grid. A line
        # observation is not evidence that the rest of the world is clear.
        local[name]['preserve_unknown_space'] = True
    # The rolling window is wider, but its unobserved cells are not evidence
    # of free space. Never permit unexplored cells merely because it grew.
    # A 12 m window with 6 m obstacle marking was tried on 2026-11-05: tires
    # were mapped ~1.8 m earlier, but offline laps showed no gentler lane
    # changes and planning time rose 1.5-2x, so 8 m is retained.
    local['width'] = 8
    local['height'] = 8
    local['track_unknown_space'] = True
    # The simulated classified obstacle cloud emits observed returns beyond
    # 4.5 m; allow the larger local window to mark those instead of truncating
    # the existing mark source at 2.5 m. Clearing and footprint checks remain
    # unchanged.
    local['obstacle_layer']['mark_scan']['obstacle_max_range'] = 4.5
    # The standalone controller subscribes to full OccupancyGrid snapshots;
    # it does not reconstruct nav2's incremental costmap_updates topic.
    local['always_send_full_costmap'] = True
    # Drive with the reference robot's MPPI instead of the full-course
    # calibrated/filled-body variant. The progress and goal checkers in the
    # base profile already match the reference values; odom_topic and
    # use_sim_time remain the simulator's.
    controller = config['controller_server']['ros__parameters']
    controller['controller_frequency'] = REFERENCE_CONTROLLER_FREQUENCY
    controller['controller_plugins'] = ['FollowPath']
    controller['FollowPath'] = reference_controller()
    output.write_text(yaml.dump(config, Dumper=_ParameterDumper, sort_keys=False), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workspace-manifest', type=Path, required=True)
    args = parser.parse_args()
    try:
        validate_workspace(args.workspace_manifest, Path(__file__).resolve().parents[2])
        make_config(args.base, args.output)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2, f'Self-drive cost map configuration rejected: {exc}\n')


if __name__ == '__main__':
    main()
