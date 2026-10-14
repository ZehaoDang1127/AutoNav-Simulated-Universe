"""Check clean patch application, repeated setup detection, and volume upgrades.

Run in the pinned Linux container with this repository at /host and the
bootstrapped upstream checkouts at /ws/src. Temporary checkouts are discarded;
the source workspaces are inspected without modification.
"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--host-repo', type=Path, default=Path('/host'))
parser.add_argument('--sim-source', type=Path, default=Path('/ws/src/autonav_sim'))
parser.add_argument('--robot-source', type=Path, default=Path('/ws/src/AutoNav_25-26'))
parser.add_argument('--nav2-source', type=Path, default=Path('/ws/upstream_navigation2'))
parser.add_argument('--clean-only', action='store_true', help='Skip installed-workspace reverse checks')
args = parser.parse_args()
host = args.host_repo.resolve()
bootstrap = (host / 'integrations/ros_humble_fortress/bootstrap_workspace.sh').read_text(encoding='utf-8')
patches = re.findall(r'apply_once "\$(SIM|ROBOT|NAV2)" "\$HOST_REPO/([^"]+)"', bootstrap)
sources = {'SIM': args.sim_source.resolve(), 'ROBOT': args.robot_source.resolve(), 'NAV2': args.nav2_source.resolve()}


def git(directory, *args):
    return subprocess.run(['git', '-C', str(directory), *args], check=True, capture_output=True)


def verify_repeat(checkout, selected):
    overlays = [host / 'integrations/robot/gps_arrival_reference.patch',
                host / 'integrations/autonav_sim/ordered_finish.patch',
                host / 'integrations/nav2_smac_planner/traveled_corridor.patch',
                host / 'integrations/nav2_smac_planner/filled_query_shortcut.patch',
                host / 'integrations/nav2_mppi_controller/final_control_validation.patch',
                host / 'integrations/robot/mirror_source_snapshot.patch',
                host / 'integrations/autonav_sim/obstacle_volume.patch',
                host / 'integrations/autonav_sim/physics_pose_timestamp.patch',
                host / 'integrations/autonav_sim/sensor_pose_snapshot.patch',
                host / 'integrations/autonav_sim/physics_imu.patch']
    for overlay in overlays:
        if overlay in selected:
            git(checkout, 'apply', '--reverse', '--check', str(overlay))
            git(checkout, 'apply', '--reverse', str(overlay))
    for patch in selected:
        if patch not in overlays:
            git(checkout, 'apply', '--reverse', '--check', str(patch))


for kind, source in sources.items():
    selected = [host / patch for target, patch in patches if target == kind]
    with tempfile.TemporaryDirectory(prefix='full-fix-patches-') as temporary:
        checkout = Path(temporary)
        git(checkout, 'init', '-q')
        touched = set()
        created = set()
        for patch in selected:
            patch_text = patch.read_text(encoding='utf-8')
            touched.update(re.findall(r'^--- a/(.+)$', patch_text, re.MULTILINE))
            created.update(re.findall(r'^--- /dev/null\n\+\+\+ b/(.+)$', patch_text, re.MULTILINE))
        # An earlier overlay can create a file extended by a later overlay.
        for relative in touched - created:
            path = checkout / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(git(source, 'show', 'HEAD:' + relative).stdout)
        for patch in selected:
            git(checkout, 'apply', '--check', str(patch))
            git(checkout, 'apply', str(patch))
        verify_repeat(checkout, selected)
        if not args.clean_only:
            with tempfile.TemporaryDirectory(prefix='installed-fix-patches-') as installed_dir:
                installed = Path(installed_dir)
                git(installed, 'init', '-q')
                for patch in selected:
                    for relative in re.findall(r'^\+\+\+ b/(.+)$', patch.read_text(encoding='utf-8'), re.MULTILINE):
                        path = installed / relative
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_bytes((source / relative).read_bytes())
                verify_repeat(installed, selected)
        print(f'{kind}: {len(selected)} canonical patches apply and repeat setup detects all')
        if kind == 'SIM':
            heartbeat_upgrade = host / 'integrations/autonav_sim/imu_heartbeat_resume.patch'
            git(checkout, 'apply', '--reverse', str(heartbeat_upgrade))
            migration = host / 'integrations/autonav_sim/imu_sample_timing.patch'
            git(checkout, 'apply', '--reverse', str(migration))
            git(checkout, 'apply', '--check', str(migration))
            git(checkout, 'apply', str(migration))
            git(checkout, 'apply', '--check', str(heartbeat_upgrade))
            git(checkout, 'apply', str(heartbeat_upgrade))
            git(checkout, 'apply', '--reverse', '--check', str(host / 'integrations/autonav_sim/imu_local_ekf.patch'))
            print('SIM: earlier gyro-timing and heartbeat migrations also verified')
        if kind == 'ROBOT':
            migration = host / 'integrations/robot/mirror_obstacle_updates.patch'
            git(checkout, 'apply', '--reverse', str(migration))
            git(checkout, 'apply', '--check', str(migration))
            git(checkout, 'apply', str(migration))
            git(checkout, 'apply', '--reverse', '--check', str(host / 'integrations/robot/mirror_soft_clear.patch'))
            print('ROBOT: earlier-volume migration also verified')
            migration = host / 'integrations/robot/mirror_unknown_cost.patch'
            git(checkout, 'apply', '--reverse', str(migration))
            git(checkout, 'apply', '--check', str(migration))
            git(checkout, 'apply', str(migration))
            git(checkout, 'apply', '--reverse', '--check', str(host / 'integrations/robot/mirror_free_space.patch'))
            print('ROBOT: initial FREE-memory upgrade also verified')
