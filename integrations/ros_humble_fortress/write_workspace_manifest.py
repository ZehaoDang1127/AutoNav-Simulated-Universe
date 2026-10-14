"""Record the exact pinned ROS/Gazebo workspace built by bootstrap_workspace."""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import subprocess


def command(*args: str) -> str:
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT).strip()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-ws", type=Path, default=Path("/ws"))
    parser.add_argument("--host-repo", type=Path, default=Path("/host"))
    parser.add_argument("--output", type=Path, default=Path("/ws/full_track_workspace_manifest.json"))
    args = parser.parse_args()

    sim = args.ros_ws / "src/autonav_sim"
    robot = args.ros_ws / "src/AutoNav_25-26"
    nav2 = args.ros_ws / "upstream_navigation2"
    files = {
        "dockerfile": "integrations/ros_humble_fortress/Dockerfile",
        "nav2_profile": "integrations/robot/full_track_baseline.yaml",
        "behavior_tree": "integrations/robot/full_track_bt.xml",
        "dynamics_profile": "integrations/ros_humble_fortress/full_track_dynamics.yaml",
        "service_discovery_profile": "integrations/ros_humble_fortress/fastdds_services.xml",
        "sim_validator_patch": "integrations/autonav_sim/validator_robot_profile.patch",
        "sim_runner_patch": "integrations/autonav_sim/runner_course_world.patch",
        "sim_mission_patch": "integrations/autonav_sim/mission_runner_fail_safe.patch",
        "robot_detection_patch": "integrations/robot/detection_sim_time.patch",
    }
    # Include every applied patch so a run can be tied to the actual fixes,
    # rather than only the pinned upstream revisions and launch profile.
    for directory in ("integrations/robot", "integrations/autonav_sim", "integrations/nav2_smac_planner", "integrations/nav2_mppi_controller", "integrations/nav2_velocity_smoother", "integrations/nav2_controller"):
        for patch in sorted((args.host_repo / directory).glob("*.patch")):
            files[f"patch:{directory}/{patch.name}"] = f"{directory}/{patch.name}"
    for source in sorted((args.host_repo / "integrations/mppi_filled_cost_critic").rglob("*")):
        if source.is_file():
            relative = source.relative_to(args.host_repo).as_posix()
            files[f"source:{relative}"] = relative
    versions = {}
    for package in (
        "ros-humble-nav2-bringup",
        "ros-humble-nav2-mppi-controller",
        "ros-humble-ros-gz-sim",
        "ros-humble-robot-localization",
        "ros-humble-rmw-fastrtps-cpp",
        "ros-humble-rmw-fastrtps-shared-cpp",
        "ros-humble-fastrtps",
    ):
        try:
            versions[package] = command("dpkg-query", "-W", "-f=${Version}", package)
        except subprocess.CalledProcessError:
            versions[package] = None

    manifest = {
        "schema_version": 1,
        "recorded_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "gazebo_version": command("ign", "gazebo", "--version"),
        "simulator_commit": command("git", "-C", str(sim), "rev-parse", "HEAD"),
        "robot_commit": command("git", "-C", str(robot), "rev-parse", "HEAD"),
        "nav2_commit": command("git", "-C", str(nav2), "rev-parse", "HEAD"),
        "smac_library_sha256": sha256(args.ros_ws / "install/nav2_smac_planner/lib/libnav2_smac_planner_lattice.so"),
        "filled_critic_library_sha256": sha256(args.ros_ws / "install/autonav_filled_cost_critic/lib/libautonav_filled_cost_critic.so"),
        "mppi_controller_library_sha256": sha256(args.ros_ws / "install/nav2_mppi_controller/lib/libmppi_controller.so"),
        "velocity_smoother_library_sha256": sha256(args.ros_ws / "install/nav2_velocity_smoother/lib/libvelocity_smoother_core.so"),
        "controller_server_library_sha256": sha256(args.ros_ws / "install/nav2_controller/lib/libcontroller_server_core.so"),
        "simulator_submodules": command("git", "-C", str(sim), "submodule", "status", "--recursive"),
        "robot_submodules": command("git", "-C", str(robot), "submodule", "status", "--recursive"),
        "apt_versions": versions,
        "input_sha256": {
            label: sha256(args.host_repo / relative)
            for label, relative in files.items()
        },
        "built_packages": [
            "autonav_interfaces", "autonav_detection", "autonav_hybrid_planner",
            "bringup", "custom_behavior_tree_plugins", "gps_handler",
            "gps_waypoint_handler", "igvc_competition_sim", "imu_cov_inflator",
            "line_layer", "local_mirror_layer", "map_padder", "odom_handler",
            "slam", "nav2_smac_planner", "nav2_mppi_controller", "nav2_velocity_smoother", "nav2_controller", "autonav_filled_cost_critic",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
