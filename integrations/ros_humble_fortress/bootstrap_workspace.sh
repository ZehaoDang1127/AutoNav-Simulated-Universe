#!/usr/bin/env bash
# Run inside autonav-humble-fortress:20260928 with /host bound to this repository
# and a persistent /ws volume. Leaves an existing checkout untouched on a SHA
# mismatch rather than resetting local work.
set -euo pipefail

HOST_REPO="${HOST_REPO:-/host}"
ROS_WS="${ROS_WS:-/ws}"
SIM_SHA=46b8e9aa953160b2d317fcc811e209fed46a52f4
ROBOT_SHA=00875b391b9ea897681893ec063c99a3d9d32835
NAV2_SHA=a097086719c88f781aa59788eca29ac6ca5e56db
SIM="$ROS_WS/src/autonav_sim"
ROBOT="$ROS_WS/src/AutoNav_25-26"
NAV2="$ROS_WS/upstream_navigation2"

clone_pinned() {
  local url="$1" path="$2" sha="$3" sparse="${4:-}"
  if [[ ! -d "$path/.git" ]]; then
    if [[ -e "$path" ]]; then
      echo "Existing non-Git path blocks checkout: $path" >&2
      exit 2
    fi
    if [[ -n "$sparse" ]]; then
      git clone --filter=blob:none --sparse "$url" "$path"
    else
      git clone "$url" "$path"
    fi
    git -C "$path" checkout --detach "$sha"
    if [[ -n "$sparse" ]]; then
      git -C "$path" sparse-checkout set "$sparse"
    fi
  fi
  local actual
  actual="$(git -C "$path" rev-parse HEAD)"
  if [[ "$actual" != "$sha" ]]; then
    echo "Pinned revision mismatch at $path: expected $sha, got $actual" >&2
    exit 2
  fi
}

apply_once() {
  local repo="$1" patch="$2"
  if git -C "$repo" apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "Already applied: $patch"
    return
  fi
  git -C "$repo" apply --check "$patch"
  git -C "$repo" apply "$patch"
}

mkdir -p "$ROS_WS/src"
clone_pinned https://github.com/blobspire/autonav_sim.git "$SIM" "$SIM_SHA"
clone_pinned https://github.com/KazakhStallion/AutoNav_25-26.git "$ROBOT" "$ROBOT_SHA"
clone_pinned https://github.com/ros-navigation/navigation2.git "$NAV2" "$NAV2_SHA" nav2_smac_planner
git -C "$NAV2" sparse-checkout add nav2_mppi_controller
git -C "$NAV2" sparse-checkout add nav2_velocity_smoother
git -C "$NAV2" sparse-checkout add nav2_controller
# Return-route memory extends the planner's snapshot overlay. Peel only that
# exact extension before checking its parents, then restore it below.
if git -C "$NAV2" apply --reverse --check "$HOST_REPO/integrations/nav2_smac_planner/traveled_corridor.patch" >/dev/null 2>&1; then
  git -C "$NAV2" apply --reverse "$HOST_REPO/integrations/nav2_smac_planner/traveled_corridor.patch"
fi
if git -C "$NAV2" apply --reverse --check "$HOST_REPO/integrations/nav2_smac_planner/filled_query_shortcut.patch" >/dev/null 2>&1; then
  git -C "$NAV2" apply --reverse "$HOST_REPO/integrations/nav2_smac_planner/filled_query_shortcut.patch"
fi
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_smac_planner/continuous_footprint.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_smac_planner/filled_query_shortcut.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_smac_planner/traveled_corridor.patch"
# Final validation extends the calibrated model header; peel it before
# checking the complete parent overlay, then restore it below.
if git -C "$NAV2" apply --reverse --check "$HOST_REPO/integrations/nav2_mppi_controller/final_control_validation.patch" >/dev/null 2>&1; then
  git -C "$NAV2" apply --reverse "$HOST_REPO/integrations/nav2_mppi_controller/final_control_validation.patch"
fi
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_mppi_controller/calibrated_response.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_mppi_controller/final_control_validation.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_mppi_controller/lattice_maneuvers.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_velocity_smoother/ros_clock.patch"
apply_once "$NAV2" "$HOST_REPO/integrations/nav2_controller/ros_clock.patch"

# The native IMU overlay extends launch functions added by earlier patches.
# Timestamped physical poses extend the native sensor launch and module.
if git -C "$SIM" apply --reverse --check "$HOST_REPO/integrations/autonav_sim/obstacle_volume.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply --reverse "$HOST_REPO/integrations/autonav_sim/obstacle_volume.patch"
fi
if git -C "$SIM" apply --reverse --check "$HOST_REPO/integrations/autonav_sim/physics_pose_timestamp.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply --reverse "$HOST_REPO/integrations/autonav_sim/physics_pose_timestamp.patch"
fi
# Sensor snapshots extend the gyro harness; peel them off before checking it.
if git -C "$SIM" apply --reverse --check "$HOST_REPO/integrations/autonav_sim/sensor_pose_snapshot.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply --reverse "$HOST_REPO/integrations/autonav_sim/sensor_pose_snapshot.patch"
fi
# Remove only that exact overlay before checking them, then reapply it below.
if git -C "$SIM" apply --reverse --check "$HOST_REPO/integrations/autonav_sim/physics_imu.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply --reverse "$HOST_REPO/integrations/autonav_sim/physics_imu.patch"
fi
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/validator_robot_profile.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/runner_course_world.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/mission_runner_fail_safe.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/compact_waypoints.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/ramp_edge_lidar.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/ramp_edge_referee.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/ordered_finish.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/verified_spawn.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/sensor_tape_index.patch"
# Upgrade an earlier timer-derived gyro without changing pinned checkouts.
if git -C "$SIM" apply --check "$HOST_REPO/integrations/autonav_sim/imu_sample_timing.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply "$HOST_REPO/integrations/autonav_sim/imu_sample_timing.patch"
fi
if git -C "$SIM" apply --check "$HOST_REPO/integrations/autonav_sim/imu_heartbeat_resume.patch" >/dev/null 2>&1; then
  git -C "$SIM" apply "$HOST_REPO/integrations/autonav_sim/imu_heartbeat_resume.patch"
fi
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/imu_local_ekf.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/gps_global_ekf.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/gps_goal_projection.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/unknown_map_canvas.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/physics_imu.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/lidar_scan_origin.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/sensor_pose_snapshot.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/physics_pose_timestamp.patch"
apply_once "$SIM" "$HOST_REPO/integrations/autonav_sim/obstacle_volume.patch"
# Peel the full-source snapshot extension before checking older mirror patches.
if git -C "$ROBOT" apply --reverse --check "$HOST_REPO/integrations/robot/mirror_source_snapshot.patch" >/dev/null 2>&1; then
  git -C "$ROBOT" apply --reverse "$HOST_REPO/integrations/robot/mirror_source_snapshot.patch"
fi
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/detection_sim_time.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/gps_turn_heading.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/gps_turn_heading_test.patch"
# Upgrade the earlier soft-clear patch in existing volumes. A clean pinned
# checkout receives the complete correction through mirror_soft_clear below.
if git -C "$ROBOT" apply --check "$HOST_REPO/integrations/robot/mirror_obstacle_updates.patch" >/dev/null 2>&1; then
  git -C "$ROBOT" apply "$HOST_REPO/integrations/robot/mirror_obstacle_updates.patch"
fi
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_soft_clear.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/path_footprint_semantics.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/gps_single_dispatch.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/path_validation_handoff.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/route_commitment.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_line_updates.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_max_merge.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/gps_navsat_projection.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/gps_arrival_reference.patch"
# Upgrade the initial FREE-memory experiment with Humble's finite search cost.
if git -C "$ROBOT" apply --check "$HOST_REPO/integrations/robot/mirror_unknown_cost.patch" >/dev/null 2>&1; then
  git -C "$ROBOT" apply "$HOST_REPO/integrations/robot/mirror_unknown_cost.patch"
fi
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_free_space.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_sensor_frame.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/survey_once.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/global_line_unknown.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_rolling_origin.patch"
apply_once "$ROBOT" "$HOST_REPO/integrations/robot/mirror_source_snapshot.patch"

set +u
source /opt/ros/humble/setup.bash
set -u
cd "$ROS_WS"
colcon build --symlink-install \
  --base-paths "$SIM/igvc_competition_sim" "$ROBOT/isaac_ros-dev/src" "$NAV2/nav2_smac_planner" \
    "$NAV2/nav2_mppi_controller" "$NAV2/nav2_velocity_smoother" "$NAV2/nav2_controller" "$HOST_REPO/integrations/mppi_filled_cost_critic" \
  --packages-select \
    autonav_interfaces autonav_detection autonav_hybrid_planner \
    bringup custom_behavior_tree_plugins gps_handler gps_waypoint_handler \
    igvc_competition_sim imu_cov_inflator line_layer local_mirror_layer \
    map_padder odom_handler slam nav2_smac_planner nav2_mppi_controller nav2_velocity_smoother nav2_controller autonav_filled_cost_critic \
  --cmake-args -Unav2_mppi_controller_DIR -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

python3 "$HOST_REPO/integrations/ros_humble_fortress/write_workspace_manifest.py" \
  --ros-ws "$ROS_WS" --host-repo "$HOST_REPO" \
  --output "$ROS_WS/full_track_workspace_manifest.json"
mkdir -p "$HOST_REPO/runs"
cp "$ROS_WS/full_track_workspace_manifest.json" \
  "$HOST_REPO/runs/full_track_workspace_manifest.json"

echo "Pinned Humble/Fortress workspace built at $ROS_WS"
