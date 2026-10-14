#!/usr/bin/env bash
# Diagnostic compact-course run. The local Blender asset is not release ready.
# Run in the pinned container with /host bound to this repo and /ws to the
# persistent colcon volume after bootstrap_workspace.sh succeeds.
set -euo pipefail

HOST_REPO="${HOST_REPO:-/host}"
ROS_WS="${ROS_WS:-/ws}"
RUN_DIR="${RUN_DIR:-$HOST_REPO/runs/compact-ros-smoke}"
MISSION_TIMEOUT_SEC="${MISSION_TIMEOUT_SEC:-90}"
mkdir -p "$RUN_DIR"
# Snapshot the service-reply discovery window with the run. Ordinary topic
# QoS keeps the pinned middleware defaults; no XML memory/mode override.
cp "$HOST_REPO/integrations/ros_humble_fortress/fastdds_services.xml" "$RUN_DIR/fastdds_services.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUN_DIR/fastdds_services.xml"

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

SIM_SHARE="$(ros2 pkg prefix igvc_competition_sim)/share/igvc_competition_sim"
COURSE="${COURSE_CONFIG_OVERRIDE:-$SIM_SHARE/config/igvc_competition_compact.yaml}"
MISSION_COURSE="${MISSION_COURSE_OVERRIDE:-$COURSE}"
WORLD="${WORLD_OVERRIDE:-$SIM_SHARE/worlds/igvc_competition_compact.sdf}"
NAV2_PARAMS="${NAV2_PARAMS_OVERRIDE:-$HOST_REPO/integrations/robot/full_track_baseline.yaml}"
BT_XML="${BT_XML_OVERRIDE:-$HOST_REPO/integrations/robot/full_track_bt.xml}"
DYNAMICS="${DYNAMICS_OVERRIDE:-$HOST_REPO/integrations/ros_humble_fortress/full_track_dynamics.yaml}"
sha256sum "$COURSE" "$WORLD" "$NAV2_PARAMS" "$BT_XML" "$DYNAMICS" "$FASTRTPS_DEFAULT_PROFILES_FILE" \
  > "$RUN_DIR/input_hashes.sha256"
if [[ "$MISSION_COURSE" != "$COURSE" ]]; then
  sha256sum "$MISSION_COURSE" >> "$RUN_DIR/input_hashes.sha256"
fi
if [[ -f "$ROS_WS/full_track_workspace_manifest.json" ]]; then
  sha256sum "$ROS_WS/full_track_workspace_manifest.json" \
    >> "$RUN_DIR/input_hashes.sha256"
  cp "$ROS_WS/full_track_workspace_manifest.json" \
    "$RUN_DIR/workspace_manifest.json"
fi

cleanup() {
  set +e
  if [[ -n "${bag_pid:-}" ]]; then
    kill -INT -- "-$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "${gate_pid:-}" ]]; then
    kill -INT "$gate_pid" 2>/dev/null || true
    wait "$gate_pid" 2>/dev/null || true
  fi
  if [[ -n "${viewer_pid:-}" ]]; then
    kill -INT "$viewer_pid" 2>/dev/null || true
    wait "$viewer_pid" 2>/dev/null || true
  fi
  if [[ -n "${probe_pid:-}" ]]; then
    kill -INT "$probe_pid" 2>/dev/null || true
    wait "$probe_pid" 2>/dev/null || true
  fi
  if [[ -n "${stack_pid:-}" ]]; then
    kill -INT -- "-$stack_pid" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-$stack_pid" 2>/dev/null || true
    wait "$stack_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 "$HOST_REPO/integrations/ros_humble_fortress/command_gate.py" \
  > "$RUN_DIR/command_gate.log" 2>&1 &
gate_pid=$!

if [[ -n "${SPECTATOR_PORT:-}" ]]; then
  python3 "$HOST_REPO/integrations/ros_humble_fortress/compact_viewer.py" \
    --course-config "$COURSE" --port "$SPECTATOR_PORT" \
    --result-file "$RUN_DIR/result.json" \
    --replay-output "$RUN_DIR/replay.html" \
    > "$RUN_DIR/viewer.log" 2>&1 &
  viewer_pid=$!
fi

if [[ "${RECORD_COSTMAP_PROBE:-false}" == "true" ]]; then
  python3 "$HOST_REPO/integrations/ros_humble_fortress/costmap_probe.py" \
    > "$RUN_DIR/costmap_probe.jsonl" 2> "$RUN_DIR/costmap_probe.err" &
  probe_pid=$!
fi

setsid ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE" \
  world:="$WORLD" \
  line_detection_mode:=lidar \
  nav2_params:="$NAV2_PARAMS" \
  bt_xml:="$BT_XML" \
  dynamics_calibration:="$DYNAMICS" \
  gazebo_server_only:=true \
  launch_local_ekf:=true \
  launch_gps_ekf:=true \
  > "$RUN_DIR/stack.log" 2>&1 &
stack_pid=$!

set +e
python3 "$HOST_REPO/integrations/ros_humble_fortress/readiness_probe.py" \
  --line-mode lidar --require-imu --require-gps-odom --timeout-wall-s 120 \
  | tee "$RUN_DIR/readiness.json"
readiness_status="${PIPESTATUS[0]}"
set -e

if [[ "$readiness_status" -eq 0 && "${RECORD_DIAGNOSTICS:-false}" == "true" ]]; then
  setsid ros2 bag record -o "$RUN_DIR/bag" \
    /clock /tf /gps_fix /goal_pose /goal_update /plan /unsmoothed_plan \
    /odom /imu/data /local_ekf/odom /global_ekf/odom /odometry/gps /igvc_sim/ground_truth_odom \
    /igvc_sim/physics_pose /igvc_sim/physics_odom \
    /world/igvc_competition/dynamic_pose/info \
    /global_costmap/costmap_raw /local_costmap/costmap_raw \
    /local_costmap/costmap /scan_pca_filtered /scan_pca_filtered_clear \
    /scan_pca_filtered_points /scan_pca_obstacle_volume_points /igvc_sim/physics_imu \
    /lidar_line_costmap /line_costmap /gps_waypoint/debug \
    /gps_waypoint/heading_offset /gps_waypoint/heading_offset_std_deg \
    /cmd_vel_nav /cmd_vel /cmd_vel_supervised /igvc_sim/score /igvc_sim/fail \
    > "$RUN_DIR/bag.log" 2>&1 &
  bag_pid=$!
fi

set +e
if [[ "$readiness_status" -eq 0 ]]; then
hard_timeout_sec=$((MISSION_TIMEOUT_SEC + 20))
timeout --signal=TERM --kill-after=8s "${hard_timeout_sec}s" \
  ros2 run igvc_competition_sim igvc_mission_runner \
    --course-config "$MISSION_COURSE" \
    --timeout-sec "$MISSION_TIMEOUT_SEC" \
    --total-timeout-sec "$MISSION_TIMEOUT_SEC" \
    --require-command-gate \
  2>&1 | tee "$RUN_DIR/mission.log"
mission_status="${PIPESTATUS[0]}"
else
  echo "Mission skipped: startup readiness failed." | tee "$RUN_DIR/mission.log"
  mission_status=2
fi
set -e

set +e
python3 "$HOST_REPO/integrations/ros_humble_fortress/postrun_gate_probe.py" \
  | tee "$RUN_DIR/postrun_gate.json"
gate_status="${PIPESTATUS[0]}"
set -e

timeout 8s ros2 topic echo /igvc_sim/score std_msgs/msg/String \
  --once --full-length --field data \
  > "$RUN_DIR/final_score.txt" 2>&1 || true
echo "$mission_status" > "$RUN_DIR/mission_exit_code.txt"
set +e
python3 "$HOST_REPO/integrations/ros_humble_fortress/run_summary.py" \
  --score "$RUN_DIR/final_score.txt" \
  --gate "$RUN_DIR/postrun_gate.json" \
  --mission-log "$RUN_DIR/mission.log" \
  --mission-exit "$mission_status" \
  --readiness "$RUN_DIR/readiness.json" \
  | tee "$RUN_DIR/result.json"
summary_status="${PIPESTATUS[0]}"
set -e
if [[ "$gate_status" -ne 0 ]]; then
  echo "The post-run command gate did not publish verified zero commands." >&2
  exit 3
fi
exit "$summary_status"
