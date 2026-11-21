#!/usr/bin/env bash
# Standalone, waypoint-free track using the pinned ROS / Gazebo workspace.
set -euo pipefail
HOST_REPO="${HOST_REPO:-/host}"
ROS_WS="${ROS_WS:-/ws}"
RUN_DIR="${RUN_DIR:?RUN_DIR is required}"
export PYTHONPATH="$HOST_REPO/src${PYTHONPATH:+:$PYTHONPATH}"
export DESKTOP_CONTROL=true
mkdir -p "$RUN_DIR"
cp "$HOST_REPO/integrations/ros_humble_fortress/fastdds_services.xml" "$RUN_DIR/fastdds_services.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$RUN_DIR/fastdds_services.xml"
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
# SELF_DRIVE_TRACK selects the layout: loop (default) or intersection.
SELF_DRIVE_TRACK="${SELF_DRIVE_TRACK:-loop}"
python3 "$HOST_REPO/tools/generate_self_drive_track.py" --output "$RUN_DIR/track" --layout "$SELF_DRIVE_TRACK"
COURSE="$RUN_DIR/track/course.yaml"
WORLD="$RUN_DIR/track/world.sdf"
# Derive physics, collision geometry, ground and paint from the pinned
# course generator. The sensor wrapper classifies only actual LiDAR returns.
python3 -m igvc_competition_sim.generate_world --course-config "$COURSE" \
  --output "$RUN_DIR/track/base_world.sdf"
python3 "$HOST_REPO/tools/generate_self_drive_track.py" --output "$RUN_DIR/track" \
  --layout "$SELF_DRIVE_TRACK" --base-world "$RUN_DIR/track/base_world.sdf"
python3 "$HOST_REPO/integrations/ros_humble_fortress/prepare_self_drive_nav2.py" \
  --base "$HOST_REPO/integrations/robot/full_track_baseline.yaml" --output "$RUN_DIR/nav2_params.yaml" \
  --workspace-manifest "$ROS_WS/full_track_workspace_manifest.json"
cleanup() {
  set +e
  for pid in "${readiness_pid:-}" "${controller_pid:-}" "${referee_pid:-}" "${divider_pid:-}" "${sensor_pid:-}" "${viewer_pid:-}" "${gate_pid:-}"; do
    if [[ -n "$pid" ]]; then kill -INT "$pid" 2>/dev/null || true; fi
  done
  if [[ -n "${stack_pid:-}" ]]; then
    kill -INT -- "-$stack_pid" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-$stack_pid" 2>/dev/null || true
    wait "$stack_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
python3 "$HOST_REPO/integrations/ros_humble_fortress/command_gate.py" > "$RUN_DIR/command_gate.log" 2>&1 &
gate_pid=$!
python3 "$HOST_REPO/integrations/ros_humble_fortress/compact_viewer.py" \
  --course-config "$COURSE" --port "${SPECTATOR_PORT:-8765}" \
  --result-file "$RUN_DIR/result.json" --replay-output "$RUN_DIR/replay.html" \
  > "$RUN_DIR/viewer.log" 2>&1 &
viewer_pid=$!
python3 "$HOST_REPO/integrations/ros_humble_fortress/self_drive_sensor.py" \
  --ros-args -p use_sim_time:=true -p course_config:="$COURSE" \
  -p ground_truth_pose_topic:=/igvc_sim/physics_pose -p publish_odom_tf:=false \
  -r /imu/data:=/igvc_sim/pose_gyro -r /autonomous_mode:=/igvc_sim/harness_autonomous_mode \
  > "$RUN_DIR/sensor.log" 2>&1 &
sensor_pid=$!
python3 "$HOST_REPO/integrations/ros_humble_fortress/self_drive_divider_node.py" \
  --ros-args -p use_sim_time:=true > "$RUN_DIR/divider.log" 2>&1 &
divider_pid=$!
# A launch wrapper removes the harness's competing AUTO publisher. The gate
# remains the only publisher of /cmd_vel_supervised and /autonomous_mode.
setsid ros2 launch "$HOST_REPO/integrations/ros_humble_fortress/desktop_course.launch.py" \
  course_config:="$COURSE" world:="$WORLD" line_detection_mode:=lidar \
  nav2_params:="$RUN_DIR/nav2_params.yaml" \
  bt_xml:="$HOST_REPO/integrations/robot/full_track_bt.xml" \
  dynamics_calibration:="$HOST_REPO/integrations/ros_humble_fortress/full_track_dynamics.yaml" \
  validate_world_sync:=true launch_monitor:=false launch_harness:=false \
  gazebo_server_only:=true launch_local_ekf:=true launch_gps_ekf:=true \
  > "$RUN_DIR/stack.log" 2>&1 &
stack_pid=$!
python3 "$HOST_REPO/integrations/ros_humble_fortress/readiness_probe.py" \
  --line-mode lidar --require-imu --require-gps-odom --timeout-wall-s 120 \
  > "$RUN_DIR/readiness.json" 2> "$RUN_DIR/readiness.err" &
readiness_pid=$!
while kill -0 "$readiness_pid" 2>/dev/null; do
  for critical_pid in "$gate_pid" "$viewer_pid" "$stack_pid" "$sensor_pid" "$divider_pid"; do
    if ! kill -0 "$critical_pid" 2>/dev/null; then
      echo "Critical ROS track process $critical_pid exited during startup." >&2
      kill -INT "$readiness_pid" 2>/dev/null || true
      wait "$readiness_pid" 2>/dev/null || true
      readiness_pid=
      exit 1
    fi
  done
  sleep 0.5
done
set +e
wait "$readiness_pid"
ready_status=$?
set -e
readiness_pid=
if [[ "$ready_status" -ne 0 ]]; then
  echo 'ROS readiness failed; autonomous controller was not started.' >&2
  exit "$ready_status"
fi
# Keep the command gate disabled until the original referee has established
# a clean initial pose from the stationary ground-truth stream.
python3 "$HOST_REPO/integrations/ros_humble_fortress/self_drive_referee.py" \
  --course-config "$COURSE" --ready-file "$RUN_DIR/referee_ready.json" \
  > "$RUN_DIR/referee.log" 2>&1 &
referee_pid=$!
until [[ -s "$RUN_DIR/referee_ready.json" ]]; do
  for critical_pid in "$gate_pid" "$viewer_pid" "$stack_pid" "$referee_pid" "$sensor_pid" "$divider_pid"; do
    if ! kill -0 "$critical_pid" 2>/dev/null; then
      echo "Critical ROS track process $critical_pid exited during referee startup." >&2
      exit 1
    fi
  done
  sleep 0.1
done
# This mode does not launch a waypoint mission client. Nav2's inactive action
# controllers must not publish velocity commands while this node runs.
python3 "$HOST_REPO/integrations/ros_humble_fortress/self_drive_controller.py" \
  > "$RUN_DIR/controller.log" 2>&1 &
controller_pid=$!
while kill -0 "$controller_pid" 2>/dev/null; do
  for critical_pid in "$gate_pid" "$viewer_pid" "$stack_pid" "$referee_pid" "$sensor_pid" "$divider_pid"; do
    if ! kill -0 "$critical_pid" 2>/dev/null; then
      echo "Critical ROS track process $critical_pid exited; stopping controller." >&2
      kill -INT "$controller_pid" 2>/dev/null || true
      wait "$controller_pid" 2>/dev/null || true
      exit 1
    fi
  done
  sleep 0.5
done
wait "$controller_pid"
