#!/usr/bin/env bash
# Test the exported local Course.blend scene with its matching Fortress world.
set -euo pipefail
HOST_REPO="${HOST_REPO:-/host}"
COURSE_BUNDLE="${COURSE_BUNDLE:-$HOST_REPO/runs/full-track-integrated-lanes}"
export COURSE_CONFIG_OVERRIDE="$COURSE_BUNDLE/course.yaml"
export WORLD_OVERRIDE="$COURSE_BUNDLE/world.sdf"
export RUN_DIR="${RUN_DIR:-$HOST_REPO/runs/full-track-ros-test}"
export MISSION_TIMEOUT_SEC="${MISSION_TIMEOUT_SEC:-900}"
export RECORD_DIAGNOSTICS="${RECORD_DIAGNOSTICS:-true}"
export RECORD_COSTMAP_PROBE="${RECORD_COSTMAP_PROBE:-true}"
if [[ -e "$RUN_DIR" ]]; then
  # The desktop owns a unique directory before Docker starts, so its heartbeat
  # is already available to the fail-closed gate during ROS startup.
  if [[ "${DESKTOP_CONTROL:-false}" != "true" || ! -f "$RUN_DIR/desktop_session.json" ]] || \
     find "$RUN_DIR" -mindepth 1 -maxdepth 1 \
       ! -name desktop_control.json ! -name desktop_control.json.tmp \
       ! -name desktop_session.json -print -quit | grep -q .; then
    echo "Output directory already exists: $RUN_DIR; select a fresh RUN_DIR." >&2
    exit 2
  fi
fi
python3 "$HOST_REPO/integrations/ros_humble_fortress/validate_course_bundle.py" \
  --bundle "$COURSE_BUNDLE" --source "$HOST_REPO/Mock_Course_Asset/Course.blend"
mkdir -p "$RUN_DIR"
cp "$COURSE_BUNDLE/course_manifest.json" "$RUN_DIR/course_manifest.json"
cp "$COURSE_CONFIG_OVERRIDE" "$RUN_DIR/course.yaml"
cp "$WORLD_OVERRIDE" "$RUN_DIR/world.sdf"
export COURSE_CONFIG_OVERRIDE="$RUN_DIR/course.yaml"
export WORLD_OVERRIDE="$RUN_DIR/world.sdf"
# Bash reads a running script incrementally. Keep the launch script and
# navigation inputs immutable even when integration files are edited later.
cp "$HOST_REPO/integrations/ros_humble_fortress/run_compact_smoke.sh" "$RUN_DIR/runner.sh"
cp "${LANE_SEMANTICS_FILE:-$HOST_REPO/integrations/robot/full_track_lane_semantics.yaml}" "$RUN_DIR/lane_semantics.yaml"
python3 "$HOST_REPO/integrations/ros_humble_fortress/prepare_lane_costmap.py" \
  --base "${NAV2_PARAMS_OVERRIDE:-$HOST_REPO/integrations/robot/full_track_baseline.yaml}" \
  --course "$COURSE_CONFIG_OVERRIDE" --semantics "$RUN_DIR/lane_semantics.yaml" \
  --version "${COSTMAP_VERSION:-2}" --lane-change-cost "${LANE_CHANGE_COST_M:-0.25}" \
  --workspace-manifest "${ROS_WS:-/ws}/full_track_workspace_manifest.json" \
  --output "$RUN_DIR/nav2_params.yaml" --metadata "$RUN_DIR/costmap_config.json"
cp "$HOST_REPO/integrations/robot/full_track_bt.xml" "$RUN_DIR/behavior_tree.xml"
cp "$HOST_REPO/integrations/ros_humble_fortress/full_track_dynamics.yaml" "$RUN_DIR/dynamics.yaml"
export NAV2_PARAMS_OVERRIDE="$RUN_DIR/nav2_params.yaml"
export BT_XML_OVERRIDE="$RUN_DIR/behavior_tree.xml"
export DYNAMICS_OVERRIDE="$RUN_DIR/dynamics.yaml"
sha256sum "$RUN_DIR/runner.sh" > "$RUN_DIR/runner_hash.sha256"
exec bash "$RUN_DIR/runner.sh"
