#!/usr/bin/env bash
# Run in the pinned image after bootstrap_workspace.sh.
set -euo pipefail
HOST_REPO="${HOST_REPO:-/host}"
ROS_WS="${ROS_WS:-/ws}"
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
python3 "$HOST_REPO/tests/ros/verify_integration_patches.py"
cmake -S "$HOST_REPO/tests/ros" -B "$ROS_WS/navigation_regressions" \
  -Unav2_mppi_controller_DIR \
  -DROBOT_SOURCE="$ROS_WS/src/AutoNav_25-26" \
  -DMIRROR_LIBRARY="$ROS_WS/install/local_mirror_layer/lib/liblocal_mirror_layer_core.so" \
  -DPATH_SAFETY_PLUGIN="$ROS_WS/install/custom_behavior_tree_plugins/lib/libautonav_path_footprint_safe_bt_node.so"
cmake --build "$ROS_WS/navigation_regressions" -j2
ctest --test-dir "$ROS_WS/navigation_regressions" --output-on-failure
cd "$HOST_REPO"
PYTHONPATH="$HOST_REPO/src:${PYTHONPATH:-}" python3 -m unittest discover -s tests -v
