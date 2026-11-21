# Lane-aware cost map in the full-course ROS/Gazebo system

The full-course launcher and Qt workspace now select **v2** by default, with
**v1** available for a paired baseline. Rebuild the pinned workspace with
`integrations/ros_humble_fortress/setup.cmd` before selecting v2. Startup checks
the new patch hashes in the workspace manifest and rejects an old build.
The Python scenario lab and standalone subsystem simulators are unchanged.

The implementation is wired into the ROS sources but has **not yet been
compiled or run in Gazebo**. Docker Desktop's Linux engine did not start during
implementation. Portable native geometry tests, Python configuration/control
tests, and clean/repeated source patch checks pass. The native ROS regression
`lane_traversal` is registered with CTest and awaits the Humble build.

The [October 23 comparison](../deliverables/ros-costmap-comparison-2026-10-23/comparison.md)
documents an earlier configuration; regenerate the comparison for the integrated
course to see its current classification counts. No Python scenario run is presented as ROS performance evidence.

## What v2 changes

The existing Smac lattice planner, filled-footprint collision checking, calibrated
MPPI controller, referee, supervised command gate and physical margins remain.
Local and global camera/LiDAR line layers use a quadratic soft clearance field:

```text
exact tape cell: 254 (lethal)
within the existing narrow inscribed radius: 253
beyond that radius: 252 * (1 - distance_beyond_inscribed / 0.8 m)^2
outside that 0.8 m reach: 0
```

These are Nav2 raster costs, not metres or probabilities. Layers still merge
by maximum; stock Nav2 inflation and obstacle sensing can dominate these values.
The obstacle inflation layer and controller critic weights retain the baseline
settings. This is a ROS adaptation of the lane-aware objective, not a port of
the Python grid robot or its square footprint assumptions.

For explicitly classified divider paint, both local and global line layers omit
the divider from lethal tape marking. All other tape remains hard, and a hard
boundary wins at an intersection. The obstacle/terrain layers remain independent
and cannot be cleared by divider classification. Missing classification TF keeps
the detection hard. Missing planner semantic TF fails planning.

Smac evaluates each motion primitive's sampled polyline against the divider's
single centreline. Each crossing adds `lane_change_cost_m / map_resolution` to
the native traversal cost. The default is **0.25 m equivalent per crossing**;
rotation/reverse/clearance costs retain their baseline meanings. Subdividing a
primitive or changing map resolution does not multiply the change penalty.
Points exactly on a divider use its nonnegative signed side, consistently
between adjacent primitive samples. This deterministic tie convention can
penalize touching and returning from the other side; it is not a lane-confidence
model. The current implementation does not add a separate MPPI lane-change
critic: MPPI tracks the selected path against the consistently classified local
map, and the existing guards validate that path's hard geometry.

With configured dividers, analytic completion and post-search smoothing are
disabled because those stages bypass native edge-transition scoring. This can
increase planning time. The once-per-change objective lives in the lattice
search, rather than charging repeatedly for pixels inside painted tape.

## Current diagnostic course

The exported Blender meshes contain 872 hard tape segments. A diagnostic
crossable dashed divider follows the marked corridor through the existing
obstacle course, including its open right-hand section. The cones remain
obstacles and the original tape remains hard. The source Blender asset is
unchanged. `integrations/robot/full_track_lane_semantics.yaml` classifies the
paint as crossable and supplies the transition centrelines, including gaps in
the paint.

Some of the existing corridor is narrower than two robot widths, and the
right-hand obstacle section has gaps in its outer boundaries. The overlay does
not establish two feasible lanes throughout the course or demonstrate lane
following. Rebuild the pinned workspace using `setup.cmd` to apply the referee
patch and regenerate the bundle as described in the mock course README. A
stale workspace is rejected. Full-course and crossing behavior still require a
ROS/Gazebo run.

## Explicit divider annotations

Author annotations only after assigning a segment's course role. A classification
uses exact `internal_lines[].name` or `dashed_lines[].name` values from the matching exported course. A dashed divider must explicitly set `allow_crossing: true`:

```yaml
schema_version: 1
frame_id: map
course_id: local_mock_course
crossable_line_names:
  - actual-divider-paint-edge-segment-name
divider_centerlines_m:
  - [[1.0, 2.0], [5.0, 2.0]]
classification_tolerance_m: 0.12
```

The names and coordinates above are an **authoring example**, not current
asset annotations. List all paint-edge segments belonging to the divider, then
provide its single transition centreline. Two edges of thick paint must not
become two lane changes. A centreline may be a polyline of multiple points.
Use consistently oriented connected segments and avoid overlapping centrelines.
All unspecified course line segments stay hard. Roles are immutable within a
session; restart with a newly generated profile after editing them.

The generator rejects unknown/duplicate segment names, mismatched course IDs,
nonfinite/zero-length geometry, duplicate transition segments, incompatible
frames, missing matching paint/centreline inputs and excessive classification
tolerance. Crossable annotations currently require explicit Blender boundaries
with generated outer tape disabled. The default asset satisfies that contract.

## Launch and compare

The Qt Session panel exposes the cost-map version and lane penalty. Saved
results show the recorded cost-map version. Historical runs without a recorded
version are marked `Unrecorded`.

```powershell
.\integrations\ros_humble_fortress\setup.cmd
.\Run_Full_Course.cmd

# Identical full-course runs with the previous and new map:
.\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 1 -RunName full-track-costmap-v1 -MissionTimeoutSec 900
.\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 2 -RunName full-track-costmap-v2 -MissionTimeoutSec 900 -LaneChangeCost 0.25

# Regenerate the configuration comparison without Docker:
.\.venv\Scripts\python.exe integrations/ros_humble_fortress/compare_lane_costmaps.py
```

Each full-course run snapshots `lane_semantics.yaml`, `costmap_config.json` and
the generated `nav2_params.yaml`. The metadata records version, penalty, hard
and crossable counts, source hashes and classification status. Existing run
manifests, rosbag capture, replay and scoring remain available. Compare mission
success and failure outcomes, stop gate, waypoint count, distance and time using
the same course/world/dynamics/build and mission budget; repeat laps before
attributing a performance difference to the map.

For direct Linux/container launches, `COSTMAP_VERSION`, `LANE_CHANGE_COST_M`
and optional `LANE_SEMANTICS_FILE` select those inputs. Compact-course smoke
runs retain their previous behavior unless using `run_full_track.sh`.

## Source and validation

- `integrations/nav2_smac_planner/lane_transition_cost.patch`: native primitive
  scoring, per-search frame conversion and shortcut handling.
- `integrations/robot/classified_lane_costmap.patch`: shared local/global tape
  classification and quadratic clearance.
- `integrations/ros_lane_costmap/lane_semantics.hpp`: canonical portable geometry;
  matching copies are installed by both patches.
- `tests/ros/lane_geometry_test.cpp` and `lane_traversal_test.cpp`: standalone
  geometry and ROS-linked native traversal regression checks.
- `tests/test_ros_lane_costmap.py`: profile generation, hard-boundary preservation,
  centreline accounting, desktop settings, stale-build rejection and compiled
  geometry execution.

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_ros_lane_costmap -v
```

In the rebuilt pinned container, run the existing ROS CMake/CTest workflow to
include `lane_geometry` and `lane_traversal`, then run paired full-course laps.
