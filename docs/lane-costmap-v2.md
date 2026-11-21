# Lane-aware cost map v2

The requested competition behavior is to allow lane changes when needed, with
a small preference against unnecessary changes. The default penalty is **0.25 m
of equivalent path cost per change**. It is an optimization weight, not extra
physical distance, seconds, a probability, or a competition scoring rule.

The runnable implementation is in `src/autonav_universe/lane_costmap.py`. It is
an opt-in version of the Python reference planner. Its Python v1 default is
preserved. The separate ROS implementation is described below.

## What examination of the existing maps found

- `ReferenceCostmap` only distinguished free, unknown, obstacle and steep cells.
  Its A* preferred known cells but had no lane identity or transition penalty.
- The pinned ROS `line_layer` stamps exact detections as lethal. The local and
  global camera/LiDAR line layers in `integrations/robot/full_track_baseline.yaml`
  deliberately prevent crossing those lines. Their sensor topics do not label
  internal dividers separately from outer boundaries.
- The supplied sketch requires a semantic distinction: the tire and outer
  boundary are hard constraints; the internal divider can be crossed.

## Cost definition

For a candidate path:

```text
total = length_m + clearance_cost_m + unknown_cost_m
        + lane_changes * lane_change_cost_m
```

| Input or setting | v2 behavior | Default |
|---|---|---|
| `obstacle`, `steep`, `boundary` | Swept footprint collision rejects the segment | Hard constraint |
| `lane_ids[(ix, iy)]` | An observed lane region ID, separate from occupancy | Empty if unclassified |
| `lane_change_cost_m` | Charge once when the last known lane ID changes | 0.25 m |
| `clearance_distance_m` | Soft-cost reach beyond the footprint exclusion zone | 0.8 m |
| `clearance_weight` | Maximum additional cost per metre near hard geometry | 3.0 |
| `unknown_multiplier` | Cost multiplier for unobserved occupancy | 1.15 |

The clearance field decays quadratically to zero. Overlapping halos use the
maximum, so dense detections do not multiply the penalty. Traversal integrates
the endpoint-average field over each segment in metres. Lane changes are
transition costs, not lethal raster cells or a band paid repeatedly per pixel.
A* keeps `(cell, last_known_lane)` as its state; an unclassified strip retains
the previous lane ID. First entering a classified region from an unclassified
start is not counted as a change. Labels must use consistent lane IDs.

Hard geometry takes priority over lane semantics. Collision checking intersects
each path segment with occupied cell squares enlarged by the robot's conservative
square footprint. This includes diagonal moves and the exact continuous start
and goal; the start cell is never silently cleared. The search starts with a 3 m margin around the start/goal rectangle and retries
with 6 m and 12 m margins when needed. If the reachable region does not touch
the current window edge, further expansion stops. `no_path` means no route was
found within the final explored window; a successful route is optimized within
the first window that contains one.

## Run and tune the illustrated example

From the repository root in Windows PowerShell:

```powershell
.\.venv\Scripts\autonav-universe.exe lane-costmap-demo --output runs/lane-costmap-v2
.\.venv\Scripts\autonav-universe.exe lane-costmap-demo --case optional --lane-change-cost 2 --output runs/lane-costmap-v2-optional
```

Each command saves `lane-costmap-v2.svg` and `lane-costmap-v2.json`. The JSON
contains the exact path, all cost components, assumed geometry, and settings.
Cases are `sketch` (right-to-left tire avoidance), `optional` (same-lane goal
with a shorter route across the divider twice), and `clear` (stay in lane).
All geometry and dimensions are authored examples inspired by the sketch,
not a reconstruction of measured sensor data. The illustration uses a 0.1 m
grid and a square footprint with 0.28 m half-size.

The sketch path makes one lane change and pays 0.25 m by default. The optional
case still takes the shorter two-change route with this small setting; a 2 m
penalty makes it take the longer route that stays in the original lane.

## Feed a real observed map to the Python planner

```python
from autonav_universe.lane_costmap import LaneAwareCostmap, LaneCostConfig

grid = LaneAwareCostmap(resolution_m=0.1,
    config=LaneCostConfig(lane_change_cost_m=0.25))
grid.cells = observed_occupancy       # (ix, iy) -> free/obstacle/steep/boundary
grid.lane_ids = observed_lane_ids    # (ix, iy) -> stable lane ID
result = grid.plan_with_costs(start_xy, goal_xy, half_width_m=0.54)
```

Use an enclosing footprint half-size for the actual robot. All coordinates,
occupancy and lane IDs must share the same frame and resolution. Boundaries
must be explicit occupancy labels; lane IDs alone do not establish hard edges.

`ReferenceNavigator` and `FixedPipelinePlanner` accept `--param costmap_version=2`
and `--param lane_change_cost_m=0.25`. The latter reads optional observed
`ReferenceNavigation.lane_cells` tuples `(ix, iy, lane_id)`. The existing
LiDAR-only runner does **not** infer lane IDs and leaves this field empty;
enabling v2 on an existing scenario alone does not synthesize lane perception.
The authored demo feeds semantics directly to the planner and is separate from
the benchmark runner's private ground truth.

The v2 follower tracks adjacent route points and stops on a missing or newly
blocked route instead of using direct-goal fallback or unchecked recovery.
The planner is a geometric grid prototype: it does not optimize steering,
curvature, velocity, or lane identity confidence. Closed-loop competition
performance and the full ROS course have not been established for v2.

## ROS integration

The [full-course ROS adaptation](ros-lane-costmap.md) now adds explicit divider
annotations, native Smac primitive transition costs, consistent local/global
line classification and a quadratic tape-clearance kernel. It retains Nav2's
existing footprint/controller behavior. Blender tape stays hard; a diagnostic
dashed divider is classified as crossable in the generated course. The ROS code awaits a Humble build and Gazebo runs
because Docker's Linux engine was unavailable during implementation. The
Python example remains a separate execution path and is not ROS performance
evidence.

## Validation

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_lane_costmap -v
```

Tests cover required and optional changes, a small default penalty, resolution
independence, unclassified strips, collision priority, diagonal clipping,
enclosed boundaries, cost accounting, parameter validation, and planner input
and stopping behavior.
