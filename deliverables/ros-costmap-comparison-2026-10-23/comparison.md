# Full-course ROS/Gazebo: previous cost map versus v2

The full-course ROS launcher and Qt workspace now select v1 or v2. The Python scenario lab is unchanged. The generated YAML files beside this report are the actual Nav2 profiles.

Course: `local_mock_course`. Tape geometry: **872 segments**. Current v2 classification: **872 hard segments, 0 crossable paint segments, 0 transition segments**.

As requested, every current tape segment remains hard. No existing tape is inferred to be a lane divider.

| Feature | Previous v1 | New ROS v2 |
|---|---|---|
| Unclassified tape | Lethal | Lethal |
| Obstacles, forbidden terrain | Existing sensor layers | Same layers |
| Tape clearance field | Exponential, cut off at configured 0.6 m | Quadratic to zero 0.8 m beyond the line's narrow inscribed band |
| Explicit divider paint | Lethal | Removed only from tape marking; obstacle layers retain priority |
| Lane-change preference | None | 0.25 m equivalent per centreline crossing, converted to grid cost using map resolution |
| Thick/double-edge paint | Both edges hard | Paint classification is separate from the single transition centreline |
| Global planner | Patched Smac lattice | Same lattice and filled-footprint checker, plus transition scoring |
| Shortcuts / smoothing | Existing behavior | Disabled when dividers are classified so scoring is not bypassed |
| Local controller | Calibrated MPPI | Same MPPI, using the updated local map |
| Stop gate, referee, speed limit | Existing behavior | Same behavior |
| Reproducibility | Snapshot profile | Snapshot profile, semantic annotations, version and cost settings |

## What changes on the current asset

With zero classified dividers, v2 adds no lane-transition charges and creates no new crossable tape. Its active change is the tape-clearance kernel. The common Nav2 inflation layer still merges by maximum, so many final cells can have identical v1/v2 costs even when their line-only kernels differ. Collision margins are unchanged. Faster laps, fewer stalls, or better completion are not established.

## Exact line-only kernel comparison

LiDAR tape settings from the baseline profile: inscribed radius 0.05 m, v1 cutoff 0.6 m, v1 decay 1. Costs below are analytic kernel values **before** maximum merging with obstacle and stock inflation costs. 254 is lethal; 253 is inscribed; 0–252 are soft costs.

| Distance from detected tape (m) | Previous v1 | New v2 |
|---:|---:|---:|
| 0.00 | 254 | 254 |
| 0.05 | 253 | 253 |
| 0.10 | 239 | 221 |
| 0.25 | 206 | 141 |
| 0.50 | 160 | 48 |
| 0.60 | 145 | 24 |
| 0.70 | 0 | 8 |
| 0.85 | 0 | 0 |

## Validation and limits

The standalone C++ geometry check passes crossing direction, primitive sample density, map resolution, shared endpoints, hard-boundary priority, quadratic decay and invalid input checks. Python checks cover profile preservation, annotation validation, desktop settings, and stale-workspace rejection. Clean patch application and repeated bootstrap detection pass for the pinned simulator, robot and Nav2 sources.

**ROS compilation, native Smac traversal tests and paired Gazebo laps remain unverified.** Docker Desktop's Linux engine did not start on this machine. No Python scenario result is used as full-course evidence. The v2 launcher rejects a workspace whose manifest lacks the new patch hashes, rather than silently running an old build.

## Run the comparison

Rebuild the pinned workspace first, then use the full-course Qt cost-map selector, or run these from the repository root:

```powershell
.\integrations\ros_humble_fortress\setup.cmd
.\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 1 -RunName full-track-costmap-v1 -MissionTimeoutSec 900
.\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 2 -RunName full-track-costmap-v2 -MissionTimeoutSec 900
```

Compare mission success, ordered waypoints, finish, referee failures and stop gate from both `result.json` files; use `final_score.txt`, saved maps and rosbag data for time, distance, clearance and path analysis. Keep course/world/dynamics, mission budget and build identical, and repeat runs before making performance claims.

Divider authoring and ROS adaptation details: `docs/ros-lane-costmap.md`.
