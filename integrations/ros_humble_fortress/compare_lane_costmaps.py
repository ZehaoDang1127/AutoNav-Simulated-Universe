"""Create an explicit configuration/field comparison for the ROS full course.

This reports configuration and analytic kernel values. It does not run Gazebo
or claim mission outcomes. Use matching full-course runs for performance data.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml

from prepare_lane_costmap import dump_profile, prepare


def compare(base, course, roles, output):
    output.mkdir(parents=True, exist_ok=True)
    profiles, metadata = {}, {}
    for version in ("1", "2"):
        profiles[version], metadata[version] = prepare(base, course, roles, version)
        (output / f"nav2-v{version}.yaml").write_text(
            dump_profile(profiles[version]), encoding="utf-8")
    layer = base["global_costmap"]["global_costmap"]["ros__parameters"]["lidar_line_layer"]
    inscribed, radius, decay = layer["inscribed_radius"], layer["inflation_radius"], layer["cost_scaling_factor"]
    samples = []
    for distance in (0, inscribed, .1, .25, .5, .6, .7, .85):
        old = (254 if distance == 0 else 253 if distance <= inscribed else
               int(252*math.exp(-decay*(distance-inscribed))) if distance <= radius else 0)
        new = (254 if distance == 0 else 253 if distance <= inscribed else
               int(252*max(0, 1-(distance-inscribed)/.8)**2))
        samples.append({"distance_m": distance, "v1_line_cost": old, "v2_line_cost": new})
    data = {"schema_version": 1, "scope": "ROS/Gazebo full-course configuration and line kernel, not an executed mission",
            "course_id": course.get("course_id"), "versions": metadata, "line_kernel_samples": samples,
            "runtime_validation": "Pending: Docker Desktop Linux engine unavailable during implementation"}
    (output / "comparison.json").write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    lines = ["# Full-course ROS/Gazebo: previous cost map versus v2", "",
             "The full-course ROS launcher and Qt workspace now select v1 or v2. "
             "The Python scenario lab is unchanged. The generated YAML files beside this report are the actual Nav2 profiles.", "",
             f"Course: `{course.get('course_id')}`. Solid tape: **{len(course.get('internal_lines', []))} segments**. "
             f"Current v2 classification: **{metadata['2']['hard_segments']} hard segments, "
             f"{metadata['2']['crossable_paint_segments']} crossable paint segments, "
             f"{metadata['2']['divider_segments']} transition segments**.", "",
             "Only explicitly classified paint is crossable; unclassified tape remains hard.", "",
             "| Feature | Previous v1 | New ROS v2 |", "|---|---|---|",
             "| Unclassified tape | Lethal | Lethal |",
             "| Obstacles, forbidden terrain | Existing sensor layers | Same layers |",
             "| Tape clearance field | Exponential, cut off at configured 0.6 m | Quadratic to zero 0.8 m beyond the line's narrow inscribed band |",
             "| Explicit divider paint | Lethal | Removed only from tape marking; obstacle layers retain priority |",
             "| Lane-change preference | None | 0.25 m equivalent per centreline crossing, converted to grid cost using map resolution |",
             "| Thick/double-edge paint | Both edges hard | Paint classification is separate from the single transition centreline |",
             "| Global planner | Patched Smac lattice | Same lattice and filled-footprint checker, plus transition scoring |",
             "| Shortcuts / smoothing | Existing behavior | Disabled when dividers are classified so scoring is not bypassed |",
             "| Local controller | Calibrated MPPI | Same MPPI, using the updated local map |",
             "| Referee | Explicitly crossable dashes permitted; other tape remains hard | Same behavior |",
             "| Reproducibility | Snapshot profile | Snapshot profile, semantic annotations, version and cost settings |", "",
             "## What changes on the current asset", "",
             ("V2 applies transition charges to the configured divider centrelines and permits the explicitly classified paint. "
              if metadata["2"]["divider_segments"] else
              "No dividers are classified, so v2 adds no lane-transition charges or crossable tape. ") +
             "The common Nav2 inflation layer still merges by maximum, "
             "so many final cells can have identical v1/v2 costs even when their line-only kernels differ. "
             "Collision margins are unchanged. Faster laps, fewer stalls, or better completion are not established.", "",
             "## Exact line-only kernel comparison", "",
             f"LiDAR tape settings from the baseline profile: inscribed radius {inscribed:g} m, "
             f"v1 cutoff {radius:g} m, v1 decay {decay:g}. Costs below are analytic kernel values **before** maximum merging "
             "with obstacle and stock inflation costs. 254 is lethal; 253 is inscribed; 0–252 are soft costs.", "",
             "| Distance from detected tape (m) | Previous v1 | New v2 |", "|---:|---:|---:|"]
    lines += [f"| {r['distance_m']:.2f} | {r['v1_line_cost']} | {r['v2_line_cost']} |" for r in samples]
    lines += ["", "## Validation and limits", "",
              "The standalone C++ geometry check passes crossing direction, primitive sample density, map resolution, "
              "shared endpoints, hard-boundary priority, quadratic decay and invalid input checks. "
              "Python checks cover profile preservation, annotation validation, desktop settings, and stale-workspace rejection. "
              "Clean patch application and repeated bootstrap detection pass for the pinned simulator, robot and Nav2 sources.", "",
              "**ROS compilation, native Smac traversal tests and paired Gazebo laps remain unverified.** "
              "Docker Desktop's Linux engine did not start on this machine. No Python scenario result is used as full-course evidence. "
              "The v2 launcher rejects a workspace whose manifest lacks the new patch hashes, rather than silently running an old build.", "",
              "## Run the comparison", "",
              "Rebuild the pinned workspace first, then use the full-course Qt cost-map selector, or run these from the repository root:", "",
              "```powershell", r".\integrations\ros_humble_fortress\setup.cmd",
              r".\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 1 -RunName full-track-costmap-v1 -MissionTimeoutSec 900",
              r".\integrations\ros_humble_fortress\run_compact_smoke.ps1 -FullTrack -CostmapVersion 2 -RunName full-track-costmap-v2 -MissionTimeoutSec 900",
              "```", "",
              "Compare mission success, ordered waypoints, finish, referee failures and stop gate from both `result.json` files; "
              "use `final_score.txt`, saved maps and rosbag data for time, distance, clearance and path analysis. "
              "Keep course/world/dynamics, mission budget and build identical, and repeat runs before making performance claims.", "",
              "Divider authoring and ROS adaptation details: `docs/ros-lane-costmap.md`.", ""]
    (output / "comparison.md").write_text("\n".join(lines), encoding="utf-8")
    return data


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=root / "integrations/robot/full_track_baseline.yaml")
    parser.add_argument("--course", type=Path, default=root / "runs/full-track-integrated-lanes/course.yaml")
    parser.add_argument("--semantics", type=Path, default=root / "integrations/robot/full_track_lane_semantics.yaml")
    parser.add_argument("--output", type=Path, default=root / "runs/lane-change-costmap-comparison")
    args = parser.parse_args()
    inputs = [yaml.safe_load(path.read_text(encoding="utf-8")) for path in (args.base, args.course, args.semantics)]
    result = compare(*inputs, args.output)
    print(json.dumps({"report": str((args.output / "comparison.md").resolve()), "versions": result["versions"]}))


if __name__ == "__main__":
    main()
