"""Generate a full-course Nav2 profile with explicit, conservative line roles.

Only named course segments can become crossable. An absent classification
retains hard tape. Course coordinates are semantic authoring inputs, not a
replacement for online obstacle/tape observations or a supplied occupancy map.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import yaml


class RosParameterDumper(yaml.SafeDumper):
    # rcl's ROS 2 parameter parser rejects YAML anchors and aliases.
    def ignore_aliases(self, data):
        return True


def dump_profile(profile):
    return yaml.dump(profile, Dumper=RosParameterDumper, sort_keys=False)


def prepare(base, course, semantics, version="2", penalty=.25):
    if version not in ("1", "2") or not math.isfinite(penalty) or penalty < 0:
        raise ValueError("cost map version must be 1 or 2 and lane penalty finite and nonnegative")
    if semantics.get("schema_version") != 1 or semantics.get("frame_id") != "map":
        raise ValueError("lane semantics require schema_version 1 and course frame_id map")
    names = semantics.get("crossable_line_names", [])
    if not isinstance(names, list) or any(not isinstance(n, str) for n in names) or len(set(names)) != len(names):
        raise ValueError("crossable_line_names must contain distinct course segment names")
    # The simulator expands each dashed line into physical tape segments.
    # Use the same dash/gap geometry here so only painted portions are soft.
    segments = list(course.get("internal_lines", []))
    by_name = {line["name"]: [line] for line in segments}
    if len(by_name) != len(segments):
        raise ValueError("ambiguous course line name")
    for line in course.get("dashed_lines", []):
        name = line["name"]
        if name in by_name:
            raise ValueError("ambiguous course line name")
        if name in names and line.get("allow_crossing") is not True:
            raise ValueError("classified dashed line must explicitly allow crossing")
        start = tuple(float(line["start"][axis]) for axis in ("x_m", "y_m"))
        end = tuple(float(line["end"][axis]) for axis in ("x_m", "y_m"))
        dash = float(line.get("dash_length_m", .75))
        gap = float(line.get("gap_length_m", .45))
        length = math.dist(start, end)
        if (not all(math.isfinite(v) for v in (*start, *end, dash, gap)) or
                length < 1e-8 or dash <= 0 or gap < 0):
            raise ValueError("invalid dashed course line geometry")
        if length / (dash + gap) > 10000:
            raise ValueError("too many dashed course line segments")
        parts = []
        offset = 0.0
        while offset < length:
            finish = min(offset + dash, length)
            point = lambda distance: {axis: start[i] + (end[i] - start[i]) * distance / length
                                      for i, axis in enumerate(("x_m", "y_m"))}
            if finish - offset > 1e-6:
                parts.append({"name": name, "start": point(offset), "end": point(finish)})
            offset += dash + gap
        by_name[name] = parts
        segments.extend(parts)
    if any(n not in by_name for n in names):
        raise ValueError("ambiguous or missing classified course line name")
    if names and semantics.get("course_id") != course.get("course_id"):
        raise ValueError("classified dividers must name the matching course_id")
    if names and course.get("course_geometry") != "explicit_blender":
        raise ValueError("divider classification currently requires explicit Blender course boundaries")
    if names:
        disabled = {int(b["segment_index"]) for b in course.get("line_breaks", [])
                    if b.get("boundary") == "both" and float(b.get("start_fraction", 1)) == 0
                    and float(b.get("end_fraction", 0)) == 1}
        if any(i not in disabled for i in range(len(course.get("centerline", []))-1)):
            raise ValueError("explicit boundary roles required before softening generated course tape")
    dividers, boundaries = [], []
    for line in segments:
        coords = [float(line[end][axis]) for end in ("start", "end") for axis in ("x_m", "y_m")]
        if not all(math.isfinite(v) for v in coords) or math.dist(coords[:2], coords[2:]) < 1e-8:
            raise ValueError("invalid course line geometry")
        (dividers if line["name"] in names else boundaries).extend(coords)
    transitions = []
    seen = set()
    for polyline in semantics.get("divider_centerlines_m", []):
        if not isinstance(polyline, list) or len(polyline) < 2:
            raise ValueError("each divider centreline needs at least two course-frame points")
        for a, b in zip(polyline, polyline[1:]):
            if len(a) != 2 or len(b) != 2:
                raise ValueError("divider centreline points require x,y")
            a, b = tuple(map(float, a)), tuple(map(float, b))
            if not all(math.isfinite(v) for v in (*a, *b)) or math.dist(a, b) < 1e-8:
                raise ValueError("invalid divider centreline geometry")
            key = tuple(sorted((a, b)))
            if key in seen:
                raise ValueError("duplicate divider centreline segment")
            seen.add(key)
            transitions.extend((*a, *b))
    if bool(names) != bool(transitions):
        raise ValueError("crossable paint names and divider centrelines must be supplied together")
    # Generated outer boundaries are not exposed as crossable authoring inputs.
    tolerance = float(semantics.get("classification_tolerance_m", .12))
    if not math.isfinite(tolerance) or not 0 < tolerance <= .25:
        raise ValueError("classification tolerance must be in (0, 0.25] metres")
    result = copy.deepcopy(base)
    if version == "2":
        planner = result["planner_server"]["ros__parameters"]["GridBased"]
        planner.update(lane_change_cost_m=float(penalty),
                       lane_semantics_frame="map")
        if transitions:
            planner["lane_dividers"] = transitions
        for map_name in ("local_costmap", "global_costmap"):
            params = result[map_name][map_name]["ros__parameters"]
            for layer_name in params["plugins"]:
                layer = params[layer_name]
                if layer.get("plugin") != "line_layer::LineLayer":
                    continue
                layer.update(lane_semantics_frame="map", lane_classification_tolerance_m=tolerance,
                             quadratic_clearance=True, clearance_reach_m=.8)
                if dividers:
                    layer["lane_dividers"] = dividers
                    if boundaries:
                        layer["lane_hard_boundaries"] = boundaries
        if dividers:
            planner["smooth_path"] = False
    return result, {"schema_version": 1, "version": version, "lane_change_cost_m": penalty,
                    "course_id": course.get("course_id"), "semantic_frame": "map",
                    "crossable_line_names": names if version == "2" else [],
                    "divider_segments": len(transitions)//4 if version == "2" else 0,
                    "crossable_paint_segments": len(dividers)//4 if version == "2" else 0,
                    "hard_segments": len(boundaries)//4 if version == "2" else len(segments),
                    "classification_tolerance_m": tolerance,
                    "semantic_status": "classified_dividers" if names and version == "2" else "all_tape_hard",
                    "note": "Only explicitly classified course segments can be crossable; all other tape stays hard."}


def main():
    parser = argparse.ArgumentParser()
    for name in ("base", "course", "semantics", "output", "metadata"):
        parser.add_argument("--"+name, type=Path, required=True)
    parser.add_argument("--version", choices=("1", "2"), default="2")
    parser.add_argument("--lane-change-cost", type=float, default=.25)
    parser.add_argument("--workspace-manifest", type=Path)
    args = parser.parse_args()
    try:
        inputs = {name: yaml.safe_load(getattr(args, name).read_text(encoding="utf-8"))
                  for name in ("base", "course", "semantics")}
        if args.workspace_manifest is not None:
            manifest = json.loads(args.workspace_manifest.read_text(encoding="utf-8"))
            root = Path(__file__).resolve().parents[2]
            required = []
            if args.version == "2":
                required.extend(("integrations/robot/classified_lane_costmap.patch",
                                 "integrations/nav2_smac_planner/lane_transition_cost.patch"))
            if any(line.get("allow_crossing") is True for line in inputs["course"].get("dashed_lines", [])):
                required.append("integrations/autonav_sim/crossable_dashed_tape.patch")
            for relative in required:
                expected = hashlib.sha256((root / relative).read_bytes()).hexdigest()
                if manifest.get("input_sha256", {}).get("patch:"+relative) != expected:
                    raise ValueError("ROS lane cost map or referee build is missing or stale; run setup.cmd")
        profile, metadata = prepare(**inputs, version=args.version, penalty=args.lane_change_cost)
        metadata["input_sha256"] = {name: hashlib.sha256(getattr(args, name).read_bytes()).hexdigest()
                                     for name in inputs}
        args.output.write_text(dump_profile(profile), encoding="utf-8")
        args.metadata.write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2, f"Lane cost map configuration rejected: {exc}\n")
    print(json.dumps(metadata))


if __name__ == "__main__":
    main()
