"""Build a diagnostic Fortress world and audit the local Blender course.

This reuses the pinned autonav_sim course loader and world generator. It keeps
public GPS targets separate from private or provisional course geometry and
fails a release build when the source asset lacks required semantics.

Run the two Blender exporters first; see Mock_Course_Asset/README.md.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import runpy
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

from tools.audit_course_export import audit


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def semantic_gaps(metadata: dict, semantics: dict, course) -> list[str]:
    objects = {item["name"]: item for item in metadata["objects"]}
    roles = Counter(str(item.get("custom_properties", {}).get("type", "")).lower()
                    for item in metadata["objects"])
    gaps: list[str] = []

    expected_types = {
        "line_objects": "line", "ramp_objects": "ramp",
        "pothole_objects": "pothole", "no_mans_land_objects": "no_mans_land",
        "internal_boundary_objects": "internal_boundary",
        "finish_gate_objects": "finish_gate",
        "referee_checkpoint_objects": "referee_checkpoint",
    }

    def require_objects(key: str, minimum: int = 1) -> None:
        names = semantics.get(key, [])
        if len(names) < minimum:
            gaps.append(f"{key}: expected at least {minimum} authored object(s)")
        for name in names:
            if name not in objects:
                gaps.append(f"{key}: missing Blender object {name}")
            elif objects[name].get("custom_properties", {}).get("type") != expected_types[key]:
                gaps.append(f"{key}: {name} needs type={expected_types[key]}")

    if semantics.get("start_object") not in objects:
        gaps.append("start_object: missing Blender object")
    for key in ("line_objects", "ramp_objects", "pothole_objects",
                "no_mans_land_objects", "internal_boundary_objects",
                "finish_gate_objects", "referee_checkpoint_objects"):
        require_objects(key)
    if not roles["ground"]:
        gaps.append("ground: no typed surface")
    if not roles["cone"] and not roles["barrel"]:
        gaps.append("obstacles: no typed obstacle")

    gps_names = semantics.get("provisional_gps_target_objects", [])
    if len(gps_names) != 4 or len(set(gps_names)) != 4:
        gaps.append("provisional GPS targets: exactly four distinct objects required")
    for name in gps_names:
        if name not in objects:
            gaps.append(f"GPS target: missing Blender object {name}")
    candidate = semantics.get("provisional_finish_marker_object")
    if candidate not in objects:
        gaps.append("provisional finish marker: missing Blender object")
    if len(course.mission_waypoints) != 4:
        gaps.append(f"upstream mission has {len(course.mission_waypoints)} GPS goals; release requires four")
    if [point.label for point in course.mission_waypoints[:4]] != gps_names:
        gaps.append("public GPS object order differs from exported mission order")
    if not roles["finish"]:
        gaps.append("finish: no explicitly typed finish object")

    ground = next((item for item in metadata["objects"]
                   if item.get("custom_properties", {}).get("type") == "ground"), None)
    if ground:
        low, high = ground["bounds_m"]["min"], ground["bounds_m"]["max"]
        width, depth = high[0] - low[0], high[1] - low[1]
        # The mock asset may depart from the rules, but that departure must be visible.
        if max(width, depth) > 36.576 + 0.3048 or min(width, depth) > 30.48 + 0.3048:
            gaps.append(f"ground footprint {width:.3f} x {depth:.3f} m exceeds 36.576 x 30.48 m reference")
    return gaps


def hard_geometry_course(data: dict) -> tuple[dict, int]:
    """Retain physical hard geometry while excluding explicitly crossable paint."""
    original = data.get("dashed_lines", [])
    hard = [line for line in original if line.get("allow_crossing") is not True]
    result = dict(data)
    result["dashed_lines"] = hard
    return result, len(original) - len(hard)


def prepare(source: Path, metadata_path: Path, course_yaml: Path,
            semantics_path: Path, sim_package: Path, output: Path) -> dict:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    semantics = json.loads(semantics_path.read_text(encoding="utf-8"))
    raw_audit = audit(metadata_path)
    if not raw_audit["valid_geometry_metadata"]:
        raise ValueError("invalid Blender export: " + "; ".join(raw_audit["errors"]))
    source_hash = sha256(source)
    if metadata.get("source_sha256") != source_hash:
        raise ValueError("Blender metadata does not match the current source asset")
    if semantics.get("source_sha256") != source_hash:
        raise ValueError("course semantics are stale for the current source asset")
    if semantics.get("schema_version") != 1:
        raise ValueError("unsupported course semantics schema")
    if not (sim_package / "igvc_competition_sim" / "course.py").is_file():
        raise ValueError("--sim-package must point to the pinned igvc_competition_sim package root")

    sys.path.insert(0, str(sim_package.resolve()))
    from igvc_competition_sim.course import load_course, local_to_latlon
    from igvc_competition_sim.generate_world import generate_world

    # The diagnostic divider is an explicit overlay, kept separate from the
    # Blender inventory. Generate and hash one YAML before generating the world.
    import yaml
    data = yaml.safe_load(course_yaml.read_text(encoding="utf-8"))
    overlay = semantics.get("diagnostic_dashed_lines", [])
    road_edges = semantics.get("diagnostic_boundary_lines", [])
    road_surfaces = semantics.get("diagnostic_road_surfaces", [])
    if overlay or road_edges or road_surfaces:
        if data.get("course_id") != semantics["course_id"]:
            raise ValueError("diagnostic divider overlay course ID mismatch")
        for surface in road_surfaces:
            corners = surface.get("corners_m", [])
            if (len(corners) != 4 or any(len(p) != 2 for p in corners) or
                    not all(math.isfinite(float(v)) for p in corners for v in p)):
                raise ValueError("invalid diagnostic road surface")
        data.setdefault("diagnostic_road_surfaces", []).extend(road_surfaces)
        boundaries = data.setdefault("internal_lines", [])
        used_names = {line.get("name") for line in boundaries + data.get("dashed_lines", [])}
        for line in road_edges:
            name = line.get("name")
            try:
                coords = [float(line[end][axis]) for end in ("start", "end") for axis in ("x_m", "y_m")]
                width = float(line.get("width_m", .0762))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid diagnostic road edge") from exc
            if (not isinstance(name, str) or not name or name in used_names or
                    not all(math.isfinite(v) for v in (*coords, width)) or width <= 0 or
                    math.dist(coords[:2], coords[2:]) < 1e-8):
                raise ValueError("invalid or duplicate diagnostic road edge")
            boundaries.append(line)
            used_names.add(name)
        existing = data.setdefault("dashed_lines", [])
        existing_names = used_names
        for line in overlay:
            name = line.get("name")
            if not isinstance(name, str) or not name:
                raise ValueError("diagnostic divider requires a name")
            try:
                coords = [float(line[end][axis]) for end in ("start", "end") for axis in ("x_m", "y_m")]
                dash, gap = float(line.get("dash_length_m", .75)), float(line.get("gap_length_m", .45))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid diagnostic dashed line") from exc
            length = math.dist(coords[:2], coords[2:])
            if (not all(math.isfinite(v) for v in (*coords, dash, gap)) or length < 1e-8
                    or dash <= 0 or gap < 0 or length / (dash + gap) > 10000):
                raise ValueError("invalid diagnostic dashed line geometry")
            if line.get("allow_crossing") is not True:
                raise ValueError("diagnostic dashed divider must explicitly allow crossing")
            if line.get("name") in existing_names:
                raise ValueError("diagnostic divider name already exists in course")
            existing.append(line)
            existing_names.add(line.get("name"))
    output.mkdir(parents=True, exist_ok=True)
    saved_yaml = output / "course.yaml"
    if overlay or road_edges or road_surfaces:
        saved_yaml.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    elif course_yaml.resolve() != saved_yaml.resolve():
        shutil.copyfile(course_yaml, saved_yaml)
    course = load_course(saved_yaml)
    if course.course_id != semantics["course_id"]:
        raise ValueError("course ID differs from semantic overlay")
    gaps = semantic_gaps(metadata, semantics, course)
    world = output / "world.sdf"
    world.write_text("\n".join(line.rstrip() for line in generate_world(course).splitlines()) + "\n",
                     encoding="utf-8")
    ET.parse(world)
    crossable = [line for line in data.get("dashed_lines", [])
                 if line.get("allow_crossing") is True]
    validator_path = sim_package / "autoresearch" / "lib" / "validate_course.py"
    if validator_path.is_file():
        try:
            validator = runpy.run_path(str(validator_path))
            # The upstream validator treats every tape segment as lethal. For
            # the v2 diagnostic course, exclude only paint explicitly marked
            # crossable; all hard tape and all obstacles remain in the check.
            # The actual saved course and generated SDF retain the paint.
            if crossable:
                validation_data, _ = hard_geometry_course(data)
                with tempfile.TemporaryDirectory(dir=output) as temp_dir:
                    validation_path = Path(temp_dir) / "hard_geometry.yaml"
                    validation_path.write_text(yaml.safe_dump(validation_data, sort_keys=False), encoding="utf-8")
                    geometry_ok, geometry_detail = validator["validate"](validation_path)
                geometry_detail = "v2 hard geometry only (crossable paint omitted): " + geometry_detail
            else:
                geometry_ok, geometry_detail = validator["validate"](saved_yaml)
        except Exception as exc:  # Preserve a failed validator as a failed gate.
            geometry_ok, geometry_detail = False, f"validator error: {exc}"
    else:
        geometry_ok, geometry_detail = False, "upstream validator missing"
    if not geometry_ok:
        gaps.append(f"geometric feasibility: {geometry_detail}")
    if crossable:
        gaps.append("crossable paint excluded from v2 hard-geometry check; full-tape and v1 feasibility not established")

    public_targets = []
    for waypoint in course.mission_waypoints[:4]:
        lat, lon = local_to_latlon(waypoint.x_m, waypoint.y_m,
                                   course.datum_latitude_deg, course.datum_longitude_deg)
        public_targets.append({"label": waypoint.label, "longitude_deg": lon,
                               "latitude_deg": lat, "radius_m": waypoint.radius_m})
    (output / "public_mission.json").write_text(json.dumps({
        "schema_version": 1, "course_id": course.course_id,
        "gps_targets": public_targets,
        "coordinate_contract": "NavigateToWaypoint wgs84: x=longitude, y=latitude",
        "provisional": True,
    }, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1, "status": "asset_diagnostic" if gaps else "asset_candidate",
        "asset_semantics_complete": not gaps,
        "release_ready": False,
        "course_id": course.course_id,
        "source_asset_sha256": source_hash,
        "metadata_sha256": sha256(metadata_path),
        "semantics_sha256": sha256(semantics_path),
        "course_yaml_sha256": sha256(saved_yaml),
        "world_sdf_sha256": sha256(world),
        "simulator_revision": "46b8e9aa953160b2d317fcc811e209fed46a52f4",
        "robot_revision": "00875b391b9ea897681893ec063c99a3d9d32835",
        "blender_version": metadata.get("blender_version"),
        "object_count": len(metadata["objects"]),
        "geometry_check": {"passed": geometry_ok, "detail": geometry_detail,
                           "scope": "v2_hard_geometry" if crossable else "full_tape_geometry",
                           "crossable_dashed_lines_excluded": len(crossable)},
        "semantic_gaps": gaps,
        "limitations": [
            "The SDF is generated from semantic YAML, not a direct collision export of every Blender mesh.",
            "Connectivity, sensor visibility, localization, and ROS/Gazebo operation require runtime verification.",
        ],
    }
    (output / "course_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                                   encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--course-yaml", type=Path, required=True)
    parser.add_argument("--semantics", type=Path, required=True)
    parser.add_argument("--sim-package", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-complete-assets", action="store_true")
    args = parser.parse_args()
    result = prepare(args.source, args.metadata, args.course_yaml,
                     args.semantics, args.sim_package, args.output)
    print(json.dumps(result, indent=2))
    return 1 if args.require_complete_assets and not result["asset_semantics_complete"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
