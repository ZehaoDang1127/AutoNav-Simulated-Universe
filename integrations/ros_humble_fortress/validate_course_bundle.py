"""Verify the generated course/world pair before a local full-scene test."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def validate(bundle: Path, source: Path) -> dict:
    manifest = json.loads((bundle / "course_manifest.json").read_text(encoding="utf-8"))
    for path, key in ((bundle / "course.yaml", "course_yaml_sha256"),
                      (bundle / "world.sdf", "world_sdf_sha256"),
                      (source, "source_asset_sha256")):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != manifest.get(key):
            raise ValueError(f"Stale or mismatched course bundle: {path} differs from {key}")
    ET.parse(bundle / "world.sdf")
    if not manifest.get("geometry_check", {}).get("passed"):
        raise ValueError("The exported course has not passed its geometry check")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    args = parser.parse_args()
    manifest = validate(args.bundle, args.source)
    print(json.dumps({
        "course_id": manifest["course_id"],
        "course_world_source_hashes_match": True,
        "geometry_check": manifest["geometry_check"],
        "asset_semantics_complete": manifest.get("asset_semantics_complete", False),
        "semantic_gaps": manifest.get("semantic_gaps", []),
    }, indent=2))


if __name__ == "__main__":
    main()
