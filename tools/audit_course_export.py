"""Audit a Blender metadata export before assigning simulator semantics."""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path


def audit(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    errors, warnings = [], []
    if data.get("schema_version") not in (1, 2):
        errors.append("unsupported export schema")
    if data.get("schema_version") == 2:
        digest = data.get("source_sha256")
        if not isinstance(digest, str) or len(digest) != 64 or any(
                char not in "0123456789abcdef" for char in digest):
            errors.append("invalid source SHA-256")
        if not isinstance(data.get("source_size_bytes"), int) or data["source_size_bytes"] <= 0:
            errors.append("invalid source size")
    if data.get("unit_system") != "METRIC" or data.get("scale_length") != 1.0:
        errors.append("course must export in metric meters with scale length 1")
    objects = data.get("objects", [])
    if not objects:
        errors.append("no scene objects")
    names = [obj.get("name") for obj in objects]
    if len(names) != len(set(names)):
        errors.append("duplicate object names")
    keys = Counter()
    for obj in objects:
        matrix = obj.get("matrix_world", [])
        if (len(matrix) != 4 or any(len(row) != 4 for row in matrix)
                or not all(isinstance(v, (int, float)) and math.isfinite(v)
                           for row in matrix for v in row)):
            errors.append(f"invalid transform: {obj.get('name')}")
        bounds = obj.get("bounds_m", {})
        low, high = bounds.get("min", []), bounds.get("max", [])
        if (len(low) != 3 or len(high) != 3
                or not all(isinstance(v, (int, float)) and math.isfinite(v)
                           for v in (*low, *high))
                or any(a > b for a, b in zip(low, high))):
            errors.append(f"invalid bounds: {obj.get('name')}")
        props = obj.get("custom_properties", {})
        keys.update(props.keys())
    if not keys:
        warnings.append("no custom properties; waypoint/ramp/fault schema cannot be derived")
    return {"valid_geometry_metadata": not errors, "object_count": len(objects),
            "custom_property_keys": dict(sorted(keys.items())),
            "errors": errors, "warnings": warnings}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("metadata", type=Path)
    args = parser.parse_args()
    result = audit(args.metadata)
    print(json.dumps(result, indent=2))
    return 0 if result["valid_geometry_metadata"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
