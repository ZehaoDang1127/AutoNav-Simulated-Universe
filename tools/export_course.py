"""Run inside Blender: blender -b Course.blend --python tools/export_course.py -- OUTPUT_DIR.

Exports a GLB and an unmodified audit of object custom properties. This does
not guess the course's property schema or silently assign navigation meaning.
"""
from __future__ import annotations

import json
import hashlib
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def plain(value):
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if hasattr(value, "to_list"):
        return plain(value.to_list())
    if hasattr(value, "items"):
        return {str(key): plain(item) for key, item in value.items()}
    return str(value)


def main() -> None:
    if "--" not in sys.argv or len(sys.argv) <= sys.argv.index("--")+1:
        raise SystemExit("usage: blender -b Course.blend --python tools/export_course.py -- OUTPUT_DIR")
    output = Path(sys.argv[sys.argv.index("--")+1]).resolve()
    output.mkdir(parents=True, exist_ok=True)
    scene = bpy.context.scene
    objects = []
    for obj in scene.objects:
        corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
        bounds = {"min": [min(c[i] for c in corners) for i in range(3)],
                  "max": [max(c[i] for c in corners) for i in range(3)]}
        materials = []
        for slot in obj.material_slots:
            material = slot.material
            if material is not None:
                materials.append({"name": material.name,
                                  "diffuse_rgba": [float(v) for v in material.diffuse_color],
                                  "uses_nodes": bool(material.use_nodes)})
        objects.append({"name": obj.name, "type": obj.type,
                        "matrix_world": [[float(obj.matrix_world[i][j]) for j in range(4)]
                                         for i in range(4)],
                        "bounds_m": bounds,
                        "dimensions_m": [float(v) for v in obj.dimensions],
                        "materials": materials,
                        "vertex_count": len(obj.data.vertices) if obj.type == "MESH" else 0,
                        "polygon_count": len(obj.data.polygons) if obj.type == "MESH" else 0,
                        "custom_properties": {str(key): plain(obj[key]) for key in obj.keys()
                                              if not key.startswith("_")}})
    source = Path(bpy.data.filepath)
    metadata = {"schema_version": 2, "source_blend": source.name,
                "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "source_size_bytes": source.stat().st_size,
                "blender_version": bpy.app.version_string,
                "unit_system": scene.unit_settings.system,
                "scale_length": scene.unit_settings.scale_length,
                "objects": objects}
    (output / "course_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    bpy.ops.export_scene.gltf(filepath=str(output / "course.glb"), export_format="GLB")
    print(f"Exported {len(objects)} objects to {output}")


if __name__ == "__main__":
    main()
