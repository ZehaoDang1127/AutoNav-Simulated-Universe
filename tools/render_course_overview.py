"""Render a top-down overview without changing the source .blend file.

blender -b Course.blend --python tools/render_course_overview.py -- OUTPUT.png
"""
from __future__ import annotations

import sys
from pathlib import Path

import bpy
from mathutils import Vector


def main() -> None:
    if "--" not in sys.argv or len(sys.argv) <= sys.argv.index("--") + 1:
        raise SystemExit("usage: blender -b Course.blend --python render_course_overview.py -- OUTPUT.png")
    output = Path(sys.argv[sys.argv.index("--") + 1]).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    scene = bpy.context.scene
    meshes = [obj for obj in scene.objects if obj.type == "MESH"]
    if not meshes:
        raise SystemExit("course has no meshes")
    lows = [min((obj.matrix_world @ Vector(corner))[axis]
                for obj in meshes for corner in obj.bound_box) for axis in range(3)]
    highs = [max((obj.matrix_world @ Vector(corner))[axis]
                 for obj in meshes for corner in obj.bound_box) for axis in range(3)]
    width, depth = highs[0] - lows[0], highs[1] - lows[1]
    bpy.ops.object.camera_add(location=((lows[0] + highs[0]) / 2,
                                        (lows[1] + highs[1]) / 2,
                                        highs[2] + max(width, depth)))
    camera = bpy.context.object
    camera.name = "Course_Overview_Camera"
    camera.rotation_euler = (0, 0, 0)
    camera.data.type = "ORTHO"
    camera.data.ortho_scale = max(width, depth) * 1.12
    scene.camera = camera
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "MATERIAL"
    scene.display.shading.light = "STUDIO"
    scene.render.resolution_x = 1400
    scene.render.resolution_y = 1400
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(output)
    bpy.ops.render.render(write_still=True)
    print(f"Rendered overview to {output}")


if __name__ == "__main__":
    main()
