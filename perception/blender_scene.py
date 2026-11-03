"""Render a labeled synthetic obstacle scene from Blender primitives."""
import csv
import json
from pathlib import Path
import sys

import bpy
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view


def arguments():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(args) != 1:
        raise SystemExit("usage: blender --background --python blender_scene.py -- OUTPUT_DIR")
    return Path(args[0]).resolve()


def look_at(obj, point):
    obj.rotation_euler = (Vector(point) - obj.location).to_track_quat("-Z", "Y").to_euler()


def main():
    out = arguments()
    out.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0.62))
    body = bpy.context.object
    body.name = "body"
    body.dimensions = (0.30, 0.20, 0.10)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    for index, x in enumerate((-0.22, 0.22)):
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=32, radius=0.12, depth=0.07,
            location=(x, 0, 0.12), rotation=(1.5708, 0, 0),
        )
        bpy.context.object.name = f"wheel_{index}"

    bpy.ops.mesh.primitive_cube_add(size=1, location=(1.15, 0, 0.08))
    obstacle = bpy.context.object
    obstacle.name = "obstacle"
    obstacle.dimensions = (0.22, 0.7, 0.16)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    bpy.ops.mesh.primitive_plane_add(size=5, location=(0.4, 0, 0))
    ground = bpy.context.object
    ground.name = "ground"

    bpy.ops.object.camera_add(location=(2.5, -4.2, 1.8))
    camera = bpy.context.object
    look_at(camera, (0.45, 0, 0.45))
    scene = bpy.context.scene
    scene.camera = camera
    scene.render.engine = "CYCLES"
    scene.cycles.device = "CPU"
    scene.cycles.samples = 8
    scene.render.resolution_x = 640
    scene.render.resolution_y = 480
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"

    bpy.ops.object.light_add(type="AREA", location=(0, -2.5, 3))
    bpy.context.object.data.energy = 700
    bpy.context.object.data.shape = "DISK"
    bpy.context.object.data.size = 3

    targets = {"body": (0, body), "obstacle": (1, obstacle)}
    image_names = []
    with (out / "labels.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", "class_id", "x_center", "y_center", "width", "height"])
        viewpoints = [(2.5, -4.2, 1.8), (3.0, -3.2, 1.5), (1.8, -4.5, 1.4), (3.5, -2.0, 1.8), (1.3, -3.6, 1.9)]
        for index, location in enumerate(viewpoints):
            camera.location = location
            look_at(camera, (0.45, 0, 0.45))
            image_name = f"synthetic_{index:04d}.png"
            scene.render.filepath = str(out / image_name)
            bpy.ops.render.render(write_still=True)
            image_names.append(image_name)
            for _, (class_id, obj) in targets.items():
                projected = [world_to_camera_view(scene, camera, obj.matrix_world @ Vector(corner)) for corner in obj.bound_box]
                xs = [point.x for point in projected]
                ys = [point.y for point in projected]
                x0, x1 = max(0.0, min(xs)), min(1.0, max(xs))
                y0, y1 = max(0.0, min(ys)), min(1.0, max(ys))
                if x1 > x0 and y1 > y0:
                    writer.writerow([image_name, class_id, (x0 + x1) / 2, 1 - (y0 + y1) / 2, x1 - x0, y1 - y0])
    (out / "synthetic_renders.json").write_text(json.dumps({"status": "SYNTHETIC", "generator": "blender_scene.py", "images": image_names}, indent=2) + "\n")


if __name__ == "__main__":
    main()
