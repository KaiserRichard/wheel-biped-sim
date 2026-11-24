from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from tools.simcam.scenario import (
    CAMERA_NAME, CAMERA_POS, CAMERA_PITCH_DEG, FAR_M, NEAR_M, STEP_WIDTH_M,
    build_xml, camera_fovy, intrinsics, load_scenario, validate_scenario,
)

EXAMPLE = Path(__file__).resolve().parents[3] / "docs/simcam/example_box132.json"


def example():
    return load_scenario(EXAMPLE)


def vector(element, key):
    return np.fromstring(element.get(key), sep=" ")


class ScenarioTests(unittest.TestCase):
    def test_shared_example_and_copy(self):
        value = example()
        result = validate_scenario(value)
        result["robot"]["start_x"] = 99
        self.assertEqual(value["robot"]["start_x"], 0.0)
        self.assertEqual(value["name"], "box132_approach")

    def test_clear_validation_errors(self):
        mutations = [
            (lambda s: s.pop("robot"), "missing robot"),
            (lambda s: s.update(speed=1), "unknown fields speed"),
            (lambda s: s["robot"].update(duration_s=0), "robot.duration_s: must be positive"),
            (lambda s: s["robot"].update(speed_m_s=-1), "robot.speed_m_s: must be nonnegative"),
            (lambda s: s["robot"].update(start_x=True), "robot.start_x: expected a finite number"),
            (lambda s: s["robot"].update(start_x=float("nan")), "robot.start_x: expected a finite number"),
            (lambda s: s["robot"].update(l0=9), "robot.l0: five-bar unreachable"),
            (lambda s: s["robot"].update(jump_at_s=6), "robot.jump_at_s: must be before"),
            (lambda s: s["obstacles"][0].update(type="wall"), "unsupported 'wall'"),
            (lambda s: s["obstacles"][0].update(height=-1), "height: must be positive"),
            (lambda s: s["obstacles"][0].update(width=float("inf")), "width: expected a finite number"),
            (lambda s: s["expected"][0].update(advisory="JUMP"), "advisory: expected one of"),
            (lambda s: s["expected"][0].update(t_to=7), "require t_from < t_to"),
            (lambda s: s["expected"].append(deepcopy(s["expected"][0])), "intervals must not overlap"),
            (lambda s: s.update(obstacles={}), "obstacles: expected an array"),
            (lambda s: s.update(expected=None), "expected: expected an array"),
        ]
        for mutate, message in mutations:
            value = example()
            mutate(value)
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                validate_scenario(value)

    def test_empty_scene_optional_expected_and_null_jump(self):
        value = example()
        value["obstacles"] = []
        value.pop("expected")
        self.assertEqual(validate_scenario(value), value)

    def test_file_error_includes_path(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.json"
            path.write_text('{"name":')
            with self.assertRaisesRegex(ValueError, "broken.json"):
                load_scenario(path)
            path.write_text(json.dumps({"name": "missing fields"}))
            with self.assertRaisesRegex(ValueError, "broken.json.*missing"):
                load_scenario(path)


class GeometryTests(unittest.TestCase):
    def test_box_at_requested_face_and_size(self):
        root = ET.fromstring(build_xml(example()))
        geom = root.find("worldbody/geom[@name='obstacle_0']")
        np.testing.assert_allclose(vector(geom, "pos"), (1.3, 0, 0.066))
        np.testing.assert_allclose(vector(geom, "size"), (0.1, 0.15, 0.066))
        self.assertEqual(root.find("worldbody/body[@name='body']").get("mocap"), "true")

    def test_yawed_box_face_is_minimum_world_x(self):
        value = example()
        ob = value["obstacles"][0]
        ob.update(y=0.4, yaw_deg=40)
        geom = ET.fromstring(build_xml(value)).find("worldbody/geom[@name='obstacle_0']")
        center, half = vector(geom, "pos"), vector(geom, "size")
        yaw = vector(geom, "euler")[2]
        corners_x = [center[0] + x * half[0] * math.cos(yaw) - y * half[1] * math.sin(yaw)
                     for x in (-1, 1) for y in (-1, 1)]
        self.assertAlmostEqual(min(corners_x), ob["x_face"])
        self.assertAlmostEqual(center[1], 0.4)

    def test_step_spans_floor_and_continues_past_approach(self):
        value = example()
        value["obstacles"] = [{"type": "step", "x_face": 2.0, "height": 0.08}]
        geom = ET.fromstring(build_xml(value)).find("worldbody/geom[@name='obstacle_0']")
        center, half = vector(geom, "pos"), vector(geom, "size")
        self.assertAlmostEqual(center[0] - half[0], 2.0)
        self.assertAlmostEqual(half[1] * 2, STEP_WIDTH_M)
        self.assertAlmostEqual(center[2] - half[2], 0.0)
        self.assertAlmostEqual(center[2] + half[2], 0.08)
        self.assertGreater(center[0] + half[0], 1.5 + 3.0)

    def test_ramp_wedge_endpoints_width_and_outward_volume(self):
        value = example()
        value["obstacles"] = [dict(type="ramp", x_start=1, length=0.8, rise=0.1, width=0.6)]
        root = ET.fromstring(build_xml(value))
        geom = root.find("worldbody/geom[@name='obstacle_0']")
        mesh = root.find(f"asset/mesh[@name='{geom.get('mesh')}']")
        vertices = vector(mesh, "vertex").reshape(-1, 3)
        faces = vector(mesh, "face").astype(int).reshape(-1, 3)
        world_vertices = vertices + vector(geom, "pos")
        np.testing.assert_allclose(world_vertices.min(axis=0), (1, -0.3, 0))
        np.testing.assert_allclose(world_vertices.max(axis=0), (1.8, 0.3, 0.1))
        triangles = vertices[faces]
        volume = np.einsum("ij,ij->i", triangles[:, 0],
                           np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6
        self.assertAlmostEqual(volume, 0.8 * 0.6 * 0.1 / 2)

    def test_camera_mount_axes_and_fovy(self):
        root = ET.fromstring(build_xml(example()))
        camera = root.find(f"worldbody/body[@name='body']/camera[@name='{CAMERA_NAME}']")
        np.testing.assert_allclose(vector(camera, "pos"), CAMERA_POS)
        axes = vector(camera, "xyaxes").reshape(2, 3)
        view = -np.cross(axes[0], axes[1])
        np.testing.assert_allclose(view, (math.cos(math.radians(CAMERA_PITCH_DEG)),
                                         0, -math.sin(math.radians(CAMERA_PITCH_DEG))), atol=1e-11)
        self.assertAlmostEqual(float(camera.get("fovy")), camera_fovy(), places=9)
        self.assertAlmostEqual(camera_fovy(), math.degrees(2 * math.atan(240 / intrinsics()[1, 1])))
        self.assertEqual(root.find("statistic").get("extent"), "1")
        self.assertEqual(float(root.find("visual/map").get("znear")), NEAR_M)
        self.assertEqual(float(root.find("visual/map").get("zfar")), FAR_M)


if __name__ == "__main__":
    unittest.main()
