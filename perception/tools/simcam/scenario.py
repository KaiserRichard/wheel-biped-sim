"""Scenario validation and MJCF construction, without importing MuJoCo."""

from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from sim.models import make_wbr_3d_simple as gen
from tools.ground_plane.synth import FX, FY, CX, CY
from tools.obstacle_decision.decide import Advisory
from tools.scene_twin.replay import CAMERA_OFFSET
from tools.wbr_pose.fivebar import forward_kinematics
from tools.wbr_pose.model import build_xml as pose_xml

WIDTH, HEIGHT = 640, 480
CAMERA_NAME = "simcam_d435"
# ASSUMED: the downward 3-degree case in obstacle_decision/evaluate.py.
# Neither this angle nor scene_twin's front/top mount is measured calibration.
CAMERA_PITCH_DEG = 3.0
CAMERA_POS = (CAMERA_OFFSET, 0.0, gen.BODY_HALF[2])
STEP_WIDTH_M = 4.0
NEAR_M, FAR_M = 0.01, 6.0


def intrinsics():
    return np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])


def camera_fovy(fy=FY, height=HEIGHT):
    """MJCF fovy is always in degrees, including with compiler angle=radian."""
    if not math.isfinite(fy) or fy <= 0 or not math.isfinite(height) or height <= 0:
        raise ValueError("fy and image height must be positive and finite")
    return math.degrees(2 * math.atan(height / (2 * fy)))


def _object(value, path, required, optional=()):
    if not isinstance(value, dict):
        raise ValueError(f"{path}: expected an object")
    missing = set(required) - value.keys()
    extra = value.keys() - set(required) - set(optional)
    if missing:
        raise ValueError(f"{path}: missing {', '.join(sorted(missing))}")
    if extra:
        raise ValueError(f"{path}: unknown fields {', '.join(sorted(extra))}")


def _number(value, path, *, positive=False, nonnegative=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError(f"{path}: expected a finite number")
    if positive and value <= 0:
        raise ValueError(f"{path}: must be positive")
    if nonnegative and value < 0:
        raise ValueError(f"{path}: must be nonnegative")


def _string(value, path):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path}: expected a nonempty string")


def validate_scenario(value):
    """Return an independent copy of the shared JSON schema; reject typos early."""
    _object(value, "scenario", ("name", "description", "status", "robot", "obstacles"),
            ("expected",))
    for key in ("name", "description", "status"):
        _string(value[key], key)
    robot = value["robot"]
    _object(robot, "robot", ("start_x", "speed_m_s", "duration_s", "l0", "jump_at_s"))
    for key in ("start_x", "speed_m_s", "duration_s", "l0"):
        _number(robot[key], f"robot.{key}", positive=key in ("duration_s", "l0"),
                nonnegative=key == "speed_m_s")
    try:
        forward_kinematics(robot["l0"])
    except ValueError as exc:
        raise ValueError(f"robot.l0: {exc}") from exc
    jump = robot["jump_at_s"]
    if jump is not None:
        _number(jump, "robot.jump_at_s", nonnegative=True)
        if jump >= robot["duration_s"]:
            raise ValueError("robot.jump_at_s: must be before duration_s")
    if not isinstance(value["obstacles"], list):
        raise ValueError("obstacles: expected an array")
    schemas = {
        "box": (("type", "x_face", "depth", "width", "height"), ("y", "yaw_deg")),
        "step": (("type", "x_face", "height"), ()),
        "ramp": (("type", "x_start", "length", "rise", "width"), ()),
    }
    for i, ob in enumerate(value["obstacles"]):
        path = f"obstacles[{i}]"
        if not isinstance(ob, dict) or not isinstance(ob.get("type"), str):
            raise ValueError(f"{path}.type: expected box, step, or ramp")
        kind = ob["type"]
        if kind not in schemas:
            raise ValueError(f"{path}.type: unsupported {kind!r}; expected box, step, or ramp")
        _object(ob, path, *schemas[kind])
        for key in ob.keys() - {"type"}:
            _number(ob[key], f"{path}.{key}",
                    positive=key in ("depth", "width", "height", "length", "rise"))
    expected = value.get("expected", [])
    if not isinstance(expected, list):
        raise ValueError("expected: expected an array")
    for i, interval in enumerate(expected):
        path = f"expected[{i}]"
        _object(interval, path, ("t_from", "t_to", "advisory"), ("note",))
        for key in ("t_from", "t_to"):
            _number(interval[key], f"{path}.{key}", nonnegative=True)
        if not interval["t_from"] < interval["t_to"] <= robot["duration_s"]:
            raise ValueError(f"{path}: require t_from < t_to <= duration_s")
        if (not isinstance(interval["advisory"], str)
                or interval["advisory"] not in Advisory.__members__):
            raise ValueError(f"{path}.advisory: expected one of {', '.join(Advisory.__members__)}")
        if "note" in interval:
            _string(interval["note"], f"{path}.note")
    ordered = sorted(expected, key=lambda item: item["t_from"])
    if any(a["t_to"] > b["t_from"] for a, b in zip(ordered, ordered[1:])):
        raise ValueError("expected: intervals must not overlap")
    return deepcopy(value)


def load_scenario(path):
    path = Path(path)
    try:
        return validate_scenario(json.loads(path.read_text(encoding="utf-8")))
    except (ValueError, OSError) as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _vector(values):
    return " ".join(f"{v:.12g}" for v in values)


def obstacle_geoms(scenario, asset):
    """Return world geoms; add wedge meshes to the supplied MJCF asset element.

    A yawed box rotates about its footprint centre. x_face denotes the minimum
    world x of its footprint even after yaw, making approach placement unambiguous.
    """
    geoms = []
    robot = scenario["robot"]
    end_x = robot["start_x"] + robot["speed_m_s"] * robot["duration_s"]
    for i, ob in enumerate(scenario["obstacles"]):
        attrs = dict(name=f"obstacle_{i}", rgba="0.65 0.35 0.15 1")
        if ob["type"] in ("box", "step"):
            step = ob["type"] == "step"
            depth = max(4.0, end_x + 4.0 - ob["x_face"]) if step else ob["depth"]
            width = STEP_WIDTH_M if step else ob["width"]
            yaw = 0.0 if step else math.radians(ob.get("yaw_deg", 0.0))
            half_x = (abs(math.cos(yaw)) * depth + abs(math.sin(yaw)) * width) / 2
            attrs.update(type="box", pos=_vector((ob["x_face"] + half_x,
                         ob.get("y", 0.0), ob["height"] / 2)),
                         size=_vector((depth / 2, width / 2, ob["height"] / 2)),
                         euler=_vector((0, 0, yaw)))
        else:
            length, half_w, rise = ob["length"], ob["width"] / 2, ob["rise"]
            vertices = ((0, -half_w, 0), (0, half_w, 0),
                        (length, -half_w, 0), (length, half_w, 0),
                        (length, -half_w, rise), (length, half_w, rise))
            mesh = f"obstacle_{i}_mesh"
            ET.SubElement(asset, "mesh", name=mesh,
                          vertex=_vector(v for vertex in vertices for v in vertex),
                          face="0 3 2 0 1 3 0 2 4 1 5 3 2 5 4 2 3 5 0 5 1 0 4 5")
            attrs.update(type="mesh", mesh=mesh, pos=_vector((ob["x_start"], 0, 0)))
        geoms.append(ET.Element("geom", attrs))
    return geoms


def build_xml(scenario):
    scenario = validate_scenario(scenario)
    root = ET.fromstring(pose_xml())
    world = root.find("worldbody")
    asset = ET.SubElement(root, "asset")
    world.extend(obstacle_geoms(scenario, asset))
    # The wbr_pose floor is only +-2 m; a floor edge in view looks like an obstacle to the
    # detector. Cover the whole drive plus the depth range (FAR_M) beyond its end.
    robot = scenario["robot"]
    reach = abs(robot["start_x"]) + abs(robot["speed_m_s"]) * robot["duration_s"] + FAR_M + 1.0
    world.find("geom[@name='floor']").set("size", f"{reach:.3f} {reach:.3f} 0.01")
    body = world.find("body[@name='body']")
    pitch = math.radians(CAMERA_PITCH_DEG)
    # Local camera +X = robot right (-Y), +Y = image up; view is along -Z.
    ET.SubElement(body, "camera", name=CAMERA_NAME, pos=_vector(CAMERA_POS),
                  xyaxes=_vector((0, -1, 0, math.sin(pitch), 0, math.cos(pitch))),
                  fovy=f"{camera_fovy():.12g}", ipd="0")
    # Fix extent so adding distant obstacles cannot silently move clip planes.
    ET.SubElement(root, "statistic", extent="1")
    visual = root.find("visual")
    ET.SubElement(visual, "map", znear=str(NEAR_M), zfar=str(FAR_M))
    return ET.tostring(root, encoding="unicode")
