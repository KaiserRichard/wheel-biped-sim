"""Scenario -> robot-camera depth -> real detector/decision -> research jump gate."""

import argparse
import csv
from dataclasses import replace
import math
from pathlib import Path

import numpy as np

from tools.jump_gate.gate import JumpGate, Tick
from tools.jump_gate.timeline import SyntheticPose, synthetic_pose
from tools.obstacle_decision import detect, decide
from tools.wbr_pose.jump_demo import LAND_L0
from tools.wbr_pose.model import set_pose
from .depth import clipping_distances, optical_depth, pinhole_frustum, stereo_noise
from .scenario import (CAMERA_NAME, HEIGHT, WIDTH, build_xml, intrinsics,
                       load_scenario)

DETECTOR_HZ = 10.0  # ASSUMED acquisition and processing rate, no real-time claim.
CSV_FIELDS = (
    "t", "robot_x", "camera_height_m", "camera_pitch_deg", "raw_advisory",
    "height_m", "distance_m", "plane_valid", "gated_advisory", "phase",
    "expected_advisory", "match", "raw_match", "raw_confidence", "gated_confidence",
    "synthetic_phase", "gate_reason", "plane_reused", "plane_age_frames",
    "history_reset", "body_pitch_deg", "leg_length_m", "vertical_offset_m",
    "scene_status", "depth_status", "phase_source",
)


def tick_times(duration_s, hz=DETECTOR_HZ):
    """Uniform ticks in [0,duration], including duration only if on the grid."""
    if not (math.isfinite(duration_s) and duration_s > 0
            and math.isfinite(hz) and hz > 0):
        raise ValueError("duration_s and hz must be positive and finite")
    last = math.floor(math.nextafter(duration_s * hz, math.inf))
    return [k / hz for k in range(last + 1)]


def pose_at(t, robot):
    """Jump time is CROUCH onset, reusing the demo's exact phase/contact mapping.

    The current wbr_pose timeline has no attitude trajectory: body pitch is zero.
    Its fixed 0.07/0.20/0.10 m jump landmarks are retained; crouch/settle endpoints
    interpolate to the scenario's neutral l0 instead of the demo's 0.12 m.
    """
    l0, jump_at = robot["l0"], robot["jump_at_s"]
    if jump_at is None:
        return SyntheticPose("STAND", l0, 0.0, 3, False, False)
    pose = synthetic_pose(t, jump_at)
    length = pose.leg_length_m
    if pose.phase == "STAND":
        length = l0
    elif pose.phase == "CROUCH":
        length = 0.07 + (length - 0.07) * (l0 - 0.07) / (0.12 - 0.07)
    elif pose.phase == "SETTLE":
        length = LAND_L0 + (length - LAND_L0) * (l0 - LAND_L0) / (0.12 - LAND_L0)
    return replace(pose, leg_length_m=length)


def expected_at(scenario, t):
    """Half-open intervals; include the scenario endpoint in an ending interval."""
    end = scenario["robot"]["duration_s"]
    for interval in scenario.get("expected", []):
        if (interval["t_from"] <= t < interval["t_to"]
                or t == end == interval["t_to"]):
            return interval["advisory"]
    return None


def advisory_match(expected, actual):
    """Blank means no expected label; False remains a real, reportable mismatch."""
    if expected is None:
        return ""
    actual = actual.name if isinstance(actual, decide.Advisory) else actual
    return actual == expected


class PerceptionLoop:
    """Stateful integration, independent of rendering and transport.

    Always run the unmodified detector, including during flight. Clear its plane
    cache and decision EMA/votes only on crouch entry/first tick after flight.
    Recovery therefore requires fresh post-impact geometry and confirmation.
    """

    def __init__(self, K=None):
        self.K = intrinsics() if K is None else K
        self.detector_state = {}
        self.decision_state = {}
        self.gate = JumpGate()
        self.previous_pose = None

    def update(self, t, depth, pose, speed):
        first_crouch = pose.phase == "CROUCH" and self.previous_pose != "CROUCH"
        first_landing = self.previous_pose == "FLIGHT" and pose.phase != "FLIGHT"
        reset = first_crouch or first_landing
        if reset:
            self.detector_state, self.decision_state = {}, {}
        frame, self.detector_state = detect.process_frame(
            depth, self.K, state=self.detector_state)
        decision, self.decision_state = decide.update(
            frame, self.decision_state, forward_speed_m_s=speed)
        tick = Tick(
            t=float(t), jump_status=pose.jump_status,
            airborne_l=pose.airborne_l, airborne_r=pose.airborne_r,
            telemetry_age_s=0.0, pitch_rate_deg_s=0.0, roll_rate_deg_s=0.0,
            plane_valid=frame.plane_valid, raw_advisory=decision.advisory,
            raw_confidence=decision.confidence, plane_stamp_s=float(t))
        gated = self.gate.update(tick)
        self.previous_pose = pose.phase
        return frame, decision, gated, reset


def place_robot(model, data, robot, t, pose):
    """Place animated legs/body first, then translate every mocap part together."""
    import mujoco
    set_pose(model, data, pose.leg_length_m, 0.0, pitch=0.0,
             vertical_offset=pose.vertical_offset_m)
    robot_x = robot["start_x"] + robot["speed_m_s"] * t
    data.mocap_pos[:, 0] += robot_x
    data.time = t
    mujoco.mj_forward(model, data)
    return robot_x


def update_camera(renderer, data, K, near):
    renderer.update_scene(data, camera=CAMERA_NAME)
    # fovy supplies the right focal scale. Correct the small off-centre CX/CY
    # through the GL frustum so the detector uses synth.py's exact K, with no
    # image interpolation across depth discontinuities. Update resets these.
    projection = pinhole_frustum(K, WIDTH, HEIGHT, near)
    for camera in renderer.scene.camera:
        for field, value in projection.items():
            setattr(camera, field, value)


def camera_measurement(data, camera_id):
    forward = -np.asarray(data.cam_xmat[camera_id]).reshape(3, 3)[:, 2]
    pitch = math.degrees(math.atan2(-forward[2], math.hypot(*forward[:2])))
    return float(data.cam_xpos[camera_id, 2]), pitch


def timeline_row(scenario, t, robot_x, camera_height, camera_pitch, pose,
                 frame, decision, gated, reset, noise):
    expected = expected_at(scenario, t)
    return dict(
        t=t, robot_x=robot_x, camera_height_m=camera_height,
        camera_pitch_deg=camera_pitch, raw_advisory=decision.advisory.name,
        height_m=decision.height_m, distance_m=decision.distance_m,
        plane_valid=frame.plane_valid, gated_advisory=gated.advisory.name,
        phase=gated.phase.value, expected_advisory=expected or "",
        match=advisory_match(expected, gated.advisory),
        raw_match=advisory_match(expected, decision.advisory),
        raw_confidence=decision.confidence, gated_confidence=gated.confidence,
        synthetic_phase=pose.phase, gate_reason=gated.reason,
        plane_reused=frame.plane_reused, plane_age_frames=frame.plane_age,
        history_reset=reset, body_pitch_deg=0.0, leg_length_m=pose.leg_length_m,
        vertical_offset_m=pose.vertical_offset_m, scene_status=scenario["status"],
        depth_status=f"ASSUMED stereo sigma_d={noise:g}px" if noise else "IDEAL rendered depth",
        phase_source="SYNTHETIC wbr_pose timeline")


def record_frame(rgb, depth, row, path):
    from PIL import Image, ImageDraw
    image = Image.fromarray(rgb).convert("RGB")
    # Bright = near, black = invalid. Fixed 0..6 m scale across all frames.
    valid = np.isfinite(depth) & (depth > 0)
    gray = np.where(valid, 255 * (1 - np.clip(depth / 6.0, 0, 1)), 0).astype(np.uint8)
    inset = Image.fromarray(gray).convert("RGB").resize((192, 144))
    image.paste(inset, (WIDTH - 200, HEIGHT - 152))
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, WIDTH, 100), fill="white")
    text = (
        f"t={row['t']:.2f}s x={row['robot_x']:.3f}m  SYNTHETIC {row['synthetic_phase']}\n"
        f"raw {row['raw_advisory']} -> {row['gated_advisory']} ({row['phase']})\n"
        f"height={row['height_m']:.3f}m gap={row['distance_m']:.3f}m plane={row['plane_valid']}\n"
        f"camera H={row['camera_height_m']:.3f}m pitch down={row['camera_pitch_deg']:.1f}deg\n"
        f"{row['scene_status']} | {row['depth_status']} | advisory only")
    draw.multiline_text((8, 6), text, fill="black", spacing=3)
    draw.text((WIDTH - 198, HEIGHT - 168), "Optical depth: bright=near", fill="black")
    image.save(path)


def run(scenario, *, out="timeline.csv", record=None, noise=0.0, seed=0):
    import mujoco
    if not math.isfinite(noise) or noise < 0:
        raise ValueError("noise must be a finite nonnegative disparity standard deviation")
    record = Path(record) if record is not None else None
    if record is not None:
        record.mkdir(parents=True, exist_ok=True)
        if any(record.iterdir()):
            raise ValueError(f"record directory must be empty: {record}")
    model = mujoco.MjModel.from_xml_string(build_xml(scenario))
    data = mujoco.MjData(model)
    near, far = clipping_distances(model.vis.map.znear, model.vis.map.zfar, model.stat.extent)
    camera_id = model.camera(CAMERA_NAME).id
    K, loop, rng = intrinsics(), PerceptionLoop(), np.random.default_rng(seed)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    # Construct the renderer before opening the CSV, preserving previous results
    # when the graphics context is unavailable. Errors propagate to the caller.
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        with out.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            count = 0
            for index, t in enumerate(tick_times(scenario["robot"]["duration_s"])):
                pose = pose_at(t, scenario["robot"])
                robot_x = place_robot(model, data, scenario["robot"], t, pose)
                update_camera(renderer, data, K, near)
                renderer.enable_depth_rendering()
                depth = optical_depth(renderer.render(), near, far)
                if noise:
                    depth = stereo_noise(depth, sigma_d=noise, fx=K[0, 0], rng=rng,
                                         near=near, far=far)
                frame, decision, gated, reset = loop.update(
                    t, depth, pose, scenario["robot"]["speed_m_s"])
                height, pitch = camera_measurement(data, camera_id)
                row = timeline_row(scenario, t, robot_x, height, pitch, pose,
                                   frame, decision, gated, reset, noise)
                writer.writerow(row)
                stream.flush()  # Preserve completed rows if rendering later fails.
                if record is not None:
                    renderer.disable_depth_rendering()
                    record_frame(renderer.render(), depth, row, record / f"frame_{index:05d}.png")
                count += 1
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", type=Path)
    parser.add_argument("--record", type=Path, metavar="OUT", help="empty directory for annotated PNGs")
    parser.add_argument("--out", type=Path, default=Path("timeline.csv"))
    parser.add_argument("--noise", type=float, default=0.0, metavar="SIGMA_D_PX",
                        help="ASSUMED disparity sigma in pixels (default: ideal depth)")
    parser.add_argument("--seed", type=int, default=0, help="noise RNG seed")
    args = parser.parse_args(argv)
    try:
        scenario = load_scenario(args.scenario)
        count = run(scenario, out=args.out, record=args.record, noise=args.noise, seed=args.seed)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(f"Wrote {count} detector ticks to {args.out}; scene={scenario['status']}; phases SYNTHETIC")
    if args.record is not None:
        print(f"Wrote {count} annotated camera/depth PNGs to {args.record}")


if __name__ == "__main__":
    main()
