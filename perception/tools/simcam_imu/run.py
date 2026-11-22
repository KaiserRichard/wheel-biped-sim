"""Replay measured bench IMU tilt through simcam and compare with zero body tilt."""

import argparse
import csv
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

from tools.imu_view.view import axle_height
from tools.simcam import run as simcam
from tools.simcam.depth import clipping_distances, optical_depth
from tools.simcam.scenario import (CAMERA_NAME, HEIGHT, WIDTH, build_xml,
                                  intrinsics, load_scenario)
from tools.wbr_pose.model import set_pose
from .tilt_source import IMU_STATUS, Tilt, TiltSource

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENARIO = ROOT / "docs/simcam/example_box132.json"
TILT_FIELDS = (
    "body_roll_deg", "pitch_rate_deg_s", "roll_rate_deg_s", "imu_elapsed_s",
    "imu_timestamp_s", "imu_scale", "imu_status", "run_kind", "axle_height_m",
    "detector_height_m", "detector_distance_m", "obstacle_detected",
    "reference_height_m", "height_error_m", "abs_height_error_m",
    "before_obstacle_face",
)
CSV_FIELDS = (*simcam.CSV_FIELDS, *TILT_FIELDS)


class RateGate:
    """Adapt simcam's zero-rate Tick without duplicating its perception loop."""

    def __init__(self, gate):
        self.gate = gate
        self.pitch_rate = self.roll_rate = 0.0

    def update(self, tick):
        return self.gate.update(replace(tick, pitch_rate_deg_s=self.pitch_rate,
                                        roll_rate_deg_s=self.roll_rate))


class TiltPerceptionLoop(simcam.PerceptionLoop):
    def __init__(self, K=None):
        super().__init__(K)
        self.gate = RateGate(self.gate)

    def update(self, t, depth, pose, speed, tilt):
        self.gate.pitch_rate = tilt.pitch_rate_deg_s
        self.gate.roll_rate = tilt.roll_rate_deg_s
        return super().update(t, depth, pose, speed)


def place_robot(model, data, robot, t, pose, tilt):
    """Use wbr_pose's exact axle pivot and finite-cylinder floor clearance.

    All mocap parts rotate together; the camera remains a child of body. The
    synthetic jump vertical offset is added after wheel clearance, as in simcam.
    """
    import mujoco
    set_pose(model, data, pose.leg_length_m, 0.0,
             pitch=tilt.pitch_rad, roll=tilt.roll_rad,
             vertical_offset=pose.vertical_offset_m)
    x = robot["start_x"] + robot["speed_m_s"] * t
    data.mocap_pos[:, 0] += x
    data.time = t
    mujoco.mj_forward(model, data)
    return x


def reference_height(scenario):
    """A single flat-top target has a known assumed height; other scenes don't."""
    obstacles = scenario["obstacles"]
    if len(obstacles) == 1 and obstacles[0]["type"] in ("box", "step"):
        return float(obstacles[0]["height"])
    return math.nan


def make_row(scenario, source, imu_start, t, x, camera_height, camera_pitch,
             pose, tilt, frame, decision, gated, reset, zero_tilt=False):
    row = simcam.timeline_row(scenario, t, x, camera_height, camera_pitch, pose,
                             frame, decision, gated, reset, 0.0)
    target = reference_height(scenario)
    nearest = frame.nearest
    detected_height = float(nearest.height_m) if nearest is not None else math.nan
    detected_distance = float(nearest.distance_m) if nearest is not None else math.nan
    # A reused floor is explicitly invalid for the per-frame-fit hypothesis.
    error = detected_height - target if frame.plane_valid else math.nan
    obstacles = scenario["obstacles"]
    before_face = ""
    if len(obstacles) == 1 and obstacles[0]["type"] in ("box", "step"):
        # Necessary but insufficient for collision-free body/camera geometry.
        before_face = x < obstacles[0]["x_face"]
    row.update(
        body_pitch_deg=math.degrees(tilt.pitch_rad),
        body_roll_deg=math.degrees(tilt.roll_rad),
        pitch_rate_deg_s=tilt.pitch_rate_deg_s, roll_rate_deg_s=tilt.roll_rate_deg_s,
        imu_elapsed_s=imu_start + t, imu_timestamp_s=source.timestamp_origin_s + imu_start + t,
        imu_scale=source.scale, imu_status=IMU_STATUS,
        run_kind="zero_tilt" if zero_tilt else "imu_tilt",
        axle_height_m=axle_height(tilt.pitch_rad, tilt.roll_rad) + pose.vertical_offset_m,
        detector_height_m=detected_height, detector_distance_m=detected_distance,
        obstacle_detected=nearest is not None, reference_height_m=target,
        height_error_m=error, abs_height_error_m=abs(error),
        before_obstacle_face=before_face,
        phase_source="SYNTHETIC jump/contact; rates from filtered IMU" if not zero_tilt
                     else "SYNTHETIC jump/contact; zero attitude rates",
    )
    return row


COMPARISON_FIELDS = (
    *CSV_FIELDS, *(f"zero_{key}" for key in CSV_FIELDS if key != "t"),
    "detector_height_delta_m", "height_error_delta_m", "abs_height_error_delta_m",
    "raw_advisory_agreement", "gated_advisory_agreement", "both_planes_valid",
)


def comparison_join(tilted, zero):
    """Exact one-to-one time join; never hide a missing or duplicate tick.

    Error deltas are tilted minus zero, and remain NaN when either run lacks
    current, valid-plane target geometry. Agreement includes UNCERTAIN/NONE.
    """
    def index(rows):
        result = {}
        for row in rows:
            t = float(row["t"])
            if not math.isfinite(t) or t in result:
                raise ValueError("comparison requires finite, unique timestamps")
            result[t] = row
        return result

    left, right = index(tilted), index(zero)
    if left.keys() != right.keys():
        raise ValueError("comparison runs must have identical timestamps")
    result = []
    for t in sorted(left):
        a, b = left[t], right[t]
        row = dict(a)
        row.update({f"zero_{key}": value for key, value in b.items() if key != "t"})
        row.update(
            detector_height_delta_m=a["detector_height_m"] - b["detector_height_m"],
            height_error_delta_m=a["height_error_m"] - b["height_error_m"],
            abs_height_error_delta_m=a["abs_height_error_m"] - b["abs_height_error_m"],
            raw_advisory_agreement=a["raw_advisory"] == b["raw_advisory"],
            gated_advisory_agreement=a["gated_advisory"] == b["gated_advisory"],
            both_planes_valid=a["plane_valid"] and b["plane_valid"],
        )
        result.append(row)
    return result


def write_csv(path, rows, fields):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def output_paths(out):
    out = Path(out)
    return dict(tilted=out, zero=out.with_name(f"{out.stem}_zero_tilt.csv"),
                comparison=out.with_name(f"{out.stem}_comparison.csv"),
                metadata=out.with_name(f"{out.stem}_metadata.json"))


def record_frame(rgb, depth, row, path):
    """Keep simcam's camera/depth annotation and add measured replay provenance."""
    from PIL import Image, ImageDraw
    simcam.record_frame(rgb, depth, row, path)
    with Image.open(path) as image:
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 102, WIDTH, 145), fill="white")
        draw.multiline_text((8, 104),
                            f"{row['run_kind']} | IMU MEASURED bench by hand; pose ASSUMED\n"
                            f"IMU t={row['imu_elapsed_s']:.2f}s scale={row['imu_scale']:g} "
                            f"body pitch={row['body_pitch_deg']:+.2f} roll={row['body_roll_deg']:+.2f} deg",
                            fill="black")
        image.save(path)


def render_run(scenario, source, *, imu_start=0.0, zero_tilt=False, out, record=None):
    """The only renderer entry point. Never called by non-rendering tests."""
    import mujoco
    model = mujoco.MjModel.from_xml_string(build_xml(scenario))
    data = mujoco.MjData(model)
    near, far = clipping_distances(model.vis.map.znear, model.vis.map.zfar, model.stat.extent)
    camera_id = model.camera(CAMERA_NAME).id
    K, loop = intrinsics(), TiltPerceptionLoop()
    rows = []
    # Fail before opening CSV if the graphics context is unavailable.
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        with Path(out).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for index, t in enumerate(simcam.tick_times(scenario["robot"]["duration_s"])):
                tilt = Tilt(0.0, 0.0, 0.0, 0.0) if zero_tilt else source.at(imu_start + t)
                pose = simcam.pose_at(t, scenario["robot"])
                x = place_robot(model, data, scenario["robot"], t, pose, tilt)
                simcam.update_camera(renderer, data, K, near)
                renderer.enable_depth_rendering()
                depth = optical_depth(renderer.render(), near, far)
                frame, decision, gated, reset = loop.update(
                    t, depth, pose, scenario["robot"]["speed_m_s"], tilt)
                height, pitch = simcam.camera_measurement(data, camera_id)
                row = make_row(scenario, source, imu_start, t, x, height, pitch, pose,
                               tilt, frame, decision, gated, reset, zero_tilt)
                writer.writerow(row)
                stream.flush()
                rows.append(row)
                if record is not None:
                    renderer.disable_depth_rendering()
                    record_frame(renderer.render(), depth, row,
                                 Path(record) / f"frame_{index:05d}.png")
    return rows


def run(scenario, source, *, imu_start=0.0, out="timeline.csv", record=None):
    source.require_interval(imu_start, scenario["robot"]["duration_s"])
    paths = output_paths(out)
    record = Path(record) if record is not None else None
    if record is not None:
        record.mkdir(parents=True, exist_ok=True)
        if any(record.iterdir()):
            raise ValueError(f"record directory must be empty: {record}")
        for name in ("imu_tilt", "zero_tilt"):
            (record / name).mkdir()
    tilted = render_run(scenario, source, imu_start=imu_start, out=paths["tilted"],
                        record=record / "imu_tilt" if record is not None else None)
    zero = render_run(scenario, source, imu_start=imu_start, zero_tilt=True, out=paths["zero"],
                      record=record / "zero_tilt" if record is not None else None)
    comparison = comparison_join(tilted, zero)
    write_csv(paths["comparison"], comparison, COMPARISON_FIELDS)
    metadata = dict(source.metadata(), imu_start_s=imu_start, scenario=scenario,
                    imu_csv_sha256=hashlib.sha256(source.path.read_bytes()).hexdigest(),
                    outputs={key: str(path) for key, path in paths.items()},
                    depth_status="IDEAL rendered optical depth; no stereo noise",
                    gate_contact_status="ASSUMED from synthetic jump; not measured wheel contact",
                    detector_hz=simcam.DETECTOR_HZ)
    paths["metadata"].write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return comparison


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", nargs="?", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--imu-csv", type=Path, required=True)
    parser.add_argument("--imu-start", type=float, default=0.0, help="elapsed CSV seconds")
    parser.add_argument("--bias-window", nargs=2, type=float, default=(0.0, 2.0),
                        metavar=("START", "END"), help="elapsed CSV seconds, assumed still")
    parser.add_argument("--scale", type=float, default=1.0, help="multiply pitch/roll and rates")
    parser.add_argument("--record", type=Path, metavar="OUT", help="empty directory for paired PNGs")
    parser.add_argument("--out", type=Path, default=Path("timeline.csv"))
    args = parser.parse_args(argv)
    try:
        scenario = load_scenario(args.scenario)
        source = TiltSource(args.imu_csv, bias_window=args.bias_window, scale=args.scale)
        rows = run(scenario, source, imu_start=args.imu_start, out=args.out, record=args.record)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    print(f"Wrote {len(rows)} paired ticks: " + ", ".join(map(str, output_paths(args.out).values())))


if __name__ == "__main__":
    main()
