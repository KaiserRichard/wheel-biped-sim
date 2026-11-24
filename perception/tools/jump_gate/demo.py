"""Recorded obstacle advisory + SYNTHETIC kinematic jump, research only.

MuJoCo rendering is deliberately separate from --csv-only replay/export.
"""

import argparse
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from tools.scene_twin.replay import BOX_DEPTH, BOX_WIDTH, COLORS
from .timeline import (ASSUMED_DEFAULT_JUMP_AT_S, DEFAULT_PACKETS, export_csv,
                       load_timeline)


ASSUMED_DISPLAY_FPS = 10.0
ASSUMED_FRAME_WIDTH, ASSUMED_FRAME_HEIGHT = 960, 640


def build_xml(replay):
    from tools.wbr_pose.model import build_xml as pose_xml
    root = ET.fromstring(pose_xml())
    world = root.find("worldbody")
    world.find("geom[@name='floor']").set("size", "4 4 .01")
    ET.SubElement(world, "geom", name="obstacle", type="box",
                  pos=f"{replay.face + BOX_DEPTH / 2} 0 {replay.height / 2}",
                  size=f"{BOX_DEPTH / 2} {BOX_WIDTH / 2} {replay.height / 2}",
                  rgba="0.5 0.5 0.5 1")
    return ET.tostring(root, encoding="unicode")


def make_scene(replay):
    import mujoco
    model = mujoco.MjModel.from_xml_string(build_xml(replay))
    return model, mujoco.MjData(model)


def apply(model, data, row):
    from tools.wbr_pose.model import set_pose
    pose = row.pose
    set_pose(model, data, pose.leg_length_m, 0.0, vertical_offset=pose.vertical_offset_m)
    model.geom("obstacle").rgba[:] = COLORS[int(row.output.advisory)]


def overlay(row):
    tick, out = row.inputs, row.output
    return (
        f"t={tick.t:.2f}s  raw={tick.raw_advisory.name} -> gated={out.advisory.name}  "
        f"phase={out.phase.value}  reason={out.reason}\n"
        f"SYNTHETIC {row.pose.phase}: status={tick.jump_status}, "
        f"airborne L/R={int(tick.airborne_l)}/{int(tick.airborne_r)}; "
        f"confidence raw={tick.raw_confidence:.2f} ASSUMED / gated={out.confidence:.2f}\n"
        f"source frame_stamp_ns={row.frame_stamp_ns}; "
        f"observation age={tick.t - tick.plane_stamp_s:.3f}s\n"
        "RESEARCH ONLY | contact, rates, freshness, plane validity SYNTHETIC/ASSUMED | STM32 owns authority"
    )


def camera(replay):
    from tools.scene_twin.scene import camera as replay_camera
    cam = replay_camera(replay)
    # ASSUMED framing includes the existing scripted ballistic arc (~0.37 m).
    cam.lookat[2] = 0.32
    cam.distance = max(1.8, cam.distance)
    cam.elevation = -20
    return cam


def record(args, timeline, model, data):
    import mujoco
    from PIL import Image, ImageDraw
    out = Path(args.record)
    out.mkdir(parents=True, exist_ok=True)
    if any(out.iterdir()):
        raise ValueError(f"record directory must be empty: {out}")
    export_csv(timeline, out / "timeline.csv")
    frames, cam = [], camera(timeline.replay)
    with mujoco.Renderer(model, height=ASSUMED_FRAME_HEIGHT,
                         width=ASSUMED_FRAME_WIDTH) as renderer:
        for k in range(math.ceil(timeline.replay.duration * args.fps) + 1):
            t = min(k / args.fps, timeline.replay.duration)
            row = timeline.at(t)
            apply(model, data, row)
            renderer.update_scene(data, camera=cam)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, ASSUMED_FRAME_WIDTH, 90), fill="white")
            draw.multiline_text((8, 6), overlay(row), fill="black", spacing=4)
            frame.save(out / f"frame_{k:05d}.png")
            frames.append(frame.convert("P", palette=Image.Palette.ADAPTIVE))
    frames[0].save(out / "jump_gate.gif", save_all=True, append_images=frames[1:],
                   duration=max(10, round(1000 / args.fps / 10) * 10), loop=0)
    print(f"Recorded {len(frames)} frames, jump_gate.gif, timeline.csv in {out}")


def window(args, timeline, model, data):
    import mujoco
    import mujoco.viewer
    cam = camera(timeline.replay)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False,
                                     show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = cam.lookat
        viewer.cam.distance, viewer.cam.azimuth = cam.distance, cam.azimuth
        viewer.cam.elevation = cam.elevation
        start = time.monotonic()
        while viewer.is_running():
            t = min(time.monotonic() - start, timeline.replay.duration)
            row = timeline.at(t)
            with viewer.lock():
                apply(model, data, row)
            viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, overlay(row), ""))
            viewer.sync()
            if t >= timeline.replay.duration:
                break
            time.sleep(1 / args.fps)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, metavar="OUT")
    parser.add_argument("--fps", type=float, default=ASSUMED_DISPLAY_FPS)
    parser.add_argument("--jump-at", type=float, default=ASSUMED_DEFAULT_JUMP_AT_S)
    parser.add_argument("--packets", type=Path, default=DEFAULT_PACKETS)
    parser.add_argument("--csv-only", type=Path, metavar="CSV",
                        help="export timeline without importing MuJoCo or rendering")
    args = parser.parse_args(argv)
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be positive and finite")
    if args.csv_only and args.record:
        parser.error("--csv-only and --record are mutually exclusive")
    try:
        timeline = load_timeline(args.packets, args.jump_at)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.csv_only:
        path = export_csv(timeline, args.csv_only)
        print(f"Exported {len(timeline.rows)} gate ticks to {path}; all injected inputs SYNTHETIC")
        return
    model, data = make_scene(timeline.replay)
    (record if args.record else window)(args, timeline, model, data)


if __name__ == "__main__":
    main()
