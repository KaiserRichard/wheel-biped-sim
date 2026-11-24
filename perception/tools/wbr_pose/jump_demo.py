"""ASSUMED kinematic jump illustration; no dynamics, forces, or contact solver."""

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np

from .model import make_model, set_pose

# All cycle timing and flight parameters below are ASSUMED for a readable demo.
STAND_S = 1.00
CROUCH_S = 0.40
PUSH_S = 0.12
FLIGHT_S = 0.55
SETTLE_S = 0.40
LAND_L0 = 0.10
GRAVITY = 9.81


def _smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


def state_at(t):
    """Return phase, l0, flight vertical offset for continuous piecewise trajectory."""
    t = max(0.0, t)
    t1 = STAND_S
    t2 = t1 + CROUCH_S
    t3 = t2 + PUSH_S
    t4 = t3 + FLIGHT_S
    t5 = t4 + 0.04  # ASSUMED short landing/compression display interval.
    t6 = t5 + SETTLE_S
    if t < t1:
        return "STAND", 0.12, 0.0
    if t < t2:
        u = (t - t1) / CROUCH_S
        return "CROUCH", 0.12 + (0.07 - 0.12) * _smooth(u), 0.0
    if t < t3:
        u = (t - t2) / PUSH_S
        return "PUSH", 0.07 + (0.20 - 0.07) * _smooth(u), 0.0
    if t < t4:
        u = (t - t3) / FLIGHT_S
        # Ballistic arc with zero height and equal launch/landing vertical speed magnitude.
        peak = GRAVITY * FLIGHT_S**2 / 8
        z = 4 * peak * u * (1 - u)
        return "FLIGHT", 0.20 + (LAND_L0 - 0.20) * _smooth(u), z
    if t < t5:
        return "LAND", LAND_L0, 0.0
    if t < t6:
        u = (t - t5) / SETTLE_S
        return "SETTLE", LAND_L0 + (0.12 - LAND_L0) * _smooth(u), 0.0
    return "STAND", 0.12, 0.0


def total_time():
    return STAND_S + CROUCH_S + PUSH_S + FLIGHT_S + 0.04 + SETTLE_S


def render_frame(model, data, renderer, t):
    from PIL import Image, ImageDraw
    phase, l0, z = state_at(t)
    set_pose(model, data, l0, 0.0, vertical_offset=z)
    renderer.update_scene(data, camera="demo")
    image = Image.fromarray(renderer.render())
    ImageDraw.Draw(image).rectangle((0, 0, 310, 48), fill="white")
    ImageDraw.Draw(image).text((10, 8), f"{phase}   t={t:.2f}s   l0={l0:.3f}m", fill="black")
    return image


def software_frame(model, data, t):
    """Orthographic Pillow fallback using the MuJoCo forward-kinematics geom poses."""
    from PIL import Image, ImageDraw
    import mujoco
    from sim.models import make_wbr_3d_simple as gen
    phase, l0, z = state_at(t)
    set_pose(model, data, l0, 0.0, vertical_offset=z)
    image = Image.new("RGB", (640, 480), (245, 247, 249))
    draw = ImageDraw.Draw(image)
    scale, ground = 620.0, 420.0
    def point(p):
        return (int(320 + p[0] * scale), int(ground - p[2] * scale))
    draw.line((0, ground, 640, ground), fill=(70, 75, 80), width=3)
    # Draw links from MuJoCo capsule geom centers/orientations and their source lengths.
    link_colors = {"upper": (157, 153, 51), "lower": (64, 166, 166)}
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid]) or ""
        if "upper_" in name or "lower_" in name:
            length = gen.L_UP if "upper_" in name else gen.L_LOW
            center, rot = data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3)
            a = center - rot[:, 2] * (length / 2)
            b = center + rot[:, 2] * (length / 2)
            color = link_colors["upper" if "upper_" in name else "lower"]
            draw.line((*point(a), *point(b)), fill=color, width=13)
            draw.line((*point(a), *point(b)), fill=(45, 48, 50), width=2)
            for endpoint in (a, b):
                px, py = point(endpoint)
                draw.ellipse((px-4, py-4, px+4, py+4), fill=(25, 28, 30))
    # Outline the body box using transformed corners from its MuJoCo geom pose.
    bid = model.body("body").id
    gid = next(g for g in range(model.ngeom) if model.geom_bodyid[g] == bid)
    center, rot = data.geom_xpos[gid], data.geom_xmat[gid].reshape(3, 3)
    hx, hy, hz = gen.BODY_HALF
    corners = [center + rot @ [sx*hx, sy*hy, sz*hz]
               for sx, sy, sz in ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                  (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))]
    for a, b in ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)):
        draw.line((*point(corners[a]), *point(corners[b])), fill=(92, 98, 107), width=5)
    # Wheels: the viewed cross-section is circular; center and radius are source geometry.
    for side in ("left", "right"):
        body = model.body(f"wheel_{side}").id
        center = data.xpos[body]
        p = point(center)
        r = int(gen.R_WHEEL * scale)
        draw.ellipse((p[0]-r, p[1]-r, p[0]+r, p[1]+r), fill=(36, 38, 40), outline=(15, 16, 18), width=3)
    draw.rectangle((8, 8, 330, 42), fill="white")
    draw.text((16, 17), f"{phase}   t={t:.2f}s   l0={l0:.3f}m", fill="black")
    return image


def record(out, fps):
    from PIL import Image
    import mujoco
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    model, data = make_model(), None
    data = mujoco.MjData(model)
    old_unraisablehook = sys.unraisablehook
    def construction_hook(unraisable):
        # MuJoCo 3.3.7's partially initialized Renderer destructor raises here
        # after a CoreGraphics context failure; keep other unraisable errors visible.
        if not (isinstance(unraisable.exc_value, AttributeError)
                and "Renderer.__del__" in str(unraisable.object)):
            old_unraisablehook(unraisable)
    sys.unraisablehook = construction_hook
    try:
        try:
            renderer = mujoco.Renderer(model, 640, 480)
        except Exception as exc:
            print(f"MuJoCo offscreen renderer unavailable ({exc}); using Pillow projection of MuJoCo geom poses")
            renderer = None
    finally:
        sys.unraisablehook = old_unraisablehook
    times = np.arange(0, total_time() + 1e-9, 1 / fps)
    frames = []
    for idx, t in enumerate(times):
        frame = render_frame(model, data, renderer, float(t)) if renderer else software_frame(model, data, float(t))
        key_times = (0.50, 1.46, 1.80)
        if idx in tuple(round(key * fps) for key in key_times):
            phase, _, _ = state_at(float(t))
            frame.save(out / f"{phase.lower()}_key.png")
        frames.append(frame.resize((480, 360)).convert("P", palette=Image.Palette.ADAPTIVE))
    frames[0].save(out / "jump.gif", save_all=True, append_images=frames[1:],
                   duration=round(1000 / fps), loop=0, optimize=True)
    if renderer:
        renderer.close()
    print(f"recorded {len(frames)} frames to {out}")


def show(fps):
    import mujoco
    import mujoco.viewer
    model, data = make_model(), None
    data = mujoco.MjData(model)
    start = time.monotonic()
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        viewer.cam.fixedcamid = model.camera("demo").id
        while viewer.is_running():
            t = (time.monotonic() - start) % total_time()
            phase, l0, z = state_at(t)
            set_pose(model, data, l0, 0.0, vertical_offset=z)
            viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, phase, f"l0 = {l0:.3f} m"))
            viewer.sync()
            time.sleep(1 / fps)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", metavar="OUT", help="headless GIF and key PNGs")
    parser.add_argument("--fps", type=float, default=20)
    args = parser.parse_args(argv)
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.record:
        record(args.record, args.fps)
    else:
        show(args.fps)


if __name__ == "__main__":
    main()
