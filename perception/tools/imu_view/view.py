"""Tilt the imu_sensor by hand, watch the 3D WBR tilt the same way.

  .venv/bin/mjpython -m tools.imu_view.view --fake
  .venv/bin/mjpython -m tools.imu_view.view --replay run.csv
  ssh pi-host 'stty -F /dev/ttyAMA0 115200 raw -echo cs8 -parenb -cstopb && cat /dev/ttyAMA0' \
      | .venv/bin/mjpython -m tools.imu_view.view --stdin --save run.csv
  .venv/bin/python -m tools.imu_view.view --fake --record out/ --seconds 12   # headless PNG/GIF

Solid robot = complementary filter, blue ghost = accelerometer only,
orange ghost = gyro integration only. Visualisation only: no balance, no dynamics.
"""

import argparse
import math
from pathlib import Path
import time

import mujoco

from tools.imu_view import stream
from sim.models import make_wbr_3d_simple as gen
from tools.imu_view.filter import Madgwick, TiltFilters, quat_from_pitch_roll

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "sim/models/wbr_3d_simple.xml"


class Pipeline:
    """Samples -> (optional) gyro bias estimate -> filters -> mocap quaternions."""

    def __init__(self, model, axes, tau_s, calib_s, beta):
        self.axes = axes
        self.filters = TiltFilters(tau_s)
        self.madgwick = Madgwick(beta)
        self.calib_s = calib_s
        self.bias = (0.0, 0.0, 0.0)
        self._calib = []
        self.last_t = None
        self.status = "starting"
        self.line_stats = None
        self.mocap = {n: model.body(n).mocapid[0] for n in ("robot", "ghost_acc", "ghost_gyro")}

    def push(self, raw):
        s = stream.remap(raw, self.axes)
        if self.calib_s > 0:
            self._calib.append(s)
            elapsed = s.t - self._calib[0].t
            if elapsed < self.calib_s:
                self.status = f"HOLD STILL: measuring gyro bias ({elapsed:.1f}/{self.calib_s:.1f} s)"
                return False
            n = len(self._calib)
            self.bias = tuple(sum(c.gyro[i] for c in self._calib) / n for i in range(3))
            self.calib_s = 0
        gyro = tuple(g - b for g, b in zip(s.gyro, self.bias))
        dt = 0.01 if self.last_t is None else min(max(s.t - self.last_t, 1e-4), 0.1)
        self.last_t = s.t
        self.filters.update(s.accel, gyro, dt)
        self.madgwick.update(s.accel, gyro, dt)
        self.status = "running"
        self.truth = getattr(raw, "truth", None)
        return True

    def apply(self, data):
        f = self.filters
        for name, est in (("robot", f.comp), ("ghost_acc", f.acc), ("ghost_gyro", f.gyro)):
            i = self.mocap[name]
            data.mocap_quat[i] = quat_from_pitch_roll(est.pitch, est.roll)
            if data.mocap_pos[i][2] > -1:   # hidden ghosts stay hidden
                data.mocap_pos[i][2] = axle_height(est.pitch, est.roll)

    def text(self):
        f, m = self.filters, self.madgwick.pitch_roll()
        d = math.degrees
        rows = [("complementary (solid)", f.comp), ("accel only (blue)", f.acc),
                ("gyro only (orange)", f.gyro), ("Madgwick (not drawn)", m)]
        left = "\n".join(r[0] for r in rows) + "\ngyro bias (deg/s)"
        right = "\n".join(f"pitch {d(e.pitch):+6.1f}  roll {d(e.roll):+6.1f}" for _, e in rows)
        right += "\n" + " ".join(f"{d(b):+.2f}" for b in self.bias)
        if getattr(self, "truth", None):
            left += "\nfake truth"
            right += f"\npitch {d(self.truth[0]):+6.1f}  roll {d(self.truth[1]):+6.1f}"
        left += "\n" + ("" if self.status == "running" else self.status)
        if self.line_stats is not None and self.line_stats.bad:
            left += f"\nUART: {self.line_stats.bad} bad lines (check stty 8N1)"
        return left + "\npitch>0 nose down, roll>0 left up; yaw not shown", right


def axle_height(pitch, roll):
    """Axle-centre height that keeps the LOWER wheel touching the floor.

    Pitch alone rolls the wheels (axle stays at r). Roll on flat ground cannot keep both
    wheels down with fixed legs, so the robot tips onto one wheel and the other lifts.
    Only the z part of the world wheel axis matters: a_z = cos(pitch) sin(roll).
    """
    a_z = math.cos(pitch) * math.sin(roll)
    lowest_rim = gen.R_WHEEL * math.sqrt(max(0.0, 1 - a_z * a_z)) + gen.WHEEL_HALF_WIDTH * abs(a_z)
    return gen.Y_WHEEL * abs(a_z) + lowest_rim


def axes_det(axes):
    """+1 for a rotation, -1 for a mirror. Mirrors flip the gyro (a pseudovector) against the accel."""
    m = [[0.0] * 3 for _ in range(3)]
    for row, spec in enumerate(axes):
        m[row]["xyz".index(spec[-1])] = -1.0 if spec.startswith("-") else 1.0
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def source(a):
    if a.fake:
        held = stream.fake(seconds=a.seconds, still_s=a.calib_s)
        return stream.paced(held) if not a.record else held
    if a.replay:
        rows = stream.from_csv(a.replay)
        return stream.paced(rows) if not a.record else rows
    a.line_stats = stream.LineStats()
    return stream.from_stdin(a.line_stats)


def run_viewer(a, model, data, pipe, samples, writer):
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        v.cam.fixedcamid = model.camera(a.camera).id
        last_draw = 0.0
        for s in samples:
            if not v.is_running():
                break
            if writer:
                writer.write(s)
            pipe.push(s)
            now = time.monotonic()
            if now - last_draw < 1 / 60:   # draw at <= 60 Hz, filter at the full 100 Hz
                continue
            last_draw = now
            with v.lock():
                pipe.apply(data)
                mujoco.mj_forward(model, data)
            left, right = pipe.text()
            v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
            v.sync()


def run_record(a, model, data, pipe, samples):
    """Headless: render a PNG every 1/fps s of sample time, plus an animated GIF."""
    from PIL import Image, ImageDraw
    out = Path(a.record)
    out.mkdir(parents=True, exist_ok=True)
    r = mujoco.Renderer(model, 640, 960)
    frames, next_t, k = [], 0.0, 0
    for s in samples:
        pipe.push(s)
        if s.t + 1e-9 < next_t:
            continue
        next_t += 1.0 / a.fps
        pipe.apply(data)
        mujoco.mj_forward(model, data)
        r.update_scene(data, camera=a.camera)
        img = Image.fromarray(r.render())
        left, right = pipe.text()
        draw = ImageDraw.Draw(img)
        draw.rectangle((0, 0, 560, 150), fill=(255, 255, 255))
        draw.text((8, 6), f"t = {s.t:5.2f} s   (SIMULATED fake IMU)" if a.fake else f"t = {s.t:5.2f} s",
                  fill=(0, 0, 0))
        for i, (l, rr) in enumerate(zip(left.split("\n"), right.split("\n") + [""] * 9)):
            draw.text((8, 22 + 14 * i), l, fill=(0, 0, 0))
            draw.text((200, 22 + 14 * i), rr, fill=(0, 0, 0))
        img.save(out / f"frame_{k:04d}.png")
        frames.append(img.resize((480, 320)).convert("P", palette=Image.ADAPTIVE))  # half size keeps the GIF small
        k += 1
    if frames:
        frames[0].save(out / "imu_view.gif", save_all=True, append_images=frames[1:],
                       duration=int(1000 / a.fps), loop=0)
    print(f"wrote {k} frames + imu_view.gif to {out}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--fake", action="store_true", help="sine tilt, no hardware")
    src.add_argument("--replay", metavar="CSV", help="replay a --save file")
    src.add_argument("--stdin", action="store_true", help="live imu_can_bench lines on stdin")
    p.add_argument("--axes", default="x,y,z",
                   help="robot x,y,z in board axes, e.g. 'x,-y,-z' if the board is mounted upside down")
    p.add_argument("--tau", type=float, default=0.5, help="complementary time constant, s (alpha = tau/(tau+dt))")
    p.add_argument("--calib-s", type=float, default=2.0, help="hold still this long to measure gyro bias; 0 = off")
    p.add_argument("--beta", type=float, default=0.05, help="Madgwick gain")
    p.add_argument("--save", metavar="CSV", help="also record raw samples for --replay")
    p.add_argument("--camera", default="demo", choices=["demo", "side"])
    p.add_argument("--no-ghosts", action="store_true", help="solid robot only (implied by --camera side)")
    p.add_argument("--seconds", type=float, default=None, help="fake-mode length")
    p.add_argument("--record", metavar="DIR", help="headless PNG sequence + GIF instead of a window")
    p.add_argument("--fps", type=float, default=10.0)
    a = p.parse_args(argv)
    axes = tuple(x.strip() for x in a.axes.split(","))
    if len(axes) != 3 or sorted(x[-1] for x in axes) != ["x", "y", "z"]:
        p.error("--axes needs a permutation of x,y,z with optional '-' signs")
    if axes_det(axes) < 0:
        p.error("--axes is a mirror image (determinant -1): a physical board can only be rotated")
    if a.record and a.fake and a.seconds is None:
        a.seconds = 12.0

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    pipe = Pipeline(model, axes, a.tau, a.calib_s, a.beta)
    if a.no_ghosts or a.camera == "side":   # ghosts would block the side view
        for name in ("ghost_acc", "ghost_gyro"):
            data.mocap_pos[pipe.mocap[name]][2] = -5.0
    samples = source(a)
    pipe.line_stats = getattr(a, "line_stats", None)
    if a.record:
        run_record(a, model, data, pipe, samples)
        return
    writer = stream.Writer(a.save) if a.save else None
    try:
        run_viewer(a, model, data, pipe, samples, writer)
    finally:
        if writer:
            writer.close()


if __name__ == "__main__":
    main()
