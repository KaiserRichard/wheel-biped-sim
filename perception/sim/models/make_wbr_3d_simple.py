"""Write sim/models/wbr_3d_simple.xml: a visual-only WBR with closed five-bar legs.

  python -m sim.models.make_wbr_3d_simple            # nominal virtual-leg length l0 = 0.12 m
  python -m sim.models.make_wbr_3d_simple --l0 0.16

Geometry is SOURCE-SUPPORTED where noted (see tools/imu_view/README.md):
  hip motor spacing d = 0.08 m, upper links l1 = l4 = 0.075 m, lower links l2 = l3 = 0.14 m
  (report Fig. 2.1 / Section 5, and DM-jump User/Algorithm/VMC/VMC_calc.c VMC_init).
The legs are drawn in one fixed pose (theta0 = 0): this is a picture, not a mechanism.
"""

import argparse
import math
from pathlib import Path

D_HIP = 0.08      # SOURCE-SUPPORTED: distance between the two hip motor axes
L_UP = 0.075      # SOURCE-SUPPORTED: upper links OA, DC
L_LOW = 0.14      # SOURCE-SUPPORTED: lower links AB, CB
R_WHEEL = 0.0603  # SOURCE-SUPPORTED: DM-jump observer constant (report Table 5.1 rounds to 0.06)

# ASSUMED / INFERRED layout (no CAD yet), metres.
BODY_HALF = (0.15, 0.10, 0.06)   # INFERRED x/z from Fig. 2.1b scale; y ASSUMED
HIP_BELOW_TOP = 0.03             # INFERRED from Fig. 2.1b: hip axes ~3 cm below the body top
Y_UPPER, Y_LOWER, Y_WHEEL = 0.112, 0.126, 0.162   # ASSUMED lateral stack: body | upper | lower | wheel
WHEEL_HALF_WIDTH = 0.018         # ASSUMED (tyre ~36 mm wide in the photo)
LINK_R = 0.008                   # ASSUMED drawing radius of the links


def knee(hip, foot, l_hip, l_foot, outward):
    """Circle-circle intersection in the (x, z) plane; pick the knee on the `outward` x side."""
    dx, dz = foot[0] - hip[0], foot[1] - hip[1]
    dist = math.hypot(dx, dz)
    if not abs(l_hip - l_foot) <= dist <= l_hip + l_foot:
        raise ValueError(f"five-bar cannot reach: hip-foot distance {dist:.4f} m")
    a = (l_hip ** 2 - l_foot ** 2 + dist ** 2) / (2 * dist)
    h = math.sqrt(max(l_hip ** 2 - a ** 2, 0.0))
    mx, mz = hip[0] + a * dx / dist, hip[1] + a * dz / dist
    k1 = (mx + h * dz / dist, mz - h * dx / dist)
    k2 = (mx - h * dz / dist, mz + h * dx / dist)
    return min(k1, k2) if outward < 0 else max(k1, k2)


def leg_points(l0):
    """(x, z) of hips O, D, knees A, C and wheel axle B, with B at the origin and theta0 = 0."""
    o, d, b = (-D_HIP / 2, l0), (D_HIP / 2, l0), (0.0, 0.0)
    a = knee(o, b, L_UP, L_LOW, outward=-1)   # front/back knees splay outwards (Fig. 2.1)
    c = knee(d, b, L_UP, L_LOW, outward=+1)
    return o, d, a, c, b


def robot_xml(name, pos_y, l0, alpha, solid):
    o, d, a, c, b = leg_points(l0)
    def rgba(r, g, bl):
        return f"{r} {g} {bl} {alpha}"

    def nm(n):  # only the solid robot gets geom names
        return f'name="{n}" ' if solid else ""

    body_z = l0 + HIP_BELOW_TOP - BODY_HALF[2]
    g = []
    for side, s in (("l", 1), ("r", -1)):
        yu, yl, yw = s * Y_UPPER, s * Y_LOWER, s * Y_WHEEL
        for tag, p in (("O", o), ("D", d)):
            g.append(f'<geom type="cylinder" size="0.0285 0.012" pos="{p[0]:.4f} {s * 0.098:.4f} {p[1]:.4f}" '
                     f'euler="90 0 0" rgba="{rgba(0.3, 0.3, 0.32)}" mass="0"/>')  # DM-J4310 face, 57 mm
        for p, q, y, col in ((o, a, yu, (0.62, 0.6, 0.2)), (d, c, yu, (0.62, 0.6, 0.2)),
                             (a, b, yl, (0.25, 0.65, 0.65)), (c, b, yl, (0.25, 0.65, 0.65))):
            g.append(f'<geom type="capsule" size="{LINK_R}" fromto="{p[0]:.4f} {y:.4f} {p[1]:.4f} '
                     f'{q[0]:.4f} {y:.4f} {q[1]:.4f}" rgba="{rgba(*col)}" mass="0.05"/>')
        for p in (a, c):  # knee pins
            g.append(f'<geom type="cylinder" size="0.011 0.012" pos="{p[0]:.4f} {s * (Y_UPPER + Y_LOWER) / 2:.4f} '
                     f'{p[1]:.4f}" euler="90 0 0" rgba="{rgba(0.15, 0.15, 0.15)}" mass="0"/>')
        g.append(f'<geom {nm("wheel_" + side)}type="cylinder" '
                 f'size="{R_WHEEL} {WHEEL_HALF_WIDTH}" pos="0 {yw:.4f} 0" euler="90 0 0" '
                 f'rgba="{rgba(0.1, 0.1, 0.1)}" mass="0.7"/>')
        g.append(f'<geom type="cylinder" size="0.034 {WHEEL_HALF_WIDTH + 0.001}" pos="0 {yw:.4f} 0" euler="90 0 0" '
                 f'rgba="{rgba(0.6, 0.6, 0.62)}" mass="0"/>')  # DM-H6215 hub, 68 mm
    body_col = (0.55, 0.57, 0.6) if solid else ((0.2, 0.4, 1) if "acc" in name else (1, 0.5, 0.1))
    g.append(f'<geom {nm("body")}type="box" '
             f'size="{BODY_HALF[0]} {BODY_HALF[1]} {BODY_HALF[2]}" pos="0 0 {body_z:.4f}" '
             f'rgba="{rgba(*body_col)}" mass="3.5"/>')
    top = body_z + BODY_HALF[2]
    g.append(f'<geom type="sphere" size="0.015" pos="{BODY_HALF[0] + 0.005:.4f} 0 {body_z:.4f}" '
             f'rgba="0.9 0.1 0.1 {max(alpha, 0.6)}" mass="0"/>')  # nose = +x
    if solid:
        g += [f'<site name="imu" pos="0 0 {top + 0.0015:.4f}" size="0.005"/>',
              f'<geom name="imu_sensor" type="box" size="0.0105 0.008 0.00075" pos="0 0 {top + 0.0015:.4f}" '
              f'rgba="0.1 0.3 0.7 1" mass="0"/>']
        for ax, vec, col in (("x", (0.08, 0, 0), "1 0 0"), ("y", (0, 0.08, 0), "0 0.8 0"), ("z", (0, 0, 0.08), "0 0.3 1")):
            z0 = top + 0.003
            g.append(f'<geom name="imu_{ax}" type="capsule" fromto="0 0 {z0:.4f} {vec[0]} {vec[1]} {z0 + vec[2]:.4f}" '
                     f'size="0.003" rgba="{col} 1" mass="0"/>')
    inner = "\n      ".join(g)
    return f'    <body name="{name}" mocap="true" pos="0 {pos_y} {R_WHEEL}">\n      {inner}\n    </body>'


def build(l0):
    o, d, a, c, b = leg_points(l0)
    top = l0 + HIP_BELOW_TOP
    robots = "\n\n".join([robot_xml("robot", 0, l0, 1, True),
                          robot_xml("ghost_acc", 0.55, l0, 0.25, False),
                          robot_xml("ghost_gyro", -0.55, l0, 0.25, False)])
    return f"""<!--
  GENERATED by sim/models/make_wbr_3d_simple.py (l0 = {l0} m). Edit the script, not this file.
  WBR 3D visual model for the IMU tilt demo (tools/imu_view). VISUALISATION ONLY:
  no joints, no actuators, no dynamics claims. The planar control model is sim/planar.py.

  Each side is a closed five-bar drawn in one pose (theta0 = 0), from the SOURCE-SUPPORTED
  link lengths d = {D_HIP}, l1 = l4 = {L_UP}, l2 = l3 = {L_LOW} m (team report Fig. 2.1, DM-jump VMC_calc.c).
  Knees A = ({a[0]:.4f}, {a[1]:.4f}), C = ({c[0]:.4f}, {c[1]:.4f}) m relative to the wheel axle.
  Body size, lateral stack and IMU location are ASSUMED/INFERRED (tools/imu_view/README.md).

  Frame: x forward, y left, z up. Each robot is a mocap body whose origin is the
  wheel-axle centre, so a pitch rotates the robot about the axle, like the real
  inverted pendulum. The viewer writes mocap_quat from the filtered pitch/roll.
    robot        complementary filter (solid)
    ghost_acc    accelerometer only   (translucent blue, left)
    ghost_gyro   gyro integration only (translucent orange, right)
-->
<mujoco model="wbr_3d_simple">
  <compiler angle="degree"/>
  <visual>
    <global offwidth="960" offheight="640"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.72 0.74 0.76" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <light pos="0.5 -0.5 1.5" dir="-0.3 0.3 -1"/>
    <geom name="floor" type="plane" size="1.5 1.5 0.01" material="grid"/>
    <!-- Upright reference post behind the robot: the zero-tilt direction. -->
    <geom name="upright_ref" type="capsule" fromto="-0.3 0 0 -0.3 0 {R_WHEEL + top + 0.1:.3f}" size="0.003" rgba="0.3 0.3 0.3 0.5"/>
    <camera name="demo" pos="1.05 -1.05 0.6" xyaxes="0.707 0.707 0 -0.25 0.25 0.935"/>
    <camera name="side" pos="0 -1.2 0.2" xyaxes="1 0 0 0 0 1"/>

{robots}
  </worldbody>
</mujoco>
"""


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--l0", type=float, default=0.12, help="virtual leg length hip-midpoint to axle, m "
                   "(report range 0.06-0.20; 0.12 is an ASSUMED standing height)")
    p.add_argument("--out", default=str(Path(__file__).with_name("wbr_3d_simple.xml")))
    a = p.parse_args()
    Path(a.out).write_text(build(a.l0))
    print(f"wrote {a.out} (l0 = {a.l0} m)")


if __name__ == "__main__":
    main()
