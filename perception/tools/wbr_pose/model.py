"""Mocap-only MuJoCo model and pose placement for the WBR."""

import math

from sim.models import make_wbr_3d_simple as gen
from .fivebar import forward_kinematics


def build_xml() -> str:
    bodies = []
    for side, ysign in (("left", 1), ("right", -1)):
        for part, color, radius, length in (
            ("upper_a", "0.62 0.60 0.20 1", gen.LINK_R, gen.L_UP),
            ("upper_b", "0.62 0.60 0.20 1", gen.LINK_R, gen.L_UP),
            ("lower_a", "0.25 0.65 0.65 1", gen.LINK_R, gen.L_LOW),
            ("lower_b", "0.25 0.65 0.65 1", gen.LINK_R, gen.L_LOW),
        ):
            bodies.append(f'<body name="{side}_{part}" mocap="true"><geom type="capsule" size="{radius}" fromto="0 0 {-length/2} 0 0 {length/2}" rgba="{color}"/></body>')
        bodies.append(f'<body name="wheel_{side}" mocap="true"><geom type="cylinder" size="{gen.R_WHEEL} {gen.WHEEL_HALF_WIDTH}" euler="90 0 0" rgba="0.10 0.10 0.10 1"/><geom type="cylinder" size="0.034 {gen.WHEEL_HALF_WIDTH + .001}" euler="90 0 0" rgba="0.6 0.6 0.62 1"/></body>')
    bodies.append(f'<body name="body" mocap="true"><geom type="box" size="{gen.BODY_HALF[0]} {gen.BODY_HALF[1]} {gen.BODY_HALF[2]}" rgba="0.55 0.57 0.60 1"/><geom type="sphere" size="0.015" pos="{gen.BODY_HALF[0]+.005} 0 0" rgba="0.9 0.1 0.1 1"/></body>')
    return f'''<mujoco model="wbr_kinematic_jump"><compiler angle="radian"/><visual><global offwidth="960" offheight="640"/><headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual><default><geom contype="0" conaffinity="0"/></default><worldbody>
      <light pos="0.5 -0.5 1.5" dir="-0.3 0.3 -1"/><geom name="floor" type="plane" size="2 2 .01" rgba="0.78 0.81 0.84 1"/>
      <camera name="demo" pos="1.5 -1.5 0.75" xyaxes="0.707 0.707 0 -0.2 0.2 0.96"/><camera name="side" pos="0 -1.4 0.22" xyaxes="1 0 0 0 0 1"/>
      {' '.join(bodies)}
      </worldbody></mujoco>'''


def make_model():
    import mujoco
    return mujoco.MjModel.from_xml_string(build_xml())


def _quat(pitch, roll):
    # q = q_pitch-about-y * q_roll-about-x, MuJoCo w,x,y,z order.
    cy, sy = math.cos(pitch / 2), math.sin(pitch / 2)
    cx, sx = math.cos(roll / 2), math.sin(roll / 2)
    return (cy * cx, cy * sx, sy * cx, -sy * sx)


def _rotate(v, q):
    w, x, y, z = q
    # q*v*q^-1
    tx, ty, tz = 2 * (y*v[2] - z*v[1]), 2 * (z*v[0] - x*v[2]), 2 * (x*v[1] - y*v[0])
    return (v[0] + w*tx + (y*tz - z*ty), v[1] + w*ty + (z*tx - x*tz), v[2] + w*tz + (x*ty - y*tx))


def _axis_quat(a, b):
    v = tuple(b[i] - a[i] for i in range(3))
    norm = math.sqrt(sum(x*x for x in v))
    if norm < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    x, y, z = (u / norm for u in v)
    # Rotate local +z onto v.
    dot = max(-1.0, min(1.0, z))
    q = (1 + dot, -y, x, 0.0)
    n = math.sqrt(sum(t*t for t in q))
    if n < 1e-10:  # antiparallel
        return (0.0, 1.0, 0.0, 0.0)
    return tuple(t / n for t in q)


def _id(model, name):
    return int(model.body(name).mocapid[0])


def set_pose(model, data, l0, theta0, pitch=0.0, roll=0.0, vertical_offset=0.0):
    """Place all mocap bodies; ``vertical_offset`` raises the whole robot in flight."""
    kin = forward_kinematics(l0, theta0)
    q = _quat(pitch, roll)
    axle_local = kin.axle
    # Transform wheel cylinder's local y-axis and account for its finite width.
    world_y = _rotate((0.0, 1.0, 0.0), q)
    wheel_clearance = gen.R_WHEEL * math.sqrt(max(0.0, 1 - world_y[2]**2)) + gen.WHEEL_HALF_WIDTH * abs(world_y[2])
    wheel_clearance += abs(gen.Y_WHEEL * world_y[2])
    axle_z = wheel_clearance + vertical_offset
    axle_world = (0.0, 0.0, axle_z)

    def world(local):
        v = _rotate((local[0], local[1], local[2]), q)
        return (axle_world[0] + v[0], axle_world[1] + v[1], axle_world[2] + v[2])

    # In the neutral leg frame, hip midpoint is the origin; transform points relative to axle.
    ax, az = kin.axle
    hip_l = (-gen.D_HIP / 2 - ax, 0.0, -az)
    hip_r = ( gen.D_HIP / 2 - ax, 0.0, -az)
    knee_l = (kin.knee_left[0] - ax, 0.0, kin.knee_left[1] - az)
    knee_r = (kin.knee_right[0] - ax, 0.0, kin.knee_right[1] - az)
    axle = (0.0, 0.0, 0.0)
    for side, ysign in (("left", 1), ("right", -1)):
        upper_y, lower_y = ysign * gen.Y_UPPER, ysign * gen.Y_LOWER
        hl, hr = (hip_l[0], upper_y, hip_l[2]), (hip_r[0], upper_y, hip_r[2])
        kl, kr = (knee_l[0], upper_y, knee_l[2]), (knee_r[0], upper_y, knee_r[2])
        kll, krr = (knee_l[0], lower_y, knee_l[2]), (knee_r[0], lower_y, knee_r[2])
        bl, br = (0.0, lower_y, 0.0), (0.0, lower_y, 0.0)
        for name, p, end in ((f"{side}_upper_a", hl, kl), (f"{side}_upper_b", hr, kr),
                             (f"{side}_lower_a", kll, bl), (f"{side}_lower_b", br, krr)):
            p0, p1 = world(p), world(end)
            i = _id(model, name)
            data.mocap_pos[i] = [(p0[j] + p1[j]) / 2 for j in range(3)]
            data.mocap_quat[i] = _axis_quat(p0, p1)
        i = _id(model, f"wheel_{side}")
        data.mocap_pos[i], data.mocap_quat[i] = world((0.0, ysign * gen.Y_WHEEL, 0.0)), q
    body_z = gen.HIP_BELOW_TOP - gen.BODY_HALF[2] - az
    i = _id(model, "body")
    data.mocap_pos[i] = world((-ax, 0.0, body_z))
    data.mocap_quat[i] = q
    import mujoco
    mujoco.mj_forward(model, data)
    return data
