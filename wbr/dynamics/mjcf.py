"""MuJoCo model generated from the parameter file. Primitive shapes, closed five-bar legs."""
import numpy as np

from . import fivebar


def _rod_inertia(m, l):
    return f"{m * l**2 / 12 + 1e-7:.8g} {1e-7:.8g} {m * l**2 / 12 + 1e-7:.8g}"


def _leg(side, y, p):
    fb, w = p["fivebar"], p["wheel"]
    d, l1, l2 = fb["hip_spacing"], fb["upper_length"], fb["lower_length"]
    m1, m2 = fb["upper_mass"], fb["lower_mass"]
    wi = w["inertia_axial"]
    wt = w["mass"] * (3 * w["radius"] ** 2 + w["width"] ** 2) / 12
    def link(name, axis, x, l, m, inner, jname):
        return f"""
      <body name="{name}" pos="{x} {y if name.startswith('u') else 0} {0 if name.startswith('u') else -l1}">
        <joint name="{jname}" type="hinge" axis="0 {axis} 0" damping="0.001"/>
        <inertial pos="0 0 {-l / 2}" mass="{m}" diaginertia="{_rod_inertia(m, l)}"/>
        <geom type="capsule" fromto="0 0 0 0 0 {-l}" size="0.006" contype="0" conaffinity="0" rgba=".25 .3 .4 1"/>{inner}
      </body>"""
    wheel = f"""
        <site name="foot_{side}" pos="0 0 {-l2}" size="0.005"/>
        <body name="wheel_{side}" pos="0 0 {-l2}">
          <joint name="wheel_{side}" type="hinge" axis="0 1 0" damping="0.0005"/>
          <inertial pos="0 0 0" mass="{w['mass']}" diaginertia="{wt} {wi} {wt}"/>
          <geom name="wheel_{side}" type="cylinder" size="{w['radius']} {w['width'] / 2}" zaxis="0 1 0" contype="1" conaffinity="1" condim="4" friction="{p['friction']} 0.01 0.001" rgba=".1 .1 .1 1"/>
          <geom type="box" size="{w['radius'] * .7} {w['width'] / 2 + .001} .004" contype="0" conaffinity="0" rgba=".85 .55 .15 1"/>
        </body>"""
    lower_f = link(f"l_front_{side}", -1, 0, l2, m2, wheel, f"knee_front_{side}")
    rear_site = f'\n        <site name="foot_rear_{side}" pos="0 0 {-l2}" size="0.004"/>'
    lower_r = link(f"l_rear_{side}", 1, 0, l2, m2, rear_site, f"knee_rear_{side}")
    return link(f"u_front_{side}", -1, d / 2, l1, m1, lower_f, f"hip_front_{side}") + link(f"u_rear_{side}", 1, -d / 2, l1, m1, lower_r, f"hip_rear_{side}")


def build_mjcf(p, planar=False, extra_world="", timestep=0.001):
    b = p["body"]
    cx, cz = b["com_offset_xz"]
    ixx, iyy, izz = b["inertia_diag"]
    l2 = p["fivebar"]["lower_length"]
    tr = p["track_width"]
    root = (
        '<joint name="root_x" type="slide" axis="1 0 0"/><joint name="root_z" type="slide" axis="0 0 1"/><joint name="root_pitch" type="hinge" axis="0 1 0"/>'
        if planar
        else '<freejoint name="root"/>'
    )
    legs = _leg("L", tr / 2, p) + _leg("R", -tr / 2, p)
    motors, conn = "", ""
    for side in "LR":
        hp, wm = p["hip_motor"]["peak_torque"], p["wheel_motor"]["peak_torque"]
        motors += f"""
    <motor name="hip_front_{side}" joint="hip_front_{side}" ctrlrange="{-hp} {hp}" ctrllimited="true"/>
    <motor name="hip_rear_{side}" joint="hip_rear_{side}" ctrlrange="{-hp} {hp}" ctrllimited="true"/>
    <motor name="wheel_{side}" joint="wheel_{side}" ctrlrange="{-wm} {wm}" ctrllimited="true"/>"""
        # Sites, not body1/anchor: with an anchor, the point on body2 would be taken from qpos0,
        # where the legs hang straight and the two tips are hip_spacing apart.
        conn += f'\n    <connect name="close_{side}" site1="foot_{side}" site2="foot_rear_{side}" solref="0.002 1" solimp="0.99 0.999 0.0005"/>'
    return f"""<!-- ASSUMED inputs from models/design.json; no physical-mass validation is represented here. -->
<mujoco model="wbr_own">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep}" integrator="implicitfast" gravity="0 0 {-p['gravity']}" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1920" offheight="1080"/><quality shadowsize="4096"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".78 .8 .82" rgb2=".62 .65 .68"/>
    <material name="floor" texture="grid" texrepeat="8 8" reflectance=".05"/>
    <texture type="skybox" builtin="gradient" rgb1=".55 .7 .85" rgb2=".95 .97 1" width="256" height="256"/>
  </asset>
  <worldbody>
    <light pos="1 -2 4" dir="-.2 .4 -1" diffuse=".85 .85 .85"/>
    <geom name="floor" type="plane" size="40 40 .1" material="floor" contype="1" conaffinity="1" friction="{p['friction']} 0.01 0.001"/>
    {extra_world}
    <body name="torso" pos="0 0 0.3">
      {root}
      <inertial pos="{cx} 0 {cz}" mass="{b['mass']}" diaginertia="{ixx} {iyy} {izz}"/>
      <geom type="box" size=".09 {tr / 2 - .03} .035" pos="{cx} 0 {cz}" contype="0" conaffinity="0" rgba=".82 .84 .88 1"/>
      <site name="imu" pos="0 0 0" size=".005"/>{legs}
    </body>
  </worldbody>
  <equality>{conn}
  </equality>
  <contact>
    <exclude body1="wheel_L" body2="torso"/>
    <exclude body1="wheel_R" body2="torso"/>
  </contact>
  <actuator>{motors}
  </actuator>
  <sensor>
    <framequat name="torso_quat" objtype="site" objname="imu"/>
    <gyro name="torso_gyro" site="imu"/>
    <accelerometer name="torso_acc" site="imu"/>
  </sensor>
</mujoco>"""


def leg_joint_angles(L, alpha, p):
    """Hip and knee hinge angles for one leg at this virtual pose."""
    q1, q2 = fivebar.ik(L, alpha, p)
    kf, kr = fivebar.knees(q1, q2, p)
    foot = fivebar.fk(q1, q2, p)
    uf, ur = foot - kf, foot - kr
    return q1, np.arctan2(uf[0], -uf[1]) - q1, q2, np.arctan2(-ur[0], -ur[1]) - q2
