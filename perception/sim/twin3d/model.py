"""MJCF contact plant. All unspecified numerical choices are ASSUMED (README)."""

import math

import mujoco
import numpy as np

from sim.models.make_wbr_3d_simple import R_WHEEL, BODY_HALF, Y_WHEEL

DT = 0.001  # ASSUMED synchronous integration/control period, s.
STAND_LENGTH = 0.12  # ASSUMED m.
BODY_MASS, LEG_MASS, WHEEL_MASS = 3.5, 0.2, 0.7  # SOURCE-SUPPORTED kg, per leg/wheel.
BODY_COM = -0.03  # ASSUMED m relative to hip, matching visual-model placement.
WHEEL_LIMIT, HIP_LIMIT, LEG_LIMIT = 1.0, 3.0, 100.0  # ASSUMED per-side Nm, Nm, N.
BOX_HEIGHT, BOX_DEPTH, BOX_FACE = 0.08, 0.12, 1.0  # ASSUMED test fixture, m.
CAMERA_FOVY = math.degrees(2*math.atan(480/(2*386.953)))  # CALCULATED from simcam's ASSUMED intrinsics.


def make_mjcf(box_face=None):
    """Planar joint set: world x/z slides and y pitch; no mocap/equalities."""
    legs, motors = [], []
    for side, sign in (("left", 1), ("right", -1)):
        legs.append(f'''<body name="hip_{side}" pos="0 {sign*Y_WHEEL} 0">
          <joint name="hip_{side}" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.001" diaginertia="0.000001 0.000001 0.000001"/>
          <body name="leg_{side}">
            <joint name="length_{side}" type="slide" axis="0 0 -1" limited="true" range="0.06 0.21"/>
            <inertial pos="0 0 0.06" mass="{LEG_MASS-.001}" diaginertia="0.0003 0.0003 0.00002"/>
            <geom name="leg_{side}" type="capsule" fromto="0 0 0.015 0 0 0.105" size="0.008" rgba="0.2 0.5 0.7 1"/>
            <body name="wheel_{side}">
              <joint name="wheel_{side}" type="hinge" axis="0 1 0"/>
              <geom name="wheel_{side}" type="cylinder" size="{R_WHEEL} 0.018" euler="1.5707963267948966 0 0" mass="{WHEEL_MASS}" rgba="0.1 0.1 0.1 1"/>
            </body>
          </body>
        </body>''')
        for name, limit in (("wheel", WHEEL_LIMIT), ("hip", HIP_LIMIT), ("length", LEG_LIMIT)):
            motors.append(f'<motor name="{name}_{side}" joint="{name}_{side}" ctrllimited="true" ctrlrange="{-limit} {limit}"/>')
    box = (f'<geom name="box" type="box" pos="{box_face+BOX_DEPTH/2} 0 {BOX_HEIGHT/2}" '
           f'size="{BOX_DEPTH/2} 0.6 {BOX_HEIGHT/2}" rgba="0.65 0.35 0.15 1"/>' if box_face is not None else '')
    return f'''<mujoco model="ASSUMED_contact_virtual_leg_WBR">
      <compiler angle="radian"/>
      <option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" iterations="60"/>
      <statistic extent="1"/>
      <visual><global offwidth="640" offheight="480"/><map znear="0.01" zfar="6"/></visual>
      <default>
        <joint damping="0.001" armature="0.00001"/>
        <geom friction="1.0 0.005 0.0001" condim="3" solref="0.004 1" solimp="0.95 0.99 0.001"/>
      </default>
      <worldbody>
        <light pos="0 -2 3"/>
        <geom name="floor" type="plane" size="10 2 0.05" rgba="0.8 0.8 0.8 1"/>
        {box}
        <camera name="side" pos="1 -3 0.6" xyaxes="1 0 0 0 0 1"/>
        <body name="base" pos="0 0 {R_WHEEL+STAND_LENGTH}">
          <joint name="base_x" type="slide" axis="1 0 0"/>
          <joint name="base_z" type="slide" axis="0 0 1"/>
          <joint name="pitch" type="hinge" axis="0 1 0"/>
          <geom name="base" type="box" pos="0 0 {BODY_COM}" size="{' '.join(map(str, BODY_HALF))}" mass="{BODY_MASS}" rgba="0.8 0.5 0.2 1"/>
          <camera name="simcam_d435" pos="0.16 0 0.06" xyaxes="0 -1 0 0.052335956 0 0.998629535" fovy="{CAMERA_FOVY}" ipd="0"/>
          {''.join(legs)}
        </body>
      </worldbody>
      <contact><exclude body1="leg_left" body2="leg_right"/></contact>
      <actuator>{''.join(motors)}</actuator>
    </mujoco>'''


class Plant:
    def __init__(self, box_face=None):
        self.model = mujoco.MjModel.from_xml_string(make_mjcf(box_face))
        self.q = {name: int(self.model.joint(name).qposadr[0]) for name in
                  ("base_x", "base_z", "pitch", "hip_left", "hip_right", "length_left", "length_right", "wheel_left", "wheel_right")}
        self.u = {name: self.model.actuator(name).id for name in
                  ("wheel_left", "wheel_right", "hip_left", "hip_right", "length_left", "length_right")}

    def data(self, pitch=0.0):
        data = mujoco.MjData(self.model)
        data.qpos[self.q['pitch']] = pitch
        for side in ('left', 'right'):
            data.qpos[self.q['length_'+side]] = STAND_LENGTH
            data.qpos[self.q['hip_'+side]] = -pitch
        mujoco.mj_forward(self.model, data)
        return data

    def state(self, d):
        """CALCULATED reduced-model state [axle x, leg, body, vx, leg rate, body rate]."""
        pitch = d.qpos[self.q['pitch']]
        pitchdot = d.qvel[self.q['pitch']]
        h = np.mean([d.qpos[self.q['hip_'+s]] for s in ('left', 'right')])
        hd = np.mean([d.qvel[self.q['hip_'+s]] for s in ('left', 'right')])
        length, ld = self.length(d), np.mean([d.qvel[self.q['length_'+s]] for s in ('left', 'right')])
        theta = pitch+h
        x = d.qpos[self.q['base_x']] - length*np.sin(theta)
        vx = d.qvel[self.q['base_x']] - ld*np.sin(theta) - length*np.cos(theta)*(pitchdot+hd)
        return np.array([x, theta, pitch, vx, pitchdot+hd, pitchdot])

    def length(self, d):
        return float(np.mean([d.qpos[self.q['length_'+s]] for s in ('left', 'right')]))

    def contacts(self, d):
        flags = dict(left=False, right=False, base_ground=False, bad_box=False, wheel_box=False)
        for contact in d.contact:
            if contact.dist > 0:
                continue
            names = {self.model.geom(int(i)).name for i in (contact.geom1, contact.geom2)}
            for side in ('left', 'right'):
                if 'wheel_'+side in names and names & {'floor', 'box'}:
                    flags[side] = True
            flags['base_ground'] |= {'base', 'floor'} <= names
            flags['bad_box'] |= 'box' in names and bool(names & {'base', 'leg_left', 'leg_right'})
            flags['wheel_box'] |= 'box' in names and bool(names & {'wheel_left', 'wheel_right'})
        return flags
