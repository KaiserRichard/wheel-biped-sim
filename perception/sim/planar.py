"""Three-coordinate MuJoCo plant with exact ideal rolling reduction."""

import json
from pathlib import Path

import mujoco
import numpy as np


STATE_NAMES = ["x_m", "leg_rad", "body_rad", "xdot_m_s", "legdot_rad_s", "bodydot_rad_s"]
INPUT_NAMES = ["wheel_pair_Nm", "virtual_hip_pair_Nm"]
# [x, theta, phi] -> MuJoCo [x, theta, phi-theta].
TO_JOINT = np.array([[1., 0., 0.], [0., 1., 0.], [0., -1., 1.]])
FROM_JOINT = np.linalg.inv(TO_JOINT)


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    if cfg["controller"]["state_order"] != STATE_NAMES or cfg["controller"]["input_order"] != INPUT_NAMES:
        raise ValueError("Unsupported state/input order")
    for name, item in cfg["physics"].items():
        if item["evidence"] not in ("ASSUMED", "SOURCE-SUPPORTED") or not item["source"] or not item["unit"]:
            raise ValueError(f"Missing parameter provenance: {name}")
        if not np.isfinite(item["value"]) or item["value"] <= 0:
            raise ValueError(f"Expected positive finite parameter: {name}")
    s, p, c = cfg["simulation"], cfg["protocol"], cfg["controller"]
    if s["integrator"] != "RK4" or c["solver"] not in ("CARE", "DARE"):
        raise ValueError("Supported choices: RK4; CARE or DARE")
    dt = s["dt_s"]
    if not 0 < dt <= 0.01 or not 0 < s["finite_difference_epsilon"] <= 0.001:
        raise ValueError("Invalid dt or finite difference epsilon")
    for name in ("duration_s", "push_start_s", "push_duration_s", "settle_window_s"):
        steps = p[name] / dt
        if not np.isfinite(steps) or steps <= 0 or abs(steps - round(steps)) > 1e-8:
            raise ValueError(f"{name} must be positive and align with dt")
    if p["push_start_s"] + p["push_duration_s"] + p["settle_window_s"] > p["duration_s"]:
        raise ValueError("Pulse and settle window must fit the run")
    for key, n in (("Q_diagonal", 6), ("R_diagonal", 2)):
        a = np.asarray(c[key], float)
        if a.shape != (n,) or not np.all(np.isfinite(a)) or np.any(a <= 0):
            raise ValueError(f"Invalid {key}")
    for key in ("initial_state", "state_abs_tolerance"):
        a = np.asarray(p[key], float)
        if a.shape != (6,) or not np.all(np.isfinite(a)):
            raise ValueError(f"Invalid {key}")
    if np.any(np.asarray(p["state_abs_tolerance"]) <= 0):
        raise ValueError("Recovery tolerances must be positive")
    return cfg


def parameters(cfg):
    return {k: float(v["value"]) for k, v in cfg["physics"].items()}


def make_mjcf(cfg):
    p = parameters(cfg)
    r, length = p["wheel_radius"], p["leg_length"]
    # No springs, damping, position servos, equality stabilization or contact.
    # The armature retains exactly the eliminated rolling kinetic energy.
    return f"""<mujoco model="ASSUMED_reduced_planar_WBR">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="{cfg['simulation']['dt_s']}" gravity="0 0 {-p['gravity']}" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <default>
    <joint damping="0" frictionloss="0" stiffness="0" limited="false"/>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <geom name="visual_floor" type="plane" size="4 1 0.05" rgba="0.8 0.8 0.8 1"/>
    <body name="axle" pos="0 0 {r}">
      <joint name="translation" type="slide" axis="1 0 0" armature="{p['wheel_inertia']/r**2}"/>
      <inertial pos="0 0 0" mass="{p['wheel_mass']}" diaginertia="{p['wheel_inertia']} {p['wheel_inertia']} {p['wheel_inertia']}"/>
      <geom name="wheel_visual" type="cylinder" size="{r} 0.035" euler="1.5707963267948966 0 0" rgba="0.1 0.1 0.1 1"/>
      <body name="leg">
        <joint name="leg_pitch" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 {p['leg_com']}" mass="{p['leg_mass']}" diaginertia="{p['leg_inertia']} {p['leg_inertia']} {p['leg_inertia']}"/>
        <geom type="capsule" fromto="0 0 0 0 0 {length}" size="0.014" rgba="0.2 0.5 0.8 1"/>
        <body name="body" pos="0 0 {length}">
          <joint name="hip_pitch" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 {p['body_com']}" mass="{p['body_mass']}" diaginertia="{p['body_inertia']} {p['body_inertia']} {p['body_inertia']}"/>
          <geom type="box" pos="0 0 {p['body_com']}" size="0.06 0.08 0.06" rgba="0.9 0.5 0.2 1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="wheel_relative_rotation">
      <joint joint="translation" coef="{1/r}"/>
      <joint joint="leg_pitch" coef="-1"/>
    </fixed>
  </tendon>
  <actuator>
    <motor name="wheel_pair" tendon="wheel_relative_rotation" ctrllimited="true" ctrlrange="{-p['wheel_torque_limit']} {p['wheel_torque_limit']}"/>
    <motor name="hip_pair" joint="hip_pitch" ctrllimited="true" ctrlrange="{-p['hip_torque_limit']} {p['hip_torque_limit']}"/>
  </actuator>
</mujoco>
"""


class Plant:
    def __init__(self, cfg):
        self.cfg = cfg
        self.xml = make_mjcf(cfg)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.body_id = self.model.body("body").id
        self.limits = self.model.actuator_ctrlrange[:, 1].copy()

    def data(self, state=None):
        d = mujoco.MjData(self.model)
        if state is not None:
            d.qpos[:] = TO_JOINT @ np.asarray(state)[:3]
            d.qvel[:] = TO_JOINT @ np.asarray(state)[3:]
        mujoco.mj_forward(self.model, d)
        return d

    @staticmethod
    def state(d):
        return np.r_[FROM_JOINT @ d.qpos, FROM_JOINT @ d.qvel]

    def forward(self, d, control, push=0.):
        d.ctrl[:] = control
        d.xfrc_applied[:] = 0.
        d.xfrc_applied[self.body_id, 0] = push
        mujoco.mj_forward(self.model, d)

    def derivative(self, state, control):
        d = self.data(state)
        self.forward(d, control)
        return np.r_[self.state(d)[3:], FROM_JOINT @ d.qacc]

    def transition(self, state, control):
        d = self.data(state)
        self.forward(d, control)
        mujoco.mj_step(self.model, d)
        return self.state(d)


def finite_difference(function, epsilon):
    """Centered differences in declared SI state/control coordinates."""
    state, control = np.zeros(6), np.zeros(2)
    A = np.column_stack([
        (function(state + dx, control) - function(state - dx, control)) / (2 * epsilon)
        for dx in epsilon * np.eye(6)
    ])
    B = np.column_stack([
        (function(state, control + du) - function(state, control - du)) / (2 * epsilon)
        for du in epsilon * np.eye(2)
    ])
    return A, B
