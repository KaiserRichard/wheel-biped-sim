"""Primitive MuJoCo plant and ASSUMED torque-speed limiter."""
import mujoco
import numpy as np
from wbr.dynamics import fivebar, model
from wbr.dynamics import mjcf
class Sim:
    """Free-joint MuJoCo plant with truth readout and motor limits."""

    def __init__(self, p, world='', L0=0.18, x0=0.0, truth=None, timestep=0.001, backend=model):
        self.backend = backend
        self.p = p
        self.model_p = truth or p
        self.m = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(self.model_p, planar=False, extra_world=world, timestep=timestep))
        self.d = mujoco.MjData(self.m)
        n = lambda t, s: mujoco.mj_name2id(self.m, t, s)
        (J, A) = (mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_ACTUATOR)
        self.ja = {s: {k: self.m.jnt_qposadr[n(J, f'{k}_{s}')] for k in ('hip_front', 'hip_rear', 'knee_front', 'knee_rear', 'wheel')} for s in 'LR'}
        self.jv = {s: {k: self.m.jnt_dofadr[n(J, f'{k}_{s}')] for k in ('hip_front', 'hip_rear', 'knee_front', 'knee_rear', 'wheel')} for s in 'LR'}
        self.act = {s: [n(A, f'hip_front_{s}'), n(A, f'hip_rear_{s}'), n(A, f'wheel_{s}')] for s in 'LR'}
        self.wheel = {s: n(mujoco.mjtObj.mjOBJ_BODY, f'wheel_{s}') for s in 'LR'}
        self.wgeom = {n(mujoco.mjtObj.mjOBJ_GEOM, f'wheel_{s}') for s in 'LR'}
        self.reset(L0, x0)

    def reset(self, L, x0=0.0, z0=0.0):
        (p, d) = (self.model_p, self.d)
        mujoco.mj_resetData(self.m, d)
        (th0, ph0, _) = self.backend.equilibrium(p, L)
        d.qpos[0:3] = [x0 + L * np.sin(th0), 0, z0 + p['wheel']['radius'] + L * np.cos(th0)]
        d.qpos[3:7] = [np.cos(ph0 / 2), 0, np.sin(ph0 / 2), 0]
        (q1, k1, q2, k2) = mjcf.leg_joint_angles(L, th0 - ph0, p)
        for s in 'LR':
            a = self.ja[s]
            (d.qpos[a['hip_front']], d.qpos[a['knee_front']], d.qpos[a['hip_rear']], d.qpos[a['knee_rear']]) = (q1, k1, q2, k2)
        mujoco.mj_forward(self.m, d)

    def airborne(self):
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if c.geom1 in self.wgeom or c.geom2 in self.wgeom:
                return False
        return True

    def read(self):
        (d, p) = (self.d, self.p)
        R = d.xmat[1].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        h = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        phi = np.arctan2(R[:, 2] @ h, R[2, 2])
        roll = np.arcsin(np.clip(R[2, 1], -1, 1))
        w = d.qvel[3:6]
        o = dict(phi=phi, roll=roll, yaw=yaw, phid=w[1], rolld=w[0], yawd=w[2], pos=d.qpos[:3].copy(), legs={})
        v = []
        res = np.zeros(6)
        for s in 'LR':
            (a, b) = (self.ja[s], self.jv[s])
            (q1, q2) = (d.qpos[a['hip_front']], d.qpos[a['hip_rear']])
            qd = np.array([d.qvel[b['hip_front']], d.qvel[b['hip_rear']]])
            J = fivebar.jacobian(q1, q2, p)
            (L, al) = fivebar.polar(q1, q2, p)
            (Ld, ald) = J @ qd
            o['legs'][s] = dict(L=L, alpha=al, Ld=Ld, alphad=ald, J=J, qd=qd, wheel_speed=d.qvel[b['wheel']])
            mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, self.wheel[s], res, 0)
            v.append(res[3:] @ h)
        o['v'] = float(np.mean(v))
        o['v_each'] = v
        return o

    def actuate(self, hips, wheels, delay_buf=None):
        """Applies torque with a linear torque-speed limit for each motor, then the actuator range."""
        (p, d) = (self.model_p, self.d)
        (hm, wm) = (p['hip_motor'], p['wheel_motor'])
        for s in 'LR':
            (ids, b) = (self.act[s], self.jv[s])
            for (i, (tau, dof)) in enumerate(((hips[s][0], b['hip_front']), (hips[s][1], b['hip_rear']))):
                d.ctrl[ids[i]] = _limit(tau, d.qvel[dof], hm)
            d.ctrl[ids[2]] = _limit(wheels[s], d.qvel[b['wheel']], wm)
def _limit(tau, speed, motor):
    """Torque limit falls linearly with speed in the driving direction; ASSUMED shape, real curve UNKNOWN."""
    (peak, w0) = (motor['peak_torque'], motor['max_speed'])
    if tau * speed > 0:
        peak = peak * max(0.0, 1 - abs(speed) / w0)
    return float(np.clip(tau, -peak, peak))
