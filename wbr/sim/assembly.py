"""Assembly-derived numerical plant, visual meshes withheld."""
import copy
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
from wbr.dynamics import model, fivebar
MODEL=Path(__file__).resolve().parents[2]/"models"/"assembly.xml"
class AssemblySim:
    instances = []
    templates = {}
    variant = 'built'
    com_mm = 0

    def __init__(self, p, world='', L0=0.14, x0=0.0, truth=None, timestep=0.001, backend=model):
        self.backend=backend
        self.p = p
        self.model_p = truth or p
        path = MODEL
        root = ET.parse(path).getroot()
        
        for sensor in root.find('sensor'):
            if sensor.get('name') == 'model_base_angular_velocity':
                sensor.set('name', 'torso_gyro')
            if sensor.get('name') == 'model_base_specific_force':
                sensor.set('name', 'torso_acc')
        if 'platform_L' in world:
            root.find('worldbody/geom').set('pos', '0 0 -.06')
        for elem in ET.fromstring('<worldbody>' + world + '</worldbody>'):
            root.find('worldbody').append(elem)
        root.find('option').set('timestep', str(timestep))
        self.xml = ET.tostring(root, encoding='unicode')
        key = (str(path), world, timestep)
        if key not in self.templates:
            self.templates[key] = mujoco.MjModel.from_xml_string(self.xml)
        self.m = copy.copy(self.templates[key])
        self.d = mujoco.MjData(self.m)
        self.m.body_ipos[1, 0] += self.com_mm / 1000
        if self.variant == 'lump':
            self.m.body_ipos[self.m.body('ASSUMED_team_body_mass_lump').id, 0] += self.com_mm / 1000
        mujoco.mj_setConst(self.m, self.d)
        if self.com_mm:
            saved = ET.fromstring(self.xml)
            for node in saved.iter('body'):
                if node.get('name') in ('chassis', 'ASSUMED_team_body_mass_lump'):
                    node.find('inertial').set('pos', ' '.join(map(str, self.m.body_ipos[self.m.body(node.get('name')).id])))
            self.xml = ET.tostring(saved, encoding='unicode')
        self.ja = {}
        self.jv = {}
        self.act = {}
        self.wheel = {}
        self.wgeom = set()
        self.offset = {}
        self.hips = {}
        self.sign = np.array([-1.0, 1.0, 1.0])
        for (s, side) in zip('LR', ('left', 'right')):
            names = {k: side + '_' + v for (k, v) in dict(hip_front='front_motor', hip_rear='rear_motor', knee_front='front_knee', knee_rear='rear_knee', wheel='wheel').items()}
            self.ja[s] = {k: self.m.joint(v).qposadr[0] for (k, v) in names.items()}
            self.jv[s] = {k: self.m.joint(v).dofadr[0] for (k, v) in names.items()}
            self.act[s] = [self.m.actuator(names[k] + '_torque').id for k in ('hip_front', 'hip_rear', 'wheel')]
            self.wheel[s] = self.m.body(side + '_wheel').id
            self.wgeom.add(self.m.geom(side + '_wheel_collision').id)
            hf = self.m.body_pos[self.m.body(side + '_front_upper').id]
            hr = self.m.body_pos[self.m.body(side + '_rear_upper').id]
            self.hips[s] = (hf + hr) / 2
            off = []
            for (b, sgn) in [('front', 1), ('rear', -1)]:
                v = self.m.body_pos[self.m.body(side + '_' + b + '_lower').id]
                off.append(np.arctan2(sgn * v[0], -v[2]))
            self.offset[s] = np.array(off + [0.0])
        self.hip_mid = np.mean(list(self.hips.values()), axis=0)
        self.hip_mid[1] = 0
        self.request_peak = np.zeros(6)
        self.applied_peak = np.zeros(6)
        self.saturation = np.zeros(6)
        self.calls = 0
        self.qrecord = []
        self.reset(L0, x0)
        self.instances.append(self)

    def pose(self, s, theta, phi, L):
        q = self.m.qpos0.copy()
        r = self.p['wheel']['radius']
        R = np.array([[np.cos(phi), 0, np.sin(phi)], [0, 1, 0], [-np.sin(phi), 0, np.cos(phi)]])
        q[:3] = np.array([s + L * np.sin(theta), 0, r + L * np.cos(theta)]) - R @ self.hip_mid
        q[3:7] = [np.cos(phi / 2), 0, np.sin(phi / 2), 0]
        for (side, word) in zip('LR', ('left', 'right')):
            foot = self.hips[side] + np.array([-L * np.sin(theta - phi), 0, -L * np.cos(theta - phi)])
            for (b, sgn) in [('front', 1), ('rear', -1)]:
                u = self.m.body(word + '_' + b + '_upper').id
                l = self.m.body(word + '_' + b + '_lower').id
                hip = self.m.body_pos[u]
                v = self.m.body_pos[l][[0, 2]]
                if b == 'front':
                    end = self.m.body_pos[self.wheel[side]][[0, 2]]
                else:
                    end = self.m.site_pos[self.m.site(word + '_rear_closure').id][[0, 2]]
                target = (foot - hip)[[0, 2]]
                dist = np.linalg.norm(target)
                a = np.linalg.norm(v)
                c = np.linalg.norm(end)
                angle = np.arctan2(sgn * target[0], -target[1]) + np.arccos(np.clip((a * a + dist * dist - c * c) / (2 * a * dist), -1, 1))
                upper = np.array([sgn * a * np.sin(angle), -a * np.cos(angle)])
                du = np.arctan2(v[0], v[1]) - np.arctan2(upper[0], upper[1])
                du = -du
                lower = target - upper
                total = np.arctan2(lower[0], lower[1]) - np.arctan2(end[0], end[1])
                total = (total + np.pi) % (2 * np.pi) - np.pi
                du = (du + np.pi) % (2 * np.pi) - np.pi
                q[self.ja[side]['hip_' + b]] = du
                q[self.ja[side]['knee_' + b]] = total - du
                if b == 'front':
                    q[self.ja[side]['wheel']] = s / r - phi - total
        return q

    def set_state(self, x, L):
        (s, sd, th, thd, ph, phd) = x
        self.d.qpos[:] = self.pose(s, th, ph, L)
        eps = 1e-06
        plus = self.pose(s + eps * sd, th + eps * thd, ph + eps * phd, L)
        minus = self.pose(s - eps * sd, th - eps * thd, ph - eps * phd, L)
        mujoco.mj_differentiatePos(self.m, self.d.qvel, 2 * eps, minus, plus)
        self.d.ctrl[:] = 0
        mujoco.mj_forward(self.m, self.d)

    def reset(self, L, x0=0, z0=0):
        mujoco.mj_resetData(self.m, self.d)
        (th, ph, _) = self.backend.equilibrium(self.p, L)
        self.set_state([x0, 0, th, 0, ph, 0], L)
        self.d.qpos[2] += z0
        mujoco.mj_forward(self.m, self.d)

    def encoder_arrays(self, data):
        q = data.qpos.copy()
        v = data.qvel.copy()
        for s in 'LR':
            for (i, k) in enumerate(('hip_front', 'hip_rear', 'wheel')):
                q[self.ja[s][k]] = self.sign[i] * q[self.ja[s][k]] + self.offset[s][i]
                v[self.jv[s][k]] *= self.sign[i]
        return (q, v)

    def read(self):
        d = self.d
        R = d.xmat[1].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        h = np.array([np.cos(yaw), np.sin(yaw), 0])
        phi = np.arctan2(R[:, 2] @ h, R[2, 2])
        (q, v) = self.encoder_arrays(d)
        o = dict(phi=phi, roll=np.arcsin(np.clip(R[2, 1], -1, 1)), yaw=yaw, phid=d.qvel[4], rolld=d.qvel[3], yawd=d.qvel[5], pos=d.qpos[:3] + R @ self.hip_mid, legs={})
        speeds = []
        res = np.zeros(6)
        for s in 'LR':
            (a, b) = (self.ja[s], self.jv[s])
            qd = np.array([v[b['hip_front']], v[b['hip_rear']]])
            J = fivebar.jacobian(q[a['hip_front']], q[a['hip_rear']], self.p)
            (L, al) = fivebar.polar(q[a['hip_front']], q[a['hip_rear']], self.p)
            (Ld, ad) = J @ qd
            o['legs'][s] = dict(L=L, alpha=al, Ld=Ld, alphad=ad, J=J, qd=qd, wheel_speed=v[b['wheel']])
            mujoco.mj_objectVelocity(self.m, d, mujoco.mjtObj.mjOBJ_BODY, self.wheel[s], res, 0)
            speeds.append(res[3:] @ h)
        o['v'] = float(np.mean(speeds))
        o['v_each'] = speeds
        return o

    def airborne(self):
        return not any((c.geom1 in self.wgeom or c.geom2 in self.wgeom for c in self.d.contact))

    def actuate(self, hips, wheels, delay_buf=None):
        if self.calls % 16 == 0:
            self.qrecord.append(self.d.qpos.copy())
        if self.m.nmocap:
            t = self.d.time
            for (j, sgn) in enumerate((1, -1)):
                self.d.mocap_pos[j, 2] = -0.015 + sgn * 0.01 * np.sin(2 * np.pi * 0.5 * t) if 6 <= t < 9 else -0.015
        requested = []
        for s in 'LR':
            vals = np.r_[hips[s], wheels[s]] * self.sign
            self.d.ctrl[self.act[s]] = np.clip(vals, [-7, -7, -2], [7, 7, 2])
            requested.extend(vals)
        requested = np.array(requested)
        applied = self.d.ctrl.copy()
        self.request_peak = np.maximum(self.request_peak, abs(requested))
        self.applied_peak = np.maximum(self.applied_peak, abs(applied))
        self.saturation += abs(requested) > np.array([7, 7, 2] * 2)
        self.calls += 1
