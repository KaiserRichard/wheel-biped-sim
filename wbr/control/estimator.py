"""Own complementary attitude filter and [body forward speed, accel bias] KF.

Capability boundary: accepts only Packet plus nominal geometry, never a plant.
All filter constants are ASSUMED tuning choices, not hardware measurements.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from wbr.dynamics import fivebar
from wbr.sim.sensors import Packet


class Estimator:
    def __init__(self, p):
        self.p = p
        self.R = np.eye(3)  # ASSUMED upright startup, yaw zero; no truth initialisation.
        self.x = np.zeros(2)
        self.P = np.diag([.1, .1])
        self.position = np.zeros(3)
        self.last_foot = None
        self.air = False
        self.low_force_time = 0.
        self.updates = 0

    def update(self, samples, dt=.002):
        if not samples or any(type(s) is not Packet for s in samples):
            raise TypeError("Only sensor packets may enter the estimator")
        # Consume both 1 kHz samples at the 500 Hz control invocation.
        h = dt / len(samples)
        for s in samples:
            a, g = s.accel, s.gyro
            norm = np.linalg.norm(a)
            expected = self.R.T @ np.array([0., 0., 1.])
            # Reject free fall and impact. Linear acceleration near 1 g is
            # intrinsically ambiguous; weak correction limits false tilt.
            weight = max(0., 1. - abs(norm / self.p["gravity"] - 1.) / .15)
            correction = .25 * weight * np.cross(a / max(norm, 1e-9), expected)
            self.R = self.R @ Rotation.from_rotvec((g + correction) * h).as_matrix()
        s = samples[-1]
        a = np.mean([z.accel for z in samples], axis=0)
        g = np.mean([z.gyro for z in samples], axis=0)
        R = self.R
        yaw = np.arctan2(R[1, 0], R[0, 0])
        heading = np.array([np.cos(yaw), np.sin(yaw), 0.])
        phi = np.arctan2(R[:, 2] @ heading, R[2, 2])
        roll = np.arcsin(np.clip(R[2, 1], -1, 1))
        legs, v_each, feet, foot_vels = {}, [], [], []
        for i, side in enumerate("LR"):
            q1, q2, _ = s.angles[3*i:3*i+3]
            rates = s.rates[3*i:3*i+2]
            J = fivebar.jacobian(q1, q2, self.p)
            L, alpha = fivebar.polar(q1, q2, self.p)
            Ld, alphad = J @ rates
            foot = fivebar.fk(q1, q2, self.p)
            knee = fivebar.knees(q1, q2, self.p)[0]
            # Differentiate front lower-link orientation using hip encoders.
            eps = 1e-5
            f2 = fivebar.fk(q1 + eps*rates[0], q2 + eps*rates[1], self.p)
            k2 = fivebar.knees(q1 + eps*rates[0], q2 + eps*rates[1], self.p)[0]
            u, ud = foot-knee, (f2-k2-foot+knee)/eps
            lower_rate = (u[1]*ud[0] - u[0]*ud[1]) / (u @ u)
            # Joint wheel rate is relative to the moving front lower link.
            v_each.append(self.p["wheel"]["radius"] * (s.rates[3*i+2] + lower_rate + g[1]))
            body_foot = np.array([foot[0], 0., foot[1]])
            feet.append(R @ body_foot)
            foot_vels.append(R @ (np.cross(g, body_foot) +
                np.array([(f2-foot)[0]/eps, 0., (f2-foot)[1]/eps])))
            legs[side] = dict(L=L, alpha=alpha, Ld=Ld, alphad=alphad, J=J,
                             qd=rates.copy(), wheel_speed=float(s.rates[3*i+2]))
        rel_v = float(np.mean(foot_vels, axis=0) @ heading)
        norm = np.linalg.norm(a)
        self.low_force_time = self.low_force_time + dt if norm < .3*self.p["gravity"] else 0.
        if self.low_force_time >= .004:
            self.air = True
        elif norm > .65*self.p["gravity"]:
            self.air = False
        A = np.array([[1., -dt], [0., 1.]])
        forward_acc = float((R @ a) @ heading)
        self.x = A @ self.x + np.array([dt * forward_acc, 0.])
        self.P = A @ self.P @ A.T + np.diag([.35**2*dt**2, .005**2*dt])
        if not self.air:
            innovation = np.mean(v_each) - rel_v - self.x[0]
            variance = .04**2
            K = self.P[:, 0] / (self.P[0, 0] + variance)
            self.x += K * innovation
            self.P -= np.outer(K, self.P[0, :])
        v = float(self.x[0] + rel_v)
        self.position += heading * self.x[0] * dt
        self.position[2] = self.p["wheel"]["radius"] - np.mean(feet, axis=0)[2]
        self.updates += 1
        return dict(phi=float(phi), roll=float(roll), yaw=float(yaw),
                    phid=float(g[1]), rolld=float(g[0]), yawd=float(g[2]),
                    pos=self.position.copy(), legs=legs, v=v, v_each=v_each)
