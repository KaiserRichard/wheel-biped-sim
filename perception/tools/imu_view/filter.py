"""Tilt from raw accel + gyro: accel-only, gyro-only and complementary estimates.

Robot frame (same as MuJoCo world at rest): x forward, y left, z up.
  pitch = rotation about +y. pitch > 0 means NOSE DOWN (+x tips toward the floor).
  roll  = rotation about +x. roll  > 0 means LEFT SIDE UP (right side down).
Angles follow the ZYX (yaw-pitch-roll) order, so at rest the accelerometer reads
  a = g * (-sin(pitch), cos(pitch) sin(roll), cos(pitch) cos(roll)).
Yaw is not estimated: without a magnetometer it only drifts.
"""

import math
from dataclasses import dataclass


def accel_tilt(ax, ay, az):
    """(pitch, roll) in rad from specific force; valid only when not accelerating."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return pitch, roll


def euler_rates(pitch, roll, gx, gy, gz):
    """ZYX Euler-angle rates (pitch_dot, roll_dot) from body rates in rad/s."""
    tp = math.tan(pitch)
    roll_dot = gx + math.sin(roll) * tp * gy + math.cos(roll) * tp * gz
    pitch_dot = math.cos(roll) * gy - math.sin(roll) * gz
    return pitch_dot, roll_dot


def alpha_for(tau_s, dt_s):
    """Blend weight for a crossover time constant tau: alpha = tau / (tau + dt)."""
    return tau_s / (tau_s + dt_s)


@dataclass
class Estimate:
    pitch: float = 0.0
    roll: float = 0.0


class TiltFilters:
    """Runs three estimators side by side so their failure modes can be compared.

    tau_s = 0.5 s with 100 Hz samples gives alpha = 0.98: trust the gyro for fast
    motion, let the accelerometer pull the angle back over ~0.5 s.
    """

    def __init__(self, tau_s=0.5):
        self.tau_s = tau_s
        self.acc = Estimate()
        self.gyro = Estimate()
        self.comp = Estimate()
        self.started = False

    def update(self, accel, gyro, dt):
        """accel in any unit (only direction matters), gyro in rad/s, dt in s."""
        ap, ar = accel_tilt(*accel)
        self.acc = Estimate(ap, ar)
        if not self.started:
            # Start both integrators at the accel angle, not at zero.
            self.gyro = Estimate(ap, ar)
            self.comp = Estimate(ap, ar)
            self.started = True
            return self.comp
        pd, rd = euler_rates(self.gyro.pitch, self.gyro.roll, *gyro)
        self.gyro = Estimate(self.gyro.pitch + pd * dt, self.gyro.roll + rd * dt)
        a = alpha_for(self.tau_s, dt)
        pd, rd = euler_rates(self.comp.pitch, self.comp.roll, *gyro)
        self.comp = Estimate(a * (self.comp.pitch + pd * dt) + (1 - a) * ap,
                             a * (self.comp.roll + rd * dt) + (1 - a) * ar)
        return self.comp


class Madgwick:
    """Madgwick IMU (accel + gyro) quaternion filter, for comparison only.

    beta = 0.05 is a common default; larger trusts the accelerometer more.
    """

    def __init__(self, beta=0.05):
        self.beta = beta
        self.q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z; body -> world
        self.started = False

    def update(self, accel, gyro, dt):
        ax, ay, az = accel
        n = math.sqrt(ax * ax + ay * ay + az * az)
        if not self.started and n > 0:
            p, r = accel_tilt(ax, ay, az)
            self.q = quat_from_pitch_roll(p, r)
            self.started = True
            return self.pitch_roll()
        q0, q1, q2, q3 = self.q
        gx, gy, gz = gyro
        qd = [0.5 * (-q1 * gx - q2 * gy - q3 * gz),
              0.5 * (q0 * gx + q2 * gz - q3 * gy),
              0.5 * (q0 * gy - q1 * gz + q3 * gx),
              0.5 * (q0 * gz + q1 * gy - q2 * gx)]
        if n > 0:
            ax, ay, az = ax / n, ay / n, az / n
            # Gradient of |q* (0,0,1) q - a|^2 (standard Madgwick IMU form).
            f1 = 2 * (q1 * q3 - q0 * q2) - ax
            f2 = 2 * (q0 * q1 + q2 * q3) - ay
            f3 = 2 * (0.5 - q1 * q1 - q2 * q2) - az
            s = [-2 * q2 * f1 + 2 * q1 * f2,
                 2 * q3 * f1 + 2 * q0 * f2 - 4 * q1 * f3,
                 -2 * q0 * f1 + 2 * q3 * f2 - 4 * q2 * f3,
                 2 * q1 * f1 + 2 * q2 * f2]
            sn = math.sqrt(sum(v * v for v in s))
            if sn > 0:
                qd = [qd[i] - self.beta * s[i] / sn for i in range(4)]
        q = [self.q[i] + qd[i] * dt for i in range(4)]
        qn = math.sqrt(sum(v * v for v in q))
        self.q = [v / qn for v in q]
        return self.pitch_roll()

    def pitch_roll(self):
        w, x, y, z = self.q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        return Estimate(pitch, roll)


def quat_from_pitch_roll(pitch, roll):
    """Quaternion (w, x, y, z) for R = Ry(pitch) Rx(roll), yaw = 0."""
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return [cp * cr, cp * sr, sp * cr, -sp * sr]
