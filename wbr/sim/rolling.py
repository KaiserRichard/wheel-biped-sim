"""World-referenced rolling resistance for loaded wheels on a stationary floor."""

import mujoco
import numpy as np


class RollingLoss:
    """Own the rotational applied wrench of each supplied wheel body."""

    def __init__(self, model, data, joints, radius, coefficient, smoothing):
        if not np.isfinite([radius, coefficient, smoothing]).all():
            raise ValueError("rolling parameters must be finite")
        if radius <= 0 or coefficient < 0 or smoothing <= 0:
            raise ValueError("positive radius and smoothing and nonnegative coefficient required")
        self.model, self.data = model, data
        self.joints = tuple(joints)
        self.bodies = tuple(int(model.jnt_bodyid[joint]) for joint in self.joints)
        self.radius = radius
        self.coefficient = coefficient
        self.smoothing = smoothing
        self.velocity = np.zeros(6)

    def clear(self):
        """Clear the owned torques before updating contact forces and velocities."""
        for body in self.bodies:
            self.data.xfrc_applied[body, 3:] = 0.0

    def apply(self, loads):
        """Apply resistance from refreshed normal loads in newtons, in joint order."""
        loads = np.asarray(loads, dtype=float)
        if loads.shape != (len(self.joints),) or not np.isfinite(loads).all() or (loads < 0).any():
            raise ValueError("provide one finite nonnegative normal load per wheel")

        data = self.data
        for joint, body, load in zip(self.joints, self.bodies, loads):
            mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_BODY, body,
                                    self.velocity, 0)
            axis = data.xaxis[joint]
            speed = self.velocity[:3] @ axis
            torque = -self.coefficient * load * self.radius * np.tanh(speed / self.smoothing)
            data.xfrc_applied[body, 3:] = torque * axis
