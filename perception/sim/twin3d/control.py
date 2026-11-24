"""New length-scheduled reduced LQR and lesson-05 fixed target jump machine."""

from functools import lru_cache
from pathlib import Path

import numpy as np

from sim.planar import Plant as ReducedPlant, load_config
from sim.lqr import synthesize
from .model import STAND_LENGTH, R_WHEEL, WHEEL_LIMIT, HIP_LIMIT, LEG_LIMIT

LENGTHS = (0.08, 0.12, 0.16, 0.20)  # ASSUMED design grid, m.


@lru_cache(maxsize=16)
def design(length):
    """CALCULATED DARE from existing planar.py/lqr.py, updated mass partition."""
    cfg = load_config(Path(__file__).resolve().parents[1] / 'configs/assumed_planar.json')
    values = dict(wheel_radius=R_WHEEL, wheel_mass=1.4,
                  wheel_inertia=0.5*1.4*R_WHEEL**2, leg_length=length,
                  leg_mass=0.4, leg_com=0.06, leg_inertia=0.0006,
                  body_mass=3.5, body_com=-0.03, body_inertia=3.5*(0.30**2+0.12**2)/12)
    for key, value in values.items():
        cfg['physics'][key]['value'] = value
    cfg['simulation']['dt_s'] = 0.001
    return synthesize(ReducedPlant(cfg))


class Jump:
    """SOURCE thresholds with accumulating per-side controller-pass counters."""
    def __init__(self, stand=STAND_LENGTH):
        self.phase, self.stand = 'STAND', stand
        self.airborne_seen = False
        self.passes = {'left': 0, 'right': 0}

    def update(self, lengths, trigger=False):
        if not isinstance(lengths, dict):
            lengths = {'left': lengths, 'right': lengths}
        if self.phase == 'STAND' and trigger:
            self.phase = 'CROUCH'
        elif self.phase in ('CROUCH', 'THRUST', 'RETRACT'):
            threshold = {'CROUCH': lambda x: x < 0.10,
                         'THRUST': lambda x: x > 0.18,
                         'RETRACT': lambda x: x < self.stand+0.01}[self.phase]
            for side in ('left', 'right'):
                if threshold(lengths[side]):
                    self.passes[side] += 1
            if all(value >= 10 for value in self.passes.values()):
                self.phase = {'CROUCH': 'THRUST', 'THRUST': 'RETRACT',
                              'RETRACT': 'SETTLE'}[self.phase]
                self.passes = {'left': 0, 'right': 0}
        return dict(CROUCH=0.08, THRUST=0.21).get(self.phase, self.stand)


class Controller:
    def __init__(self):
        self.gains = np.stack([design(length)[0] for length in LENGTHS])
        self.jump = Jump()
        self.xref = 0.0
        self.flight_angle = None

    def gain(self, length):
        return np.array([[np.interp(length, LENGTHS, self.gains[:, i, j])
                          for j in range(6)] for i in range(2)])

    def update(self, plant, data, speed=0.0, trigger=False):
        state = plant.state(data)
        length = plant.length(data)
        lengths = {side: float(data.qpos[plant.q['length_'+side]]) for side in ('left', 'right')}
        target = self.jump.update(lengths, trigger)
        contact = plant.contacts(data)
        airborne = not (contact['left'] or contact['right'])
        flight = airborne and self.jump.phase not in ('STAND', 'CROUCH', 'THRUST')
        self.jump.airborne_seen |= flight
        self.xref += speed*plant.model.opt.timestep
        error = state.copy()
        error[0] -= self.xref
        error[3] -= speed
        pair = -self.gain(length) @ error
        if airborne:
            pair[0] = 0.0
        mean_hip = np.mean([data.qpos[plant.q['hip_'+s]] for s in ('left', 'right')])
        mean_rate = np.mean([data.qvel[plant.q['hip_'+s]] for s in ('left', 'right')])
        if flight:
            if self.flight_angle is None:
                self.flight_angle = mean_hip
        else:
            self.flight_angle = None
        for side in ('left', 'right'):
            ql = plant.q['length_'+side]
            # ASSUMED PD N/m, Ns/m. CALCULATED static body-weight feedforward.
            kp, kd = (5000, 16) if self.jump.phase == 'THRUST' else (1800, 24)
            force = kp*(target-data.qpos[ql]) - kd*data.qvel[ql]
            if not flight:
                force += 3.5*9.81/2
            hip = -pair[1]/2
            if flight:
                qh = plant.q['hip_'+side]
                hip = 8*(self.flight_angle-data.qpos[qh])-0.3*data.qvel[qh]
            # ASSUMED differential stabilization: the planar model cannot roll,
            # but its two independent hips still have an antisymmetric mode.
            qh = plant.q['hip_'+side]
            hip -= 20*(data.qpos[qh]-mean_hip)
            hip -= 0.8*(data.qvel[qh]-mean_rate)
            data.ctrl[plant.u['wheel_'+side]] = np.clip(pair[0]/2, -WHEEL_LIMIT, WHEEL_LIMIT)
            data.ctrl[plant.u['hip_'+side]] = np.clip(hip, -HIP_LIMIT, HIP_LIMIT)
            data.ctrl[plant.u['length_'+side]] = np.clip(force, -LEG_LIMIT, LEG_LIMIT)
        return flight
