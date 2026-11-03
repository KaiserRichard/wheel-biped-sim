"""Seeded CPU runner: sensor packets enter control, truth only enters scoring."""
from types import SimpleNamespace
import mujoco
import numpy as np
from .plant import Sim
from .sensors import Sensors
from .scenarios import CTRL_DT, rms
from wbr.control.controller import Controller
from wbr.control.estimator import Estimator
from wbr.dynamics import model

def forbidden_read(*args,**kwargs):
    raise AssertionError("Control attempted plant truth access")

class Runner:
    def __init__(self,plant=Sim,backend=model,initial_length=.18,controller_options=None,sensor_class=Sensors,slip=False):
        self.plant=plant;self.backend=backend;self.initial_length=initial_length
        self.controller_options=controller_options or {};self.sensor_class=sensor_class
        self.slip=slip;self.diagnostics={};self.trace={}
    def __call__(self, p, coef, T, script, world='', L0=None, truth=None, noise=0.0, delay_steps=0, seed=0, stop_on_fall=True):
        if noise:
            raise ValueError('Noise is defined in Sensors, not truth observations')
        L0 = self.initial_length if L0 is None else L0
        sim = self.plant(p, world=world, L0=L0, truth=truth, backend=self.backend)
        (sensors, est) = (self.sensor_class(sim, seed), Estimator(p))
        ctrl = Controller(p, coef=coef, L_ref=L0, backend=self.backend, **self.controller_options)
        ctrl.initial_length=L0
        dt = sim.m.opt.timestep
        every = int(round(CTRL_DT / dt))
        assert np.isclose(dt, sensors.config.dt)
        assert every == 2
        observations=[]
        (rows, qlog, estimates, extras, queue, packets) = ([], [], [], [], [], [])
        (hips, wheels) = ({s: np.zeros(2) for s in 'LR'}, {s: 0.0 for s in 'LR'})
        proxy_data = SimpleNamespace(qpos=est.position.copy(), xfrc_applied=sim.d.xfrc_applied)
        proxy = SimpleNamespace(d=proxy_data)
        fell = False
        friction = sim.m.geom_friction.copy()
        original_read = sim.read
        sim.read = forbidden_read
        try:
            for i in range(int(round(T / dt))):
                t = i * dt
                proxy_data.qpos = est.position.copy()
                script(t, proxy, ctrl)
                if self.slip:
                    sim.m.geom_friction[:, 0] = 0.001 if 3.0 <= t < 4.0 else friction[:, 0]
                mujoco.mj_forward(sim.m, sim.d)
                packets.append(sensors.sample(sim.d))
                if i % every == 0:
                    o = est.update(packets, len(packets) * dt)
                    norm = float(np.linalg.norm(np.mean([s.accel for s in packets], axis=0)))
                    packets = []
                    cmd = ctrl.step(o, CTRL_DT, est.air)
                    observations.append([*o["pos"],ctrl.s,ctrl.s_ref,*ctrl.integral,ctrl.log["x"][4],ctrl.phase=="flight",o["phid"]])
                    queue.append(cmd)
                    (hips, wheels) = queue.pop(0) if len(queue) > delay_steps else ({s: np.zeros(2) for s in 'LR'}, {s: 0.0 for s in 'LR'})
                    tr = original_read()
                    air = sim.airborne()
                    rows.append([t, tr['phi'], tr['roll'], tr['yaw'], tr['yawd'], tr['v'], ctrl.s, ctrl.s_ref, np.mean([tr['legs'][s]['L'] for s in 'LR']), ctrl.log['T'], ctrl.log['Tp'], *tr['pos'], float(air), wheels['L'], wheels['R'], *hips['L'], *hips['R']])
                    estimates.append([o['phi'], o['v'], o['roll'], np.mean(o['v_each'])])
                    extras.append([norm, float(est.air), float(ctrl.phase == 'flight')])
                    if i % 16 == 0:
                        qlog.append(sim.d.qpos.copy())
                    if abs(tr['phi']) > 1.0 or tr['pos'][2] < 0.1:
                        fell = True
                        if stop_on_fall:
                            break
                sim.actuate(hips, wheels)
                mujoco.mj_step(sim.m, sim.d)
                if not np.isfinite(sim.d.qacc).all():
                    fell = True
                    break
        finally:
            sim.read = original_read
        cols = 't phi roll yaw yawd v s s_ref L T Tp x y z air wL wR hfL hrL hfR hrR'.split()
        r = {c: np.array([row[j] for row in rows]) for (j, c) in enumerate(cols)}
        (estimates, extras) = (np.array(estimates), np.array(extras))
        pe = estimates[:, 0] - r['phi']
        ve = estimates[:, 1] - r['v']
        air = r['air'] > 0.5
        land = np.flatnonzero(air[:-1] & ~air[1:]) + 1
        landing = np.zeros(len(air), dtype=bool)
        for j in land:
            landing[j:j + 50] = True

        def subset(mask):
            return dict(samples=int(mask.sum()), pitch_rms_deg=float(np.degrees(rms(pe[mask]))), speed_rms_m_s=rms(ve[mask]), specific_force_mean_m_s2=float(extras[mask, 0].mean()), specific_force_peak_m_s2=float(extras[mask, 0].max())) if mask.any() else None
        self.diagnostics = dict(pitch_rms_deg=float(np.degrees(rms(pe))), speed_rms_m_s=rms(ve), wheel_ground_discrepancy_peak_m_s=float(np.max(np.abs(estimates[:, 3] - r['v']))), roll_rms_deg=float(np.degrees(rms(estimates[:, 2] - r['roll']))), flight=subset(air), landing_100ms=subset(landing), sensor_air_disagreement_fraction=float(np.mean(air != (extras[:, 1] > 0.5))), sensor_samples=sensors.samples, estimator_updates=est.updates, guarded_truth_access=True, scorer_truth_reads=len(rows), elapsed_s=float(r['t'][-1]), sensors=sensors.report())
        self.trace = dict(**r, estimated_phi=estimates[:, 0], estimated_v=estimates[:, 1], estimated_roll=estimates[:, 2], encoder_rolling_speed=estimates[:, 3], specific_force_norm=extras[:, 0], estimated_air=extras[:, 1], controller_flight=extras[:, 2])
        obs=np.asarray(observations)
        for j,name in enumerate("estimated_x estimated_y estimated_z integrated_s reference_s integral_s integral_phi pitch_error controller_flight_logged estimated_phid".split()):
            self.trace[name]=obs[:,j]
        return (r, np.array(qlog), fell, sim)
