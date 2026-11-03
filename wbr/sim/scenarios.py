"""Seeded MuJoCo behaviours and fixed scoring rules."""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from wbr.control import controller as W
from .plant import Sim

OUT = Path(__file__).resolve().parents[2] / "results"
CTRL_DT = 0.002  # ASSUMED 500 Hz control rate.
STEP_WORLD = '<geom name="step" type="box" size="1.5 1 0.04" pos="{x} 0 0.04" rgba=".55 .45 .35 1" contype="1" conaffinity="1" friction="{mu} 0.01 0.001"/>'


def run(p, coef, T, script, world="", L0=0.18, truth=None, noise=0.0,
        delay_steps=0, seed=0, stop_on_fall=True, runner=None):
    if runner is not None:
        return runner(
            p, coef, T, script, world=world, L0=L0, truth=truth,
            noise=noise, delay_steps=delay_steps, seed=seed,
            stop_on_fall=stop_on_fall,
        )
    sim = Sim(p, world=world, L0=L0, truth=truth)
    ctrl = W.Controller(p, coef=coef, L_ref=L0)
    rng = np.random.default_rng(seed)
    dt = sim.m.opt.timestep
    every = int(round(CTRL_DT / dt))
    n = int(round(T / dt))
    rows, qlog = [], []
    hips, wheels = {s: np.zeros(2) for s in "LR"}, {s: 0.0 for s in "LR"}
    queue = []
    fell = False
    for i in range(n):
        t = i * dt
        script(t, sim, ctrl)
        if i % every == 0:
            o = sim.read()
            if noise:
                o["phi"] += rng.normal(0, 0.2 * noise * np.pi / 180)
                o["phid"] += rng.normal(0, noise * np.pi / 180)
                o["v"] += rng.normal(0, 0.02 * noise)
            cmd = ctrl.step(o, CTRL_DT, sim.airborne())
            queue.append(cmd)
            hips, wheels = queue.pop(0) if len(queue) > delay_steps else ({s: np.zeros(2) for s in "LR"}, {s: 0.0 for s in "LR"})
            rows.append([t, o["phi"], o["roll"], o["yaw"], o["yawd"], o["v"], ctrl.s, ctrl.s_ref, ctrl.log["Lm"], ctrl.log["T"], ctrl.log["Tp"], o["pos"][0], o["pos"][1], o["pos"][2], float(sim.airborne()), wheels["L"], wheels["R"], *hips["L"], *hips["R"]])
            if i % 16 == 0:
                qlog.append(sim.d.qpos.copy())
            if abs(o["phi"]) > 1.0 or o["pos"][2] < 0.10 and stop_on_fall:
                fell = True
                if stop_on_fall:
                    break
        sim.actuate(hips, wheels)
        mujoco.mj_step(sim.m, sim.d)
        if not np.isfinite(sim.d.qacc).all():
            fell = True
            break
    cols = "t phi roll yaw yawd v s s_ref L T Tp x y z air wL wR hfL hrL hfR hrR".split()
    return {c: np.array([r[i] for r in rows]) for i, c in enumerate(cols)}, np.array(qlog), fell, sim


def rms(a):
    return float(np.sqrt(np.mean(np.square(a))))


def stand(p, coef, **kw):
    def script(t, sim, c):
        # 2 N s forward push on the body at t = 5 s, delivered over 50 ms.
        sim.d.xfrc_applied[1, 0] = 40.0 if 5.0 <= t < 5.05 else 0.0
    r, q, fell, _ = run(p, coef, 10.0, script, **kw)
    calm = r["t"] < 5.0
    m = dict(fell=fell, phi_rms_deg=float(np.degrees(rms(r["phi"][calm]))), phi_peak_after_push_deg=float(np.degrees(np.abs(r["phi"][~calm]).max())), x_drift_m=float(abs(r["x"][calm][-1])), x_peak_push_m=float(np.abs(r["x"]).max()), final_phi_deg=float(np.degrees(r["phi"][-1])), peak_wheel_nm=float(np.abs(r["wL"]).max()))
    m["pass"] = bool(not fell and m["phi_rms_deg"] < 1.0 and m["x_drift_m"] < 0.05 and abs(m["final_phi_deg"]) < 2.0)
    return m, r, q


def drive(p, coef, **kw):
    def script(t, sim, c):
        c.v_ref = 1.0 * min(max(t - 1, 0) / 1.5, 1) if t < 6 else max(1.0 - (t - 6) / 1.5, 0)
    r, q, fell, _ = run(p, coef, 11.0, script, **kw)
    cruise = (r["t"] > 3.5) & (r["t"] < 6)
    m = dict(fell=fell, speed_err_cruise=float(np.abs(r["v"][cruise] - 1.0).mean()) if cruise.any() else None, phi_peak_deg=float(np.degrees(np.abs(r["phi"]).max())), distance_m=float(r["x"][-1]), final_speed=float(r["v"][-1]))
    m["pass"] = bool(not fell and m["speed_err_cruise"] is not None and m["speed_err_cruise"] < 0.1 and abs(m["final_speed"]) < 0.1)
    return m, r, q


def turn(p, coef, **kw):
    def script(t, sim, c):
        c.yawd_ref = 1.5 if 2 <= t < 6 else 0.0
        c.v_ref = 0.5 * min(max(t - 1, 0), 1)
    r, q, fell, _ = run(p, coef, 9.0, script, **kw)
    w = (r["t"] > 3) & (r["t"] < 6)
    yawd = float(r["yawd"][w].mean()) if w.any() else None
    m = dict(fell=fell, yaw_rate_mean=yawd, roll_peak_deg=float(np.degrees(np.abs(r["roll"]).max())), phi_peak_deg=float(np.degrees(np.abs(r["phi"]).max())), total_yaw_rad=float(np.unwrap(r["yaw"])[-1]))
    m["pass"] = bool(not fell and yawd is not None and abs(yawd - 1.5) < 0.15 and m["roll_peak_deg"] < 15)
    return m, r, q


def height(p, coef, **kw):
    fb = p["fivebar"]
    lo, hi = fb["leg_height_min"] + 0.02, fb["leg_height_max"] - 0.015

    def script(t, sim, c):
        seq = [(0, 0.18), (1, lo), (4, hi), (7, 0.18)]
        target = [v for t0, v in seq if t >= t0][-1]
        c.L_ref += np.clip(target - c.L_ref, -0.1 * 0.001, 0.1 * 0.001)  # 0.1 m/s leg rate
    r, q, fell, _ = run(p, coef, 10.0, script, **kw)
    m = dict(fell=fell, L_min=float(r["L"].min()), L_max=float(r["L"].max()), L_target=[lo, hi], phi_peak_deg=float(np.degrees(np.abs(r["phi"]).max())), x_drift=float(abs(r["x"][-1])))
    m["pass"] = bool(not fell and m["L_min"] < lo + 0.01 and m["L_max"] > hi - 0.01 and m["phi_peak_deg"] < 10)
    return m, r, q


def jump(p, coef, **kw):
    def script(t, sim, c):
        if t >= 2.0 and c.jump is None:
            c.start_jump()
    r, q, fell, sim = run(p, coef, 6.0, script, **kw)
    air = r["air"] > 0.5
    z_base = r["z"][r["t"] < 2].mean()
    r_w = p["wheel"]["radius"]
    m = dict(fell=fell, air_time_s=float(air.sum() * CTRL_DT), body_rise_m=float(r["z"].max() - z_base), phi_peak_deg=float(np.degrees(np.abs(r["phi"]).max())), recovered=bool(not fell and abs(r["phi"][-1]) < np.radians(5)))
    m["pass"] = bool(m["air_time_s"] > 0.1 and m["recovered"])
    return m, r, q


def obstacle(p, coef, trigger=0.18, v=0.6, **kw):
    """8 cm step, its front face 1.0 m ahead. Drives up, jumps at a fixed distance, must land on top and balance."""
    def script(t, sim, c):
        # Speed is held through the jump: dropping it at the crouch brakes the wheels and tips the body at take-off.
        target = v * min(t / 1.0, 1) if sim.d.qpos[0] < 1.3 else 0.0
        c.v_ref += np.clip(target - c.v_ref, -0.0007, 0.0007)
        if c.jump is None and sim.d.qpos[0] > 1.0 - trigger - 0.35 * v:
            c.start_jump(crouch_time=0.25)
    world = STEP_WORLD.format(x=2.5, mu=(kw.get("truth") or p)["friction"])
    r, q, fell, _ = run(p, coef, 8.0, script, world=world, **kw)
    m = dict(fell=fell, final_x=float(r["x"][-1]), final_z=float(r["z"][-1]), z_start=float(r["z"][0]), air_time_s=float((r["air"] > 0.5).sum() * CTRL_DT), phi_peak_deg=float(np.degrees(np.abs(r["phi"]).max())))
    on_top = m["final_x"] > 1.03 and m["final_z"] - m["z_start"] > 0.04
    m["pass"] = bool(not fell and on_top and abs(r["phi"][-1]) < np.radians(10))
    return m, r, q


def rolling_into_step(p, coef, **kw):
    """Control case: no jump, only drive into the 8 cm step. The trade study predicts this cannot climb."""
    def script(t, sim, c):
        c.v_ref = 0.6 * min(t, 1) if t < 6 else 0.0
    r, q, fell, _ = run(p, coef, 7.0, script, world=STEP_WORLD.format(x=2.5, mu=(kw.get("truth") or p)["friction"]), **kw)
    m = dict(fell=fell, final_x=float(r["x"][-1]), final_z=float(r["z"][-1]), z_start=float(r["z"][0]))
    m["pass"] = bool(m["final_x"] > 1.03 and m["final_z"] - m["z_start"] > 0.04)
    return m, r, q


ALL = dict(stand=stand, drive=drive, turn=turn, height=height, jump=jump, obstacle=obstacle, rolling_into_step=rolling_into_step)


def main():
    p = W.load_design()
    coef, rep = W.design_gains(p)
    print("gains: worst eig", rep["worst_eig"], "fitted", rep["worst_eig_fit"], "fit err", rep["fit_max_err"])
    names = sys.argv[1:] or list(ALL)
    for name in names:
        m, r, q = ALL[name](p, coef)
        print(name, json.dumps(m))


if __name__ == "__main__":
    main()
