"""Own WBR controller: LQR K(L) for balance, VMC for the legs, PD for yaw, roll and leg sync, and a jump state machine."""
import json
from pathlib import Path

import numpy as np
from scipy.linalg import solve_continuous_are
from wbr.dynamics import fivebar, model

HERE = Path(__file__).parent

from wbr.control.lqr import design_gains, gain


class Controller:
    def __init__(
        self, p, coef=None, L_ref=0.18, gains=None, backend=model,
        delay_mode=False, phase_schedule=True, step_face_x=None,
        compact_jump=False,
    ):
        self.p, self.L_ref = p, L_ref
        self.backend = backend
        self.delay_mode = delay_mode
        self.phase_schedule = phase_schedule
        self.step_face_x = step_face_x
        self.compact_jump = compact_jump
        self.coef = coef if coef is not None else design_gains(p)[0]
        self.g = dict(kL=800.0, dL=50.0, kr=40.0, dr=5.0, ks=30.0, ds=2.0, ky=0.3, kyi=1.0, yi_max=0.5)
        if delay_mode:
            self.g.update(dL=15.0, dr=2.0, ds=0.5)
        self.g.update(gains or {})
        self.s_ref, self.v_ref, self.yawd_ref, self.s, self.t = 0.0, 0.0, 0.0, 0.0, 0.0
        self.yaw_i = 0.0
        self.integral = np.zeros(2)
        self.phase = "balance"
        self.jump = None
        self.log = {}
        self.delay_coef = None
        if delay_mode:
            lengths = np.linspace(p["fivebar"]["leg_height_min"], p["fivebar"]["leg_height_max"], 15)
            augmented = []
            q = np.diag([30.0, 15.0, 30.0, 2.0, 120.0, 3.0, 40.0, 400.0])
            r = np.diag([12.0, 4.0])
            for length in lengths:
                a, b, *_ = backend.linearize(p, float(length))
                aa = np.zeros((8, 8)); aa[:6, :6] = a
                aa[6, 0] = 1.0; aa[7, 4] = 1.0
                bb = np.zeros((8, 2)); bb[:6] = b
                augmented.append(np.linalg.solve(r, bb.T @ solve_continuous_are(aa, bb, q, r)))
            self.delay_coef = np.polyfit(lengths, np.asarray(augmented).reshape(15, 16), 3)

    def start_jump(self, crouch=0.13, tuck=0.13, land=0.19, crouch_time=0.35):
        if self.compact_jump:
            crouch, tuck, land, crouch_time = 0.11, 0.12, 0.17, 0.25
        self.jump = dict(crouch=crouch, tuck=tuck, land=land, crouch_time=crouch_time, t0=self.t, L_prev=self.L_ref)
        self.phase = "crouch"

    def _jump_logic(self, o, air):
        j = self.jump
        Lm = np.mean([o["legs"][s]["L"] for s in "LR"])
        Lmax = self.p["fivebar"]["leg_height_max"]
        if self.phase == "crouch":
            if self.step_face_x is not None:
                lead_time = 0.28
                minimum_crouch = 0.10
                ready = o["pos"][0] >= self.step_face_x - lead_time * max(o["v"], 0.0)
                if ready and self.t - j["t0"] >= minimum_crouch:
                    j["crouch_time"] = 0.0
            self.L_ref = j["crouch"]
            if self.t - j["t0"] > j["crouch_time"]:
                self.phase = "push"
                j["t_push"] = self.t
        elif self.phase == "push":
            if Lm > Lmax - 0.012 or air or self.t - j["t_push"] > 0.35:
                self.phase = "flight"
                j["t_flight"] = self.t
        elif self.phase == "flight":
            self.L_ref = j["tuck"]
            if self.t - j["t_flight"] > 0.05 and not air:
                self.phase = "land"
                j["t_land"] = self.t
                self.s_ref = self.s
            if self.t - j["t_flight"] > 1.0:
                self.phase = "land"
                j["t_land"] = self.t
        elif self.phase == "land":
            self.L_ref = j["land"]
            if self.t - j["t_land"] > 0.5:
                self.phase = "balance"

    def step(self, o, dt, air=False):
        p, g = self.p, self.g
        self.t += dt
        self.s += o["v"] * dt
        self.s_ref += self.v_ref * dt
        if self.phase != "balance":
            self._jump_logic(o, air)
        fb = p["fivebar"]
        L_l, L_r = o["legs"]["L"]["L"], o["legs"]["R"]["L"]
        Lm = float(np.clip((L_l + L_r) / 2, fb["leg_height_min"], fb["leg_height_max"]))
        al_m = (o["legs"]["L"]["alpha"] + o["legs"]["R"]["alpha"]) / 2
        ald_m = (o["legs"]["L"]["alphad"] + o["legs"]["R"]["alphad"]) / 2
        th, thd = o["phi"] + al_m, o["phid"] + ald_m
        th0, ph0, Tp0 = self.backend.equilibrium(p, Lm)
        x = np.array([np.clip(self.s - self.s_ref, -0.3, 0.3), o["v"] - self.v_ref, th - th0, thd, o["phi"] - ph0, o["phid"]])
        T, Tp = -gain(self.coef, Lm) @ x
        delayed_phase = self.phase == "balance" if self.phase_schedule else self.phase in ("balance", "crouch", "land")
        if self.delay_coef is not None and delayed_phase and not air:
            self.integral += dt * x[[0, 4]]
            self.integral = np.clip(self.integral, [-0.5, -0.3], [0.5, 0.3])
            k = np.array([np.polyval(self.delay_coef[:, i], Lm) for i in range(16)]).reshape(2, 8)
            correction = -(k[:, :6] - gain(self.coef, Lm)) @ x - k[:, 6:] @ self.integral
            T += correction[0]
            Tp += correction[1]
        Tp += Tp0
        if self.phase == "flight":
            # No ground to push on: hold the legs in line with the body, spin nothing.
            T = 0.0
            Tp = -40.0 * al_m - 3.0 * ald_m
        elif self.phase == "push":
            T = 0.5 * T
        # PI on yaw rate: tyre scrub is a steady load, so proportional alone leaves an offset. No ground, no integration.
        yaw_err = self.yawd_ref - o["yawd"]
        if not air:
            self.yaw_i = float(np.clip(self.yaw_i + g["kyi"] * yaw_err * dt, -g["yi_max"], g["yi_max"]))
        Ty = g["ky"] * yaw_err + self.yaw_i
        if self.phase == "flight":
            Ty = 0.0
        roll_f = g["kr"] * o["roll"] + g["dr"] * o["rolld"]
        F0 = (p["body"]["mass"] + self.backend.constants(p, Lm)["ml"] / 2) * p["gravity"] / 2 * 1.0
        hips, wheels = {}, {}
        for s, sign in (("L", 1.0), ("R", -1.0)):
            leg = o["legs"][s]
            other = o["legs"]["R" if s == "L" else "L"]
            if self.phase == "push":
                # Full extension: each hip gets its largest usable torque.
                J = leg["J"]
                F = p["hip_motor"]["peak_torque"] / max(abs(J[0, 0]), abs(J[0, 1]), 1e-3)
            else:
                F = F0 + g["kL"] * (self.L_ref - leg["L"]) - g["dL"] * leg["Ld"]
                F -= sign * roll_f
                if self.phase == "flight":
                    F = g["kL"] * (self.L_ref - leg["L"]) - g["dL"] * leg["Ld"]
            sync = g["ks"] * (other["alpha"] - leg["alpha"]) + g["ds"] * (other["alphad"] - leg["alphad"])
            hips[s] = leg["J"].T @ np.array([F, Tp / 2 + sync])
            wheels[s] = T / 2 - sign * Ty
        self.log = dict(x=x, T=T, Tp=Tp, Ty=Ty, Lm=Lm, phase=self.phase)
        return hips, wheels


def load_design(path=None):
    return fivebar.load_params(path or HERE.parent.parent / "models" / "design.json")


def save_gains(path, report, coef):
    Path(path).write_text(json.dumps(dict(report, poly_coef=coef.tolist()), indent=1))
