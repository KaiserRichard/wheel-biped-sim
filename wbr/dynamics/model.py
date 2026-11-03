"""Numeric reduced model built on generated_model.py: equilibrium, nonlinear dynamics, A(L), B(L)."""
import numpy as np

from . import fivebar
from . import generated_model as gm

# State order: [s, s_dot, theta, theta_dot, phi, phi_dot]. Input: [T_wheel, T_hip].


def constants(p, L):
    leg = fivebar.lumped_leg(L, p)
    b, w = p["body"], p["wheel"]
    return dict(
        mw=2 * w["mass"],
        Iw=2 * w["inertia_axial"],
        r=w["radius"],
        ml=leg["mass"],
        lc=leg["lc"],
        Il=leg["inertia"],
        L=L,
        mb=b["mass"],
        Ib=b["inertia_diag"][1],
        cx=b["com_offset_xz"][0],
        cz=b["com_offset_xz"][1],
        g=p["gravity"],
    )


def _call(f, q, qd, c):
    return f(q[0], q[1], q[2], qd[0], qd[1], qd[2], c["mw"], c["Iw"], c["r"], c["ml"], c["lc"], c["Il"], c["L"], c["mb"], c["Ib"], c["cx"], c["cz"], c["g"])


def equilibrium(p, L, phi0=0.0):
    """Leg lean and hip torque that hold still with the body at phi0 and no wheel torque."""
    c = constants(p, L)
    zero = np.zeros(3)
    # Adding the theta and phi rows cancels Tp, leaving a scalar gravity balance in theta.
    bal = lambda th: _call(gm.bias, np.array([0.0, float(th), phi0]), zero, c).ravel()
    f = lambda th: float(bal(th)[1] + bal(th)[2])
    lo, hi = -0.8, 0.8
    flo = f(lo)
    for _ in range(80):
        mid = (lo + hi) / 2
        fm = f(mid)
        if fm * flo > 0:
            lo, flo = mid, fm
        else:
            hi = mid
    th0 = (lo + hi) / 2
    return th0, phi0, float(-bal(th0)[2])


def xdot(x, u, p, L, c=None):
    c = c or constants(p, L)
    q, qd = x[[0, 2, 4]], x[[1, 3, 5]]
    qdd = np.linalg.solve(_call(gm.mass_matrix, q, qd, c), gm.input_map(c["r"]) @ u - _call(gm.bias, q, qd, c).ravel())
    return np.array([qd[0], qdd[0], qd[1], qdd[1], qd[2], qdd[2]])


def linearize(p, L, phi0=0.0):
    """Returns A, B, x0, u0 about the balance point at this leg length."""
    c = constants(p, L)
    th0, ph0, Tp0 = equilibrium(p, L, phi0)
    q0, qd0 = np.array([0.0, th0, ph0]), np.zeros(3)
    Minv = np.linalg.inv(_call(gm.mass_matrix, q0, qd0, c))
    G, D = _call(gm.dh_dq, q0, qd0, c), _call(gm.dh_dqd, q0, qd0, c)
    A = np.zeros((6, 6))
    B = np.zeros((6, 2))
    pos, vel = [0, 2, 4], [1, 3, 5]
    for i in range(3):
        A[pos[i], vel[i]] = 1.0
    A[np.ix_(vel, pos)] = -Minv @ G
    A[np.ix_(vel, vel)] = -Minv @ D
    B[vel, :] = Minv @ gm.input_map(c["r"])
    x0 = np.array([0.0, 0.0, th0, 0.0, ph0, 0.0])
    return A, B, x0, np.array([0.0, Tp0])


def simulate(x0, ufun, T, p, L, dt=1e-3):
    """RK4 on the nonlinear reduced model."""
    c = constants(p, L)
    n = int(round(T / dt))
    xs = np.zeros((n + 1, 6))
    xs[0] = x0
    for i in range(n):
        x, ti = xs[i], i * dt
        k1 = xdot(x, ufun(ti, x), p, L, c)
        k2 = xdot(x + dt / 2 * k1, ufun(ti + dt / 2, x), p, L, c)
        k3 = xdot(x + dt / 2 * k2, ufun(ti + dt / 2, x), p, L, c)
        k4 = xdot(x + dt * k3, ufun(ti + dt, x), p, L, c)
        xs[i + 1] = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return xs


def yaw_model(p):
    """Differential wheel torque to yaw: Izz_eff * psi_dd = (T_right - T_left) * track / (2 r)."""
    w, tr = p["wheel"], p["track_width"]
    fb = p["fivebar"]
    leg_m = 2 * (fb["upper_mass"] + fb["lower_mass"])
    izz = p["body"]["inertia_diag"][2] + 2 * (w["mass"] + leg_m) * (tr / 2) ** 2 + 2 * w["inertia_axial"] * (tr / (2 * w["radius"])) ** 2
    return {"Izz_eff": izz, "gain_per_diff_torque": tr / (2 * w["radius"] * izz)}


def roll_model(p, L):
    """Static body roll from a leg-length difference, for level ground: roll = atan(dL / track)."""
    return {"roll_per_dL": 1.0 / p["track_width"], "track": p["track_width"], "L": L}
