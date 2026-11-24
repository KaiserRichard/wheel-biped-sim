"""Loop delay: from IMU sample to torque on the shaft.

Budget (LOG_SPEC 4.4):  imu_age + (t_TX_axis - t_CTRL) + d_motor + lag_motor
  imu_age        CTRL time minus the stamp of the IMU sample it used (logged)
  t_TX - t_CTRL  queue position: DM-jump sends three frames 1 ms apart
  d_motor, lag   from the stand experiment E-M3 (ident.motor)
The INS attitude filter's group delay on the pitch ANGLE is not visible in
time stamps; method B below measures the composite on the rate path.

Method A: time stamps only (exact to the 1 us clock).
Method B: no instrumentation. Regress the logged torque command on the
logged gyro history, tau_k = K1 * phi(t_k - D) + K2 * w(t_k - D) + c,
and pick D by least squares (1-D search, sub-sample by interpolation).
"""
import bisect
import math
import random

from .linalg import lstsq, percentile


# ------------------------------------------------------------- method A
def delay_from_timestamps(ds, axes):
    """Per axis: list of (imu_age + TX offset) over controller ticks."""
    ctrl_t = ds.ch["ctrl.imu_age"].t
    ages = ds.ch["ctrl.imu_age"].y
    out = {}
    for ax in axes:
        tx = ds.ch[f"ctrl.applied.{ax}"].t
        vals = []
        for k, (tc, age) in enumerate(zip(ctrl_t, ages)):
            i = bisect.bisect_left(tx, tc)
            end = ctrl_t[k + 1] if k + 1 < len(ctrl_t) else (
                tc + ctrl_t[-1] - ctrl_t[-2] if len(ctrl_t) > 1 else tc)
            if i < len(tx) and tx[i] < end:
                vals.append(age + (tx[i] - tc))
        out[ax] = vals
    return out


def summarize(vals):
    return {"mean": sum(vals) / len(vals), "p50": percentile(vals, 0.5),
            "p99": percentile(vals, 0.99), "max": max(vals), "n": len(vals)}


# ------------------------------------------------------------- method B
def rocking_signal(rng, n_sines=12, f_lo=0.5, f_hi=8.0, amp=0.4):
    comps = [(rng.uniform(f_lo, f_hi), rng.uniform(0, 2 * math.pi)) for _ in range(n_sines)]
    scale = amp / math.sqrt(n_sines / 2)

    def w(t):
        return scale * sum(math.sin(2 * math.pi * f * t + p) for f, p in comps)

    def phi(t):
        return scale * sum((-math.cos(2 * math.pi * f * t + p) + math.cos(p)) / (2 * math.pi * f) for f, p in comps)
    return w, phi


def simulate_rocking(T, D, rng, K1=-8.0, K2=-0.8, other_std=0.05, gyro_std=0.0115,
                     imu_fs=1000.0, period=0.003, gyro_lsb=2000.0 / 32768 * math.pi / 180,
                     tau_lsb=20.0 / 4095):
    """Chassis rocked by hand on the stand, motors in a low-gain torque mode
    (EXPERIMENTS R4). The controller uses phi integrated from the gyro and the
    gyro itself, delayed by D; 'other_std' lumps every other state term."""
    w, phi = rocking_signal(rng)
    imu_t = [k / imu_fs for k in range(int(T * imu_fs))]
    imu_w = [round((w(t) + rng.gauss(0, gyro_std)) / gyro_lsb) * gyro_lsb for t in imu_t]
    tx_t, tx_u = [], []
    t = 0.05
    while t < T:
        u = K1 * phi(t - D) + K2 * w(t - D) + rng.gauss(0, other_std)
        tx_t.append(t)
        tx_u.append(round(u / tau_lsb) * tau_lsb)
        t += period
    return imu_t, imu_w, tx_t, tx_u


def fit_delay_regression(imu_t, imu_w, tx_t, tx_u, d_max=0.012, step=2.5e-4):
    dt = imu_t[1] - imu_t[0]
    # integrate gyro to angle (trapezoid), then high-pass by removing a fit constant
    ang = [0.0]
    for i in range(1, len(imu_w)):
        ang.append(ang[-1] + 0.5 * (imu_w[i] + imu_w[i - 1]) * dt)

    def at(arr, t):
        x = (t - imu_t[0]) / dt
        i = int(math.floor(x))
        if i < 0 or i + 1 >= len(arr):
            return None
        f = x - i
        return arr[i] + f * (arr[i + 1] - arr[i])

    def rss(D):
        X, y = [], []
        for t, u in zip(tx_t, tx_u):
            a, b = at(ang, t - D), at(imu_w, t - D)
            if a is None or b is None:
                continue
            X.append([a, b, 1.0, t])
            y.append(u)
        beta, cov, r, dof = lstsq(X, y)
        return r / dof, beta, cov

    grid = [k * step for k in range(int(d_max / step) + 1)]
    costs = [rss(D)[0] for D in grid]
    i = min(range(len(grid)), key=lambda k: costs[k])
    D = grid[i]
    if 0 < i < len(grid) - 1:
        a, b, c = costs[i - 1], costs[i], costs[i + 1]
        den = a - 2 * b + c
        if den > 0:
            D += 0.5 * (a - c) / den * step
    r, beta, cov = rss(D)
    return {"D": D, "K1": beta[0], "K2": beta[1], "cost_curve": list(zip(grid, costs))}
