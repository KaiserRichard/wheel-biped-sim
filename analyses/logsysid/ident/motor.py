"""One DM motor on the stand, wheel free: simulator and fits.

Studio values (SOURCE wbr-studio/params/drive_system_params.m, wheel column):
equivalent torque constant 0.2 N*m/A, first-order torque lag 5 ms, Coulomb
0.01 N*m, viscous 0.001 N*m*s/rad. Encoder-rate noise 0.02 rad/s
(sensor_params.m). Wheel inertia: ASSUMED 1.0e-3 kg*m^2 (the CAD tensor lives
in wbr_data.mat and was not read here).

What the DM motor reports (INFERRED): 'tau' in the feedback frame is the
motor's own estimate Kt_fw * iq. So, with k = Kt_true / Kt_fw:
    tau_fb    = lag(u(t - d))            (commanded units, k not visible)
    tau_shaft = k * tau_fb
    J dw/dt   = tau_shaft - Tc tanh(w/w_s) - b w + cogging(theta)
Hence from free-spin data alone only k/J, Tc/J, b/J are identifiable.
A static lever test gives k; J follows as k / (k/J).
"""
import bisect
import math
import random

from wbrlog import dm
from .linalg import lstsq, nelder_mead, corr_from_cov, inv

STUDIO_WHEEL = {"Kt": 0.2, "lag": 0.005, "Tc": 0.01, "b": 0.001}
J_ASSUMED = 1.0e-3
W_SMOOTH = 0.05     # rad/s, Coulomb smoothing (tanh)

# Rehearsal truth: Studio values deliberately perturbed so that a fit that
# returned the nominal values would fail the recovery check.
TRUTH = {"k": 1.08, "J": 1.0e-3, "Tc": 0.012, "b": 0.0012, "lag": 0.004, "d": 0.0007,
         "cog_amp": 0.004, "cog_per_rev": 20}
SENSOR = {"tau_noise": 0.003, "w_noise": 0.020, "pmax": 12.5, "vmax": 45.0, "tmax": 10.0}


def quant(x, lo, hi, bits):
    return dm.uint_to_float(dm.float_to_uint(x, lo, hi, bits), lo, hi, bits)


# ------------------------------------------------------------ E-M1 lever
def lever_experiment(levels, truth, rng, r_lever=0.100, r_err_rel=0.005, f_noise=0.02,
                     readings_per_level=1):
    """Wheel held by a lever on a scale. Sweep up then down through levels.
    Returns rows (tau_cmd, F_measured, direction) and the lever length used
    (with its calibration error, which is systematic)."""
    r_used = r_lever * (1.0 + rng.gauss(0.0, r_err_rel))
    rows = []
    seq = [(l, +1) for l in levels] + [(l, -1) for l in reversed(levels)]
    for tau, direction in seq:
        true_torque = truth["k"] * tau - truth["Tc"] * direction * 0.5  # static hysteresis band
        f = true_torque / r_lever
        f_meas = sum(f + rng.gauss(0.0, f_noise) for _ in range(readings_per_level)) / readings_per_level
        rows.append((tau, f_meas, direction))
    return rows, r_used


def fit_lever(rows, r_used):
    X = [[tau, d, 1.0] for tau, f, d in rows]
    y = [f * r_used for tau, f, d in rows]
    beta, cov, rss, dof = lstsq(X, y)
    return {"k": beta[0], "k_se": math.sqrt(cov[0][0]), "hyst": beta[1], "offset": beta[2]}


# ------------------------------------------------------- E-M2 plateaus
def plateau_samples(w, T, truth, sensor, rng, fs=1000.0, settle=0.3):
    """tau_fb samples on a constant-speed plateau (MIT velocity damping)."""
    n = int(T * fs)
    out = []
    theta = rng.uniform(0, 2 * math.pi)
    for i in range(n):
        t = settle + i / fs
        th = theta + w * t
        cog = truth["cog_amp"] * math.sin(truth["cog_per_rev"] * th)
        shaft = truth["Tc"] * math.tanh(w / W_SMOOTH) + truth["b"] * w + cog
        tfb = shaft / truth["k"] + rng.gauss(0.0, sensor["tau_noise"])
        out.append(quant(tfb, -sensor["tmax"], sensor["tmax"], 12))
    return out


def friction_experiment(speeds, T, truth, sensor, rng, fs=1000.0):
    rows = []
    for w in speeds:
        s = plateau_samples(w, T, truth, sensor, rng, fs)
        rows.append((w, sum(s) / len(s), len(s)))
    return rows


def fit_friction(rows, k):
    """Regress mean tau_fb on [sign(w), w]; scale by k to shaft torque."""
    X = [[math.copysign(1.0, w), w] for w, m, n in rows]
    y = [m for w, m, n in rows]
    beta, cov, _, _ = lstsq(X, y)
    return {"Tc": k * beta[0], "b": k * beta[1],
            "Tc_se": k * math.sqrt(cov[0][0]), "b_se": k * math.sqrt(cov[1][1])}


# --------------------------------------------- E-M3 torque steps (spin)
def step_experiment(T, truth, sensor, rng, period=0.003, amp=0.15, kd_host=0.004,
                    rtt=300e-6, dt=1e-4):
    """Square-wave torque with random switch times; host adds a weak speed
    damping so the wheel stays inside VMAX. One command per 'period'
    (3 ms = DM-jump cadence, 1 ms = bench build). Returns logged arrays:
    tx times, commanded torque (as transmitted, quantised), rx times,
    tau_fb, w_fb, plus the truth trace for checks.
    rtt, d and period must be multiples of dt: the reply samples the state
    at the first grid step at or after tx + rtt, so an off-grid rtt shows up
    as a false delay bias (a 250 us rtt on a 100 us grid gave -50 us)."""
    for v in (rtt, truth["d"], period):
        if abs(v / dt - round(v / dt)) > 1e-6:
            raise ValueError("rtt, d and period must be multiples of dt")
    tmax, vmax = sensor["tmax"], sensor["vmax"]
    k, J, Tc, b, lag, d = (truth[x] for x in ("k", "J", "Tc", "b", "lag", "d"))
    sw_times = []
    t = 0.05
    while t < T:
        sw_times.append(t)
        t += rng.uniform(0.06, 0.14)
    level = amp
    tau_m, w, th = 0.0, 0.0, rng.uniform(0, 2 * math.pi)
    tx_t, tx_u, rx_t, rx_tau, rx_w = [], [], [], [], []
    pending = []          # (apply_time, u)
    u_applied = 0.0
    w_fb_last = 0.0
    next_tx = 0.0
    si = 0
    nsteps = int(T / dt)
    for i in range(nsteps):
        t = i * dt
        if t >= next_tx - 1e-12:
            while si < len(sw_times) and sw_times[si] <= t:
                level = -level
                si += 1
            u = quant(level - kd_host * w_fb_last, -tmax, tmax, 12)
            tx_t.append(t)
            tx_u.append(u)
            pending.append((t + d, u))
            # reply: state sampled at t + rtt (approximation: current state
            # advanced below once rtt has elapsed)
            next_tx += period
        while pending and pending[0][0] <= t + 1e-12:
            u_applied = pending.pop(0)[1]
        # record replies whose time has come
        while len(rx_t) < len(tx_t) and tx_t[len(rx_t)] + rtt <= t + 1e-12:
            tf = quant(tau_m + rng.gauss(0, sensor["tau_noise"]), -tmax, tmax, 12)
            wf = quant(w + rng.gauss(0, sensor["w_noise"]), -vmax, vmax, 12)
            rx_t.append(tx_t[len(rx_t)] + rtt)
            rx_tau.append(tf)
            rx_w.append(wf)
            w_fb_last = wf
        # integrate (exact for the lag over dt, Euler for the mechanics)
        a = math.exp(-dt / lag)
        tau_m = u_applied + (tau_m - u_applied) * a
        cog = truth["cog_amp"] * math.sin(truth["cog_per_rev"] * th)
        wd = (k * tau_m - Tc * math.tanh(w / W_SMOOTH) - b * w + cog) / J
        w += wd * dt
        th += w * dt
    return {"tx_t": tx_t, "tx_u": tx_u, "rx_t": rx_t, "rx_tau": rx_tau, "rx_w": rx_w}


def lag_response(tx_t, tx_u, d, lag, tq):
    """First-order response to the piecewise-constant input u(t - d), at sorted
    query times tq, starting from 0."""
    out = []
    y, t_last, u = 0.0, 0.0, 0.0
    j = 0
    n = len(tx_t)
    for q in tq:
        while j < n and tx_t[j] + d <= q:
            te = tx_t[j] + d
            y = u + (y - u) * math.exp(-(te - t_last) / lag)
            t_last, u = te, tx_u[j]
            j += 1
        out.append(u + (y - u) * math.exp(-(q - t_last) / lag))
        y, t_last = out[-1], q
    return out


def fit_lag_delay(log, x0=(0.001, 0.005)):
    """Output-error fit of (d, lag) to tau_fb. Returns estimates, 1-sigma
    errors and the correlation of the two from the local Jacobian."""
    tq, yq = log["rx_t"], log["rx_tau"]

    def cost(p):
        d, lag = p
        if d < 0 or lag <= 1e-5:
            return 1e9
        m = lag_response(log["tx_t"], log["tx_u"], d, lag, tq)
        return sum((a - b) ** 2 for a, b in zip(m, yq)) / len(yq)

    p, c = nelder_mead(cost, list(x0), [0.0005, 0.002], tol=1e-12, max_iter=400)
    d, lag = p
    sigma2 = c
    J = _jac(lambda pp: lag_response(log["tx_t"], log["tx_u"], pp[0], pp[1], tq), p, [1e-5, 1e-5])
    info = [[sum(J[k][i] * J[k][j] for k in range(len(J))) for j in range(2)] for i in range(2)]
    cov = [[v * sigma2 for v in r] for r in inv(info)]
    corr = corr_from_cov(cov)
    return {"d": d, "lag": lag, "d_se": math.sqrt(cov[0][0]), "lag_se": math.sqrt(cov[1][1]),
            "corr": corr[0][1], "rms": math.sqrt(c)}


def _jac(model, p, h):
    base = model(p)
    cols = []
    for i in range(len(p)):
        q = list(p)
        q[i] += h[i]
        y = model(q)
        cols.append([(a - b) / h[i] for a, b in zip(y, base)])
    return [[cols[i][k] for i in range(len(p))] for k in range(len(base))]


def fit_spin(log, d, lag, window=0.3, fine=2e-4, friction=None):
    """Integral equation error on the speed record, windowed:
        w(t) - w(t0) = a * I_tau - (Tc/J) * I_s - (b/J) * I_w
    with I_tau = int tau_fb_model, I_s = int tanh(w/ws), I_w = int w.
    friction=None: fit (a, Tc/J, b/J) jointly (shows they are poorly
    identifiable from slow spin data).
    friction=(Tc, b) in N*m from the plateau test: fit (a, 1/J) with
        dw = a * I_tau - (1/J) * (Tc * I_s + b * I_w)."""
    rows = _spin_rows(log, d, lag, window, fine)
    y = [r[3] for r in rows]
    if friction is None:
        X = [[r[0], -r[1], -r[2]] for r in rows]
        beta, cov, rss, dof = lstsq(X, y)
        return {"a": beta[0], "c": beta[1], "beta": beta[2],
                "se": [math.sqrt(cov[i][i]) for i in range(3)], "corr": corr_from_cov(cov)}
    Tc, b = friction
    X = [[r[0], -(Tc * r[1] + b * r[2])] for r in rows]
    beta, cov, rss, dof = lstsq(X, y)
    return {"a": beta[0], "invJ": beta[1], "se": [math.sqrt(cov[i][i]) for i in range(2)],
            "corr": corr_from_cov(cov)}


def _spin_rows(log, d, lag, window, fine):
    t_end = log["rx_t"][-1]
    grid = [i * fine for i in range(int(t_end / fine) + 1)]
    tau_g = lag_response(log["tx_t"], log["tx_u"], d, lag, grid)
    rt, rw = log["rx_t"], log["rx_w"]

    def w_at(t):
        i = bisect.bisect_right(rt, t)
        if i == 0:
            return rw[0]
        if i == len(rt):
            return rw[-1]
        f = (t - rt[i - 1]) / (rt[i] - rt[i - 1])
        return rw[i - 1] + f * (rw[i] - rw[i - 1])

    w_g = [w_at(t) for t in grid]
    rows = []
    win_start = rt[0]
    Itau = Is = Iw = 0.0
    g0 = int(win_start / fine)
    gi = g0
    w0 = w_at(win_start)
    for t_i, w_i in zip(rt, rw):
        if t_i - win_start > window:
            win_start, w0 = t_i, w_i
            Itau = Is = Iw = 0.0
            gi = int(t_i / fine)
            continue
        while gi + 1 < len(grid) and grid[gi + 1] <= t_i:
            Itau += 0.5 * (tau_g[gi] + tau_g[gi + 1]) * fine
            Is += 0.5 * (math.tanh(w_g[gi] / W_SMOOTH) + math.tanh(w_g[gi + 1] / W_SMOOTH)) * fine
            Iw += 0.5 * (w_g[gi] + w_g[gi + 1]) * fine
            gi += 1
        if Itau == 0.0:
            continue
        rows.append((Itau, Is, Iw, w_i - w0))
    return rows


def speed_channel_corr(truth, sensor, T=3.0, period=0.001, seed=5):
    """Local identifiability of (d, lag) from the SPEED record alone:
    correlation of the two sensitivities (finite differences of noise-free
    simulations driven by the same command)."""
    def run(d, lag):
        tr = dict(truth, d=d, lag=lag, cog_amp=0.0)
        sn = dict(sensor, tau_noise=0.0, w_noise=0.0)
        rng = random.Random(seed)
        return step_experiment(T, tr, sn, rng, period=period, kd_host=0.0,
                               dt=2e-5)["rx_w"]
    base = run(truth["d"], truth["lag"])
    h = 2e-5
    jd = [(a - b) / h for a, b in zip(run(truth["d"] + h, truth["lag"]), base)]
    jl = [(a - b) / h for a, b in zip(run(truth["d"], truth["lag"] + h), base)]
    sdd = sum(x * x for x in jd)
    sll = sum(x * x for x in jl)
    sdl = sum(x * y for x, y in zip(jd, jl))
    # covariance ~ inv(info); correlation of the estimates is -sdl/sqrt(sdd*sll)
    return -sdl / math.sqrt(sdd * sll)
