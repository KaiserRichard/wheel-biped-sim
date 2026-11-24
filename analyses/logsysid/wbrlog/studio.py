"""Load Studio logs, align a real log to them in time, resample and compare."""
import csv
import math

from . import spec
from .importer import Dataset, Series, interp, wrap_pi

_AX = ("x", "y", "z")


def _rows(path):
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            try:
                yield [float(v) for v in row]
            except ValueError:
                continue  # header line


def load_estimation_csv(path):
    """M13/M14 '*_estimation.csv' (SOURCE m13_score_prefix.m line 31):
    t, est pitch, truth pitch, est speed, truth speed, est L(side1,2),
    truth L(side1,2), ready. Side 1 = left (Studio axes L first)."""
    ds = Dataset({"source": "studio estimation csv"})
    names = ("est.pitch", "truth.pitch", "est.body_speed", "truth.speed",
             "est.L.L", "est.L.R", "truth.L.L", "truth.L.R", "est.ready")
    for r in _rows(path):
        if len(r) != 10:
            raise ValueError(f"expected 10 columns, got {len(r)}")
        for n, v in zip(names, r[1:]):
            ds.add(n, r[0], v)
    return ds


def load_wire_csv(path):
    """Studio M13 packet wire with time: t, imu_tick, gyro_S(3), accel_S(3),
    enc_tick, q(6), dq(6) = 21 columns (m13_sensor_block.m output + time).
    Ticks of -1 mean a dropped packet."""
    ds = Dataset({"source": "studio wire csv"})
    for r in _rows(path):
        if len(r) != 21:
            raise ValueError(f"expected 21 columns, got {len(r)}")
        t = r[0]
        if r[1] >= 0:
            ta = r[1] * 1e-3
            for i, a in enumerate(_AX):
                ds.add(f"imu.gyro_S.{a}", ta, r[2 + i])
                ds.add(f"imu.accel_S.{a}", ta, r[5 + i])
        if r[8] >= 0:
            ta = r[8] * 1e-3
            for i, ax in enumerate(spec.STUDIO_AXES):
                ds.add(f"enc.q.{ax}", ta, r[9 + i])
                ds.add(f"enc.dq.{ax}", ta, r[15 + i])
    return ds


def write_wire_csv(ds, path, t0, t1, tick=1e-3):
    """Write a robot log as the Studio M13 packet wire (21 columns, see
    load_wire_csv), one row per 1 ms tick on [t0, t1], for a packet-only replay
    of the Studio estimator (m13_boundary_worker.m pattern).

    IMU: the newest sample acquired in (t-tick, t] is sent with imu_tick = k
    (the tick it is released in; the sub-ms age is lost, which matches the
    Studio's nominal 0-tick transport latency); none -> -1 (dropped packet). Encoder: each axis is held
    at its newest CAN feedback (zero-order hold, as the firmware sees it);
    enc_tick = current tick once every axis has data, else -1. Ticks count
    from t0. Gyro/accel stay in the sensor frame S: replay with the robot's
    measured R_BS in the estimator configuration, not the Studio fixture."""
    if tick != 1e-3 or t1 < t0:
        raise ValueError("M13 wire requires a 1 ms tick and t1 >= t0")
    imu = [ds.ch[f"imu.gyro_S.{a}"] for a in _AX] + [ds.ch[f"imu.accel_S.{a}"] for a in _AX]
    qs = [ds.ch.get(f"enc.q.{ax}") for ax in spec.STUDIO_AXES]
    dqs = [ds.ch.get(f"enc.dq.{ax}") for ax in spec.STUDIO_AXES]
    import bisect
    n = int(round((t1 - t0) / tick))
    with open(path, "w", newline="") as f:
        for k in range(n + 1):
            t = t0 + k * tick
            row = [k * tick]
            i = bisect.bisect_right(imu[0].t, t + 1e-9) - 1
            if i >= 0 and imu[0].t[i] > t - tick + 1e-9:
                row += [k] + [s.y[i] for s in imu]
            else:
                row += [-1] + [0.0] * 6
            vals = []
            for s in qs + dqs:
                j = bisect.bisect_right(s.t, t + 1e-9) - 1 if s is not None else -1
                vals.append(s.y[j] if j >= 0 else None)
            if any(v is None for v in vals):
                row += [-1] + [0.0] * 12
            else:
                row += [k] + vals
            f.write(",".join(repr(v) if isinstance(v, float) else str(v) for v in row) + "\n")


def load_controller_csv(path):
    """Studio controller log with time: t + y(23) (motion_control_eval.m)."""
    ds = Dataset({"source": "studio controller csv"})
    xn = ("axle_x", "axle_v", "leg_minus_pitch", "leg_minus_pitch_rate", "pitch", "pitch_rate")
    for r in _rows(path):
        if len(r) != 24:
            raise ValueError(f"expected 24 columns, got {len(r)}")
        t, y = r[0], r[1:]
        for i, ax in enumerate(spec.STUDIO_AXES):
            ds.add(f"ctrl.applied.{ax}", t, y[i])
            ds.add(f"ctrl.request.{ax}", t, y[6 + i])
        for i, n in enumerate(xn):
            ds.add(f"ctrl.x.{n}", t, y[12 + i])
        ds.add("ctrl.L.L", t, y[18])
        ds.add("ctrl.L.R", t, y[19])
        ds.add("ctrl.phi", t, y[20])
        ds.add("ctrl.roll", t, y[21])
        ds.add("ctrl.yaw", t, y[22])
    return ds


def event_time(ds, label):
    """Time of the first EVENT whose label matches (primary sync, LOG_SPEC 4.2)."""
    for t, code, arg, lab in ds.events:
        if lab == label:
            return t
    raise KeyError(label)


def uniform_grid(t0, t1, dt):
    if dt <= 0 or t1 < t0:
        raise ValueError("require dt > 0 and t1 >= t0")
    n = int(math.floor((t1 - t0) / dt + 1e-9)) + 1
    return [t0 + k * dt for k in range(n)]


def resample(series, grid, method="linear"):
    return Series(list(grid), interp(series, grid, method))


def _sse(ref_vals, sig_vals, shift):
    """Mean squared difference ref[k] vs sig[k+shift] over the overlap."""
    n = len(ref_vals)
    lo, hi = max(0, -shift), min(n, len(sig_vals) - shift)
    s, c = 0.0, 0
    for k in range(lo, hi):
        a, b = ref_vals[k], sig_vals[k + shift]
        if a == a and b == b:
            d = a - b
            s += d * d
            c += 1
    return s / c if c else float("inf"), c


def align_offset(ref, sig, max_lag=0.5, dt=5e-4, min_overlap=0.5, t_range=None):
    """Return lag L (s) that minimises mean (ref(t) - sig(t + L))^2.

    Coarse integer search on a dt grid, parabolic refinement, then a golden
    search on the continuous lag. Works on any shared signal with motion
    (pitch, a wheel speed, a gyro axis). t_range limits the ref window."""
    if t_range is not None:
        keep = [i for i, t in enumerate(ref.t) if t_range[0] <= t <= t_range[1]]
        ref = Series([ref.t[i] for i in keep], [ref.y[i] for i in keep])
    t0 = max(ref.t[0], sig.t[0] - max_lag)
    t1 = min(ref.t[-1], sig.t[-1] + max_lag)
    grid = uniform_grid(ref.t[0], ref.t[-1], dt)
    rv = interp(ref, grid, "linear")
    sgrid = uniform_grid(ref.t[0] - max_lag, ref.t[-1] + max_lag, dt)
    sv = interp(sig, sgrid, "linear")
    m = int(round(max_lag / dt))
    need = int(min_overlap * len(grid))
    best, best_k = float("inf"), None
    costs = {}
    for k in range(-m, m + 1):
        e, c = _sse(rv, sv, k + m)
        costs[k] = e
        if c >= need and e < best:
            best, best_k = e, k
    if best_k is None:
        raise ValueError("no overlap")
    lag = best_k * dt
    if -m < best_k < m:
        a, b, c = costs[best_k - 1], costs[best_k], costs[best_k + 1]
        den = a - 2 * b + c
        if den > 0:
            lag += 0.5 * (a - c) / den * dt
    # golden refinement on the continuous lag, ref times inside [t0, t1]
    ref_t = [t for t in ref.t if t0 <= t <= t1]
    stride = max(1, len(ref_t) // 4000)
    ref_t = ref_t[::stride]
    ref_y = interp(ref, ref_t, "linear")

    def cost(L):
        y = interp(sig, [t + L for t in ref_t], "linear")
        s, c = 0.0, 0
        for a, b in zip(ref_y, y):
            if a == a and b == b:
                s += (a - b) ** 2
                c += 1
        return s / c if c else float("inf")

    lo, hi = max(-max_lag, lag - dt), min(max_lag, lag + dt)
    gr = (math.sqrt(5) - 1) / 2
    x1, x2 = hi - gr * (hi - lo), lo + gr * (hi - lo)
    f1, f2 = cost(x1), cost(x2)
    for _ in range(40):
        if f1 < f2:
            hi, x2, f2 = x2, x1, f1
            x1 = hi - gr * (hi - lo)
            f1 = cost(x1)
        else:
            lo, x1, f1 = x1, x2, f2
            x2 = lo + gr * (hi - lo)
            f2 = cost(x2)
    return 0.5 * (lo + hi)


def compare(real, studio, names, offset=0.0, method="linear", t_range=None):
    """Per channel: RMS, max |error| and sample count of real(t+offset) - studio(t),
    evaluated at the Studio sample times inside the overlap."""
    out = {}
    for n in names:
        if n not in real.ch or n not in studio.ch:
            out[n] = None
            continue
        rs, ss = real.ch[n], studio.ch[n]
        lo = max(ss.t[0], rs.t[0] - offset)
        hi = min(ss.t[-1], rs.t[-1] - offset)
        if t_range:
            lo, hi = max(lo, t_range[0]), min(hi, t_range[1])
        tq = [t for t in ss.t if lo <= t <= hi]
        angular = n.startswith("enc.q.") or n in ("ctrl.yaw", "est.yaw")
        effective_method = "angle" if angular and method == "linear" else method
        yr = interp(rs, [t + offset for t in tq], effective_method)
        ys = interp(ss, tq, "angle" if angular else "linear")
        errs = [(wrap_pi(a - b) if angular else a - b)
                for a, b in zip(yr, ys) if math.isfinite(a) and math.isfinite(b)]
        if not errs:
            out[n] = None
            continue
        out[n] = {"rms": math.sqrt(sum(e * e for e in errs) / len(errs)),
                  "max_abs": max(abs(e) for e in errs), "n": len(errs)}
    return out
