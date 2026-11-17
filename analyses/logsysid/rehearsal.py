"""P3 identification rehearsal on synthetic data built from Studio parameters.

    python rehearsal.py            # full run, writes rehearsal_results.json, REHEARSAL.md
    python rehearsal.py --quick    # small Monte Carlo (used by the tests)

Standard library only. Every random draw is seeded; the run is reproducible.
"""
import json
import math
import os
import random
import sys
import time
import platform

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ident import allan, delay, motor  # noqa: E402
from ident.linalg import percentile  # noqa: E402
from wbrlog import importer, spec, studio, synth  # noqa: E402

D2R = math.pi / 180


def p90abs(x):
    return percentile([abs(v) for v in x], 0.9)


def medabs(x):
    return percentile([abs(v) for v in x], 0.5)


def first_ok(rows, key, target):
    for r in rows:
        if r[key] <= target:
            return r["T"]
    return None


# ------------------------------------------------------------------ 1
def run_allan(quick):
    durations = [60, 300, 900] if quick else [60, 300, 900, 3600]
    seeds = 2 if quick else 5
    dt = 0.01  # analysis rate; N is preserved by rescaling (allan.simulate)
    scen = {
        "gyro_studio": dict(allan.STUDIO_GYRO),
        "gyro_board": dict(allan.STUDIO_GYRO, sigma_w=allan.BOARD_GYRO_STD),
        "accel_studio": dict(allan.STUDIO_ACCEL),
    }
    out = {}
    for name, p in scen.items():
        N_true = p["sigma_w"] * math.sqrt(1e-3)
        K_true = p["drift"]
        rows = []
        for T in durations:
            eN, eK, eb = [], [], []
            for s in range(seeds):
                rng = random.Random(1000 * T + s)
                y, b0 = allan.simulate(int(T / dt), dt, rng=rng, **p)
                f = allan.fit(allan.allan(y, dt), terms=("N", "K"))
                eN.append(f["N"] / N_true - 1)
                eK.append(f["K"] / K_true - 1 if f["K"] > 0 else -1.0)
                Tb = min(T, allan.best_bias_window(N_true, K_true))
                nb = max(1, int(Tb / dt))
                eb.append(sum(y[:nb]) / nb - b0)
            rows.append({"T": T, "N_med": medabs(eN), "N_p90": p90abs(eN),
                         "K_med": medabs(eK), "K_p90": p90abs(eK),
                         "bias_err_rms": math.sqrt(sum(e * e for e in eb) / len(eb))})
        T_star = allan.best_bias_window(N_true, K_true)
        out[name] = {"N_true": N_true, "K_true": K_true, "bias_std_prior": p["bias_std"],
                     "per_sample_std_1kHz": p["sigma_w"], "rows": rows,
                     "T_best_bias": T_star, "bias_err_at_T_best": allan.bias_error_std(N_true, K_true, T_star),
                     "bias_err_3s": allan.bias_error_std(N_true, K_true, 3.0),
                     "need_N10": first_ok(rows, "N_p90", 0.10), "need_K25": first_ok(rows, "K_p90", 0.25),
                     "seeds": seeds}
    return out


# ------------------------------------------------------------------ 2
def run_lever(quick):
    seeds = 50 if quick else 400
    rows = []
    for n in (3, 5, 10):
        levels = [0.1 * (i + 1) * 5 / n for i in range(n)]  # 0..0.5 N*m
        err = []
        for s in range(seeds):
            rng = random.Random(7000 + 100 * n + s)
            data, r = motor.lever_experiment(levels, motor.TRUTH, rng)
            err.append(motor.fit_lever(data, r)["k"] / motor.TRUTH["k"] - 1)
        rows.append({"levels_per_sweep": n, "T": 2 * n * 3.0, "k_med": medabs(err), "k_p90": p90abs(err)})
    return {"rows": rows, "need_k2": first_ok(rows, "k_p90", 0.02), "seeds": seeds,
            "note": "3 s per level, up and down sweeps; lever 0.100 m with 0.5 % length error, scale noise 0.02 N"}


# ------------------------------------------------------------------ 3
def run_friction(quick):
    speeds = [-32, -16, -8, -4, -2, -1, 1, 2, 4, 8, 16, 32]
    Tps = [0.5, 2.0] if quick else [0.5, 1.0, 2.0, 6.3, 12.6]
    seeds = 3 if quick else 12
    rows = []
    for Tp in Tps:
        eT, eb = [], []
        for s in range(seeds):
            rng = random.Random(9000 + int(Tp * 10) * 100 + s)
            fr = motor.friction_experiment(speeds, Tp, motor.TRUTH, motor.SENSOR, rng)
            f = motor.fit_friction(fr, motor.TRUTH["k"])
            eT.append(f["Tc"] / motor.TRUTH["Tc"] - 1)
            eb.append(f["b"] / motor.TRUTH["b"] - 1)
        rows.append({"T_plateau": Tp, "T": Tp * len(speeds) + 0.3 * len(speeds),
                     "Tc_med": medabs(eT), "Tc_p90": p90abs(eT), "b_med": medabs(eb), "b_p90": p90abs(eb)})
    need = None
    for r in rows:
        if r["Tc_p90"] <= 0.05 and r["b_p90"] <= 0.05:
            need = r["T"]
            break
    return {"rows": rows, "speeds": speeds, "need_5pct": need, "seeds": seeds,
            "lsb_tau": 20.0 / 4095, "Tc_over_lsb": motor.TRUTH["Tc"] / (20.0 / 4095)}


# ------------------------------------------------------------------ 4
def run_steps(quick):
    Ts = [1.0, 2.0] if quick else [1.0, 2.0, 5.0, 10.0]
    seeds = 2 if quick else 6
    tr = motor.TRUTH
    a_true = tr["k"] / tr["J"]
    out = {}
    for per in (0.001, 0.003):
        rows = []
        for T in Ts:
            ed, el, ea, cr = [], [], [], []
            for s in range(seeds):
                rng = random.Random(4000 + int(T * 10) * 10 + s + int(per * 1e4))
                log = motor.step_experiment(T, tr, motor.SENSOR, rng, period=per)
                ld = motor.fit_lag_delay(log)
                sp = motor.fit_spin(log, ld["d"], ld["lag"], friction=(tr["Tc"], tr["b"]))
                ed.append(ld["d"] - tr["d"])
                el.append(ld["lag"] / tr["lag"] - 1)
                ea.append(sp["a"] / a_true - 1)
                cr.append(ld["corr"])
            rows.append({"T": T, "d_med_us": medabs(ed) * 1e6, "d_p90_us": p90abs(ed) * 1e6,
                         "lag_p90": p90abs(el), "a_p90": p90abs(ea), "corr_d_lag": sum(cr) / len(cr)})
        need = None
        for r in rows:
            if r["d_p90_us"] <= 200 and r["lag_p90"] <= 0.10 and r["a_p90"] <= 0.02:
                need = r["T"]
                break
        out[f"period_{int(per * 1e3)}ms"] = {"rows": rows, "need": need}
    # joint friction from spin data only (identifiability check)
    rng = random.Random(4242)
    log = motor.step_experiment(5.0 if quick else 10.0, tr, motor.SENSOR, rng, period=0.001)
    ld = motor.fit_lag_delay(log)
    j = motor.fit_spin(log, ld["d"], ld["lag"])
    out["joint_spin"] = {"a_rel": j["a"] / a_true - 1, "Tc_over_J_rel": j["c"] / (tr["Tc"] / tr["J"]) - 1,
                         "b_over_J_rel": j["beta"] / (tr["b"] / tr["J"]) - 1,
                         "corr_c_beta": j["corr"][1][2], "corr_a_c": j["corr"][0][1]}
    out["speed_only_corr_d_lag"] = motor.speed_channel_corr(tr, motor.SENSOR, T=2.0 if quick else 3.0)
    out["truth"] = dict(tr)
    out["seeds"] = seeds
    return out


# ------------------------------------------------------------------ 5
def run_calibration_chain(quick):
    """Propagate fitted lever/friction estimates into an independent spin fit."""
    rows = []
    for seed in range(3 if quick else 12):
        rng = random.Random(19000 + seed)
        lever, radius = motor.lever_experiment([.1, .2, .3, .4, .5], motor.TRUTH, rng)
        k = motor.fit_lever(lever, radius)["k"]
        plateaus = motor.friction_experiment([-16, -4, -1, 1, 4, 16], 6.3,
                                             motor.TRUTH, motor.SENSOR, rng)
        friction = motor.fit_friction(plateaus, k)
        log = motor.step_experiment(5.0, motor.TRUTH, motor.SENSOR, rng)
        lag = motor.fit_lag_delay(log)
        spin = motor.fit_spin(log, lag["d"], lag["lag"],
                              friction=(friction["Tc"], friction["b"]))
        rows.append({"seed": 19000 + seed, "k": k, "Kt_conditional": k * motor.STUDIO_WHEEL["Kt"],
                     "Tc": friction["Tc"], "b": friction["b"], "lag": lag["lag"],
                     "delay": lag["d"], "k_over_J": spin["a"], "J": k / spin["a"]})
    truth = {"k": motor.TRUTH["k"], "Tc": motor.TRUTH["Tc"], "b": motor.TRUTH["b"],
             "lag": motor.TRUTH["lag"], "J": motor.TRUTH["J"],
             "k_over_J": motor.TRUTH["k"] / motor.TRUTH["J"]}
    return {"rows": rows, "relative_error_p90": {
        key: p90abs([r[key] / value - 1 for r in rows]) for key, value in truth.items()},
        "duration_s": 30 + 6 * (6.3 + .3) + 5,
        "note": "Independent lever and plateau data; no true friction or k passed to fits."}


def _flat_signals():
    zero3 = lambda t: (0.0, 0.0, 9.80665)  # noqa: E731
    return {"gyro_S": lambda t: (0.0, 0.0, 0.0), "accel_S": zero3,
            "q": lambda a, t: 0.0, "dq": lambda a, t: 0.0,
            "tau_fb": lambda a, t: 0.0, "applied": lambda a, t: 0.1}


def _ctrl_zero(t):
    return {k: 0.0 for k in spec.CTRL_FIELDS}


def run_delay_timestamps(quick):
    meta = spec.default_meta()
    durations = [1.0, 10.0] if quick else [1.0, 10.0, 30.0]
    rows = []
    for T in durations:
        w, info = synth.synth_log(T, meta, _flat_signals(), imu_phase_us=411, jitter_us=100,
                                  seed=int(T), ctrl=_ctrl_zero)
        ds = importer.import_log(w.to_bytes(), t0="none")
        got = delay.delay_from_timestamps(ds, spec.STUDIO_AXES)
        # truth from the generator
        imu = info["imu_times"]
        import bisect
        tx_by_axis = {}
        for ax, tx in info["tx_log"]:
            tx_by_axis.setdefault(ax, []).append(tx)
        worst = 0.0
        summ = {}
        ticks = [int(v) for v in ds.ch["ctrl.tick"].y]
        first = ticks[0]
        for ax in spec.STUDIO_AXES:
            truth = []
            for c in ticks:
                c_us = info["t0_us"] + c * 3000
                i = bisect.bisect_right(imu, c_us) - 1
                tx = tx_by_axis[ax][c - first]
                truth.append((c_us - imu[i] + tx - c_us) * 1e-6)
            worst = max(worst, max(abs(a - b) for a, b in zip(got[ax], truth)))
            summ[ax] = delay.summarize(got[ax])
        rows.append({"T": T, "ticks": len(got[spec.STUDIO_AXES[0]]), "max_abs_error_s": worst, "per_axis": summ})
    return {"rows": rows, "note": "IMU phase 411 us, TX jitter 0 to 100 us, three sends 1 ms apart per bus"}


def run_delay_regression(quick):
    D = 0.0036
    Ts = [2.0, 5.0] if quick else [2.0, 5.0, 10.0, 30.0]
    seeds = 2 if quick else 6
    out = {}
    for name, gstd in (("board_noise", allan.BOARD_GYRO_STD), ("studio_noise", allan.STUDIO_GYRO["sigma_w"])):
        rows = []
        for T in Ts:
            err = []
            for s in range(seeds):
                rng = random.Random(5000 + int(T) * 10 + s + (0 if name == "board_noise" else 777))
                d = delay.simulate_rocking(T, D, rng, gyro_std=gstd)
                err.append(delay.fit_delay_regression(*d)["D"] - D)
            rows.append({"T": T, "D_med_us": medabs(err) * 1e6, "D_p90_us": p90abs(err) * 1e6})
        out[name] = {"rows": rows, "need_300us": next((r["T"] for r in rows if r["D_p90_us"] <= 300), None)}
    out["D_true"] = D
    out["seeds"] = seeds
    return out


# ------------------------------------------------------------------ 6
def run_studio_roundtrip(quick):
    path = os.path.join(HERE, "m13_loop3_estimation.csv")
    if not os.path.exists(path):
        return {"skipped": "m13_loop3_estimation.csv not found"}
    st = studio.load_estimation_csv(path)
    T = min(st.ch["est.pitch"].t[-1], 4.0 if quick else 1e9)
    meta = spec.default_meta()
    off_true = 0.2371  # Studio t=0 happens 0.2371 s after the robot log starts
    pitch, speed = st.ch["est.pitch"], st.ch["est.body_speed"]
    LL, LR = st.ch["est.L.L"], st.ch["est.L.R"]

    def est(t):
        ts = [t - off_true]
        v = lambda s: importer.interp(s, ts, "linear")[0]  # noqa: E731
        p = v(pitch)
        if p != p:
            p, sp, l1, l2 = 0.0, 0.0, 0.12, 0.12
        else:
            sp, l1, l2 = v(speed), v(LL), v(LR)
        return {"ready": 1.0, "pitch": p, "roll": 0.0, "yaw": 0.0, "body_speed": sp, "L_L": l1, "L_R": l2,
                "gbias_x": 0.0, "gbias_y": 0.0, "gbias_z": 0.0, "accel_bias": 0.0, "accel_norm": 9.8}

    w, info = synth.synth_log(T + off_true + 0.05, meta, _flat_signals(), est=est,
                              t0_us=2**32 - 100_000, events=[(off_true, 1, "studio_t0")])
    ds = importer.import_log(w.to_bytes(), t0="first")
    # first record is META at t0-10 us -> robot-log time of Studio t=0 is off_true + 10 us
    off_expected = off_true + 10e-6
    lag = studio.event_time(ds, "studio_t0")
    ready = st.ch["est.ready"]
    t_ready = next(t for t, r in zip(ready.t, ready.y) if r > 0)
    names = ["est.pitch", "est.body_speed", "est.L.L", "est.L.R"]
    cmp_ = studio.compare(ds, st, names, offset=lag, t_range=(t_ready + 0.006, T - 0.01))
    # Signal-based alignment: on this run (smooth 2 mrad decay after 'ready')
    # and on a chirp with motion.
    post = [y for t, y in zip(st.ch["est.pitch"].t, st.ch["est.pitch"].y) if t > t_ready]
    span = max(post) - min(post)
    lag_run = studio.align_offset(st.ch["est.pitch"], ds.ch["est.pitch"], max_lag=0.35,
                                  t_range=(t_ready + 0.006, T - 0.01))
    chirp = lambda t: 0.05 * math.sin(2 * math.pi * (0.5 + 1.5 * t) * t)  # noqa: E731
    st2 = importer.Dataset()
    for k in range(int(T * 1000) + 1):
        st2.add("est.pitch", k * 1e-3, chirp(k * 1e-3))
    w2, _ = synth.synth_log(T + off_true + 0.05, meta, _flat_signals(), t0_us=7_000_000,
                            est=lambda t: dict(est(t), pitch=chirp(t - off_true)))
    ds2 = importer.import_log(w2.to_bytes(), t0="first")
    lag2 = studio.align_offset(st2.ch["est.pitch"], ds2.ch["est.pitch"], max_lag=0.5)
    return {"offset_true_s": off_expected, "offset_found_s": lag,
            "offset_error_us": (lag - off_expected) * 1e6, "compare": cmp_, "T": T,
            "t_ready": t_ready, "pitch_span_after_ready": span,
            "xcorr_run_offset_error_us": (lag_run - off_expected) * 1e6,
            "xcorr_offset_error_us": (lag2 - off_expected) * 1e6, "t_wrap_crossed": True}


# ------------------------------------------------------------------ report
def fmt(x, n=3):
    if x is None:
        return "not reached"
    if isinstance(x, float):
        if x == 0:
            return "0"
        if abs(x) < 1e-3 or abs(x) >= 1e4:
            return f"{x:.{n}g}"
        return f"{x:.{n}g}"
    return str(x)


def pct(x):
    return "n/a" if x is None else f"{100 * x:.1f} %"


def write_report(R, path):
    L = []
    a = L.append
    a("# REHEARSAL: identification on synthetic data (generated)")
    a("")
    a(f"Generated by `python rehearsal.py` on {R['generated']} in {R['runtime_s']:.0f} s "
      f"({'QUICK' if R['quick'] else 'full'} Monte Carlo). Standard-library Python {R['python_version']}, no MATLAB. "
      "Numbers below are simulation results from ASSUMED or Studio parameters, not hardware data.")
    a("")
    a("p90 = empirical 90th percentile of |relative error| over the seeds, not a confidence bound. "
      "'Seconds needed' = the shortest tested record meeting the target under this model; "
      "it is not a proven minimum or guaranteed hardware duration. See IDENTIFIABILITY.md for limits.")
    a("")
    # Allan
    A = R["allan"]
    a("## 1. IMU bias and noise (Allan variance)")
    a("")
    a("Model: the Studio M13 noise (constant turn-on bias + white noise + rate random walk), simulated at 100 Hz "
      "with the white level rescaled so that N is the Studio value at the 1 ms tick. Fit: weighted least squares "
      "of sigma_A^2 = N^2/tau + K^2 tau/3 on octave-spaced taus.")
    a("")
    a("| Scenario | per-sample std at 1 kHz | N (ARW) | K (RRW) | best bias window sqrt(3)N/K | bias error at best window | bias error with 3 s window |")
    a("|---|---|---|---|---|---|---|")
    for k, v in A.items():
        a(f"| {k} | {fmt(v['per_sample_std_1kHz'])} | {fmt(v['N_true'])} | {fmt(v['K_true'])} | "
          f"{fmt(v['T_best_bias'])} s | {fmt(v['bias_err_at_T_best'])} | {fmt(v['bias_err_3s'])} |")
    a("")
    a("Units: gyro rad/s, rad/sqrt(s), rad/s/sqrt(s); accel m/s^2, m/s/sqrt(s), m/s^2/sqrt(s).")
    a("")
    for k, v in A.items():
        a(f"**{k}** (seeds {v['seeds']}): seconds needed for N within 10 % (p90): **{fmt(v['need_N10'])}**; "
          f"for K within 25 % (p90): **{fmt(v['need_K25'])}**.")
        a("")
        a("| Record T (s) | N med | N p90 | K med | K p90 | bias error rms (window min(T, best)) |")
        a("|---|---|---|---|---|---|")
        for r in v["rows"]:
            a(f"| {r['T']} | {pct(r['N_med'])} | {pct(r['N_p90'])} | {pct(r['K_med'])} | {pct(r['K_p90'])} | {fmt(r['bias_err_rms'])} |")
        a("")
    # lever
    Lv = R["lever"]
    a("## 2. Motor torque constant: static lever test (E-M1)")
    a("")
    a(f"Truth k = Kt_true/Kt_fw = {motor.TRUTH['k']} (Studio wheel Kt 0.2 N*m/A, so Kt_true = "
      f"{0.2 * motor.TRUTH['k']:.3f} N*m/A). {Lv['note']}. Seeds {Lv['seeds']}.")
    a("")
    a("| Levels per sweep | Seconds | k med | k p90 |")
    a("|---|---|---|---|")
    for r in Lv["rows"]:
        a(f"| {r['levels_per_sweep']} | {r['T']:.0f} | {pct(r['k_med'])} | {pct(r['k_p90'])} |")
    a("")
    a(f"Seconds needed for k within 2 % (p90): **{fmt(Lv['need_k2'])}** per motor. The 0.5 % lever-length "
      "error is systematic and does not average out; measure the lever with calipers.")
    a("")
    # friction
    F = R["friction"]
    a("## 3. Wheel drive friction: constant-speed plateaus (E-M2)")
    a("")
    a(f"Truth Tc = {motor.TRUTH['Tc']} N*m, b = {motor.TRUTH['b']} N*m*s/rad (Studio 0.01 and 0.001, perturbed). "
      f"Cogging ripple {motor.TRUTH['cog_amp']} N*m at {motor.TRUTH['cog_per_rev']} per rev (ASSUMED). "
      f"Speeds {F['speeds']} rad/s, 1 kHz feedback. The DM torque LSB is {F['lsb_tau']:.4f} N*m (12 bit over +-10 N*m), "
      f"so Tc is only {F['Tc_over_lsb']:.1f} LSB: the estimate relies on noise dithering and averaging. Seeds {F['seeds']}.")
    a("")
    a("| Plateau (s) | Total seconds | Tc med | Tc p90 | b med | b p90 |")
    a("|---|---|---|---|---|---|")
    for r in F["rows"]:
        a(f"| {r['T_plateau']} | {r['T']:.0f} | {pct(r['Tc_med'])} | {pct(r['Tc_p90'])} | {pct(r['b_med'])} | {pct(r['b_p90'])} |")
    a("")
    a(f"Seconds needed for Tc and b within 5 % (p90): **{fmt(F['need_5pct'])}** (12 plateaus). "
      "Optimistic: the synthetic friction has exactly the fitted form (Coulomb + viscous). Real drives add "
      "Stribeck, temperature and position-dependent terms, so plan plateaus of at least one wheel revolution "
      "(6.3 s at 1 rad/s) and repeat after warm-up.")
    a("")
    # steps
    S = R["steps"]
    a("## 4. Torque lag, motor delay and k/J: torque steps, wheel free (E-M3)")
    a("")
    a(f"Truth: lag {S['truth']['lag'] * 1e3:.1f} ms (Studio 5 ms, perturbed), TX-to-torque delay "
      f"{S['truth']['d'] * 1e6:.0f} us (ASSUMED), k/J = {S['truth']['k'] / S['truth']['J']:.0f} 1/(s^2) per N*m. "
      "Square-wave torque 0.15 N*m with random switch times, weak host speed damping. (d, lag) are fitted on the "
      "motor's torque feedback by output error; k/J on the speed record by windowed integral equation error with "
      f"friction from E-M2. Seeds {S['seeds']}.")
    a("")
    for key in ("period_1ms", "period_3ms"):
        v = S[key]
        a(f"**Command period {key.split('_')[1]}** (1 ms = bench build, 3 ms = DM-jump cadence). "
          f"Seconds needed for |d| < 200 us, lag < 10 %, k/J < 2 % (p90): **{fmt(v['need'])}**.")
        a("")
        a("| Seconds | d med (us) | d p90 (us) | lag p90 | k/J p90 | corr(d, lag) |")
        a("|---|---|---|---|---|---|")
        for r in v["rows"]:
            a(f"| {r['T']:.0f} | {r['d_med_us']:.0f} | {r['d_p90_us']:.0f} | {pct(r['lag_p90'])} | {pct(r['a_p90'])} | {r['corr_d_lag']:.2f} |")
        a("")
    j = S["joint_spin"]
    a(f"Identifiability checks. From the speed record alone, corr(d, lag) = {S['speed_only_corr_d_lag']:.2f} "
      "(local sensitivities with noise disabled but quantisation retained). "
      f"Fitting friction from the spin record jointly with k/J gives Tc/J off by {pct(j['Tc_over_J_rel'])} and "
      f"b/J off by {pct(j['b_over_J_rel'])} (corr {j['corr_c_beta']:.2f}): friction is poorly estimated from slow "
      "spin data with cogging in this experiment, so it is taken from the plateau test. "
      "The step duration table is conditional on exact friction; section 8 propagates fitted friction. "
      "k and J are never separable from uncalibrated spin data "
      "(only k/J); the lever test supplies k.")
    a("")
    # delay
    DA = R["delay_timestamps"]
    a("## 5. Loop delay from time stamps (method A)")
    a("")
    a(f"Synthetic firmware log ({DA['note']}), written in the binary format, imported, and the delay "
      "imu_age + (t_TX - t_CTRL) computed per axis, compared with the generator's truth.")
    a("")
    a("| Seconds | Controller ticks | max absolute error (s) |")
    a("|---|---|---|")
    for r in DA["rows"]:
        a(f"| {r['T']:.0f} | {r['ticks']} | {fmt(r['max_abs_error_s'])} |")
    a("")
    last = DA["rows"][-1]["per_axis"]
    a("| Axis | mean (ms) | p50 (ms) | p99 (ms) | max (ms) |")
    a("|---|---|---|---|---|")
    for ax, s in last.items():
        a(f"| {ax} | {s['mean'] * 1e3:.3f} | {s['p50'] * 1e3:.3f} | {s['p99'] * 1e3:.3f} | {s['max'] * 1e3:.3f} |")
    a("")
    DB = R["delay_regression"]
    a("## 6. Loop delay without instrumentation (method B)")
    a("")
    a(f"Synthetic rocking signal (0.5 to 8 Hz; no hardware acquisition), controller torque = K1 phi + K2 w delayed by "
      f"D = {DB['D_true'] * 1e3:.1f} ms plus other terms (0.05 N*m rms). D found by least squares over a delay grid. Seeds {DB['seeds']}.")
    a("")
    for k in ("board_noise", "studio_noise"):
        v = DB[k]
        a(f"**{k}**: seconds needed for |D error| < 300 us (p90): **{fmt(v['need_300us'])}**.")
        a("")
        a("| Seconds | D med (us) | D p90 (us) |")
        a("|---|---|---|")
        for r in v["rows"]:
            a(f"| {r['T']:.0f} | {r['D_med_us']:.0f} | {r['D_p90_us']:.0f} |")
        a("")
    ST = R["studio_roundtrip"]
    a("## 7. Importer round trip against a Studio simulation export")
    a("")
    if "skipped" in ST:
        a(ST["skipped"])
    else:
        a(f"Studio `loop3_estimation.csv` (M13 simulation export, {ST['T']:.2f} s) was written into a robot-format "
          "log at the 3 ms firmware cadence, with the MCU clock crossing its 32-bit wrap and an EVENT "
          "'studio_t0' marking Studio t = 0. The log was imported, synchronised on the EVENT and compared on "
          f"the Studio grid after the estimator is ready (t >= {ST['t_ready']:.3f} s). "
          f"EVENT offset error: {ST['offset_error_us']:.3f} us.")
        a("")
        a("| Channel | RMS error | max error | samples |")
        a("|---|---|---|---|")
        for k, v in ST["compare"].items():
            if v:
                a(f"| {k} | {fmt(v['rms'])} | {fmt(v['max_abs'])} | {v['n']} |")
        a("")
        a("The residual is float32 rounding plus the linear-interpolation error of resampling 3 ms data onto "
          "the Studio 1 ms grid. There is no timing error.")
        a("")
        a(f"Signal-based alignment (cross-correlation on est.pitch after 'ready'): on this run the pitch only "
          f"decays smoothly by {ST['pitch_span_after_ready']:.2g} rad (simulated robot standing), and the offset error is "
          f"{ST['xcorr_run_offset_error_us']:.0f} us. On a 0.05 rad chirp through the same pipeline it is "
          f"{ST['xcorr_offset_error_us']:.1f} us. Signal alignment is only as good as the motion in the "
          "signal, so the EVENT is the primary sync and cross-correlation is a check.")
    a("")
    a("## 8. Sequential calibration with uncertainty propagation")
    a("")
    chain = R["calibration_chain"]
    a(f"{chain['note']} {len(chain['rows'])} seeds; {chain['duration_s']:.1f} s of nominal data "
      "per motor (30 s lever, 39.6 s plateaus including settling, 5 s independent steps).")
    a("")
    a("| Parameter | Absolute relative error p90 |")
    a("|---|---|")
    for key, value in chain["relative_error_p90"].items():
        a(f"| {key} | {pct(value)} |")
    a("")
    a("Kt = k * 0.2 N*m/A is conditional on the assumed firmware torque/current scale. "
      "Without calibrated torque-producing current, hardware data identifies k, not absolute Kt. "
      "No real stationary log was supplied; the original measured-data hardware acceptance gate remains open.")
    a("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def main():
    quick = "--quick" in sys.argv
    t0 = time.time()
    R = {"quick": quick, "generated": time.strftime("%Y-%m-%d %H:%M"),
         "python_version": platform.python_version(), "measurement_domain": "synthetic"}
    for key, fn in (("allan", run_allan), ("lever", run_lever), ("friction", run_friction),
                    ("steps", run_steps), ("calibration_chain", run_calibration_chain),
                    ("delay_timestamps", run_delay_timestamps),
                    ("delay_regression", run_delay_regression), ("studio_roundtrip", run_studio_roundtrip)):
        t = time.time()
        R[key] = fn(quick)
        print(f"{key}: {time.time() - t:.1f} s", flush=True)
    R["runtime_s"] = time.time() - t0
    out = os.path.join(HERE, "out_quick" if quick else ".")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "rehearsal_results.json"), "w") as f:
        json.dump(R, f, indent=1, default=str)
    write_report(R, os.path.join(out, "REHEARSAL.md"))
    print("done", round(R["runtime_s"], 1), "s")
    return R


if __name__ == "__main__":
    main()
