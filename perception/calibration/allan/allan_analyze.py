#!/usr/bin/env python3
"""OPEN: native-rate overlapping Allan analysis of D435i imu_sensor / imu_camera CSVs."""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

HEADER = ["sdk_ts_ms", "domain", "frame_number", "host_steady_ns", "x", "y", "z"]
RATES = {"gyro": 200.0, "accel": 100.0}
UNITS = {"gyro": {"N": "rad/s/sqrt(Hz)", "K": "rad/s^2/sqrt(Hz)", "deviation": "rad/s"},
         "accel": {"N": "m/s^2/sqrt(Hz)", "K": "m/s^3/sqrt(Hz)", "deviation": "m/s^2"}}


def load_csv(path):
    """Parse the exact recorder schema; retain integer counters without float rounding."""
    with Path(path).open(newline="") as stream:
        reader = csv.reader(stream)
        if next(reader, None) != HEADER:
            raise ValueError(f"{path}: incorrect CSV header")
        domains, frames, hosts = set(), [], []
        for line, row in enumerate(reader, 2):
            if len(row) != len(HEADER):
                raise ValueError(f"{path}:{line}: incomplete CSV row")
            domains.add(row[1])
            frames.append(int(row[2]))
            hosts.append(int(row[3]))
            if frames[-1] < 0 or hosts[-1] < 0:
                raise ValueError(f"{path}:{line}: negative counter")
    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(0, 4, 5, 6), ndmin=2)
    if len(data) != len(frames) or len(data) < 4:
        raise ValueError(f"{path}: insufficient/inconsistent samples")
    return data[:, 0], data[:, 1:], domains, np.array(frames, dtype=np.uint64), np.array(hosts, dtype=np.uint64)


def clean_segment(stamps_ms, values, domains, rate, allow_system_time=False, frames=None):
    stamps_ms, values = np.asarray(stamps_ms), np.asarray(values)
    if values.shape != (len(stamps_ms), 3) or len(stamps_ms) < 4:
        raise ValueError("expected at least four XYZ samples")
    if not np.isfinite(stamps_ms).all() or not np.isfinite(values).all():
        raise ValueError("nonfinite SDK stamps or XYZ values")
    normalized = {d.upper().replace(" ", "_") for d in domains}
    if len(domains) != 1:
        raise ValueError("multiple/changing timestamp domains")
    if not normalized <= {"HARDWARE_CLOCK", "GLOBAL_TIME", "SYSTEM_TIME"}:
        raise ValueError(f"unsupported timestamp domain: {domains}")
    diagnostic = "SYSTEM_TIME" in normalized
    if diagnostic and not allow_system_time:
        raise ValueError("SYSTEM_TIME rejected; --allow-system-time is DIAGNOSTIC only")
    dt = np.diff(stamps_ms) / 1000.0
    if np.any(dt <= 0):
        raise ValueError("duplicate/backward SDK stamps; analysis refused")
    period = 1.0 / rate
    median = float(np.median(dt))
    if abs(median / period - 1) > 0.05:
        raise ValueError(f"median dt {median:.9g}s differs from nominal {period}s by >5%")
    breaks = np.flatnonzero((dt > 1.5 * period) | (dt < 0.5 * period)) + 1
    boundaries = np.r_[0, breaks, len(values)]
    lengths = np.diff(boundaries)
    best = int(np.argmax(lengths))
    start, end = int(boundaries[best]), int(boundaries[best + 1])
    chosen_dt = dt[start:end - 1]
    if len(chosen_dt) < 3:
        raise ValueError("no sufficiently long clean segment")
    if np.any(np.abs(chosen_dt / period - 1) > 0.10):
        raise ValueError("longest segment has >10% period jitter; no resampling performed")
    warnings = []
    if diagnostic:
        warnings.append("DIAGNOSTIC: SYSTEM_TIME permitted explicitly")
    if len(breaks):
        warnings.append("Discontinuities: only the longest clean segment was analyzed")
    frame_gaps = frame_nonincreasing = 0
    if frames is not None:
        increasing = frames[1:] > frames[:-1]
        frame_nonincreasing = int(np.count_nonzero(~increasing))
        frame_gaps = int(np.sum(frames[1:][increasing] - frames[:-1][increasing] - 1))
        if frame_gaps or frame_nonincreasing:
            warnings.append("Frame-number discontinuities found; inspect raw evidence")
    report = {"nominal_rate_hz": rate, "median_dt_s": median, "timestamp_domain": next(iter(domains)),
              "diagnostic": diagnostic, "input_samples": len(values), "used_samples": end - start,
              "dropped_samples": len(values) - (end - start), "selected_indices": [start, end],
              "segment_lengths": lengths.tolist(), "gap_count": int(np.count_nonzero(dt > 1.5 * period)),
              "short_interval_count": int(np.count_nonzero(dt < 0.5 * period)),
              "break_indices": breaks.tolist(), "max_gap_s": float(dt.max()),
              "missing_periods_estimate": int(np.maximum(np.rint(dt / period).astype(np.int64) - 1, 0).sum()),
              "frame_number_gaps": frame_gaps, "frame_number_nonincreasing": frame_nonincreasing,
              "input_span_s": float((stamps_ms[-1] - stamps_ms[0]) / 1000),
              "used_span_s": float((stamps_ms[end - 1] - stamps_ms[start]) / 1000),
              "nominal_used_span_s": (end - start - 1) / rate,
              "dt_percentiles_s": np.percentile(dt, [0, 1, 50, 99, 100]).tolist(), "warnings": warnings}
    return values[start:end], report


def overlapping_allan(values, rate, points=72):
    """Adjacent m-sample means: sigma^2 = mean(difference^2)/2.

    Overlapping terms n-2m+1 are correlated. Independent cluster counts n//m
    are reported separately and must be >=3. No resampling across gaps.
    """
    n = len(values)
    if n < 6:
        raise ValueError("too few samples for Allan deviation")
    sizes = np.unique(np.geomspace(1, n // 3, points).astype(np.int64))
    centered = np.asarray(values, dtype=np.float64) - np.mean(values, axis=0)
    sums = np.vstack((np.zeros((1, 3)), np.cumsum(centered, axis=0)))
    deviations, terms, clusters = [], [], []
    for m in sizes:
        difference = (sums[2*m:] - 2*sums[m:-m] + sums[:-2*m]) / m
        deviations.append(np.sqrt(np.mean(difference * difference, axis=0) / 2))
        terms.append(len(difference))
        clusters.append(n // int(m))
    return sizes / rate, np.array(deviations), np.array(terms), np.array(clusters)


def fit_region(tau, deviation, clusters, slope, evaluation_tau):
    """Require >=1 decade, >=5 points, >=30 independent clusters, slope within
    0.1 and R^2 >=0.95. Fit fixed model slope for the coefficient; report a free
    slope regression as a quality check.
    """
    good = np.isfinite(deviation) & (deviation > 0) & (clusters >= 30)
    x, y = np.log10(tau), np.log10(np.maximum(deviation, np.finfo(float).tiny))
    candidates = []
    for a in range(len(tau)):
        for b in range(a + 5, len(tau) + 1):
            if not good[a:b].all() or x[b-1] - x[a] < 1:
                continue
            xx, yy = x[a:b], y[a:b]
            measured, intercept = np.polyfit(xx, yy, 1)
            residual = yy - (measured * xx + intercept)
            total = float(np.sum((yy - yy.mean())**2))
            r2 = 1 - float(np.sum(residual**2)) / total if total > 0 else 0
            if abs(measured - slope) > 0.1 or r2 < 0.95:
                continue
            fixed_intercept = float(np.mean(yy - slope * xx))
            fixed_rmse = float(np.sqrt(np.mean((yy - (slope * xx + fixed_intercept))**2)))
            # Favor agreement with the physical slope over breadth: the broadest
            # admissible window can include the curved N/K crossover and bias N.
            candidates.append((-fixed_rmse, x[b-1] - x[a], a, b, measured, r2, fixed_intercept))
    if not candidates:
        return {"value": None, "status": "UNKNOWN", "observed": False,
                "reason": "No decade with slope within ±0.1, R²>=0.95 and >=30 independent clusters",
                "target_slope": slope, "evaluation_tau_s": evaluation_tau}
    _, _, a, b, measured, r2, intercept = max(candidates)
    return {"value": float(10**intercept * evaluation_tau**slope), "status": "FIT", "observed": True,
            "target_slope": slope, "measured_slope": float(measured), "r_squared": float(r2),
            "fixed_slope_log10_rmse": float(np.sqrt(np.mean((y[a:b] - (slope*x[a:b] + intercept))**2))),
            "range_tau_s": [float(tau[a]), float(tau[b-1])], "indices": [a, b],
            "minimum_independent_clusters": int(min(clusters[a:b])), "fit_points": b-a,
            "evaluation_tau_s": evaluation_tau, "evaluation_is_extrapolation": not (tau[a] <= evaluation_tau <= tau[b-1])}


def analyze_samples(stamps_ms, values, domains, sensor, allow_system_time=False, frames=None):
    selected, report = clean_segment(stamps_ms, values, domains, RATES[sensor], allow_system_time, frames)
    tau, deviation, terms, clusters = overlapping_allan(selected, RATES[sensor])
    report.update({"units": UNITS[sensor], "tau_s": tau.tolist(), "overlapping_terms": terms.tolist(),
                   "independent_clusters": clusters.tolist(), "axes": {}})
    for axis, curve in zip("xyz", deviation.T):
        n = fit_region(tau, curve, clusters, -0.5, 1.0)
        k = fit_region(tau, curve, clusters, 0.5, 3.0)
        minimum = int(np.argmin(curve))
        report["axes"][axis] = {"N": n, "K": k, "allan_deviation": curve.tolist(),
                                "bias_instability_minimum": {"deviation": float(curve[minimum]), "tau_s": float(tau[minimum]),
                                                             "independent_clusters": int(clusters[minimum]),
                                                             "note": "Observed minimum only; not K or a flicker-model fit"}}
        for key, fit in (("N", n), ("K", k)):
            if not fit["observed"]:
                report["warnings"].append(f"{axis}: {key} UNKNOWN; required slope region not observed")
    if report["used_span_s"] < 4 * 3600:
        report["warnings"].append("Below provisional 4 h; long-term walk support may be insufficient")
    return report


def conservative_values(reports):
    result = {}
    for sensor, prefix in (("gyro", "gyroscope"), ("accel", "accelerometer")):
        for key, suffix in (("N", "noise_density"), ("K", "random_walk")):
            values = [reports[sensor]["axes"][axis][key]["value"] for axis in "xyz"]
            result[f"{prefix}_{suffix}"] = max(values) if all(v is not None for v in values) else None
    return result


def write_yaml(path, values, multiplier, diagnostic):
    lines = ["# OPEN: D435i imu_sensor / imu_camera; continuous-time coefficients.",
             f"# {'DIAGNOSTIC' if diagnostic else 'Candidate'}; multiplier={multiplier}; UNKNOWN=null (not usable by Kalibr).",
             "# x10 is an ASSUMED inflation hypothesis, not a measurement."]
    for key, value in values.items():
        lines.append(f"{key}: {value * multiplier:.17g}" if value is not None else f"{key}: null # UNKNOWN")
    lines += ["rostopic: /imu0", "update_rate: 200.0", ""]
    path.write_text("\n".join(lines))


def plot_sensor(sensor, report, destination, plt, diagnostic=False):
    fig, ax = plt.subplots(figsize=(8, 5))
    tau = np.array(report["tau_s"])
    for axis, color in zip("xyz", ("tab:blue", "tab:orange", "tab:green")):
        a = report["axes"][axis]
        ax.loglog(tau, a["allan_deviation"], color=color, label=axis)
        for key in ("N", "K"):
            fit = a[key]
            if fit["observed"]:
                bounds = np.array(fit["range_tau_s"])
                ax.loglog(bounds, fit["value"] * (bounds / fit["evaluation_tau_s"])**fit["target_slope"],
                          "--", color=color)
    ax.set(xlabel="Averaging time tau (s)", ylabel=f"Allan deviation ({UNITS[sensor]['deviation']})",
           title=f"OPEN: D435i imu_sensor {sensor}" + (" — DIAGNOSTIC" if diagnostic else ""))
    ax.grid(True, which="both", alpha=0.3); ax.legend(); fig.tight_layout()
    fig.savefig(destination, dpi=160); plt.close(fig)


def analyze_run(run_dir, out_dir, allow_system_time=False, plots=True):
    run_dir, out_dir = Path(run_dir), Path(out_dir)
    reports = {}
    for sensor in RATES:
        stamps, values, domains, frames, _ = load_csv(run_dir / f"{sensor}.csv")
        reports[sensor] = analyze_samples(stamps, values, domains, sensor, allow_system_time, frames)
    if reports["gyro"]["timestamp_domain"] != reports["accel"]["timestamp_domain"]:
        raise ValueError("gyro and accel timestamp domains differ")
    diagnostic = any(r["diagnostic"] for r in reports.values())
    provenance, warnings = {}, []
    for name in ("meta.json", "power_summary.json"):
        source = run_dir / name
        if source.exists():
            provenance[name] = json.loads(source.read_text())
        else:
            warnings.append(f"{name} absent: acquisition provenance/power UNKNOWN")
    if provenance.get("meta.json", {}).get("error"):
        diagnostic = True; warnings.append("DIAGNOSTIC: recorder reports an error")
    if provenance.get("meta.json", {}).get("serial_check_bypassed"):
        diagnostic = True; warnings.append("DIAGNOSTIC: expected SDK serial check bypassed")
    power = provenance.get("power_summary.json", {})
    if power.get("undervoltage_occurred") or power.get("monitoring_complete") is False:
        diagnostic = True; warnings.append("DIAGNOSTIC: power not clean or monitoring incomplete")
    values = conservative_values(reports)
    if out_dir.exists() and (not out_dir.is_dir() or any(out_dir.iterdir())):
        raise ValueError("output must be a new or empty directory")
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_status = "disabled"
    if plots:
        # Keep matplotlib's writable font/config cache beside this report.
        os.environ["MPLCONFIGDIR"] = str((out_dir / ".matplotlib").resolve())
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            plot_status = "UNKNOWN: matplotlib unavailable; plots skipped"
            warnings.append(plot_status)
        else:
            for sensor, report in reports.items():
                plot_sensor(sensor, report, out_dir / f"allan_{sensor}.png", plt, diagnostic)
            plot_status = "generated"
    result = {"status": "OPEN", "evidence_label": "DIAGNOSTIC" if diagnostic else "UNQUALIFIED FITS",
              "frame": "imu_camera", "sensor": "D435i imu_sensor", "run_dir": str(run_dir.resolve()),
              "method": "Overlapping native-rate Allan deviation; longest clean segment; no resampling",
              "coefficient_convention": "N: -1/2 line at 1s; K: +1/2 line at 3s (Kalibr)",
              "minimum_clusters_per_tau": 3, "minimum_fit_clusters": 30,
              "fit_selection": "lowest fixed-slope log residual among admissible decades; widest on ties",
              "warnings": warnings, "provenance": provenance, "sensors": reports,
              "conservative_max_all_axes": values, "plots": plot_status}
    (out_dir / "allan.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    for multiplier, name in ((1, "raw"), (10, "x10")):
        write_yaml(out_dir / f"imu_allan_{name}.yaml", values, multiplier, diagnostic)
    assumed = {"gyroscope_noise_density": 1e-3, "accelerometer_noise_density": 1e-2,
               "gyroscope_random_walk": 1e-6, "accelerometer_random_walk": 1e-4}
    lines = [f"# OPEN — {result['evidence_label']}", "", "D435i imu_sensor / imu_camera; STM32 imu_body is out of scope.", "",
             "| Continuous-time quantity | Max across axes | Current ASSUMED ORB | Ratio |",
             "|---|---:|---:|---:|"]
    for key, value in values.items():
        lines.append(f"| {key} | {value:.6g} | {assumed[key]:.6g} | {value/assumed[key]:.4g} |" if value is not None
                     else f"| {key} | UNKNOWN | {assumed[key]:.6g} | UNKNOWN |")
    flyer = 120e-6 * 9.80665
    acc = values["accelerometer_noise_density"]
    lines += ["", f"Bosch imu_sensor accel flyer (plan [R8]): 120 ug/sqrt(Hz) = {flyer:.7g} m/s²/sqrt(Hz).",
              f"Measured-fit candidate/flyer ratio: {acc/flyer:.4g}." if acc is not None else "Fit/flyer ratio: UNKNOWN.", "",
              "Raw maxima require supported fits on all three axes. A null coefficient is UNKNOWN and the YAML is unusable until resolved.",
              "x10 is an ASSUMED hypothesis for later paired evaluation. No ORB configuration is changed.", ""]
    for sensor, report in reports.items():
        lines += [f"{sensor}: {report['used_samples']} / {report['input_samples']} samples; "
                  f"used {report['used_span_s']:.3f}s; dropped {report['dropped_samples']}; "
                  f"gaps {report['gap_count']}; domain {report['timestamp_domain']}.", ""]
        for axis in "xyz":
            for key in ("N", "K"):
                fit = report["axes"][axis][key]
                lines.append(f"- {sensor}.{axis} {key}: slope {fit['measured_slope']:.3f}, R² {fit['r_squared']:.4f}, "
                             f"tau {fit['range_tau_s']}, min independent clusters {fit['minimum_independent_clusters']}."
                             if fit["observed"] else f"- {sensor}.{axis} {key}: UNKNOWN; slope region absent.")
        lines += [""]
    lines += ["Warnings:", ""] + [f"- {w}" for w in warnings + [w for r in reports.values() for w in r["warnings"]]]
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--allow-system-time", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    try:
        result = analyze_run(args.run_dir, args.out_dir, args.allow_system_time, not args.no_plots)
    except (ValueError, OSError, OverflowError) as error:
        print(f"OPEN: analysis refused: {error}", file=sys.stderr); return 1
    print(f"OPEN: {result['evidence_label']}; outputs in {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
