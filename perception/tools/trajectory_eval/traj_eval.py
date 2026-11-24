#!/usr/bin/env python3
"""Offline ATE/RPE evaluation. See README.md for conventions and assumptions."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass, field
from datetime import datetime, timezone
from decimal import Decimal, DecimalException, InvalidOperation, ROUND_HALF_EVEN, localcontext
import hashlib
import io
import json
from pathlib import Path
import re
import sys
from typing import Any

import numpy as np

TOOL_NAME = "trajectory_eval"
VERSION = "1.0.0"
FORMAT_UNITS = {"tum": "s", "orbslam3": "ns", "euroc_gt": "ns"}
NS_PER_SECOND = 1_000_000_000
INT64 = np.iinfo(np.int64)
QUATERNION_TOLERANCE = 1e-3


def _decimal_ns(value: str, unit: str) -> tuple[int, Decimal]:
    """Convert decimal text directly to ns; round sub-ns precision half-even."""
    if unit not in {"s", "ns"}:
        raise ValueError(f"unsupported timestamp unit: {unit}")
    try:
        with localcontext() as ctx:
            ctx.prec = max(60, len(value) + 20)
            number = Decimal(value)
            if not number.is_finite():
                raise ValueError("timestamp must be finite")
            scaled = number * (NS_PER_SECOND if unit == "s" else 1)
            rounded = scaled.to_integral_value(rounding=ROUND_HALF_EVEN)
            if rounded < INT64.min or rounded > INT64.max:
                raise ValueError("timestamp exceeds int64 nanosecond range")
            return int(rounded), abs(scaled - rounded)
    except DecimalException as exc:
        raise ValueError(f"invalid decimal timestamp: {value!r}") from exc


def parse_timestamp(value: str, unit: str) -> int:
    """Return int64-compatible nanoseconds without an epoch float round-trip."""
    return _decimal_ns(value, unit)[0]


def _seconds_ns(value: str | float, name: str, *, positive: bool = False) -> int:
    text = str(value)
    ns, rounding = _decimal_ns(text, "s")
    if rounding:
        raise ValueError(f"{name} must be representable as whole nanoseconds")
    if positive and ns <= 0:
        raise ValueError(f"{name} must be positive")
    return ns


@dataclass
class Trajectory:
    """Positions in metres and active body-to-world quaternions (x,y,z,w)."""

    t_ns: np.ndarray
    positions: np.ndarray
    quaternions: np.ndarray
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        stamps = np.asarray(self.t_ns)
        if stamps.ndim != 1 or not len(stamps) or stamps.dtype.kind not in "iu":
            raise ValueError("timestamps must be a nonempty 1-D integer array")
        if any(int(t) < INT64.min or int(t) > INT64.max for t in stamps):
            raise ValueError("timestamp exceeds int64 nanosecond range")
        self.t_ns = stamps.astype(np.int64, copy=True)
        # Compare directly rather than subtracting int64s (which can overflow).
        bad = np.flatnonzero(self.t_ns[1:] <= self.t_ns[:-1])
        if len(bad):
            i = int(bad[0])
            kind = "duplicate" if self.t_ns[i] == self.t_ns[i + 1] else "reversal"
            raise ValueError(f"timestamp {kind} at pose indices {i}, {i + 1}: "
                             f"{self.t_ns[i]} -> {self.t_ns[i + 1]}")
        self.positions = np.array(self.positions, dtype=float, copy=True)
        self.quaternions = np.array(self.quaternions, dtype=float, copy=True)
        n = len(self.t_ns)
        if self.positions.shape != (n, 3) or self.quaternions.shape != (n, 4):
            raise ValueError("expected positions (n,3), quaternions (n,4)")
        if not np.isfinite(self.positions).all() or not np.isfinite(self.quaternions).all():
            raise ValueError("positions and quaternions must be finite")
        norms = np.linalg.norm(self.quaternions, axis=1)
        deviation = np.abs(norms - 1)
        if np.any(deviation > QUATERNION_TOLERANCE):
            i = int(np.argmax(deviation))
            raise ValueError(f"quaternion norm at pose {i} is {norms[i]:.12g}; "
                             f"deviation exceeds {QUATERNION_TOLERANCE}")
        self.metadata = dict(self.metadata)
        self.metadata["quaternion_max_norm_deviation"] = float(deviation.max())
        self.metadata["quaternion_normalized_after_validation"] = True
        self.quaternions /= norms[:, None]


def read_trajectory(path: str | Path, fmt: str = "tum", time_unit: str | None = None) -> Trajectory:
    """Read a strict, unsorted trajectory, recording format, hash and parsing."""
    if fmt not in FORMAT_UNITS:
        raise ValueError(f"unsupported format: {fmt}")
    unit = time_unit if time_unit is not None else FORMAT_UNITS[fmt]
    if unit not in {"s", "ns"}:
        raise ValueError(f"unsupported timestamp unit: {unit}")
    path = Path(path)
    raw = path.read_bytes()
    stamps, positions, quaternions = [], [], []
    rounded_count, max_rounding = 0, Decimal(0)
    previous_line = None
    for lineno, line in enumerate(raw.decode("utf-8-sig").splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = next(csv.reader([line], skipinitialspace=True)) if fmt == "euroc_gt" else line.split()
        try:
            if (fmt != "euroc_gt" and len(fields) != 8) or len(fields) < 8:
                raise ValueError("expected 8 columns (EuRoC GT may have extra columns)")
            stamp, rounding = _decimal_ns(fields[0].strip(), unit)
            if stamps and stamp <= stamps[-1]:
                kind = "duplicate" if stamp == stamps[-1] else "reversal"
                raise ValueError(f"timestamp {kind}: line {previous_line} ({stamps[-1]}) "
                                 f"-> line {lineno} ({stamp}); input is never sorted")
            values = np.array([float(x) for x in fields[1:8]])
            if not np.isfinite(values).all():
                raise ValueError("pose values must be finite")
            quat = values[[4, 5, 6, 3]] if fmt == "euroc_gt" else values[3:7]
            norm = np.linalg.norm(quat)
            if not np.isfinite(norm) or abs(norm - 1) > QUATERNION_TOLERANCE:
                raise ValueError(f"quaternion norm {norm:.12g} outside 1 +/- 1e-3")
        except (ValueError, InvalidOperation) as exc:
            raise ValueError(f"{path}:{lineno}: {exc}") from exc
        stamps.append(stamp)
        positions.append(values[:3])
        quaternions.append(quat)
        previous_line = lineno
        rounded_count += bool(rounding)
        max_rounding = max(max_rounding, rounding)
    metadata = {
        "path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest(),
        "format": fmt, "time_unit": unit, "time_unit_source": "explicit" if time_unit else "format default",
        "timestamp_parser": "decimal.Decimal, no float round-trip; int64 nanoseconds",
        "sub_nanosecond_policy": "round to nearest nanosecond, ties to even (ROUND_HALF_EVEN)",
        "timestamps_rounded": rounded_count, "max_rounding_ns": float(max_rounding),
        "input_quaternion_order": "wxyz" if fmt == "euroc_gt" else "xyzw",
        "internal_quaternion_order": "xyzw", "quaternion_norm_tolerance": QUATERNION_TOLERANCE,
    }
    return Trajectory(np.array(stamps, dtype=np.int64), positions, quaternions, metadata)


def quaternion_matrices(quaternions: np.ndarray) -> np.ndarray:
    """Convert normalized xyzw quaternions to active rotation matrices."""
    x, y, z, w = np.asarray(quaternions).T
    return np.stack((
        1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w),
        2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w),
        2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y),
    ), axis=1).reshape(-1, 3, 3)


def rotation_angles_deg(rotations: np.ndarray) -> np.ndarray:
    """Principal SO(3) angles in [0,180]; atan2 is stable near zero and pi."""
    skew = np.stack((rotations[:, 2, 1] - rotations[:, 1, 2],
                     rotations[:, 0, 2] - rotations[:, 2, 0],
                     rotations[:, 1, 0] - rotations[:, 0, 1]), axis=1)
    sine = np.linalg.norm(skew, axis=1) / 2
    cosine = np.clip((np.trace(rotations, axis1=1, axis2=2) - 1) / 2, -1, 1)
    return np.degrees(np.arctan2(sine, cosine))


def umeyama(source: np.ndarray, target: np.ndarray, with_scale: bool = False) -> tuple[np.ndarray, np.ndarray, float]:
    """Fit target = s * R * source + t (Umeyama, 1991); arrays are (n,3)."""
    x, y = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if x.shape != y.shape or x.ndim != 2 or x.shape[1] != 3 or len(x) < 3:
        raise ValueError("alignment needs at least three paired 3-D positions")
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError("alignment positions must be finite")
    mx, my = x.mean(axis=0), y.mean(axis=0)
    xc, yc = x - mx, y - my
    covariance = yc.T @ xc / len(x)
    u, singular, vt = np.linalg.svd(covariance)
    if np.linalg.matrix_rank(covariance) < 2:
        raise ValueError("degenerate alignment: covariance rank < 2 (collinear/stationary positions)")
    reflection = np.ones(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        reflection[-1] = -1
    rotation = (u * reflection) @ vt
    scale = float(np.dot(singular, reflection) / np.mean(np.sum(xc*xc, axis=1))) if with_scale else 1.0
    translation = my - scale * rotation @ mx
    return rotation, translation, scale


@dataclass
class Association:
    est_indices: np.ndarray
    ref_indices: np.ndarray
    shifted_est_t_ns: np.ndarray
    stats: dict[str, Any]


def associate(est: Trajectory, ref: Trajectory, t_max_diff: str | float = "0.01",
              t_offset: str | float = "0") -> Association:
    """Nearest neighbour on the shorter side; equal lengths iterate the estimate.

    This is evo sync.associate_trajectories(ref, est) pair-selection semantics,
    using integer nanoseconds. Ties pick the earliest other stamp, reuse allowed.
    """
    tolerance = _seconds_ns(t_max_diff, "t-max-diff")
    offset = _seconds_ns(t_offset, "t-offset")
    if tolerance < 0:
        raise ValueError("t-max-diff must be nonnegative")
    shifted = [int(t) + offset for t in est.t_ns]
    if shifted[0] < INT64.min or shifted[-1] > INT64.max:
        raise ValueError("offset estimate timestamps exceed int64 range")
    refs = [int(t) for t in ref.t_ns]
    # evo iterates the estimate unless the estimate is strictly longer.
    est_shorter = len(shifted) <= len(refs)
    short, long = (shifted, refs) if est_shorter else (refs, shifted)
    ei, ri, differences = [], [], []
    for i, stamp in enumerate(short):
        insertion = bisect_left(long, stamp)
        candidates = [j for j in (insertion - 1, insertion) if 0 <= j < len(long)]
        j = min(candidates, key=lambda k: (abs(long[k] - stamp), k))
        if abs(long[j] - stamp) <= tolerance:
            e, r = (i, j) if est_shorter else (j, i)
            ei.append(e)
            ri.append(r)
            differences.append(abs(shifted[e] - refs[r]))
    ue, ur = len(set(ei)), len(set(ri))
    stats = {
        "est_poses": len(shifted), "ref_poses": len(refs), "matched_pairs": len(ei),
        "unique_est_matched": ue, "unique_ref_matched": ur,
        "unmatched_est": len(shifted) - ue, "unmatched_ref": len(refs) - ur,
        "reference_sample_reused": ur < len(ri), "estimate_sample_reused": ue < len(ei),
        "reference_reuse_count": len(ri) - ur, "estimate_reuse_count": len(ei) - ue,
        "max_abs_dt_s": max(differences) / NS_PER_SECOND if differences else None,
        "mean_abs_dt_s": sum(differences) / len(differences) / NS_PER_SECOND if differences else None,
        "max_abs_dt_ns": max(differences) if differences else None,
        "iteration_side": "estimate" if est_shorter else "reference",
        "semantics": "shorter-side nearest neighbour, inclusive tolerance, earliest tie, reuse allowed; equal lengths iterate estimate",
        "t_offset_ns": offset, "t_max_diff_ns": tolerance,
    }
    return Association(np.array(ei, dtype=np.int64), np.array(ri, dtype=np.int64),
                       np.array(shifted, dtype=np.int64), stats)


def statistics(errors: np.ndarray, *, sse: bool = False) -> dict[str, Any]:
    """Population std (ddof=0); undefined empty-set statistics are JSON null."""
    errors = np.asarray(errors, dtype=float)
    if not np.isfinite(errors).all():
        raise ValueError("nonfinite metric error")
    result = {"n": len(errors)}
    if len(errors):
        result.update(rmse=float(np.sqrt(np.mean(errors**2))), mean=float(errors.mean()),
                      median=float(np.median(errors)), std=float(errors.std()),
                      min=float(errors.min()), max=float(errors.max()))
    else:
        result.update({key: None for key in ("rmse", "mean", "median", "std", "min", "max")})
    if sse:
        result["SSE"] = float(np.sum(errors**2))
    return result


def rpe_pair_indices(t_ns: np.ndarray, ref_positions: np.ndarray, delta: str | float = "1",
                     unit: str = "s") -> tuple[np.ndarray, np.ndarray]:
    """One pair per eligible start, first endpoint meeting delta, no interpolation."""
    try:
        distance = Decimal(str(delta))
    except InvalidOperation as exc:
        raise ValueError("invalid rpe-delta") from exc
    if not distance.is_finite() or distance <= 0:
        raise ValueError("rpe-delta must be finite and positive")
    n = len(t_ns)
    if unit == "frames":
        if distance != distance.to_integral_value():
            raise ValueError("rpe-delta in frames must be an integer")
        step = int(distance)
        starts = np.arange(max(0, n - step), dtype=np.int64)
        return starts, starts + step
    if unit == "s":
        step = _seconds_ns(delta, "rpe-delta", positive=True)
        progress = [int(t) for t in t_ns]
    elif unit == "m":
        step = float(distance)
        if not np.isfinite(step) or step <= 0:
            raise ValueError("rpe-delta distance is outside floating-point range")
        progress = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(ref_positions, axis=0), axis=1)))).tolist()
    else:
        raise ValueError(f"unsupported rpe-unit: {unit}")
    starts, ends = [], []
    for i in range(n):
        j = bisect_left(progress, progress[i] + step, lo=i + 1)
        if j < n:
            starts.append(i)
            ends.append(j)
    return np.array(starts, dtype=np.int64), np.array(ends, dtype=np.int64)


def relative_pose_errors(est_positions: np.ndarray, est_rotations: np.ndarray,
                         ref_positions: np.ndarray, ref_rotations: np.ndarray,
                         starts: np.ndarray, ends: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """E = inverse(inverse(Ref_i) Ref_j) (inverse(Est_i) Est_j)."""
    re_i_t = est_rotations[starts].transpose(0, 2, 1)
    rr_i_t = ref_rotations[starts].transpose(0, 2, 1)
    re = re_i_t @ est_rotations[ends]
    rr = rr_i_t @ ref_rotations[ends]
    te = np.einsum("nij,nj->ni", re_i_t, est_positions[ends] - est_positions[starts])
    tr = np.einsum("nij,nj->ni", rr_i_t, ref_positions[ends] - ref_positions[starts])
    # E.t = rr.T (te-tr); the orthogonal rr.T leaves its norm unchanged.
    translation = np.linalg.norm(te - tr, axis=1)
    rotation = rotation_angles_deg(rr.transpose(0, 2, 1) @ re)
    return translation, rotation


@dataclass
class Evaluation:
    result: dict[str, Any]
    series: dict[str, dict[str, np.ndarray]]
    ref_positions: np.ndarray
    ref_t_ns: np.ndarray


def _span(stamps: np.ndarray) -> dict[str, Any]:
    return {"start_ns": int(stamps[0]), "end_ns": int(stamps[-1]),
            "duration_s": (int(stamps[-1]) - int(stamps[0])) / NS_PER_SECOND}


def evaluate(est: Trajectory, ref: Trajectory, *, clock_assumption: str, est_frame: str,
             ref_frame: str, t_offset: str | float = "0", t_max_diff: str | float = "0.01",
             align: str = "se3", align_both: bool = False, n_to_align: int | None = None,
             rpe_delta: str | float = "1", rpe_unit: str = "s", rotation_ate: bool = False) -> Evaluation:
    """Evaluate loaded trajectories. No clock/frame relationships are inferred."""
    for name, value in (("clock-assumption", clock_assumption), ("est-frame", est_frame), ("ref-frame", ref_frame)):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be nonempty text")
    if align not in {"se3", "sim3", "none"} or (align_both and align == "none"):
        raise ValueError("invalid alignment selection (align-both cannot use align none)")
    association = associate(est, ref, t_max_diff, t_offset)
    ei, ri = association.est_indices, association.ref_indices
    if not len(ei):
        raise ValueError("no timestamp matches within the stated tolerance")
    ep, rp = est.positions[ei], ref.positions[ri]
    er, rr = quaternion_matrices(est.quaternions[ei]), quaternion_matrices(ref.quaternions[ri])
    rt = ref.t_ns[ri]
    if n_to_align is not None and (not isinstance(n_to_align, int) or n_to_align < 3 or n_to_align > len(ei)):
        raise ValueError("n-to-align must be an integer >=3 and <= matched pairs")
    count = len(ei) if n_to_align is None else n_to_align
    starts, ends = rpe_pair_indices(rt, rp, rpe_delta, rpe_unit)
    warnings = []
    if est_frame != ref_frame:
        warnings.append("WARNING: frame labels differ textually; no extrinsics are applied. Metrics may compare different physical frames.")
    if not len(starts):
        warnings.append("WARNING: no RPE pairs meet delta; RPE statistics are null with n=0.")
    modes = ["se3", "sim3"] if align_both else [align]
    metrics, series = {}, {}
    for mode in modes:
        rotation, translation, scale = (np.eye(3), np.zeros(3), 1.0) if mode == "none" else umeyama(ep[:count], rp[:count], mode == "sim3")
        aligned_positions = scale * ep @ rotation.T + translation
        aligned_rotations = rotation @ er
        ate = np.linalg.norm(aligned_positions - rp, axis=1)
        rpe_t, rpe_r = relative_pose_errors(aligned_positions, aligned_rotations, rp, rr, starts, ends)
        block = {
            "alignment": {"R": rotation.tolist(), "t": translation.tolist(), "s": scale,
                          "n_to_align": 0 if mode == "none" else count,
                          "mapping": "p_est_aligned = s * R * p_est + t; R_est_aligned = R * R_est",
                          "method": "identity" if mode == "none" else "Umeyama (1991), reflection fixed, first n associated pairs"},
            "ate_translation_m": statistics(ate, sse=True),
            "rpe_translation_m": statistics(rpe_t), "rpe_rotation_deg": statistics(rpe_r),
            "rpe_estimate_scale_applied": scale,
        }
        if rotation_ate:
            block["ate_rotation_deg"] = statistics(rotation_angles_deg(rr.transpose(0, 2, 1) @ aligned_rotations))
        metrics[mode] = block
        series[mode] = {"ate_m": ate, "rpe_translation_m": rpe_t, "rpe_rotation_deg": rpe_r,
                        "rpe_starts": starts, "rpe_ends": ends, "aligned_positions": aligned_positions}
    raw_ref_span = ref.positions[int(ri[0]):int(ri[-1]) + 1]
    result = {
        "tool": {"name": TOOL_NAME, "version": VERSION}, "evidence_label": "CALCULATED",
        "evaluated_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"python": sys.version, "numpy": np.__version__},
        "inputs": {"estimate": est.metadata, "reference": ref.metadata},
        "assumptions": {
            "clock_assumption": clock_assumption, "est_frame": est_frame, "ref_frame": ref_frame,
            "frames_differ_textually": est_frame != ref_frame, "extrinsics_applied": False,
            "pose_convention": "active body-to-world rotations; translations in metres",
            "t_offset_s": str(t_offset), "t_offset_ns": association.stats["t_offset_ns"],
            "t_max_diff_s": str(t_max_diff), "t_max_diff_ns": association.stats["t_max_diff_ns"],
            "offset_convention": "add offset to estimate timestamps before association",
            "est_time_unit": est.metadata.get("time_unit"), "ref_time_unit": ref.metadata.get("time_unit"),
            "alignments": modes, "n_to_align_requested": n_to_align,
            "alignment_evaluation": "fit first n pairs, evaluate all associated pairs",
            "rpe_delta": str(rpe_delta), "rpe_unit": rpe_unit,
            "rpe_pair_selection": "one endpoint per start; frames: i+delta; s: first reference timestamp >= t_i+delta; m: first matched-reference cumulative distance >= d_i+delta; no interpolation",
            "rpe_convention": "E = (Ref_i^-1 Ref_j)^-1 (Est_i^-1 Est_j); global rigid alignment cancels; Sim3 scale is applied to estimate translations before RPE",
            "std_convention": "population (ddof=0)",
            "scope": "Does not establish frames, extrinsics, clock synchronization, sensor readiness or gate PASS.",
        },
        "association": association.stats,
        "trajectory": {
            "est_time_span": _span(est.t_ns), "ref_time_span": _span(ref.t_ns),
            "est_offset_time_span": _span(association.shifted_est_t_ns),
            "est_matched_time_span": _span(association.shifted_est_t_ns[ei]),
            "ref_matched_time_span": _span(rt), "duration_s": _span(rt)["duration_s"],
            "reference_length_matched_span_m": float(np.linalg.norm(np.diff(raw_ref_span, axis=0), axis=1).sum()),
            "reference_matched_polyline_length_m": float(np.linalg.norm(np.diff(rp, axis=0), axis=1).sum()),
            "length_convention": "matched-span length includes all original reference poses between first and last matched reference; matched polyline uses associated samples only",
        },
        "metrics": metrics, "warnings": warnings,
    }
    return Evaluation(result, series, rp, rt)


def _format(value: Any) -> str:
    return "n/a" if value is None else f"{value:.6f}"


def _summary_table(rows: list[tuple[str, dict[str, Any]]]) -> str:
    lines = ["| Run | Align | ATE RMSE [m] | Mean [m] | Median [m] | Max [m] | RPE trans RMSE [m] | RPE rot RMSE [deg] | n pairs |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name, result in rows:
        for mode, block in result["metrics"].items():
            ate = block["ate_translation_m"]
            cells = [name, mode] + [_format(ate[k]) for k in ("rmse", "mean", "median", "max")]
            cells += [_format(block["rpe_translation_m"]["rmse"]), _format(block["rpe_rotation_deg"]["rmse"]), str(ate["n"])]
            lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def _assumption_text(result: dict[str, Any]) -> str:
    # JSON strings remain verbatim/escaped even if a label contains Markdown.
    lines = ["Evidence label: CALCULATED", "Assumptions (plain text):"]
    lines += [f"{key}: {json.dumps(value, ensure_ascii=False)}" for key, value in result["assumptions"].items()]
    for side, meta in result["inputs"].items():
        lines.append(f"{side} parsing: {json.dumps(meta, ensure_ascii=False)}")
    lines.append(f"association: {json.dumps(result['association'])}")
    lines.append(f"trajectory: {json.dumps(result['trajectory'])}")
    lines += result["warnings"]
    lines.append(f"plots: {json.dumps(result.get('plots', {}))}")
    return "\n".join(lines)


def _csv_text(header: list[str], rows: Any) -> str:
    output = io.StringIO(newline="")
    writer = csv.writer(output)
    writer.writerow(header)
    writer.writerows(rows)
    return output.getvalue()


def _relative_seconds(stamps: np.ndarray, origin: int) -> np.ndarray:
    return np.array([(int(t) - origin) / NS_PER_SECOND for t in stamps])


def _write_plots(evaluation: Evaluation, out: Path) -> dict[str, Any]:
    # Scope cache environment changes; include fontconfig's XDG cache.
    import os
    cache_environment = {"MPLCONFIGDIR": str((out / ".matplotlib").resolve()),
                         "XDG_CACHE_HOME": str((out / ".cache").resolve())}
    previous = {key: os.environ.get(key) for key in cache_environment}
    try:
        os.environ.update(cache_environment)
        return _render_plots(evaluation, out)
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def _render_plots(evaluation: Evaluation, out: Path) -> dict[str, Any]:
    # Direct Figure/Agg ownership avoids pyplot state and backend changes.
    try:
        import matplotlib
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
    except ImportError as exc:
        return {"status": "unavailable", "reason": str(exc), "files": []}
    files = []
    origin = int(evaluation.ref_t_ns[0])
    times = _relative_seconds(evaluation.ref_t_ns, origin)
    for mode, data in evaluation.series.items():
        specifications = (
            ("trajectory_xy", "x [m]", "y [m]"),
            ("ate_over_time", "Reference time from first match [s]", "ATE translation [m]"),
            ("rpe_over_time", "Reference start time from first match [s]", "RPE translation [m]"),
        )
        for kind, xlabel, ylabel in specifications:
            fig = Figure(figsize=(7, 4.5))
            FigureCanvasAgg(fig)
            ax = fig.subplots()
            try:
                if kind == "trajectory_xy":
                    ax.plot(*evaluation.ref_positions[:, :2].T, label="reference")
                    ax.plot(*data["aligned_positions"][:, :2].T, label=f"estimate ({mode})", alpha=0.8)
                    ax.set_aspect("equal", adjustable="datalim")
                    ax.legend()
                elif kind == "ate_over_time":
                    ax.plot(times, data["ate_m"])
                else:
                    ax.plot(times[data["rpe_starts"]], data["rpe_translation_m"])
                ax.set(xlabel=xlabel, ylabel=ylabel, title=f"{TOOL_NAME}: {mode} — CALCULATED")
                ax.grid(True, alpha=0.3)
                fig.tight_layout()
                filename = f"{kind}_{mode}.png"
                fig.savefig(out / filename, dpi=150)
                files.append(filename)
            finally:
                fig.clear()
    return {"status": "created", "matplotlib_version": matplotlib.__version__, "files": files}


def _prepare_output(out: Path, overwrite: bool) -> None:
    if out.is_symlink():
        raise ValueError("output directory must not be a symlink")
    if out.exists():
        if not overwrite:
            raise ValueError(f"output already exists: {out}; use --overwrite explicitly")
        if not out.is_dir():
            raise ValueError(f"output is not a directory: {out}")
    else:
        out.mkdir(parents=True)


def _check_output_artifacts(out: Path, names: list[str]) -> None:
    if out.is_symlink() or any((out / name).is_symlink() for name in names):
        raise ValueError("refusing symlinked output directory or artifacts")


def write_evaluation(evaluation: Evaluation, out: str | Path, *, overwrite: bool = False,
                     plots: bool = True) -> None:
    """Write named outputs; overwrite never recursively deletes unrelated files."""
    out = Path(out)
    _prepare_output(out, overwrite)
    targets = ["result.json", "summary.md", "ate_errors.csv", "rpe_errors.csv", ".matplotlib", ".cache"]
    targets += [f"{kind}_{mode}.png" for mode in evaluation.series
                for kind in ("trajectory_xy", "ate_over_time", "rpe_over_time")]
    _check_output_artifacts(out, targets)
    modes = list(evaluation.series)
    ate_header = ["t_ns", "error_m"] if len(modes) == 1 else ["t_ns"] + [f"error_m_{mode}" for mode in modes]
    ate_rows = ([int(t)] + [float(evaluation.series[m]["ate_m"][i]) for m in modes]
                for i, t in enumerate(evaluation.ref_t_ns))
    (out / "ate_errors.csv").write_text(_csv_text(ate_header, ate_rows), encoding="utf-8")
    first = evaluation.series[modes[0]]
    rpe_header = ["t_ns", "end_t_ns", "start_pair_index", "end_pair_index"]
    rpe_header += [f"{metric}_{mode}" if len(modes) > 1 else metric
                   for mode in modes for metric in ("translation_error_m", "rotation_error_deg")]
    rpe_rows = ([int(evaluation.ref_t_ns[i]), int(evaluation.ref_t_ns[j]), int(i), int(j)] +
                [float(evaluation.series[m][metric][k]) for m in modes
                 for metric in ("rpe_translation_m", "rpe_rotation_deg")]
                for k, (i, j) in enumerate(zip(first["rpe_starts"], first["rpe_ends"])))
    (out / "rpe_errors.csv").write_text(_csv_text(rpe_header, rpe_rows), encoding="utf-8")
    evaluation.result["plots"] = _write_plots(evaluation, out) if plots else {"status": "disabled", "files": []}
    evaluation.result["csv_convention"] = "t_ns uses associated reference timestamps; RPE t_ns is the start; align-both uses separate columns"
    (out / "result.json").write_text(json.dumps(evaluation.result, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    (out / "summary.md").write_text(_summary_table([("estimate", evaluation.result)]) + "\n\n" + _assumption_text(evaluation.result) + "\n", encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=f"{TOOL_NAME} {VERSION}")
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("single", "batch"):
        p = sub.add_parser(command, help=f"evaluate {'one trajectory' if command == 'single' else 'named runs'}")
        p.add_argument("--est", required=True, action="append" if command == "batch" else "store",
                       help="NAME=PATH (repeatable)" if command == "batch" else "estimate path")
        p.add_argument("--ref", required=True, help="reference path")
        for side in ("est", "ref"):
            p.add_argument(f"--{side}-format", choices=FORMAT_UNITS, default="tum")
            p.add_argument(f"--{side}-time-unit", choices=("s", "ns"), default=None)
            p.add_argument(f"--{side}-frame", required=True)
        p.add_argument("--clock-assumption", required=True)
        p.add_argument("--t-offset", default="0", help="seconds added to estimate timestamps")
        p.add_argument("--t-max-diff", default="0.01", help="inclusive association tolerance [s]")
        p.add_argument("--align", choices=("se3", "sim3", "none"), default="se3")
        p.add_argument("--align-both", action="store_true", help="evaluate se3 and sim3")
        p.add_argument("--n-to-align", type=int, default=None, help="first N pairs (default all)")
        p.add_argument("--rotation-ate", action="store_true")
        p.add_argument("--rpe-delta", default="1")
        p.add_argument("--rpe-unit", choices=("frames", "s", "m"), default="s")
        p.add_argument("--out", type=Path, required=True)
        p.add_argument("--overwrite", action="store_true")
        p.add_argument("--no-plots", action="store_true")
    return parser


def _aggregate(rows: list[tuple[str, dict[str, Any]]]) -> dict[str, Any]:
    result = {}
    for mode in rows[0][1]["metrics"]:
        mode_stats = {}
        for block, metric in (("ate_translation_m", "rmse"), ("ate_translation_m", "mean"),
                              ("ate_translation_m", "median"), ("ate_translation_m", "max"),
                              ("rpe_translation_m", "rmse"), ("rpe_rotation_deg", "rmse"),
                              ("ate_translation_m", "n")):
            values = [row["metrics"][mode][block][metric] for _, row in rows]
            finite = [v for v in values if v is not None]
            mode_stats[f"{block}.{metric}"] = {"n_runs": len(finite),
                "mean": float(np.mean(finite)) if finite else None,
                "min": min(finite) if finite else None, "max": max(finite) if finite else None}
        result[mode] = mode_stats
    return result


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    invoked_argv = argv.copy()
    # Accept both the compact default CLI and an explicit single subcommand.
    if argv and argv[0] not in {"single", "batch", "--version", "-h", "--help"}:
        argv.insert(0, "single")
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        entries = []
        for entry in args.est if args.command == "batch" else [args.est]:
            if args.command == "batch":
                if "=" not in entry:
                    raise ValueError("batch --est must be NAME=PATH")
                name, path = entry.split("=", 1)
                if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name) or name in {".", ".."}:
                    raise ValueError("batch names must be safe directory names: letters/digits/._-")
            else:
                name, path = "estimate", entry
            if not path or name in [n for n, _ in entries]:
                raise ValueError("batch paths must be nonempty and names unique")
            entries.append((name, Path(path)))
        if args.out.exists() and not args.overwrite:
            raise ValueError(f"output already exists: {args.out}; use --overwrite explicitly")
        # Reject outputs containing inputs, so overwrite cannot destroy evidence.
        for path in [Path(args.ref)] + [p for _, p in entries]:
            if path.resolve().is_relative_to(args.out.resolve()):
                raise ValueError("input files must not reside inside the output directory")
        ref = read_trajectory(args.ref, args.ref_format, args.ref_time_unit)
        evaluations = []
        for name, path in entries:
            est = read_trajectory(path, args.est_format, args.est_time_unit)
            evaluation = evaluate(est, ref, **{key: getattr(args, key) for key in (
                "clock_assumption", "est_frame", "ref_frame", "t_offset", "t_max_diff", "align",
                "align_both", "n_to_align", "rpe_delta", "rpe_unit", "rotation_ate")})
            evaluation.result["cli"] = {"argv": invoked_argv, "args": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}, "run_name": name}
            evaluations.append((name, evaluation))
        _check_output_artifacts(args.out, ["result.json", "summary.md"])
        if args.command == "batch":
            for name, evaluation in evaluations:
                targets = ["result.json", "summary.md", "ate_errors.csv", "rpe_errors.csv", ".matplotlib", ".cache"]
                targets += [f"{kind}_{mode}.png" for mode in evaluation.series
                            for kind in ("trajectory_xy", "ate_over_time", "rpe_over_time")]
                _check_output_artifacts(args.out / name, targets)
        _prepare_output(args.out, args.overwrite)
        rows = []
        for name, evaluation in evaluations:
            destination = args.out / name if args.command == "batch" else args.out
            write_evaluation(evaluation, destination, overwrite=args.overwrite or args.command == "single", plots=not args.no_plots)
            for warning in evaluation.result["warnings"]:
                print(warning, file=sys.stderr)
            rows.append((name, evaluation.result))
        if args.command == "batch":
            aggregate = _aggregate(rows)
            combined = _summary_table(rows)
            aggregate_rows = []
            for statistic in ("mean", "min", "max"):
                metrics = {}
                for mode, values in aggregate.items():
                    metrics[mode] = {"ate_translation_m": {k: values[f"ate_translation_m.{k}"][statistic] for k in ("rmse", "mean", "median", "max", "n")},
                                     "rpe_translation_m": {"rmse": values["rpe_translation_m.rmse"][statistic]},
                                     "rpe_rotation_deg": {"rmse": values["rpe_rotation_deg.rmse"][statistic]}}
                aggregate_rows.append((f"across runs: {statistic}", {"metrics": metrics}))
            combined += "\n\nAcross-run statistics (unweighted, per alignment; null RPE runs excluded):\n\n" + _summary_table(aggregate_rows)
            combined += "\n\n" + _assumption_text(rows[0][1]) + "\n\nPer-run parsing, hashes, alignment and warnings: each run/result.json.\n"
            (args.out / "summary.md").write_text(combined, encoding="utf-8")
            (args.out / "result.json").write_text(json.dumps({"tool": {"name": TOOL_NAME, "version": VERSION},
                "evidence_label": "CALCULATED", "evaluated_at_utc": datetime.now(timezone.utc).isoformat(),
                "cli": rows[0][1]["cli"], "aggregate": aggregate,
                "runs": {name: result for name, result in rows}}, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        print(_summary_table(rows))
        return 0
    except (ValueError, OSError, UnicodeError, np.linalg.LinAlgError) as exc:
        parser.error(str(exc))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
