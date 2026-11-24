#!/usr/bin/env python3
"""Read-only offline replay endpoint consistency and measured tracking summaries."""
import argparse
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
from pathlib import Path
import re
import sys


STATES = {-1: "SYSTEM_NOT_READY", 0: "NO_IMAGES_YET", 1: "NOT_INITIALIZED",
          2: "OK", 3: "RECENTLY_LOST", 4: "LOST", 5: "OK_KLT"}
TRACKED = {"OK", "OK_KLT"}
LOST = {"RECENTLY_LOST", "LOST"}
COVERAGES = {"complete", "partial", "unknown"}
DEFAULT_MIN_PATH_M = 1e-6


def decimal_stamp(value):
    try:
        stamp = Decimal(value)
    except InvalidOperation as error:
        raise ValueError("invalid timestamp") from error
    if not stamp.is_finite():
        raise ValueError("nonfinite timestamp")
    return stamp


def integer(value):
    if not re.fullmatch(r"\d+", value):
        raise ValueError("expected nonnegative integer, got " + repr(value))
    return int(value)


def read_tum(path):
    """Eight columns, metres; preserve timestamp precision without float epochs."""
    poses = []
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        try:
            if len(fields) != 8:
                raise ValueError("expected t x y z qx qy qz qw")
            stamp = decimal_stamp(fields[0])
            values = [float(v) for v in fields[1:]]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("nonfinite pose")
            if math.hypot(*values[3:]) == 0:
                raise ValueError("zero quaternion")
            if poses and stamp <= poses[-1][0]:
                raise ValueError("timestamps must be strictly increasing; no sorting/reset stitching")
            poses.append((stamp, tuple(values[:3])))
        except ValueError as error:
            raise ValueError(f"{path}:{line_number}: {error}") from error
    return poses


def geometry(poses, min_path_m):
    result = {"pose_count": len(poses), "first_source_stamp_s": None,
              "last_source_stamp_s": None, "endpoint_drift_m": None,
              "path_length_m": None, "endpoint_drift_percent_path": None,
              "status": "unavailable", "reason": "fewer than two tracked poses"}
    if poses:
        result.update(first_source_stamp_s=str(poses[0][0]),
                      last_source_stamp_s=str(poses[-1][0]))
    if len(poses) < 2:
        return result
    path = math.fsum(math.dist(a[1], b[1]) for a, b in zip(poses, poses[1:]))
    drift = math.dist(poses[0][1], poses[-1][1])
    if not math.isfinite(path) or not math.isfinite(drift):
        raise ValueError("nonfinite derived distance (coordinate overflow)")
    result.update(endpoint_drift_m=drift, path_length_m=path)
    if path <= min_path_m:
        result.update(status="degenerate", reason="zero/short path; percent unavailable")
    else:
        result.update(endpoint_drift_percent_path=100 * (drift / path),
                      status="available", reason=None)
    return result


def read_frames(path):
    """One completed tracking call per CSV row; no ROS/unfinished native dependency."""
    rows = []
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        headers = reader.fieldnames or []
        if len(set(headers)) != len(headers):
            raise ValueError("duplicate CSV headers")
        if not {"source_ns", "tracking_state"} <= set(headers):
            raise ValueError("CSV requires source_ns,tracking_state")
        timed = {"processing_start_steady_ns", "processing_end_steady_ns"}
        if set(headers) & timed and not timed <= set(headers):
            raise ValueError("CSV requires both processing steady timestamp columns")
        for number, row in enumerate(reader, 2):
            try:
                if None in row or any(v is None for v in row.values()):
                    raise ValueError("CSV row width differs from header")
                source = integer(row["source_ns"])
                state = row["tracking_state"]
                if state not in STATES.values():
                    state = STATES[int(state)]
                item = {"source_ns": source, "state": state}
                if "pose_published" in headers:
                    flag = row["pose_published"].lower()
                    if flag not in {"true", "false", "0", "1"}:
                        raise ValueError("pose_published must be true/false/0/1")
                    item["published"] = flag in {"true", "1"}
                    if item["published"] and state not in TRACKED:
                        raise ValueError("published pose contradicts nontracking state")
                for key in ("processed", "application_dropped"):
                    if key in headers:
                        item[key] = integer(row[key])
                if "processed" in item and item["processed"] < 1:
                    raise ValueError("processed is a one-based cumulative call counter")
                if timed <= set(headers):
                    item["start"] = integer(row["processing_start_steady_ns"])
                    item["end"] = integer(row["processing_end_steady_ns"])
                    if item["end"] <= item["start"]:
                        raise ValueError("processing end must follow start on the same steady clock")
                if rows:
                    previous = rows[-1]
                    if source <= previous["source_ns"]:
                        raise ValueError("source_ns must strictly increase")
                    if "start" in item and item["start"] < previous["end"]:
                        raise ValueError("steady processing intervals must be serial and nonoverlapping")
                    if "processed" in item and item["processed"] <= previous["processed"]:
                        raise ValueError("processed counter must strictly increase; split reset sessions")
                    if ("application_dropped" in item and
                            item["application_dropped"] < previous["application_dropped"]):
                        raise ValueError("application_dropped counter regressed")
                rows.append(item)
            except (ValueError, KeyError) as error:
                raise ValueError(f"{path}:{number}: {error}") from error
    return rows


def tracking(rows, coverage="unknown", expected_processed=None):
    result = {"coverage": coverage, "observed_processed_frames": len(rows),
              "tracked_frames": None, "nontracking_frames": None,
              "loss_episode_count": None, "loss_count_scope": "unavailable",
              "published_pose_count": None, "frames_without_pose": None,
              "successful_frames_without_pose": None, "missing_status_rows": None,
              "expected_processed": expected_processed,
              "application_dropped_last": None, "source_span_s": None,
              "measured_steady_span_s": None, "processedFPS": None, "trackedFPS": None,
              "fps_scope": "unavailable: no measured steady processing intervals"}
    if not rows:
        if expected_processed is not None:
            result["missing_status_rows"] = expected_processed
        return result
    ntracked = sum(r["state"] in TRACKED for r in rows)
    losses, armed = 0, False
    for row in rows:
        if row["state"] in LOST and armed:
            losses += 1
        armed = row["state"] in TRACKED
    result.update(tracked_frames=ntracked, nontracking_frames=len(rows) - ntracked,
                  loss_episode_count=losses, loss_count_scope="observed transitions",
                  source_span_s=float(Decimal(rows[-1]["source_ns"] - rows[0]["source_ns"]) / Decimal(10**9)))
    if "published" in rows[0]:
        published = sum(r["published"] for r in rows)
        result.update(published_pose_count=published, frames_without_pose=len(rows) - published,
                      successful_frames_without_pose=sum(r["state"] in TRACKED and not r["published"]
                                                         for r in rows))
    missing = None
    if "processed" in rows[0]:
        missing = rows[-1]["processed"] - rows[0]["processed"] + 1 - len(rows)
        if coverage == "complete":
            missing += rows[0]["processed"] - 1
    if expected_processed is not None:
        if expected_processed < len(rows) or ("processed" in rows[-1] and
                                             expected_processed < rows[-1]["processed"]):
            raise ValueError("expected_processed is smaller than observed calls/counter")
        missing = expected_processed - len(rows)
    result["missing_status_rows"] = missing
    if missing or coverage != "complete":
        result["loss_count_scope"] = "observed transitions only; incomplete/unknown capture"
    if "application_dropped" in rows[-1]:
        result["application_dropped_last"] = rows[-1]["application_dropped"]
    if "start" in rows[0]:
        # Subtract integers BEFORE conversion; never subtract a source epoch here.
        duration = (rows[-1]["end"] - rows[0]["start"]) / 1e9
        result.update(measured_steady_span_s=duration, processedFPS=len(rows) / duration,
                      trackedFPS=ntracked / duration,
                      fps_scope="observed CSV rows / first steady start to last steady end")
        if missing:
            result["fps_scope"] += "; lower bounds because status rows are missing"
    return result


def loop_events(path=None, coverage="unknown"):
    # Exact messages from Pi ORB LoopClosing.cc, not substrings or candidate counts.
    messages = {"*Loop detected": "detection", "BAD LOOP!!!": "rejected",
                "*Merge detected": "merge_detection", "Merge finished!": "merge_finished",
                "Global Bundle Adjustment finished": "gba_finished", "Map updated!": "map_updated"}
    events = []
    if path is not None:
        for number, line in enumerate(Path(path).read_text().splitlines(), 1):
            message = re.sub(r"^(?:\[[^\]\r\n]+\]\s*)+", "", line.strip())
            if message in messages:
                events.append({"line": number, "kind": messages[message], "message": message})
    detected = any(e["kind"] == "detection" for e in events)
    answer = "yes" if detected else "no" if path is not None and coverage == "complete" else "unknown"
    return {"loop_closure_detected": answer,
            "observation": "observed" if detected else "not observed" if path else "no log",
            "log_coverage": coverage if path else "unavailable",
            "correction_completed": "unknown",
            "correction_reason": "inspected source has no direct correction-completed stdout marker; detection can be rejected",
            "negative_result_meaning": "no recognized positive detection in saved log; not proof that closure never occurred",
            "events": events,
            "counts": {kind: sum(e["kind"] == kind for e in events) for kind in messages.values()}}


def evidence(path):
    return {"path": str(Path(path).resolve()),
            "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}


def analyze(label, trajectory, frames=None, orb_log=None, coverage="unknown",
            frames_coverage="unknown", log_coverage="unknown", same_start_end="unknown",
            expected_processed=None, min_path_m=DEFAULT_MIN_PATH_M):
    if not math.isfinite(min_path_m) or min_path_m < 0:
        raise ValueError("min_path_m must be finite and nonnegative")
    result = {"label": label, "coverage": coverage, "same_physical_start_end": same_start_end,
              "inputs": {}, "errors": [], "warnings": []}
    poses, rows = None, None
    for name, path, parser in (("trajectory", trajectory, read_tum), ("frames", frames, read_frames)):
        if path is None:
            continue
        try:
            result["inputs"][name] = evidence(path)
            parsed = parser(path)
            if name == "trajectory":
                poses = parsed
                result["geometry"] = geometry(poses, min_path_m)
            else:
                rows = parsed
                result["tracking"] = tracking(rows, frames_coverage, expected_processed)
        except (OSError, ValueError, OverflowError) as error:
            result["errors"].append(f"{name}: {error}")
    result.setdefault("geometry", {"status": "invalid", "endpoint_drift_m": None,
                                   "path_length_m": None, "endpoint_drift_percent_path": None})
    result.setdefault("tracking", tracking([], frames_coverage, expected_processed))
    try:
        if orb_log is not None:
            result["inputs"]["orb_log"] = evidence(orb_log)
        result["loop"] = loop_events(orb_log, log_coverage)
    except (OSError, ValueError) as error:
        result["errors"].append(f"orb_log: {error}")
        result["loop"] = loop_events()
    if coverage != "complete":
        result["warnings"].append("Endpoint metrics describe only the observed pose span; full replay coverage is not established.")
    if same_start_end != "yes":
        result["warnings"].append("Equal physical start/end is not established; endpoint displacement is not a closed-path drift measurement.")
    if frames is None:
        result["warnings"].append("No per-frame evidence: FPS, tracking-loss episodes and missing tracking outputs are unavailable.")
    if poses is not None and rows is not None and rows:
        published = result["tracking"]["published_pose_count"]
        if published is not None and published != len(poses):
            result["warnings"].append("Pose count differs from published flags; capture coverage/export scope differs or outputs are missing.")
        if rows[0]["state"] not in TRACKED or rows[-1]["state"] not in TRACKED:
            result["warnings"].append("Tracking is absent at a captured boundary; pose endpoints may omit replay start/end.")
    result["interpretation"] = "Internal endpoint consistency only; no absolute accuracy without ground truth. Static repetition is not moving-loop success."
    return result


def table(results):
    lines = ["| Replay | Coverage | Drift m (observed span) | Path m | Drift % path | Loop detected | Correction | processedFPS | trackedFPS | Loss episodes |",
             "|---|---|---:|---:|---:|---|---|---:|---:|---:|"]
    notes = []
    def fmt(value):
        return "unavailable" if value is None else format(value, ".6g")
    for result in results:
        g, t = result["geometry"], result["tracking"]
        values = [result["label"], result["coverage"], fmt(g["endpoint_drift_m"]),
                  fmt(g["path_length_m"]), fmt(g["endpoint_drift_percent_path"]),
                  result["loop"]["loop_closure_detected"], result["loop"]["correction_completed"],
                  fmt(t["processedFPS"]), fmt(t["trackedFPS"]), fmt(t["loss_episode_count"])]
        lines.append("| " + " | ".join(v.replace("|", "\\|").replace("\n", " ") for v in values) + " |")
        if result["errors"]:
            notes.append("\n" + result["label"] + ": invalid evidence; see JSON errors.\n")
    lines.extend(notes)
    lines.append("\nObserved-span consistency, not absolute accuracy. Equal physical start/end and complete tracked endpoints are required for full-loop drift.\n"
                 "Loop yes = explicit detection, no = none recognized in a declared complete log; unknown = missing/partial/unknown log. Correction is separate.\n"
                 "FPS uses measured steady processing span only; losses are observed transitions, potentially incomplete. See JSON for missing outputs and coverage limits.\n")
    return "\n".join(lines)


def outside_git(path):
    resolved = Path(path).resolve()
    if any((parent / ".git").exists() for parent in (resolved, *resolved.parents)):
        raise ValueError("output directory must be outside git checkouts")
    return resolved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", action="append", nargs=2, metavar=("LABEL", "TUM"), required=True)
    for flag, value in (("frames", "CSV"), ("orb-log", "LOG"), ("coverage", "complete|partial|unknown"),
                        ("frames-coverage", "complete|partial|unknown"), ("log-coverage", "complete|partial|unknown"),
                        ("same-start-end", "yes|no|unknown"), ("expected-processed", "COUNT")):
        parser.add_argument("--" + flag, action="append", nargs=2, metavar=("LABEL", value), default=[])
    parser.add_argument("--min-path-m", type=float, default=DEFAULT_MIN_PATH_M)
    parser.add_argument("--output-dir", required=True, help="new directory outside git; never overwrites evidence")
    args = parser.parse_args(argv)
    try:
        labels = [label for label, _ in args.replay]
        if len(labels) != 3 or len(set(labels)) != 3 or any(not label.strip() for label in labels):
            raise ValueError("provide exactly three --replay entries with unique nonempty labels")
        options = {}
        for key in ("frames", "orb_log", "coverage", "frames_coverage", "log_coverage", "same_start_end", "expected_processed"):
            mapping = {}
            for label, value in getattr(args, key):
                if label not in labels or label in mapping:
                    raise ValueError(f"{key}: unknown/duplicate label {label!r}")
                if key.endswith("coverage") and value not in COVERAGES:
                    raise ValueError("coverage must be complete, partial or unknown")
                if key == "same_start_end" and value not in {"yes", "no", "unknown"}:
                    raise ValueError("same-start-end must be yes, no or unknown")
                mapping[label] = integer(value) if key == "expected_processed" else value
            options[key] = mapping
        if not math.isfinite(args.min_path_m) or args.min_path_m < 0:
            raise ValueError("min-path-m must be finite and nonnegative")
        output = outside_git(args.output_dir)
        if output.exists():
            raise ValueError("output directory already exists; choose a fresh evidence path")
        results = [analyze(label, path, min_path_m=args.min_path_m,
                           **{key: values[label] for key, values in options.items() if label in values})
                   for label, path in args.replay]
        document = {"schema_version": 1, "min_path_m": args.min_path_m, "replays": results}
        # Serialize before any writes so invalid numbers cannot leave a misleading report.
        serialized = json.dumps(document, indent=2, allow_nan=False) + "\n"
        rendered = table(results)
        output.mkdir(parents=True, exist_ok=False)
        with (output / "replay_metrics.json").open("x") as stream:
            stream.write(serialized)
        with (output / "replay_metrics.md").open("x") as stream:
            stream.write(rendered)
        print(rendered)
        return 2 if any(r["errors"] for r in results) else 0
    except (OSError, ValueError, OverflowError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    sys.exit(main())
