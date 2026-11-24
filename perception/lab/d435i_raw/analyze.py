"""Offline qualification using only saved SDK rows, raw images and metadata."""
import csv
import hashlib
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path

from common import DOMAINS, FIELDS, STREAMS, VERSION, host, legacy_timing, utc, value, write_json


def finite(text):
    number = float(text)
    if not math.isfinite(number):
        raise ValueError("Nonfinite sample value")
    return number


def read_rows(path):
    streams = {s: [] for s in STREAMS}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != list(FIELDS):
            raise ValueError("Unexpected samples.csv schema")
        for row in reader:
            if None in row or any(v is None for v in row.values()):
                raise ValueError("Truncated or extra CSV columns")
            stream = row["stream"]
            if stream not in streams:
                raise ValueError(f"Unexpected stream {stream}")
            for key in ("frame_number", "host_clock_monotonic_ns", "host_clock_realtime_ns"):
                row[key] = int(row[key])
                if row[key] < 0:
                    raise ValueError(f"Negative {key}")
            for key in ("sdk_timestamp_ms", "hardware_timestamp_ms", "sdk_global_system_timestamp_ms", "x", "y", "z"):
                row[key] = finite(row[key]) if row[key] != "" else None
            if row["sdk_timestamp_ms"] is None:
                raise ValueError("Missing SDK timestamp")
            if stream in ("gyro", "accel") and any(row[k] is None for k in ("x", "y", "z")):
                raise ValueError("Missing motion vector")
            if row["image_file"] and (Path(row["image_file"]).name != row["image_file"] or not row["image_file"].endswith(".pgm")):
                raise ValueError("Unsafe image filename")
            streams[stream].append(row)
    return streams


def deltas(values):
    return [b - a for a, b in zip(values, values[1:])]


def clock_stats(rows):
    """Compare only identical clock identities; never cross domain transitions."""
    per_clock = defaultdict(list)
    sources = Counter()
    for row in rows:
        domain = row["sdk_timestamp_domain"]
        per_clock[f"sdk/{domain}"].append(row["sdk_timestamp_ms"])
        per_clock["host/CLOCK_MONOTONIC_ns"].append(row["host_clock_monotonic_ns"])
        per_clock["host/CLOCK_REALTIME_ns"].append(row["host_clock_realtime_ns"])
        sources[row["hardware_timestamp_source"]] += 1
        if row["hardware_timestamp_ms"] is not None:
            # Hardware get_timestamp and metadata counters need not share an epoch.
            per_clock[f"hardware/{row['hardware_timestamp_source']}"].append(row["hardware_timestamp_ms"])
        if row["sdk_global_system_timestamp_ms"] is not None:
            per_clock[f"sdk_global_system/{domain}"].append(row["sdk_global_system_timestamp_ms"])
    stats = {}
    for clock, stamps in per_clock.items():
        steps = deltas(stamps)
        stats[clock] = {"unit": "ns" if clock.startswith("host/") else "ms",
                        "count": len(stamps), "duplicates": sum(x == 0 for x in steps),
                        "reversals": sum(x < 0 for x in steps)}
    domains = Counter(r["sdk_timestamp_domain"] for r in rows)
    changes = sum(a["sdk_timestamp_domain"] != b["sdk_timestamp_domain"] for a, b in zip(rows, rows[1:]))
    consistency_errors = 0
    for r in rows:
        d, stamp = r["sdk_timestamp_domain"], r["sdk_timestamp_ms"]
        if d == "hardware_clock":
            consistency_errors += r["hardware_timestamp_ms"] != stamp or r["sdk_global_system_timestamp_ms"] is not None
        elif d in ("global_time", "system_time"):
            consistency_errors += r["sdk_global_system_timestamp_ms"] != stamp
        consistency_errors += not bool(r["hardware_timestamp_source"])
    return {"domains": dict(domains), "domain_changes": changes, "clocks": stats,
            "hardware_timestamp_sources": dict(sources), "field_consistency_errors": consistency_errors,
            "hardware_timestamp_unavailable_count": sum(r["hardware_timestamp_ms"] is None for r in rows)}


def image_stats(path, width, height):
    # This new profile is not supported by the existing fixed-640x480 PGM reader.
    raw = path.read_bytes()
    header = f"P5\n{width} {height}\n255\n".encode()
    if not raw.startswith(header) or len(raw) != len(header) + width * height:
        raise ValueError(f"Invalid captured Y8 PGM: {path.name}")
    pixels = raw[len(header):]
    return {"file": path.name, "mean": statistics.mean(pixels), "minimum": min(pixels),
            "maximum": max(pixels), "saturated_fraction": pixels.count(255) / len(pixels),
            "sha256": hashlib.sha256(raw).hexdigest()}


def analyze(directory, cfg):
    directory = Path(directory)
    if not (directory / "samples.csv").exists() and (directory / "frames.csv").exists():
        from legacy import analyze_legacy
        return analyze_legacy(directory, cfg)
    started = utc()
    criteria, metrics = [], {"evidence_label": "CALCULATED from saved evidence", "streams": {}}

    def add(identifier, description, measured, unit, specs, passed=None, blocked=False):
        threshold = {f"{g}.{k}": cfg[g][k]["value"] for g, k in specs}
        sources = {f"{g}.{k}": cfg[g][k]["source"] for g, k in specs}
        criteria.append({"id": identifier, "description": description, "measured": measured,
                         "unit": unit, "threshold": json.dumps(threshold, sort_keys=True) if threshold else "Documented evidence required",
                         "threshold_source": "; ".join(f"{key}: {source}" for key, source in sources.items()) or "SOURCE-SUPPORTED: phase-2 clock/evidence contract",
                         "status": "BLOCKED" if blocked else "PASS" if passed else "FAIL", "evidence": "metrics.json"})

    meta = None
    try:
        meta = json.loads((directory / "capture.json").read_text())
        if not isinstance(meta, dict):
            raise ValueError("capture.json must be an object")
        if not isinstance(meta.get("identity", {}), dict) or not isinstance(meta.get("profiles", []), list):
            raise ValueError("Malformed identity/profiles metadata")
        if any(not isinstance(p, dict) for p in meta.get("profiles", [])):
            raise ValueError("Malformed profile metadata")
        emitter = meta.get("emitter_readback")
        if emitter is not None and not isinstance(emitter, list):
            raise ValueError("Malformed emitter readback")
    except FileNotFoundError:
        add("capture", "Finalized capture metadata available", None, "", [], blocked=True)
    except (ValueError, OSError) as error:
        meta = None
        add("capture", "Capture metadata readable", str(error), "", [], passed=False)
    if meta is not None:
        metrics["capture"] = meta
        outcome = meta.get("outcome")
        add("capture", "Orderly capture, no SDK/writer errors or callback loss", {
            k: meta.get(k) for k in ("outcome", "complete", "errors", "queue_overflow", "callback_errors")}, "", [],
            passed=(outcome == "PASS" and meta.get("complete") is True
                    and meta.get("errors") == [] and meta.get("queue_overflow") == 0 and meta.get("callback_errors") == []),
            blocked=outcome == "BLOCKED")
    identity = meta.get("identity", {}) if meta else {}
    for name, key in (("serial", "serial"), ("firmware", "firmware"), ("usb_type", "usb_type_regex")):
        actual = identity.get(name)
        expected = value(cfg, "identity", key)
        passed = bool(re.fullmatch(expected, str(actual))) if name == "usb_type" else actual == expected
        add(f"identity.{name}", f"SDK device {name} matches configuration", actual, "", [("identity", key)],
            passed=passed, blocked=actual is None)
    profiles = meta.get("profiles", []) if meta else []
    by_name = {p.get("stream"): p for p in profiles}
    profile_ok = len(profiles) == len(STREAMS) and set(by_name) == set(STREAMS)
    for stream in STREAMS:
        p = by_name.get(stream, {})
        if stream.startswith("ir_"):
            profile_ok &= (p.get("width") == value(cfg, "profile", "width")
                           and p.get("height") == value(cfg, "profile", "height")
                           and p.get("fps") == value(cfg, "profile", "ir_hz") and p.get("format") == "y8")
        else:
            rates = [value(cfg, "profile", "gyro_hz")] if stream == "gyro" else value(cfg, "profile", "accel_hz_preference")
            profile_ok &= p.get("fps") in rates and p.get("format") == "motion_xyz32f"
    add("profiles", "Exactly stereo IR Y8 and separate native gyro/accel profiles", profiles, "",
        [("profile", k) for k in ("width", "height", "ir_hz", "gyro_hz", "accel_hz_preference")],
        passed=profile_ok, blocked=not profiles)
    emitter = meta.get("emitter_readback") if meta else None
    add("emitter", "IR emitter OFF readback during capture", emitter, "SDK option", [("profile", "emitter")],
        passed=bool(emitter) and all(x == value(cfg, "profile", "emitter") for x in emitter), blocked=emitter is None)
    start = meta.get("window_start_host_clock_monotonic_ns") if meta else None
    end = meta.get("window_end_host_clock_monotonic_ns") if meta else None
    window_ok = (type(start) is int and type(end) is int and end > start)
    duration = (end - start) / 1e9 if window_ok else None
    add("duration", "Complete declared host CLOCK_MONOTONIC measurement window", duration, "s",
        [("limits", "qualification_duration_s")],
        passed=window_ok and duration >= value(cfg, "limits", "qualification_duration_s"), blocked=not window_ok)
    streams = None
    try:
        streams = read_rows(directory / "samples.csv")
        add("csv", "Finite well-formed per-sample raw evidence", sum(map(len, streams.values())), "rows", [], passed=True)
    except FileNotFoundError:
        add("csv", "Raw samples.csv available", None, "rows", [], blocked=True)
    except (ValueError, OSError, KeyError) as error:
        add("csv", "Finite well-formed per-sample raw evidence", str(error), "", [], passed=False)
    timing = legacy_timing()
    if streams is None:
        for identifier in ("rates", "drops", "receipt_gaps", "timestamps", "stereo", "static", "ir_content"):
            add(identifier, "Required raw evidence unavailable", None, "", [], blocked=True)
    else:
        received = meta.get("received_counts") if meta else None
        written = {s: len(rows) for s, rows in streams.items()}
        add("sample_accounting", "Callback counts equal saved per-stream row counts", {"received": received, "written": written}, "samples", [],
            passed=received == written, blocked=received is None)
        for stream, rows in streams.items():
            sm = metrics["streams"][stream] = {"count": len(rows)}
            enough = len(rows) >= value(cfg, "limits", "min_samples")
            add(f"{stream}.samples", "Stream has sufficient raw observations", len(rows), "samples", [("limits", "min_samples")],
                passed=enough, blocked=not rows)
            nominal = by_name.get(stream, {}).get("fps")
            if not isinstance(nominal, (int, float)) or not math.isfinite(nominal) or nominal <= 0:
                nominal = None
            if window_ok:
                outside = sum(not start <= r["host_clock_monotonic_ns"] < end for r in rows)
                # Reuse first-party timing summary. Its stamp argument is explicitly host
                # monotonic here; SDK clock sanity is evaluated separately per domain.
                converted = [{"frame": r["frame_number"], "arrival": (r["host_clock_monotonic_ns"] - start) / 1e9,
                              "stamp": (r["host_clock_monotonic_ns"] - start) / 1e9} for r in rows]
                summary = timing.summarize(converted)
                sm["host_monotonic_timing"] = summary
                rate = len(rows) / duration
                sm["achieved_rate_hz"] = rate
                add(f"{stream}.rate", "Whole-window count / host CLOCK_MONOTONIC duration", rate, "Hz",
                    [("limits", "rate_relative_tolerance")],
                    passed=nominal is not None and abs(rate / nominal - 1) <= value(cfg, "limits", "rate_relative_tolerance"),
                    blocked=nominal is None or not rows)
                steps = [(b["host_clock_monotonic_ns"] - a["host_clock_monotonic_ns"]) / 1e9 for a, b in zip(rows, rows[1:])]
                edges = [(rows[0]["host_clock_monotonic_ns"] - start) / 1e9,
                         (end - rows[-1]["host_clock_monotonic_ns"]) / 1e9] if rows else []
                sm["receipt_jitter_s"] = {"mean_interval": statistics.mean(steps) if steps else None,
                                          "population_std_interval": statistics.pstdev(steps) if steps else None,
                                          "max_gap_with_boundaries": max(steps + edges) if rows else None}
                gap_key = "ir_max_receipt_gap_s" if stream.startswith("ir_") else "imu_max_receipt_gap_s"
                add(f"{stream}.receipt_gap", "Host CLOCK_MONOTONIC intervals and window edges; no latency claim",
                    sm["receipt_jitter_s"], "s", [("limits", gap_key)],
                    passed=enough and not outside and summary["nonpositive_arrival_steps"] == 0
                           and max(steps + edges) <= value(cfg, "limits", gap_key), blocked=not rows)
                add(f"{stream}.drops", "Missing and nonincreasing frame numbers", {
                    k: summary[k] for k in ("missing_frame_numbers", "nonpositive_frame_steps")}, "frames",
                    [("limits", "missing_frames_max"), ("limits", "nonincreasing_frames_max")],
                    passed=enough and summary["missing_frame_numbers"] <= value(cfg, "limits", "missing_frames_max")
                           and summary["nonpositive_frame_steps"] <= value(cfg, "limits", "nonincreasing_frames_max"), blocked=not rows)
            else:
                for key in ("rate", "receipt_gap", "drops"):
                    add(f"{stream}.{key}", "Declared host clock window unavailable", None, "", [], blocked=True)
            clocks = clock_stats(rows)
            sm["timestamps"] = clocks
            clock_ok = (set(clocks["domains"]) <= DOMAINS and not clocks["field_consistency_errors"]
                        and clocks["domain_changes"] <= value(cfg, "limits", "timestamp_domain_changes_max")
                        and all(c["duplicates"] <= value(cfg, "limits", "timestamp_duplicates_max")
                                and c["reversals"] <= value(cfg, "limits", "timestamp_reversals_max")
                                for c in clocks["clocks"].values()))
            add(f"{stream}.timestamps", "Strictly increasing per-stream timestamps within each named clock",
                clocks, "count / domain", [("limits", k) for k in ("timestamp_domain_changes_max", "timestamp_duplicates_max", "timestamp_reversals_max")],
                passed=enough and clock_ok, blocked=not rows)
        left, right = streams["ir_left"], streams["ir_right"]
        lm = {r["frame_number"]: r for r in left}
        rm = {r["frame_number"]: r for r in right}
        unmatched = len(lm.keys() ^ rm.keys())
        mismatched_domains, stereo_diffs = 0, []
        for number in lm.keys() & rm.keys():
            a, b = lm[number], rm[number]
            if a["sdk_timestamp_domain"] != b["sdk_timestamp_domain"] or a["sdk_timestamp_domain"] not in DOMAINS:
                mismatched_domains += 1
            else:
                stereo_diffs.append(abs(a["sdk_timestamp_ms"] - b["sdk_timestamp_ms"]))
        pairing = {"paired": len(stereo_diffs), "unpaired": unmatched, "domain_mismatches": mismatched_domains,
                   "max_sdk_delta_ms": max(stereo_diffs) if stereo_diffs else None}
        metrics["stereo"] = pairing
        add("stereo", "Pair by frame number and compare SDK stamps only in equal known domains", pairing, "ms / frames",
            [("limits", "stereo_tolerance_ms"), ("limits", "unpaired_frames_max")],
            passed=bool(stereo_diffs) and not mismatched_domains and unmatched <= value(cfg, "limits", "unpaired_frames_max")
                   and max(stereo_diffs) <= value(cfg, "limits", "stereo_tolerance_ms"), blocked=not left or not right)
        static = bool(meta and meta.get("static_confirmed") is True)
        for stream in ("gyro", "accel"):
            rows = streams[stream]
            enough = len(rows) >= value(cfg, "limits", "min_samples")
            data = None
            if enough and static:
                axes = [[r[k] for r in rows] for k in ("x", "y", "z")]
                means = list(map(statistics.mean, axes))
                stds = list(map(statistics.pstdev, axes))
                data = {"axis_mean": means, "axis_population_std": stds,
                        "mean_vector_norm": math.hypot(*means),
                        "mean_sample_norm": statistics.mean(math.hypot(*(r[k] for k in ("x", "y", "z"))) for r in rows)}
            if stream == "gyro":
                keys = ("gyro_mean_norm_rad_s", "gyro_axis_std_rad_s")
                passed = data is not None and data["mean_vector_norm"] <= value(cfg, "limits", keys[0]) and max(data["axis_population_std"]) <= value(cfg, "limits", keys[1])
                unit = "rad/s"
            else:
                keys = ("local_g_m_s2", "accel_norm_error_m_s2", "accel_axis_std_m_s2")
                passed = data is not None and abs(data["mean_sample_norm"] - value(cfg, "limits", keys[0])) <= value(cfg, "limits", keys[1]) and max(data["axis_population_std"]) <= value(cfg, "limits", keys[2])
                unit = "m/s^2"
            metrics[f"{stream}_static"] = data
            add(f"{stream}.static", "Camera IMU static plausibility only; operator --static confirmation required",
                data, unit, [("limits", k) for k in keys], passed=passed, blocked=not static or not enough)
        used_images = set()
        for stream in ("ir_left", "ir_right"):
            images, errors = [], []
            names = [r["image_file"] for r in streams[stream] if r["image_file"]]
            for name in names:
                try:
                    if name in used_images:
                        raise ValueError("Repeated image reference")
                    used_images.add(name)
                    images.append(image_stats(directory / name, value(cfg, "profile", "width"), value(cfg, "profile", "height")))
                except (OSError, ValueError) as error:
                    errors.append(str(error))
            im = {"samples": images, "errors": errors}
            metrics[f"{stream}_content"] = im
            add(f"{stream}.ir_content", "Sampled raw Y8 mean and saturated fraction", im, "Y8 / fraction",
                [("capture", "ir_samples_per_stream")] + [("limits", k) for k in ("ir_mean_min", "ir_mean_max", "ir_saturated_fraction_max")],
                passed=len(images) >= value(cfg, "capture", "ir_samples_per_stream") and not errors
                       and all(value(cfg, "limits", "ir_mean_min") <= x["mean"] <= value(cfg, "limits", "ir_mean_max")
                               and x["saturated_fraction"] <= value(cfg, "limits", "ir_saturated_fraction_max") for x in images),
                blocked=not names)
    metrics["latency"] = {"status": "UNKNOWN", "host_receipt_latency_s": None,
                          "reason": "No qualified device-to-host CLOCK_MONOTONIC mapping recorded. tools/clockmap README warns that SDK global time maps to system time, not monotonic. Only same-clock inter-arrival jitter is reported; no fit to arrivals is used as latency."}
    add("latency_policy", "Cross-clock receipt latency withheld without documented qualified mapping",
        metrics["latency"], "", [], passed=True)
    statuses = {c["status"] for c in criteria}
    status = "FAIL" if "FAIL" in statuses else "BLOCKED" if "BLOCKED" in statuses else "PASS"
    raw_files = [{"path": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                 for p in sorted(directory.iterdir()) if p.is_file() and p.name not in ("result.json", "metrics.json")]
    result = {"check": "d435i_raw", "tool_version": VERSION, "status": status,
              "started_utc": started, "ended_utc": utc(), "host": host(), "criteria": criteria,
              "raw_files": raw_files,
              "notes": ["CALCULATED: saved-data criteria only. A fixture PASS is not a hardware gate PASS.",
                        "PROPOSED: acceptance limits require lab review; static statistics are plausibility, not calibration.",
                        "UNKNOWN: actual Pi behavior, power/thermal/USB stability, camera-to-IMU alignment and exposure latency.",
                        "SDK USB descriptor is checked; physical negotiated bus speed is not independently measured."]}
    write_json(directory / "metrics.json", metrics)
    write_json(directory / "result.json", result)
    return result
