#!/usr/bin/env python3
"""Unittest PTY integration and a saved-evidence check (--outdir/--analyze-only)."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import platform
import select
import subprocess
import sys
import time
import unittest
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "py"))
from vio_to_pose_events import CONFIG, ArgumentParser, Converter, load_mapping
from perception_health import (HEARTBEAT, TYPE_HEARTBEAT, TYPE_PERCEPTION_HEALTH,
                               StreamParser, decode_health_payload, encode_frame)
from supervisor_model import Event, SupervisorModel

POLICY_PATH = HERE.parent / "config.json"
SENDER = HERE.parent / "py" / "sender.py"
REAL = ROOT / "tools" / "vio_adapter" / "real_replay" / "snapshots.jsonl"
SCENARIOS = ("no_mapping", "mapped_ok", "lost_latched", "sender_stops", "real_replay")


def utc():
    return datetime.now(timezone.utc).isoformat()


def synthetic_snapshot(state="TRACKING"):
    return {"source_ns": 1000000000, "domain": "global_time", "sequence": 1,
            "state": state, "orb_tracking_state": "LOST" if state == "LOST" else "OK",
            "valid": state == "TRACKING", "reasons": 0, "reset_counter": 0,
            "imu_initialized": True, "tracked_features": 77, "evidence": "ASSUMED synthetic fixture"}


def synthetic_mapping():
    # Entire mapping is ASSUMED for the software exercise; never applied to real replay.
    raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
    base = 1000000000
    offset = raw - CONFIG["synthetic_pose_age_ns"]["value"] - base
    exchanges = []
    for a in (base - 2000000, base - 1000000, base):
        exchanges.append([a, a + offset + 1000, a + offset + 2000, a + 3000])
    return {"valid": True, "snapshot_domain": "global_time", "camera_domain": "D435i_global_time",
            "pi_domain": "CLOCK_MONOTONIC_RAW", "camera_session": "synthetic-camera",
            "pi_session": "synthetic-host-current-process", "exchanges_ns": exchanges,
            "evidence": "ASSUMED synthetic affine exchanges; not calibration"}


def capture(directory, name):
    """Actual sender subprocess writes to a tty; receiver records bytes verbatim."""
    directory.mkdir()
    timeout = CONFIG["pty_timeout_s"]["value"]
    master, slave = os.openpty()
    proc = None
    raw = bytearray()
    reads = []
    parser = StreamParser()
    frames = []
    started = time.monotonic_ns()

    def pump(wait):
        if select.select([master], [], [], wait)[0]:
            chunk = os.read(master, 65536)
            if chunk:
                stamp = time.monotonic_ns()
                reads.append({"offset": len(raw), "length": len(chunk),
                              "host_CLOCK_MONOTONIC_ns": stamp})
                raw.extend(chunk)
                frames.extend(parser.feed(chunk))

    with (directory / "sender.stderr").open("wb") as stderr:
        try:
            # Preload the existing mapping module before readiness heartbeat.
            # This prevents NumPy import time from masquerading as synthetic pose age.
            launcher = "import runpy, sys, pathlib; sys.path.insert(0, str(pathlib.Path(sys.argv[1]).parent)); "
            if importlib.util.find_spec("numpy") is not None:
                launcher += f"sys.path.insert(0, {str(ROOT)!r}); import tools.clockmap.clockmap; "
            launcher += "runpy.run_path(__import__('sys').argv.pop(1), run_name='__main__')"
            command = [sys.executable, "-B", "-c", launcher, str(SENDER), "--device", os.ttyname(slave),
                       "--baud", str(CONFIG["uart"]["bauds"][0]), "--session-id", "9"]
            proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=stderr)
            deadline = time.monotonic() + timeout
            while not frames:
                if proc.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("sender did not emit readiness heartbeat")
                pump(min(0.05, timeout))
            mapping = None
            if name in ("mapped_ok", "lost_latched", "sender_stops"):
                # Finish the parent's first ClockMap/NumPy import before anchoring
                # the synthetic epoch. Import latency is not estimator latency.
                __import__("tools.clockmap.clockmap")
                mapping = synthetic_mapping()
                (directory / "clockmap.json").write_text(json.dumps(mapping) + "\n")
                mapping = load_mapping(directory / "clockmap.json")
            converter = Converter("synthetic-camera" if name != "real_replay" else "saved-real-replay", mapping)
            if name == "real_replay":
                snapshots = [json.loads(line) for line in REAL.read_text().splitlines()]
            elif name == "lost_latched":
                snapshots = [synthetic_snapshot(), synthetic_snapshot("LOST"), synthetic_snapshot()]
            else:
                snapshots = [synthetic_snapshot(), synthetic_snapshot(), synthetic_snapshot()]
            events = [converter.convert(s) for s in snapshots]
            (directory / "snapshots.jsonl").write_text("".join(json.dumps(s) + "\n" for s in snapshots))
            event_bytes = "".join(json.dumps(e) + "\n" for e in events).encode()
            (directory / "events.jsonl").write_bytes(event_bytes)
            # Feed one event at a time while draining; no pipe or tty-buffer deadlock.
            for index, event in enumerate(events, 1):
                proc.stdin.write((json.dumps(event) + "\n").encode())
                proc.stdin.flush()
                deadline = time.monotonic() + timeout
                while sum(t == TYPE_PERCEPTION_HEALTH for t, _, _ in frames) < index:
                    if proc.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError(f"sender failed to deliver event {index}")
                    pump(min(0.05, timeout))
            proc.stdin.close()
            deadline = time.monotonic() + timeout
            while proc.poll() is None:
                if time.monotonic() >= deadline:
                    raise RuntimeError("sender did not stop at EOF")
                pump(min(0.01, timeout))
            while select.select([master], [], [], 0)[0]:
                pump(0)
            (directory / "sender.stdout").write_bytes(proc.stdout.read())
            returncode = proc.returncode
        finally:
            if proc is not None:
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=timeout)
                if proc.stdin is not None and not proc.stdin.closed:
                    proc.stdin.close()
                if proc.stdout is not None:
                    proc.stdout.close()
            os.close(master)
            os.close(slave)
    (directory / "uart.bin").write_bytes(raw)
    (directory / "reads.json").write_text(json.dumps(reads, indent=2) + "\n")
    (directory / "capture.json").write_text(json.dumps({
        "name": name, "command": command, "returncode": returncode,
        "started_host_CLOCK_MONOTONIC_ns": started,
        "ended_host_CLOCK_MONOTONIC_ns": time.monotonic_ns(),
        "expected_health_frames": len(events), "host": {"platform": platform.platform(),
        "python": sys.version, "machine": platform.machine()},
        "evidence": "MEASURED local PTY; real replay associations MEASURED, engine estimates synthetic"
    }, indent=2) + "\n")


def replay(directory, silence=False, primary_fresh=False):
    meta = json.loads((directory / "capture.json").read_text())
    reads = json.loads((directory / "reads.json").read_text())
    wire = (directory / "uart.bin").read_bytes()
    policy = json.loads((directory.parent / "policy.json").read_text())
    config_path = directory.parent / "integration_config.json"
    config = json.loads(config_path.read_text()) if config_path.exists() else CONFIG
    if silence:
        policy.update({k: v for k, v in config["silence_policy_overrides"].items() if k != "source"})
    # Independent primary freshness is an explicit simulation input. The health
    # report itself must never supply motion authority.
    model = SupervisorModel(policy, primary_fresh=primary_fresh)
    parser = StreamParser()
    trace, health, sequences = [], [], []
    offset = 0
    encoded_bytes = 0
    heartbeat_count = 0
    previous_read_ns = meta["started_host_CLOCK_MONOTONIC_ns"]
    for read in reads:
        if read["offset"] != offset or read["length"] <= 0:
            raise ValueError("invalid raw read offsets")
        read_ns = read["host_CLOCK_MONOTONIC_ns"]
        if (type(read_ns) is not int or
                read_ns < meta["started_host_CLOCK_MONOTONIC_ns"] or
                read_ns < previous_read_ns):
            raise ValueError("read timestamps regress or precede capture start")
        previous_read_ns = read_ns
        end = offset + read["length"]
        if end > len(wire):
            raise ValueError("read extends beyond raw bytes")
        now = (read_ns - meta["started_host_CLOCK_MONOTONIC_ns"]) // 1000
        for typ, seq, payload in parser.feed(wire[offset:end]):
            sequences.append(seq)
            encoded_bytes += len(encode_frame(typ, seq, payload))
            if typ == TYPE_HEARTBEAT:
                heartbeat_count += 1
                _, session = HEARTBEAT.unpack(payload)
                event = Event("heartbeat", now, session_id=session, wire_seq=seq)
            elif typ == TYPE_PERCEPTION_HEALTH:
                value = decode_health_payload(payload)
                health.append(value)
                event = Event("health", now, value.pose_age_us, value.tracking_state, value.flags,
                              map_reset_counter=value.map_reset_counter,
                              imu_initialized=value.imu_initialized, wire_seq=seq)
            else:
                raise ValueError("unexpected message type")
            model.update(now, event)
            trace.append({"kind": event.kind, "seq": seq, "simulated_MCU_timer_us": now,
                          "state": model.state, "reason": model.reason,
                          "primary_fresh": primary_fresh})
        offset = end
    if offset != len(wire):
        raise ValueError("raw reads do not cover all bytes")
    if silence and model.last_health is not None:
        # No waiting/hardware timing claim: advance an independent simulated MCU clock.
        for delta in (policy["receipt_silence_hold_us"], policy["receipt_silence_timeout_us"]):
            now = model.last_health + delta
            model.update(now)
            trace.append({"kind": "silence_tick", "simulated_MCU_timer_us": now,
                          "state": model.state, "reason": model.reason})
    duration = (meta["ended_host_CLOCK_MONOTONIC_ns"] - meta["started_host_CLOCK_MONOTONIC_ns"]) / 1e9
    if duration <= 0:
        raise ValueError("nonpositive capture interval")
    frame_count = len(sequences)
    integrity = (meta["returncode"] == 0 and len(health) > 0 and heartbeat_count > 0
                 and len(health) == meta["expected_health_frames"]
                 and encoded_bytes == len(wire)
                 and not parser.buf and not parser.crc_errors and not parser.len_errors
                 and not parser.bytes_dropped and sequences == list(range(len(sequences))))
    rate = len(wire) / duration
    metrics = {"evidence": "MEASURED on " + meta["host"]["platform"] + "; software PTY only",
               "observation_window": "sender process launch through EOF exit (includes startup/imports)",
               "duration_s": duration, "health_frames": len(health),
               "heartbeat_frames": heartbeat_count, "total_frames": frame_count,
               "bytes_written_and_received": len(wire), "frames_per_s": frame_count / duration,
               "bytes_per_s": rate,
               "uart_budget": [{"evidence": "CALCULATED ideal 8N1; PTY does not emulate baud",
                                "baud": baud, "capacity_bytes_per_s": baud / config["uart"]["bits_per_byte"],
                                "measured_utilization": rate * config["uart"]["bits_per_byte"] / baud,
                                "within_ideal_capacity": rate * config["uart"]["bits_per_byte"] / baud <= config["uart"]["max_utilization"]}
                               for baud in config["uart"]["bauds"]]}
    return trace, health, integrity, metrics


def analyze(outdir):
    started = utc()
    criteria, metrics = [], {}

    def criterion(ident, description, measured, expected, path, blocked=False):
        criteria.append({"id": ident, "description": description, "measured": measured,
                         "unit": "boolean", "threshold": expected,
                         "status": "BLOCKED" if blocked else "PASS" if measured else "FAIL",
                         "evidence": path, "threshold_source": "SOURCE-SUPPORTED: T10b task and tools/perception_health/SPEC.md supervisor recommendation"})

    for name in SCENARIOS:
        directory = outdir / name
        try:
            trace, health, integrity, metric = replay(directory, silence=name == "sender_stops",
                                                     primary_fresh=True)
        except (FileNotFoundError, ImportError) as exc:
            criterion(name, str(exc), None, "saved capture present and dependencies available",
                      f"{name}/capture.json", blocked=True)
            continue
        except (ValueError, KeyError, TypeError) as exc:
            criterion(name, f"invalid saved evidence: {exc}", False, "valid raw evidence",
                      f"{name}/capture.json")
            continue
        metrics[name] = metric
        (directory / "trace.json").write_text(json.dumps(trace, indent=2) + "\n")
        criterion(name + ".transport", "sender frames/bytes/count/CRC/sequence complete", integrity,
                  "all input events written as valid frames", f"{name}/uart.bin")
        states = [row["state"] for row in trace if row["kind"] == "health"]
        if name == "no_mapping":
            criterion(name, "no explicit clockmap -> HOLD for every OK report", bool(health) and states == ["HOLD"] * len(health),
                      "HOLD only", f"{name}/trace.json")
        elif name == "mapped_ok":
            criterion(name, "explicit synthetic mapping + known OK/IMU/reset/features -> FRESH",
                      bool(health) and states == ["FRESH"] * len(health), "FRESH with independent primary freshness", f"{name}/trace.json")
        elif name == "lost_latched":
            criterion(name, "LOST latches FAILSAFE through subsequent OK", states == ["FRESH", "FAILSAFE", "FAILSAFE"],
                      "FRESH, FAILSAFE, FAILSAFE", f"{name}/trace.json")
        elif name == "sender_stops":
            ticks = [(row["state"], row["reason"]) for row in trace if row["kind"] == "silence_tick"]
            criterion(name, "EOF stops sender; isolated receipt-silence deadlines downgrade",
                      bool(health) and states == ["FRESH"] * len(health) and ticks == [("HOLD", "RESULT_SILENCE"), ("FAILSAFE", "RESULT_TIMEOUT")],
                      "FRESH -> HOLD/RESULT_SILENCE -> FAILSAFE/RESULT_TIMEOUT", f"{name}/trace.json")
        else:
            criterion(name, "saved association/fake engine: unknown age flags, degraded, never FRESH",
                      bool(health) and all(h.flags & 3 == 0 and h.flags & 4 for h in health) and "FRESH" not in states,
                      "unavailable age + degraded; no FRESH", f"{name}/trace.json")
    (outdir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    raw_files = []
    for path in sorted(outdir.rglob("*")):
        if path.is_file() and path.name not in ("result.json", "trace.json", "metrics.json"):
            raw_files.append({"path": str(path.relative_to(outdir)), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    statuses = {c["status"] for c in criteria}
    status = "FAIL" if "FAIL" in statuses else "BLOCKED" if "BLOCKED" in statuses else "PASS"
    provenance_path = outdir / "provenance.json"
    acquisition_started = json.loads(provenance_path.read_text())["captured_utc"] if provenance_path.exists() else started
    result = {"check": "perception_health_e2e_pty", "tool_version": CONFIG["tool_version"], "status": status,
              "started_utc": acquisition_started, "analysis_started_utc": started, "ended_utc": utc(),
              "host": {"platform": platform.platform(), "python": sys.version},
              "criteria": criteria, "raw_files": raw_files,
              "notes": ["Software fixture result is not a gate PASS. No Pi/UART/STM32 hardware exercised.",
                        "ASSUMED independent primary_fresh=True in policy scenarios; health alone cannot enable FRESH.",
                        "Real replay estimates are synthetic, and contain LOST; no clockmap was fabricated for them.",
                        "Silence uses simulated MCU-local timer increments with diagnostic overrides in integration/config.json.",
                        "Metrics are local PTY software write throughput; CALCULATED baud capacity is not physical link performance."]}
    (outdir / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"{'criterion':30} status")
    for c in criteria:
        print(f"{c['id']:30} {c['status']}")
    print(f"Overall: {status}")
    for name, m in metrics.items():
        print(f"MEASURED {name}: {m['frames_per_s']:.2f} frames/s, {m['bytes_per_s']:.2f} bytes/s; "
              + "; ".join(f"CALCULATED {b['baud']} baud {b['capacity_bytes_per_s']:.0f} bytes/s "
                          f"({b['measured_utilization']:.3f} utilization)" for b in m["uart_budget"]))
    return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[status]


def summarize(outdir, destination):
    """Archive summaries and hashes; full raw evidence stays in the capture dir."""
    result = json.loads((outdir / "result.json").read_text())
    manifest = [{"path": str(path.relative_to(outdir)), "bytes": path.stat().st_size,
                 "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                for path in sorted(outdir.rglob("*")) if path.is_file()]
    scenarios = {}
    for name in SCENARIOS:
        trace_path = outdir / name / "trace.json"
        if trace_path.exists():
            trace = json.loads(trace_path.read_text())
            counts = Counter((row["kind"], row["state"], row["reason"]) for row in trace)
            scenarios[name] = [{"kind": kind, "state": state, "reason": reason, "count": count}
                               for (kind, state, reason), count in sorted(counts.items())]
    def saved_json(name):
        path = outdir / name
        return json.loads(path.read_text()) if path.exists() else None

    summary = {"retention": "Summary only. Criterion evidence paths refer to full capture files listed in sha256.json; regenerate full captures with --outdir before --analyze-only.",
               "result": result, "trace_counts": scenarios,
               "metrics": saved_json("metrics.json"),
               "provenance": saved_json("provenance.json"),
               "policy": saved_json("policy.json"),
               "integration_config": saved_json("integration_config.json")}
    files = {"summary.json": json.dumps(summary, indent=2) + "\n",
             "sha256.json": json.dumps(manifest, indent=2) + "\n"}
    if sum(len(value.encode()) for value in files.values()) >= CONFIG["evidence_max_bytes"]["value"]:
        raise ValueError("summary archive exceeds configured evidence budget")
    destination.mkdir(parents=True, exist_ok=False)
    for name, value in files.items():
        (destination / name).write_text(value)


def main():
    parser = ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--outdir", type=Path, help="new directory for offline PTY captures")
    group.add_argument("--analyze-only", type=Path, help="re-evaluate saved raw bytes, no subprocess or tty")
    parser.add_argument("--summary-outdir", type=Path, help="new directory for compact summaries and SHA-256 hashes")
    args = parser.parse_args()
    try:
        if args.analyze_only:
            if not args.analyze_only.is_dir():
                raise ValueError("analyze directory does not exist")
            status = analyze(args.analyze_only)
            if args.summary_outdir:
                summarize(args.analyze_only, args.summary_outdir)
            return status
        args.outdir.mkdir(parents=True, exist_ok=False)
        if POLICY_PATH.is_file():
            (args.outdir / "policy.json").write_bytes(POLICY_PATH.read_bytes())
        (args.outdir / "integration_config.json").write_bytes((HERE / "config.json").read_bytes())
        inputs = (SENDER, POLICY_PATH, HERE.parent / "py/supervisor_model.py",
                  HERE.parent / "py/perception_health.py", ROOT / "tools/clockmap/clockmap.py",
                  REAL, HERE / "vio_to_pose_events.py", Path(__file__))
        missing = [str(path) for path in inputs if not path.is_file()]
        if missing:
            # A missing fixture/tool is unavailable evidence, not an internal
            # error. Let the analyzer emit the common BLOCKED result contract.
            return analyze(args.outdir)
        (args.outdir / "provenance.json").write_text(json.dumps({
            "captured_utc": utc(), "inputs": [{"path": str(p.relative_to(ROOT)),
                "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in inputs]
        }, indent=2) + "\n")
        for name in SCENARIOS:
            if name in ("mapped_ok", "lost_latched", "sender_stops") and importlib.util.find_spec("numpy") is None:
                continue
            capture(args.outdir / name, name)
        status = analyze(args.outdir)
        if args.summary_outdir:
            summarize(args.outdir, args.summary_outdir)
        return status
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"check error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    if any(arg.startswith("--outdir") or arg.startswith("--analyze-only") for arg in sys.argv[1:]):
        sys.exit(main())
    unittest.main(module="tests.test_integration", verbosity=2)
