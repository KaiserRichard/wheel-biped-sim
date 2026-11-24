"""Pi 5 runtime profiling sampler and offline latency/validity/report tools."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.runtime_observer import aggregate, patch_timing, resource_sampler, stats

HERE = Path(__file__).resolve().parent
UNKNOWN = "UNKNOWN"
THROTTLE_NAMES = ("undervoltage_now", "frequency_capped_now", "throttled_now",
                  "soft_temperature_limit_now", "undervoltage_occurred",
                  "frequency_capped_occurred", "throttled_occurred",
                  "soft_temperature_limit_occurred")


def config():
    return json.loads((HERE / "config.json").read_text())


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z")


def proc_cpu(text):
    cores = {}
    for line in (text or "").splitlines():
        columns = line.split()
        if columns and columns[0].startswith("cpu") and columns[0][3:].isdigit() and len(columns) >= 9:
            vals = [int(x) for x in columns[1:9]]
            cores[columns[0]] = vals
    return cores


def cpu_percent(before, after):
    result = {}
    for name in sorted(set(before) & set(after)):
        delta = [b - a for a, b in zip(before[name], after[name])]
        total = sum(delta)
        if total > 0 and min(delta) >= 0:
            result[name] = 100.0 * (total - delta[3] - delta[4]) / total
        else:
            result[name] = UNKNOWN
    return result


def read_optional(path):
    return resource_sampler.read_text(Path(path))


def rss_pss(pid, proc_root=Path("/proc")):
    status = resource_sampler.status_fields(read_optional(proc_root / str(pid) / "status"))
    rollup = read_optional(proc_root / str(pid) / "smaps_rollup")
    pss = None
    if rollup:
        import re
        match = re.search(r"^Pss\x3a\s+([0-9]+)\s+kB\s*$", rollup, re.M)
        if match:
            pss = int(match[1]) * 1024
    rss = status.get("rss_bytes", UNKNOWN)
    return (UNKNOWN if rss in (None, resource_sampler.UNAVAILABLE) else rss,
            pss if pss is not None else UNKNOWN)


def fan_states(sys_root=Path("/sys")):
    found = {}
    for path in sorted((sys_root / "class/thermal").glob("cooling_device*")):
        name = read_optional(path / "type")
        state = read_optional(path / "cur_state")
        found[path.name] = {"type": name.strip() if name else UNKNOWN,
                            "cur_state": state.strip() if state else UNKNOWN}
    return found or UNKNOWN


def sample_once(pid, old_cpu, old_proc, old_time, *, proc_root=Path("/proc"), sys_root=Path("/sys"),
                include_threads=False, thread_previous=None, do_throttle=True):
    now = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    row = {"monotonic_ns": now, "clock_domain": "CLOCK_MONOTONIC", "pid": pid}
    proc_stat = read_optional(proc_root / "stat")
    current_cpu = proc_cpu(proc_stat)
    per_core = cpu_percent(old_cpu, current_cpu)
    row["per_core_cpu_pct_json"] = json.dumps(per_core or UNKNOWN, sort_keys=True)
    board = resource_sampler.cpu_counters(proc_stat)
    row["cpu_count"] = board[1] if board else UNKNOWN
    process = resource_sampler.process_counters(read_optional(proc_root / str(pid) / "stat"))
    rss, pss = rss_pss(pid, proc_root)
    row["rss_bytes"], row["pss_bytes"] = rss, pss
    row["process_start_ticks"] = process[1] if process else UNKNOWN
    row["target_cpu_pct_one_core"] = (100.0 * (process[0] - old_proc[0]) / os.sysconf("SC_CLK_TCK") /
                                       ((now - old_time) / 1e9)
                                       if process and old_proc and process[1] == old_proc[1] and now > old_time
                                       and process[0] >= old_proc[0] else UNKNOWN)
    row["thread_cpu_pct_json"] = UNKNOWN
    if include_threads:
        threads = {}
        try:
            tids = sorted((proc_root / str(pid) / "task").iterdir(), key=lambda p: p.name)
        except OSError:
            tids = []
        for tid in tids:
            current = resource_sampler.process_counters(read_optional(tid / "stat"))
            previous = (thread_previous or {}).get(tid.name)
            threads[tid.name] = (100.0 * (current[0] - previous[0]) / os.sysconf("SC_CLK_TCK") /
                                 ((now - old_time) / 1e9) if current and previous and now > old_time and
                                 current[1] == previous[1] and current[0] >= previous[0] else UNKNOWN)
            if current:
                (thread_previous if thread_previous is not None else {})[tid.name] = current
        row["thread_cpu_pct_json"] = json.dumps(threads or UNKNOWN, sort_keys=True)
    frequencies = {}
    for path in sorted(sys_root.glob("devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq")):
        value = resource_sampler.nonnegative(read_optional(path))
        frequencies[path.parents[1].name] = value if value is not None else UNKNOWN
    row["cpu_freq_khz_json"] = json.dumps(frequencies or UNKNOWN, sort_keys=True)
    thermal = {}
    for p in sorted(sys_root.glob("class/thermal/thermal_zone*")):
        value = resource_sampler.nonnegative(read_optional(p / "temp"))
        zone_type = read_optional(p / "type")
        thermal[p.name] = {"type": zone_type.strip() if zone_type else UNKNOWN,
                           "c": value / 1000 if value is not None else UNKNOWN}
    row["thermal_c_json"] = json.dumps(thermal or UNKNOWN, sort_keys=True)
    row["fan_state_json"] = json.dumps(fan_states(sys_root), sort_keys=True)
    row.update({name: UNKNOWN for name in THROTTLE_NAMES})
    row["throttled_raw"] = UNKNOWN
    row["throttle_sampled"] = int(do_throttle)
    if do_throttle:
        raw = resource_sampler.get_throttled()
        import re
        match = re.fullmatch(r"throttled=(0x[0-9a-fA-F]{1,8})", raw or "")
        if match:
            bits = int(match[1], 16)
            row["throttled_raw"] = hex(bits)
            for key, bit in zip(THROTTLE_NAMES, config()["throttle_bits_invalid"]["value"]):
                row[key] = int(bool(bits & (1 << bit)))
    row["sample_duration_ns"] = time.clock_gettime_ns(time.CLOCK_MONOTONIC) - now
    return row, current_cpu, process


def sampler(args):
    out = args.out
    out.mkdir(parents=True, exist_ok=False)
    csv_path, meta_path = out / "samples.csv", out / "metadata.json"
    started_utc = utc_now()
    anchor_mono = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    anchor_utc = utc_now()
    fields = ["monotonic_ns", "clock_domain", "pid", "cpu_count", "per_core_cpu_pct_json",
              "target_cpu_pct_one_core", "thread_cpu_pct_json", "process_start_ticks", "rss_bytes",
              "pss_bytes", "cpu_freq_khz_json", "thermal_c_json", "throttled_raw", "throttle_sampled", *THROTTLE_NAMES,
              "fan_state_json", "sample_duration_ns"]
    proc_root, sys_root = Path(args.proc_root), Path(args.sys_root)
    if not (proc_root / str(args.pid)).exists():
        raise ValueError(f"target PID {args.pid} is absent under {proc_root}")
    previous_cpu, previous_process, previous_ns, threads = {}, None, anchor_mono, {}
    next_sample = time.monotonic()
    next_throttle = next_sample
    count = 0
    with csv_path.open("x", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        try:
            while args.samples is None or count < args.samples:
                now = time.monotonic()
                throttle = now >= next_throttle
                row, current_cpu, current_process = sample_once(args.pid, previous_cpu, previous_process,
                    previous_ns, proc_root=proc_root, sys_root=sys_root, include_threads=args.threads,
                    thread_previous=threads, do_throttle=throttle)
                writer.writerow(row); stream.flush(); count += 1
                previous_cpu, previous_process, previous_ns = current_cpu, current_process, row["monotonic_ns"]
                if throttle:
                    next_throttle += args.throttle_period_s
                    if next_throttle <= now:
                        next_throttle = now + args.throttle_period_s
                if args.samples is not None and count >= args.samples:
                    break
                next_sample += args.period_s
                now = time.monotonic()
                if now > next_sample:
                    next_sample += (math.floor((now - next_sample) / args.period_s) + 1) * args.period_s
                time.sleep(max(0, next_sample - now))
        except KeyboardInterrupt:
            pass
    meta = {"check": "sampler", "started_utc": started_utc, "ended_utc": utc_now(),
            "host": {"system": platform.system(), "release": platform.release(),
                     "machine": platform.machine(), "python": platform.python_version()},
            "pid": args.pid, "clock_domain": "Linux CLOCK_MONOTONIC", "utc_anchor": anchor_utc,
            "monotonic_anchor_ns": anchor_mono, "utc_anchor_method": "paired wall UTC and CLOCK_MONOTONIC reads; not a clock-map qualification",
            "period_s": args.period_s, "throttle_period_s": args.throttle_period_s,
            "threads_enabled": args.threads, "sample_count": count,
            "files": {"samples.csv": sha(csv_path)}}
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")
    return out


def read_observer(path):
    events, losses = aggregate.read_csv(Path(path).read_text())
    summary = aggregate.summarize(events, losses=losses)
    return summary


def latency(args):
    c = config()
    if args.events:
        summary = read_observer(args.events)
        sessions = summary["sessions"]
        results = {}
        budget_ns = int(round(c["camera_period_ms"]["value"] * 1e6))
        for session, report in sessions.items():
            # Re-read events to compute over-budget frame count and realtime factor using tracking service time.
            events, _ = aggregate.read_csv(Path(args.events).read_text())
            starts, ends, receives, publishes = {}, {}, {}, {}
            receive_sources = {}
            for event in events:
                if event.host_session != session: continue
                {"track_start": starts, "track_end": ends, "frame_received": receives,
                 "snapshot_published": publishes}.get(event.kind, {}).update({event.frame_id: event.host_ns})
                if event.kind == "frame_received" and event.source is not None:
                    receive_sources[event.frame_id] = event.source
            per_frame_track = [ends[k] - starts[k] for k in starts.keys() & ends.keys()]
            per_frame_e2e = [publishes[k] - receives[k] for k in receives.keys() & publishes.keys()]
            span = report["observed_span_ns"]
            source_epochs = []
            identities = {(source[1], source[2]) for source in receive_sources.values()}
            if len(identities) == 1:
                domain, source_session = next(iter(identities))
                source_epochs = [source[0] for source in receive_sources.values()]
                if domain != "host_steady" or source_session != session:
                    source_epochs = []
                    if args.clockmap:
                        mapping = aggregate.load_clockmap(args.clockmap)
                        from tools.clockmap import Stamp
                        mapped = [mapping.convert(Stamp(source[0], source[1], source[2])).stamp
                                  for source in receive_sources.values()]
                        if any(stamp.domain != "host_steady" or stamp.session != session for stamp in mapped):
                            raise ValueError("clockmap must map source stamps to this observer host_steady session")
                        source_epochs = [stamp.value_ns for stamp in mapped]
            covered_s = ((max(source_epochs) - min(source_epochs)) / span
                         if len(source_epochs) > 1 and span > 0 else None)
            results[session] = {"clock_domain": "host_steady / Linux CLOCK_MONOTONIC", "frame_count": len(per_frame_track),
                "tracking_latency_ns": stats.distribution(per_frame_track),
                "end_to_end_latency_ns": stats.distribution(per_frame_e2e),
                "over_camera_budget_count": sum(v > budget_ns for v in per_frame_track),
                "camera_budget_ns": budget_ns,
                "realtime_factor": covered_s if covered_s is not None else UNKNOWN,
                "realtime_factor_definition": "frame_received source timestamp span mapped to host_steady / full observer host_steady span; UNKNOWN unless source epochs are same-session host_steady or an explicit --clockmap is supplied"}
        output = {"evidence": "MEASURED from event timestamps; values are conditional on complete retained event pairs",
                  "quantile_method": "nearest-rank (tools/runtime_observer/stats.py)", "sessions": results,
                  "input_sha256": sha(args.events)}
    elif args.tracking:
        records = patch_timing.read_tracking(Path(args.tracking).read_text())
        durations = [row["track_ns"] for row in records]
        budget = int(round(c["camera_period_ms"]["value"] * 1e6))
        output = {"evidence": "MEASURED tracking duration parsed by tools/runtime_observer/patch_timing.py; hardware clock epoch UNKNOWN",
            "clock_domain": "host_steady duration only; image_s is a source timestamp with no cross-clock mapping",
            "quantile_method": "nearest-rank (tools/runtime_observer/stats.py)",
            "tracking_latency_ns": stats.distribution(durations), "end_to_end_latency_ns": UNKNOWN,
            "over_camera_budget_count": sum(x > budget for x in durations), "camera_budget_ns": budget,
            "realtime_factor": UNKNOWN, "frame_count": len(records), "input_sha256": sha(args.tracking)}
    else:
        raise ValueError("latency needs --events or --tracking")
    (args.out / "latency.json").write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps(output, indent=2, allow_nan=False))


def load_samples(directory):
    with (Path(directory) / "samples.csv").open(newline="") as stream:
        return list(csv.DictReader(stream))


def validity(args):
    c = config()
    samples_path = Path(args.analyze_only) / "samples.csv"
    try:
        rows = load_samples(args.analyze_only)
    except OSError:
        rows = []
    temps, observed_invalid = [], []
    unknown_throttle = False
    unknown_temperature = False
    throttle_observation_count = 0
    for row in rows:
        try:
            zones = json.loads(row["thermal_c_json"])
        except (ValueError, KeyError): zones = {}
        soc = zones.get("thermal_zone0") if isinstance(zones, dict) else None
        value_c = soc.get("c") if isinstance(soc, dict) else None
        # NaN/inf is not a temperature observation: treat it as missing evidence.
        if isinstance(value_c, (float, int)) and not isinstance(value_c, bool) and math.isfinite(value_c):
            temps.append(float(value_c))
        else:
            unknown_temperature = True
        for key in THROTTLE_NAMES:
            sampled = row.get("throttle_sampled")
            # Older saved fixtures have no marker: a parsed raw VC value proves
            # that row was sampled; otherwise telemetry status is unknown.
            if sampled == "0":
                continue
            value = row.get(key, UNKNOWN)
            if value in ("0", "1"):
                if value == "1": observed_invalid.append(key)
            else: unknown_throttle = True
        if row.get("throttle_sampled") != "0":
            throttle_observation_count += 1
    max_temp = max(temps) if temps else None
    missing_temp = max_temp is None or unknown_temperature
    temp_ok = max_temp is not None and max_temp < c["soft_temperature_limit_c"]["value"]
    criteria = []
    criteria.append({"id":"no_throttle_or_undervoltage", "description":"No current or historical under-voltage/frequency-cap/throttle/soft-temperature bit in sampled VC state",
        "measured": sorted(set(observed_invalid)) if not unknown_throttle else (sorted(set(observed_invalid)) or None), "unit":"bit names",
        "threshold":f"<={c['max_invalid_throttle_occurrences']['value']} occurrences",
        "status":"FAIL" if observed_invalid else "BLOCKED" if unknown_throttle or not throttle_observation_count else "PASS",
        "evidence":"samples.csv", "threshold_source":c["max_invalid_throttle_occurrences"]["source"]})
    criteria.append({"id":"max_temperature", "description":"Maximum sampled SoC temperature remains below configured soft limit",
        "measured":max_temp, "unit":"degC", "threshold":f"<{c['soft_temperature_limit_c']['value']} degC",
        "status":"FAIL" if max_temp is not None and not temp_ok else "BLOCKED" if missing_temp else "PASS", "evidence":"samples.csv",
        "threshold_source":c["soft_temperature_limit_c"]["source"]})
    status = "FAIL" if any(x["status"] == "FAIL" for x in criteria) else "BLOCKED" if any(x["status"] == "BLOCKED" for x in criteria) else "PASS"
    result = {"check":"profile_validity", "tool_version":"1.0", "status":status,
        "started_utc":utc_now(), "ended_utc":utc_now(), "host":{"system":platform.system(),"machine":platform.machine()},
        "criteria":criteria, "raw_files":([{"path":"samples.csv","sha256":sha(samples_path)}]
                                                if samples_path.is_file() else []),
        "notes":["Latency is VALID only when this gate is PASS.", "Historical throttle flags in vcgencmd invalidate the measured profiling window."]}
    dest=Path(args.out) if args.out else Path(args.analyze_only)
    dest.mkdir(parents=True, exist_ok=True)
    (dest/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    lines=["criterion\tstatus\tmeasured\tthreshold"]
    lines.extend(f"{x['id']}\t{x['status']}\t{x['measured']}\t{x['threshold']}" for x in criteria)
    lines.append(f"overall\t{status}")
    table="\n".join(lines)+"\n"
    (dest/"result.txt").write_text(table)
    print(table,end="")
    return {"PASS":0,"FAIL":1,"BLOCKED":2}[status]


def report(args):
    directory=Path(args.run)
    lat=json.loads((directory/"latency.json").read_text()) if (directory/"latency.json").exists() else {}
    validity_result=json.loads((directory/"result.json").read_text()) if (directory/"result.json").exists() else {}
    rows=load_samples(directory) if (directory/"samples.csv").exists() else []
    summary={"evidence":"MEASURED from saved profiling artifacts; no hardware gate is inferred",
        "validity_status":validity_result.get("status",UNKNOWN), "latency":lat,
        "latency_validity": ("VALID" if validity_result.get("status") == "PASS" else
                             "INVALID" if validity_result.get("status") in ("FAIL", "BLOCKED") else UNKNOWN),
        "sample_count":len(rows), "plots":[]}
    try:
        import matplotlib.pyplot as plt
        if rows:
            x=[(int(r["monotonic_ns"])-int(rows[0]["monotonic_ns"]))/1e9 for r in rows]
            temp=[]
            for r in rows:
                z=json.loads(r["thermal_c_json"]); soc=z.get("thermal_zone0", {})
                temp.append(soc.get("c") if isinstance(soc,dict) and isinstance(soc.get("c"),(int,float)) else float("nan"))
            fig,ax=plt.subplots(); ax.plot(x,temp); ax.set(xlabel="Elapsed CLOCK_MONOTONIC seconds",ylabel="SoC temperature (°C)")
            plot=directory/"temperature.png"; fig.savefig(plot); plt.close(fig); summary["plots"].append(plot.name)
    except ImportError: pass
    (directory/"summary.json").write_text(json.dumps(summary,indent=2,allow_nan=False)+"\n")
    lines=["# Runtime profiling report", "", f"Validity gate: **{summary['validity_status']}**",
           f"Latency validity: **{summary['latency_validity']}**", "",
           f"Samples: {len(rows)}", "", "Latency JSON: `latency.json`" if lat else "Latency: UNKNOWN",
           "", "Plots: `"+"`, `".join(summary["plots"])+"`" if summary["plots"] else "Plots: unavailable (no samples or matplotlib missing)."]
    (directory/"report.md").write_text("\n".join(lines)+"\n")


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    p=sub.add_parser("sampler"); p.add_argument("--pid",type=int,required=True); p.add_argument("--out",type=Path,required=True)
    p.add_argument("--period-s",type=float,default=config()["sample_period_s"]["value"])
    p.add_argument("--throttle-period-s",type=float,default=config()["throttle_period_s"]["value"])
    p.add_argument("--samples",type=int); p.add_argument("--threads",action="store_true")
    p.add_argument("--proc-root",default="/proc"); p.add_argument("--sys-root",default="/sys")
    p=sub.add_parser("latency"); p.add_argument("--events",type=Path); p.add_argument("--tracking",type=Path)
    p.add_argument("--clockmap",type=Path); p.add_argument("--out",type=Path,required=True)
    p=sub.add_parser("validity"); p.add_argument("--analyze-only",type=Path,required=True); p.add_argument("--out",type=Path)
    p=sub.add_parser("report"); p.add_argument("--run",type=Path,required=True)
    args=parser.parse_args(argv)
    try:
        if args.command=="sampler":
            if args.period_s<=0 or args.throttle_period_s<=0 or (args.samples is not None and args.samples<=0): raise ValueError("periods/sample count must be positive")
            print(sampler(args))
        elif args.command=="latency":
            if bool(args.events)==bool(args.tracking): raise ValueError("provide exactly one of --events or --tracking")
            args.out.mkdir(parents=True,exist_ok=True); latency(args)
        elif args.command=="validity": return validity(args)
        else: report(args)
        return 0
    except (OSError,ValueError,TypeError,KeyError,ImportError) as exc:
        print(f"profile: {exc}",file=sys.stderr); return 3

if __name__=="__main__": raise SystemExit(main())
