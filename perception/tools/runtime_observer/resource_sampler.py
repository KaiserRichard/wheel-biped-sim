"""Read-only 1 Hz Linux/Pi sampler, with UNAVAILABLE fields on other hosts."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time

UNAVAILABLE = "UNAVAILABLE"
FIELDS = ("host_ns", "host_domain", "host_session", "pid", "sample_duration_ns",
          "cpu_busy_pct_board", "cpu_count", "process_cpu_pct_one_core",
          "process_start_ticks", "rss_bytes", "threads", "thermal_c_json",
          "cpufreq_khz_json", "throttled_raw", "undervoltage_now", "frequency_capped_now",
          "throttled_now", "soft_temperature_limit_now", "undervoltage_occurred",
          "frequency_capped_occurred", "throttled_occurred", "soft_temperature_limit_occurred")
THROTTLE_FIELDS = FIELDS[14:]


def host_now_ns():
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def read_text(path):
    try:
        return path.read_text()
    except (OSError, UnicodeError):
        return None


def nonnegative(text):
    if text is None or not re.fullmatch(r"[0-9]+", text.strip()):
        return None
    return int(text.strip())


def cpu_counters(text):
    if text is None:
        return None
    lines = text.splitlines()
    aggregate = next((line.split()[1:] for line in lines if line.startswith("cpu ")), [])
    if len(aggregate) < 8:
        return None
    fields = [nonnegative(value) for value in aggregate[:8]]
    if any(value is None for value in fields):
        return None
    count = sum(bool(re.match(r"cpu[0-9]+\s", line)) for line in lines)
    # Guest/guest_nice are already included in user/nice; ignore the last two.
    return tuple(fields), count


def process_counters(text):
    if text is None:
        return None
    # comm may contain spaces and ')'; fields following its FINAL ')' are numeric.
    end = text.rfind(")")
    if end < 0:
        return None
    fields = text[end + 1:].split()  # starts at field 3, state
    if len(fields) < 20:
        return None
    values = [nonnegative(fields[index]) for index in (11, 12, 19)]
    if any(value is None for value in values):
        return None
    return values[0] + values[1], values[2]  # utime+stime, starttime


def status_fields(text):
    result = {"rss_bytes": UNAVAILABLE, "threads": UNAVAILABLE}
    if text is None:
        return result
    rss = re.search(r"^VmRSS\x3a\s+([0-9]+)\s+kB\s*$", text, re.MULTILINE)
    threads = re.search(r"^Threads\x3a\s+([0-9]+)\s*$", text, re.MULTILINE)
    if rss:
        result["rss_bytes"] = int(rss[1]) * 1024
    if threads:
        result["threads"] = int(threads[1])
    return result


def get_throttled():
    executable = shutil.which("vcgencmd")
    if executable is None:
        return None
    try:
        return subprocess.run([executable, "get_throttled"], capture_output=True,
                              text=True, timeout=.2, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError, UnicodeError):
        return None


def throttling(text):
    result = {field: UNAVAILABLE for field in ("throttled_raw", *THROTTLE_FIELDS)}
    match = re.fullmatch(r"throttled=(0x[0-9a-fA-F]{1,8})", text or "")
    if match:
        value = int(match[1], 16)
        result["throttled_raw"] = hex(value)
        for field, bit in zip(THROTTLE_FIELDS, (0, 1, 2, 3, 16, 17, 18, 19)):
            result[field] = int(bool(value & (1 << bit)))
    return result


class Sampler:
    def __init__(self, pid, host_session, *, proc_root=Path("/proc"), sys_root=Path("/sys"),
                 system=None, clock=host_now_ns, throttle_reader=get_throttled, clock_ticks=None):
        if type(pid) is not int or pid <= 0:
            raise ValueError("pid must be positive")
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,47}", host_session):
            raise ValueError("session must be a 1..47 character token")
        self.pid, self.session = pid, host_session
        self.proc, self.sys = Path(proc_root), Path(sys_root)
        self.linux = (system or platform.system()) == "Linux"
        self.clock, self.throttle_reader = clock, throttle_reader
        self.ticks = clock_ticks if clock_ticks is not None else os.sysconf("SC_CLK_TCK")
        if self.ticks <= 0:
            raise ValueError("clock tick rate must be positive")
        self.previous_cpu = self.previous_process = None

    def sample(self):
        start = self.clock()
        row = {field: UNAVAILABLE for field in FIELDS}
        row.update(host_ns=start, host_domain="host_steady", host_session=self.session, pid=self.pid)
        if self.linux:
            cpu = cpu_counters(read_text(self.proc / "stat"))
            if cpu is not None:
                fields, count = cpu
                row["cpu_count"] = count or UNAVAILABLE
                if self.previous_cpu is not None:
                    old_fields, old_count = self.previous_cpu
                    delta = [value - old for value, old in zip(fields, old_fields)]
                    if count == old_count and all(value >= 0 for value in delta) and sum(delta) > 0:
                        row["cpu_busy_pct_board"] = 100 * (sum(delta) - delta[3] - delta[4]) / sum(delta)
            self.previous_cpu = cpu
            pid_dir = self.proc / str(self.pid)
            process = process_counters(read_text(pid_dir / "stat"))
            row.update(status_fields(read_text(pid_dir / "status")))
            # Check starttime again to avoid attributing a reused PID's RSS to the
            # previous process. This is a best-effort snapshot, not atomic procfs.
            after = process_counters(read_text(pid_dir / "stat"))
            if process is None or after is None or process[1] != after[1]:
                row.update(rss_bytes=UNAVAILABLE, threads=UNAVAILABLE)
                self.previous_process = None
            else:
                ticks, identity = process
                row["process_start_ticks"] = identity
                if self.previous_process is not None:
                    old_ticks, old_identity, old_ns = self.previous_process
                    if identity == old_identity and ticks >= old_ticks and start > old_ns:
                        row["process_cpu_pct_one_core"] = 100 * (ticks - old_ticks) / self.ticks / ((start - old_ns) / 1e9)
                self.previous_process = (ticks, identity, start)
            temperatures = {}
            for zone in sorted((self.sys / "class/thermal").glob("thermal_zone*")):
                text = read_text(zone / "temp")
                if text is not None and re.fullmatch(r"-?[0-9]+", text.strip()):
                    value = int(text.strip()) / 1000
                else:
                    value = UNAVAILABLE
                zone_type = read_text(zone / "type")
                temperatures[zone.name] = {"type": zone_type.strip() if zone_type else UNAVAILABLE, "c": value}
            frequencies = {}
            for policy in sorted((self.sys / "devices/system/cpu/cpufreq").glob("policy*")):
                freq = nonnegative(read_text(policy / "scaling_cur_freq"))
                frequencies[policy.name] = freq if freq is not None else UNAVAILABLE
            row["thermal_c_json"] = json.dumps(temperatures, sort_keys=True) if temperatures else UNAVAILABLE
            row["cpufreq_khz_json"] = json.dumps(frequencies, sort_keys=True) if frequencies else UNAVAILABLE
            row.update(throttling(self.throttle_reader()))
        row["sample_duration_ns"] = max(0, self.clock() - start)
        return row


def write_samples(sampler, stream, count=None, *, clock=host_now_ns, sleep=time.sleep):
    """Sample immediately, then at 1 Hz. Skip missed slots instead of catch-up bursts."""
    writer = csv.DictWriter(stream, fieldnames=FIELDS)
    writer.writeheader()
    deadline = clock()
    written = 0
    while count is None or written < count:
        writer.writerow(sampler.sample())
        stream.flush()
        written += 1
        if count is not None and written == count:
            break
        deadline += 1_000_000_000
        now = clock()
        if now > deadline:
            deadline += ((now - deadline) // 1_000_000_000 + 1) * 1_000_000_000
        sleep(max(0, deadline - now) / 1e9)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="new CSV path (existing files are refused)")
    parser.add_argument("--pid", required=True, type=int)
    parser.add_argument("--session", required=True)
    parser.add_argument("--samples", type=int, help="otherwise continue until Ctrl-C")
    args = parser.parse_args(argv)
    try:
        if args.samples is not None and args.samples <= 0:
            raise ValueError("samples must be positive")
        sampler = Sampler(args.pid, args.session)
        with args.output.open("x", newline="") as stream:
            try:
                write_samples(sampler, stream, args.samples)
            except KeyboardInterrupt:
                pass
        return 0
    except (OSError, ValueError) as exc:
        print(f"resource sampler: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
