#!/usr/bin/env python3
"""Read the imu_can_bench firmware output and print PASS/FAIL per check.

Run on the Pi with the micro-ROS agent stopped (it owns /dev/ttyAMA0):
    python3 bench_check.py --seconds 20 --out ~/wbr-evidence/imu-can-bench-20261029/run-01
For the CANH-unplug test, pull CANH during the run and pass --expect-can-fail.
Parsing and verdicts are pure functions so tests run without hardware.
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import sys
import time

G = 9.80665
ACC_LSB_PER_G = 8192.0   # +-4 g
GYRO_LSB_PER_DPS = 65.5  # +-500 dps

# Thresholds. Gyro bias: imu_sensor datasheet initial ZRO tolerance +-20 dps.
# Gyro noise: datasheet 0.005 dps/sqrt(Hz) rms is 0.03 dps at 44 Hz; 1 dps only flags motion.
ACC_TOL = 0.5            # m/s^2 on |a|
GYRO_BIAS_MAX = 20.0     # dps per axis
GYRO_STD_MAX = 1.0       # dps per axis
RATE_HZ, RATE_TOL = 100.0, 3.0
PING_HZ = 100.0
RT_MIN_FRACTION = 0.99   # round trips matched / pings due


def parse_kv(line: str) -> tuple[str, dict[str, int]]:
    tag, *pairs = line.split()
    out = {}
    for p in pairs:
        k, _, v = p.partition("=")
        out[k] = int(v, 0)
    return tag, out


def parse(lines):
    """Return latest boot records, IMU samples and STAT records from raw lines."""
    boot, samples, stats = {}, [], []
    for raw in lines:
        line = raw.strip()
        if not line:
            continue
        try:
            if line.startswith("I,"):
                f = [int(x) for x in line.split(",")[1:]]
                if len(f) == 9:
                    samples.append(f)
            elif line.split()[0] in ("BOOT", "PIN", "IMUID", "LOOP", "NORM"):
                tag, kv = parse_kv(line)
                boot[tag] = kv
            elif line.startswith("STAT "):
                stats.append(parse_kv(line)[1])
        except ValueError:
            continue  # partial line at start of capture
    return boot, samples, stats


def esr(v: int) -> dict[str, int]:
    return {"tec": (v >> 16) & 0xFF, "rec": (v >> 24) & 0xFF, "lec": (v >> 4) & 7,
            "boff": (v >> 2) & 1, "epv": (v >> 1) & 1}


def check(boot, samples, stats, expect_can_fail=False):
    """Return a list of (name, PASS/FAIL, detail) and a dict of measured values."""
    res, m = [], {}

    def add(name, ok, detail):
        res.append((name, "PASS" if ok else "FAIL", detail))

    pin = boot.get("PIN")
    if pin is None:
        add("pins", False, "no PIN line (firmware not running?)")
    else:
        want = {"c1rx_idle": 1, "c2rx_idle": 1, "c1tx_c1rx": 0, "c2tx_c2rx": 0,
                "c1tx_c2rx": 0, "c2tx_c1rx": 0, "scl": 1, "sda": 1}
        bad = {k: pin.get(k) for k, v in want.items() if pin.get(k) != v}
        add("pins", not bad, "all as expected" if not bad else f"unexpected {bad} (want {want})")

    imu = boot.get("IMUID", {})
    m["imu_addr"], m["who_am_i"] = imu.get("addr"), imu.get("who")
    add("imu_who_am_i", imu.get("who") == 0x68,
        f"addr={imu.get('addr', 0):#04x} who={imu.get('who', 0):#04x} (want 0x68)")

    if len(samples) >= 2:
        t = [s[0] for s in samples]
        span = (t[-1] - t[0]) / 1000.0
        rate = (len(samples) - 1) / span if span > 0 else 0.0
        acc = [[s[2 + i] / ACC_LSB_PER_G * G for i in range(3)] for s in samples]
        mag = [math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2) for a in acc]
        gyro = [[s[6 + i] / GYRO_LSB_PER_DPS for s in samples] for i in range(3)]
        bias = [statistics.fmean(g) for g in gyro]
        std = [statistics.pstdev(g) for g in gyro]
        m.update(imu_samples=len(samples), imu_rate_hz=round(rate, 2),
                 accel_mean=[round(statistics.fmean(a[i] for a in acc), 3) for i in range(3)],
                 accel_mag=round(statistics.fmean(mag), 3), gyro_bias_dps=[round(b, 3) for b in bias],
                 gyro_std_dps=[round(s, 3) for s in std],
                 temp_c=round(statistics.fmean(s[5] for s in samples) / 340.0 + 36.53, 1))
        add("imu_rate", abs(rate - RATE_HZ) <= RATE_TOL, f"{rate:.2f} Hz (want {RATE_HZ}+-{RATE_TOL})")
        add("imu_accel_1g", abs(m["accel_mag"] - G) <= ACC_TOL,
            f"|a|={m['accel_mag']} m/s^2 mean={m['accel_mean']} (want {G:.2f}+-{ACC_TOL}, board flat)")
        add("imu_gyro_rest", all(abs(b) <= GYRO_BIAS_MAX for b in bias) and all(s <= GYRO_STD_MAX for s in std),
            f"bias={m['gyro_bias_dps']} dps std={m['gyro_std_dps']} (want |bias|<={GYRO_BIAS_MAX}, std<={GYRO_STD_MAX})")
    else:
        add("imu_samples", False, f"{len(samples)} samples")

    loop = boot.get("LOOP", {})
    add("can_loopback", loop.get("c1_tx") == 10 and loop.get("c1_rx") == 10 and loop.get("c2_tx") == 10
        and loop.get("c2_rx") == 10, f"{loop} (internal, no wiring involved; want 10/10 each)")
    norm = boot.get("NORM", {})
    add("can_init_normal", norm.get("c1_init") == 1 and norm.get("c2_init") == 1,
        f"{norm} (0 = RX never saw 11 recessive bits)")

    if len(stats) >= 2:
        a, b = stats[0], stats[-1]
        d = {k: b[k] - a[k] for k in b if k in a and not k.endswith("_esr")}
        secs = d["ms"] / 1000.0
        e1, e2 = esr(b["c1_esr"]), esr(b["c2_esr"])
        m.update(window_s=secs, imu_err=d["imu_err"], imu_reinit=d["imu_reinit"],
                 pings_sent=d["sent"], round_trips=d["match"], bad=d["bad"],
                 c1_busy=d["c1_busy"], c1_txok=d["c1_txok"], c2_txok=d["c2_txok"],
                 c1_boff_events=d["c1_boff_n"], c2_boff_events=d["c2_boff_n"],
                 c1_esr=e1, c2_esr=e2, uart_drop=d["uart_drop"])
        add("imu_no_errors", d["imu_err"] == 0 and d["imu_reinit"] == 0,
            f"read errors {d['imu_err']}, re-inits {d['imu_reinit']} in {secs:.0f} s")
        due = PING_HZ * secs
        can_ok = (d["match"] >= RT_MIN_FRACTION * due and d["bad"] == 0 and d["c1_boff_n"] == 0
                  and d["c2_boff_n"] == 0 and e1["tec"] == 0 and e2["tec"] == 0)
        detail = (f"{d['match']}/{due:.0f} round trips, bad={d['bad']}, CAN1 busy={d['c1_busy']}, "
                  f"bus-off events {d['c1_boff_n']}/{d['c2_boff_n']}, CAN1 TEC={e1['tec']} REC={e1['rec']} "
                  f"LEC={e1['lec']}, CAN2 TEC={e2['tec']} REC={e2['rec']} LEC={e2['lec']}")
        if expect_can_fail:
            add("can_fault_detected", not can_ok, detail + " (CANH pulled: want failure)")
        else:
            add("can_round_trip", can_ok, detail)
        add("uart_no_drop", d["uart_drop"] == 0, f"{d['uart_drop']} bytes dropped")
    else:
        add("stats", False, f"{len(stats)} STAT lines (need >=2)")
    return res, m


def capture(port, seconds, raw_path):
    import serial  # pyserial is on the Pi
    lines = []
    with serial.Serial(port, 115200, timeout=0.5) as s, open(raw_path, "w") as raw:
        s.reset_input_buffer()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = s.readline().decode("ascii", "replace")
            if line:
                raw.write(line)
                lines.append(line)
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", required=True, type=pathlib.Path)
    ap.add_argument("--from-log", type=pathlib.Path, help="re-check a saved raw.log instead of the port")
    ap.add_argument("--expect-can-fail", action="store_true")
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    lines = a.from_log.read_text().splitlines() if a.from_log else capture(a.port, a.seconds, a.out / "raw.log")
    res, m = check(*parse(lines), expect_can_fail=a.expect_can_fail)
    for name, verdict, detail in res:
        print(f"{verdict} {name}: {detail}")
    ok = all(v == "PASS" for _, v, _ in res)
    print(f"SUMMARY {'PASS' if ok else 'FAIL'}: {sum(v == 'PASS' for _, v, _ in res)}/{len(res)}")
    (a.out / "summary.json").write_text(json.dumps({"checks": res, "measured": m}, indent=2) + "\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
