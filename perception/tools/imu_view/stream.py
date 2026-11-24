"""IMU sample sources: imu_can_bench text lines, CSV replay, and a fake sine tilt.

imu_can_bench (PR #155) prints on Nucleo USART1 (PA9/PA10, 115200 8N1) to the Pi's
/dev/ttyAMA0, 100 Hz:
    I,ms,fresh,ax,ay,az,temp,gx,gy,gz      raw int16, +-4 g (8192 LSB/g), +-500 dps (65.5 LSB/dps)
All other lines (BOOT, PIN, STAT, ...) are ignored.
Every source yields Sample(t_s, accel_g (x,y,z), gyro_rad_s (x,y,z)) in BOARD axes.
"""

import csv
import math
import random
import sys
import time
from dataclasses import dataclass

ACC_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 65.5
CSV_HEADER = ["t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps"]


@dataclass
class Sample:
    t: float
    accel: tuple
    gyro: tuple


def parse_line(line):
    """One bench line -> Sample, or None for non-IMU / partial lines."""
    line = line.strip()
    if not line.startswith("I,"):
        return None
    try:
        f = [int(x) for x in line.split(",")[1:]]
    except ValueError:
        return None
    if len(f) == 9:      # I,ms,fresh,ax,ay,az,temp,gx,gy,gz (imu_can_bench)
        ms, _fresh, ax, ay, az, _temp, gx, gy, gz = f
    elif len(f) == 8:    # same without temp
        ms, _fresh, ax, ay, az, gx, gy, gz = f
    else:
        return None
    k = math.radians(1.0) / GYRO_LSB_PER_DPS
    return Sample(ms / 1000.0, (ax / ACC_LSB_PER_G, ay / ACC_LSB_PER_G, az / ACC_LSB_PER_G),
                  (gx * k, gy * k, gz * k))


class LineStats:
    """Counts what a live stream contained; bad = undecodable or malformed I-lines."""

    def __init__(self):
        self.ok = self.bad = 0


def from_lines(lines, stats=None):
    """str or bytes lines -> Samples. Garbage (wrong baud/parity, partial lines) is skipped and counted."""
    stats = stats if stats is not None else LineStats()
    for line in lines:
        if isinstance(line, bytes):
            line = line.decode("ascii", errors="replace")
        s = parse_line(line)
        if s is not None:
            stats.ok += 1
            yield s
        elif "\ufffd" in line or line.lstrip().startswith("I,"):
            stats.bad += 1


def from_stdin(stats=None):
    """Live: pipe the Pi's UART in as bytes (8N1 set by stty, see README) ... | view --stdin."""
    return from_lines(sys.stdin.buffer, stats)


def from_csv(path):
    """Replay a CSV written by --save (or any file with CSV_HEADER columns)."""
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            k = math.radians(1.0)
            yield Sample(float(row["t_s"]),
                         (float(row["ax_g"]), float(row["ay_g"]), float(row["az_g"])),
                         (float(row["gx_dps"]) * k, float(row["gy_dps"]) * k, float(row["gz_dps"]) * k))


def fake(seconds=None, still_s=0.0, rate_hz=100.0, pitch_amp_deg=20.0, roll_amp_deg=10.0,
         gyro_bias_dps=(-3.25, 0.02, -0.18), noise_g=0.01, noise_dps=0.05, seed=0):
    """Board tilted by hand: still for still_s, then pitch sine 0.25 Hz and roll sine 0.1 Hz.

    Board axes = robot axes.
    The default gyro bias is the one MEASURED at rest on 29 Oct (imu_can_bench run-02),
    so gyro-only drift looks like the real board. Truth is attached as s.truth.
    """
    rng = random.Random(seed)
    dt = 1.0 / rate_hz
    n = 0
    while seconds is None or n * dt < seconds:
        t = n * dt
        tm = max(0.0, t - still_s)
        wp, wr = 2 * math.pi * 0.25, 2 * math.pi * 0.1
        p = math.radians(pitch_amp_deg) * math.sin(wp * tm)
        r = math.radians(roll_amp_deg) * math.sin(wr * tm)
        pd = math.radians(pitch_amp_deg) * wp * math.cos(wp * tm)
        rd = math.radians(roll_amp_deg) * wr * math.cos(wr * tm)
        # Body rates from ZYX Euler rates with yaw rate 0 (inverse of filter.euler_rates).
        gx = rd
        gy = math.cos(r) * pd
        gz = -math.sin(r) * pd
        acc = (-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))
        acc = tuple(a + rng.gauss(0, noise_g) for a in acc)
        gyr = tuple(w + math.radians(b + rng.gauss(0, noise_dps))
                    for w, b in zip((gx, gy, gz), gyro_bias_dps))
        s = Sample(t, acc, gyr)
        s.truth = (p, r)
        yield s
        n += 1


def paced(samples, speed=1.0):
    """Release samples at their own timestamps (for replay/fake in a live viewer)."""
    t0_wall = t0 = None
    for s in samples:
        if t0 is None:
            t0_wall, t0 = time.monotonic(), s.t
        wait = (s.t - t0) / speed - (time.monotonic() - t0_wall)
        if wait > 0:
            time.sleep(wait)
        yield s


def remap(sample, axes):
    """Board -> robot axes. axes like ("x", "-y", "-z"): robot x = board x, robot y = -board y, ...

    The same map applies to accel and gyro: both are vectors in the board frame.
    """
    def pick(v, spec):
        sign = -1.0 if spec.startswith("-") else 1.0
        return sign * v["xyz".index(spec[-1])]
    return Sample(sample.t, tuple(pick(sample.accel, a) for a in axes),
                  tuple(pick(sample.gyro, a) for a in axes))


class Writer:
    """--save: record samples (board axes, g and dps) so a bench run can be replayed."""

    def __init__(self, path):
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(CSV_HEADER)

    def write(self, s):
        d = math.degrees(1.0)
        self.w.writerow([f"{s.t:.3f}", *(f"{a:.5f}" for a in s.accel),
                         *(f"{g * d:.4f}" for g in s.gyro)])

    def close(self):
        self.fh.close()
