"""CSV -> bias-corrected complementary tilt, without graphics or hardware access."""

import argparse
from bisect import bisect_right
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import numpy as np

from tools.imu_view.filter import TiltFilters
from tools.imu_view.stream import from_csv

TAU_S = 0.5
IMU_STATUS = "MEASURED bench hand motion; filtered/scaled body pose is ASSUMED"


@dataclass(frozen=True)
class Tilt:
    pitch_rad: float
    roll_rad: float
    pitch_rate_deg_s: float
    roll_rate_deg_s: float


@dataclass(frozen=True)
class Window:
    start_s: float
    end_s: float
    duration_s: float
    peak_pitch_deg: float
    peak_roll_deg: float


def find_windows(times, pitch_rad, roll_rad, threshold_deg=3.0, min_duration_s=2.0,
                 max_gap_s=0.1):
    """List continuous sampled spans strictly above threshold for >min_duration.

    Either absolute Euler angle can qualify. No demeaning, short-gap bridging or
    extrapolation beyond the last qualifying sample. Times are elapsed CSV time.
    """
    times, pitch, roll = (np.asarray(v, dtype=float) for v in (times, pitch_rad, roll_rad))
    if (times.ndim != 1 or pitch.shape != times.shape or roll.shape != times.shape
            or not all(np.all(np.isfinite(v)) for v in (times, pitch, roll))
            or np.any(np.diff(times) <= 0)):
        raise ValueError("window inputs must be finite, equal-length vectors with increasing times")
    if (not all(math.isfinite(v) for v in (threshold_deg, min_duration_s, max_gap_s))
            or threshold_deg < 0 or min_duration_s < 0 or max_gap_s <= 0):
        raise ValueError("invalid window threshold, duration or maximum gap")
    active = np.maximum(np.abs(pitch), np.abs(roll)) > math.radians(threshold_deg)
    windows, start = [], None

    def finish(end):
        duration = float(times[end] - times[start])
        if duration > min_duration_s + 1e-9:
            windows.append(Window(float(times[start]), float(times[end]), duration,
                                  float(np.degrees(np.max(np.abs(pitch[start:end + 1])))),
                                  float(np.degrees(np.max(np.abs(roll[start:end + 1]))))))

    for i, qualifies in enumerate(active):
        if start is not None and (not qualifies or times[i] - times[i - 1] > max_gap_s + 1e-9):
            finish(i - 1)
            start = None
        if qualifies and start is None:
            start = i
    if start is not None:
        finish(len(times) - 1)
    return windows


class TiltSource:
    """Filter the entire file once, then interpolate angles at elapsed CSV time.

    Board x,y,z are treated as robot forward,left,up, matching imu_view defaults.
    Bias is a three-axis gyro mean over an explicitly assumed still interval.
    Scaling applies to angles and their slopes, never to the bias calibration.
    """

    def __init__(self, path, *, bias_window=(0.0, 2.0), scale=1.0):
        self.path = Path(path).expanduser().resolve()
        if not math.isfinite(scale) or scale < 0:
            raise ValueError("scale must be finite and nonnegative")
        self.scale = float(scale)
        samples = list(from_csv(self.path))
        if len(samples) < 2:
            raise ValueError("IMU CSV needs at least two samples")
        raw = np.asarray([(s.t, *s.accel, *s.gyro) for s in samples], dtype=float)
        if not np.all(np.isfinite(raw)) or np.any(np.diff(raw[:, 0]) <= 0):
            raise ValueError("IMU CSV needs finite samples and strictly increasing timestamps")
        self.timestamp_origin_s = float(raw[0, 0])
        self.times = raw[:, 0] - self.timestamp_origin_s
        self.duration_s = float(self.times[-1])
        start, end = bias_window
        if (not all(math.isfinite(v) for v in (start, end))
                or not 0 <= start < end <= self.duration_s + 1e-9):
            raise ValueError("bias window must satisfy 0 <= START < END <= CSV duration")
        self.bias_window = (float(start), float(end))
        selected = (self.times >= start - 1e-9) & (self.times <= end + 1e-9)
        if np.count_nonzero(selected) < 2:
            raise ValueError("bias window needs at least two IMU samples")
        self.bias = np.mean(raw[selected, 4:7], axis=0)
        self.bias_std_dps = np.degrees(np.std(raw[selected, 4:7], axis=0))
        self.bias_sample_count = int(np.count_nonzero(selected))
        filters = TiltFilters(tau_s=TAU_S)
        angles = []
        for i, sample in enumerate(samples):
            # Same dt clamp as imu_view.Pipeline. The first update seeds from
            # gravity; subsequent updates include the calibrated gyro bias.
            dt = 0.01 if i == 0 else min(max(self.times[i] - self.times[i - 1], 1e-4), 0.1)
            estimate = filters.update(sample.accel, raw[i, 4:7] - self.bias, dt)
            angles.append((estimate.pitch, estimate.roll))
        self.angles = np.asarray(angles) * scale

    def require_interval(self, start_s, duration_s):
        if (not all(math.isfinite(v) for v in (start_s, duration_s))
                or start_s < 0 or duration_s < 0
                or start_s + duration_s > self.duration_s + 1e-9):
            raise ValueError(f"requested IMU interval exceeds [0, {self.duration_s:g}] s")
        # A dropped span must not silently become interpolated 'measured' motion.
        gaps = np.diff(self.times)
        overlaps = (self.times[:-1] < start_s + duration_s) & (self.times[1:] > start_s)
        if np.any((gaps > 0.1 + 1e-9) & overlaps):
            raise ValueError("requested IMU interval crosses a sample gap >0.1 s")

    def at(self, t_s):
        """Interpolated pitch/roll and the slope of that same linear segment.

        Rates are Euler angle rates, in deg/s, suitable for JumpGate. Yaw is zero.
        Endpoints use the adjacent segment; out-of-range queries fail explicitly.
        """
        if not math.isfinite(t_s) or not 0 <= t_s <= self.duration_s + 1e-9:
            raise ValueError(f"IMU time must be in [0, {self.duration_s:g}] s")
        i = min(max(bisect_right(self.times, t_s) - 1, 0), len(self.times) - 2)
        dt = self.times[i + 1] - self.times[i]
        if dt > 0.1 + 1e-9:
            raise ValueError("cannot interpolate across an IMU sample gap >0.1 s")
        fraction = np.clip((t_s - self.times[i]) / dt, 0.0, 1.0)
        pitch, roll = self.angles[i] + fraction * (self.angles[i + 1] - self.angles[i])
        pd, rd = np.degrees((self.angles[i + 1] - self.angles[i]) / dt)
        return Tilt(float(pitch), float(roll), float(pd), float(rd))

    def windows(self, **kwargs):
        return find_windows(self.times, self.angles[:, 0], self.angles[:, 1], **kwargs)

    def metadata(self):
        return dict(imu_csv=str(self.path), sample_count=len(self.times),
                    timestamp_origin_s=self.timestamp_origin_s, duration_s=self.duration_s,
                    scale=self.scale, tau_s=TAU_S, bias_window_s=self.bias_window,
                    bias_dps=np.degrees(self.bias).tolist(),
                    bias_std_dps=self.bias_std_dps.tolist(), bias_samples=self.bias_sample_count,
                    max_sample_gap_s=float(np.max(np.diff(self.times))),
                    axes="board x,y,z -> robot forward,left,up; yaw fixed at zero",
                    imu_status=IMU_STATUS)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("imu_csv", type=Path)
    parser.add_argument("--bias-window", nargs=2, type=float, default=(0.0, 2.0),
                        metavar=("START", "END"), help="elapsed CSV seconds, assumed still")
    parser.add_argument("--scale", type=float, default=1.0)
    args = parser.parse_args(argv)
    try:
        source = TiltSource(args.imu_csv, bias_window=args.bias_window, scale=args.scale)
        print(json.dumps(dict(metadata=source.metadata(),
                              windows=[asdict(w) for w in source.windows()]), indent=2))
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
