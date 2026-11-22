"""Small robot-centred 2.5D elevation grid implemented with NumPy."""
from dataclasses import dataclass
import numpy as np


@dataclass
class StereoNoise:
    """Axial depth noise: sigma_Z = Z^2 sigma_d / (f B)."""
    sigma_d_px: float = 0.0
    fx_px: float = 386.953
    baseline_m: float = 0.05

    def variance(self, z_m):
        return (np.asarray(z_m, dtype=float) ** 2 * self.sigma_d_px /
                (self.fx_px * self.baseline_m)) ** 2


class ElevationGrid:
    """Square map centred on the robot; points and robot positions are x/y/z-up."""
    def __init__(self, size_m=4.0, resolution_m=0.05, *, center_xy=(0., 0.),
                 decay_s=2.0, unknown_after_s=1.0):
        if size_m <= 0 or resolution_m <= 0 or size_m / resolution_m < 1:
            raise ValueError("size and resolution must be positive")
        self.resolution_m = float(resolution_m)
        self.n = int(round(size_m / resolution_m))
        self.size_m = self.n * self.resolution_m
        self.center_xy = np.asarray(center_xy, dtype=float)
        self.decay_s, self.unknown_after_s = float(decay_s), float(unknown_after_s)
        self.mean = np.full((self.n, self.n), np.nan)
        self.variance = np.full((self.n, self.n), np.inf)
        self.max_height = np.full((self.n, self.n), np.nan)
        self.count = np.zeros((self.n, self.n), dtype=np.uint32)
        self.last_seen = np.full((self.n, self.n), -np.inf)

    def _index(self, xy):
        # Column increases right; row increases toward the robot-facing edge.
        col = np.floor((xy[:, 0] - (self.center_xy[0] - self.size_m/2)) / self.resolution_m).astype(int)
        row = np.floor(((self.center_xy[1] + self.size_m/2) - xy[:, 1]) / self.resolution_m).astype(int)
        return row, col

    def scroll(self, center_xy):
        """Move map centre while retaining cells at the same world coordinates."""
        new = np.asarray(center_xy, dtype=float)
        delta = np.rint((new - self.center_xy) / self.resolution_m).astype(int)
        if np.any(delta):
            # A positive forward shift moves preserved cells toward larger rows.
            dr, dc = int(delta[1]), int(-delta[0])
            for name, fill in (("mean", np.nan), ("variance", np.inf),
                               ("max_height", np.nan), ("count", 0), ("last_seen", -np.inf)):
                old = getattr(self, name); shifted = np.full_like(old, fill)
                src_r0, src_r1 = max(0, -dr), min(self.n, self.n - dr)
                src_c0, src_c1 = max(0, -dc), min(self.n, self.n - dc)
                dst_r0, dst_r1 = max(0, dr), min(self.n, self.n + dr)
                dst_c0, dst_c1 = max(0, dc), min(self.n, self.n + dc)
                if src_r1 > src_r0 and src_c1 > src_c0:
                    shifted[dst_r0:dst_r1, dst_c0:dst_c1] = old[src_r0:src_r1, src_c0:src_c1]
                setattr(self, name, shifted)
            self.center_xy += delta * self.resolution_m

    def update(self, points_world, sensor_noise_model=None, *, time_s=0.0, floor_z=0.0,
               sensor_range_m=None):
        pts = np.asarray(points_world, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("points_world must have shape (N,3)")
        valid = np.all(np.isfinite(pts), axis=1)
        ranges = None if sensor_range_m is None else np.asarray(sensor_range_m, dtype=float)[valid]
        pts = pts[valid]
        if len(pts) == 0:
            self._decay(float(time_s)); return
        rows, cols = self._index(pts[:, :2])
        inside = (rows >= 0) & (rows < self.n) & (cols >= 0) & (cols < self.n)
        rows, cols, pts = rows[inside], cols[inside], pts[inside]
        if ranges is not None:
            ranges = ranges[inside]
        if not len(pts):
            self._decay(float(time_s)); return
        h = pts[:, 2] - floor_z
        if sensor_noise_model is None:
            meas_var = np.full(len(pts), 1e-6)
        elif hasattr(sensor_noise_model, "variance"):
            noise_ranges = pts[:, 2] if ranges is None else ranges
            meas_var = np.asarray(sensor_noise_model.variance(np.maximum(noise_ranges, 0)), dtype=float)
        else:
            meas_var = np.broadcast_to(np.asarray(sensor_noise_model, dtype=float), (len(pts),))
        meas_var = np.maximum(meas_var, 1e-12)
        # Aggregate same-frame returns first, then perform one Kalman update per cell.
        flat = rows * self.n + cols
        for cell in np.unique(flat):
            sel = flat == cell; r, c = divmod(int(cell), self.n)
            vals, vars_ = h[sel], meas_var[sel]
            weights = 1.0 / vars_
            z = float(np.sum(vals * weights) / np.sum(weights))
            rv = float(1.0 / np.sum(weights))
            if self.count[r, c] == 0:
                self.mean[r, c], self.variance[r, c] = z, rv
                self.max_height[r, c] = float(np.max(vals))
            else:
                prior_v = max(float(self.variance[r, c]), 1e-12)
                gain = prior_v / (prior_v + rv)
                self.mean[r, c] += gain * (z - self.mean[r, c])
                self.variance[r, c] = (1 - gain) * prior_v
                self.max_height[r, c] = max(self.max_height[r, c], float(np.max(vals)))
            self.count[r, c] += int(sel.sum())
            self.last_seen[r, c] = float(time_s)
        self._decay(float(time_s))

    def _decay(self, time_s):
        age = time_s - self.last_seen
        known = np.isfinite(self.mean)
        old = known & (age > self.unknown_after_s)
        if self.decay_s > 0 and np.any(old):
            self.variance[old] += (age[old] - self.unknown_after_s) / self.decay_s * 1e-4

    def outputs(self, *, time_s=None, obstacle_threshold_m=0.05, unknown_after_s=None):
        timeout = self.unknown_after_s if unknown_after_s is None else unknown_after_s
        age = (np.inf if time_s is None else float(time_s)) - self.last_seen
        known = np.isfinite(self.mean) & (age <= timeout)
        obstacle = known & (self.max_height > obstacle_threshold_m)
        # 0 unknown, 1 free, 2 obstacle
        classes = np.zeros(self.mean.shape, dtype=np.uint8)
        classes[known & ~obstacle] = 1
        classes[obstacle] = 2
        height = self.mean.copy(); height[~known] = np.nan
        var = self.variance.copy(); var[~known] = np.inf
        return {"height": height, "variance": var, "max_height": self.max_height.copy(),
                "obstacle": obstacle, "classes": classes, "known": known,
                "count": self.count.copy(), "last_seen": self.last_seen.copy()}
