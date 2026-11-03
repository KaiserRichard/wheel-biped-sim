"""Seeded sensor boundary. All noise specifications below are ASSUMED, not BMI088 claims.

Units SI. Independent Gaussian samples and Brownian bias drift, no calibration.
MuJoCo's accelerometer returns specific force at the existing torso IMU site.
Only motor joint coordinates/velocities cross the encoder boundary.
"""
from dataclasses import dataclass, asdict
import numpy as np


@dataclass(frozen=True)
class SensorConfig:
    # ASSUMED engineering test conditions; hardware calibration is UNKNOWN.
    dt: float = .001
    gyro_sigma: float = np.deg2rad(.03)
    gyro_bias_sigma: float = np.deg2rad(.10)
    gyro_drift: float = np.deg2rad(.003)  # rad/s / sqrt(s)
    accel_sigma: float = .035
    accel_bias_sigma: float = .020
    accel_drift: float = .002  # m/s² / sqrt(s)
    encoder_counts: int = 65536
    encoder_velocity_sigma: float = .020


@dataclass(frozen=True, slots=True)
class Packet:
    gyro: np.ndarray
    accel: np.ndarray
    angles: np.ndarray  # L front, rear, wheel; R front, rear, wheel
    rates: np.ndarray


class RawSensors:
    def __init__(self, sim, seed=0, config=None):
        self.config = config or SensorConfig()
        self.rng = np.random.default_rng(seed)
        self.gbias = self.rng.normal(0, self.config.gyro_bias_sigma, 3)
        self.abias = self.rng.normal(0, self.config.accel_bias_sigma, 3)
        self.gadr = sim.m.sensor("torso_gyro").adr[0]
        self.aadr = sim.m.sensor("torso_acc").adr[0]
        self.qa = [sim.ja[s][k] for s in "LR" for k in ("hip_front", "hip_rear", "wheel")]
        self.va = [sim.jv[s][k] for s in "LR" for k in ("hip_front", "hip_rear", "wheel")]
        self.samples = 0

    def sample(self, data):
        c, r = self.config, self.rng
        self.gbias += r.normal(0, c.gyro_drift * np.sqrt(c.dt), 3)
        self.abias += r.normal(0, c.accel_drift * np.sqrt(c.dt), 3)
        quantum = 2 * np.pi / c.encoder_counts
        self.samples += 1
        return Packet(
            data.sensordata[self.gadr:self.gadr + 3].copy() + self.gbias + r.normal(0, c.gyro_sigma, 3),
            data.sensordata[self.aadr:self.aadr + 3].copy() + self.abias + r.normal(0, c.accel_sigma, 3),
            np.round(data.qpos[self.qa] / quantum) * quantum,
            data.qvel[self.va].copy() + r.normal(0, c.encoder_velocity_sigma, 6),
        )

    def report(self):
        return {"status":"ASSUMED", "reason":"Synthetic noise, drift, quantisation; calibration uses a static fixture."}

from dataclasses import replace
from scipy.spatial.transform import Rotation
class Sensors(RawSensors):
    variant = 'nominal'

    def __init__(self, sim, seed):
        c = SensorConfig()
        if self.variant != 'assumed_reference':
            c = replace(c, gyro_sigma=np.deg2rad(0.014) * np.sqrt(500), accel_sigma=np.array([160, 160, 190]) * 1e-06 * 9.80665 * np.sqrt(500))
        super().__init__(sim, seed, c)
        angle = {'mount_1deg': 1, 'mount_3deg': 3}.get(self.variant, 0)
        self.mount = Rotation.from_euler('y', angle, degrees=True).as_matrix()
        if 'gyro_startup' in self.variant:
            self.gbias += np.deg2rad([0.0, 1.0, 0.0])
        self.correction = np.zeros(3)
        self.clipped_accel = self.clipped_gyro = 0
        self.raw_accel_peak = self.raw_gyro_peak = 0.0
        self.calibration_samples = 0
        self.calibration_residual_dps = None
        if True:
            samples = [self._packet(sim.d) for _ in range(1000)]
            self.correction = np.mean([s.gyro for s in samples], axis=0)
            self.calibration_samples = len(samples)
            validation = [self._packet(sim.d).gyro - self.correction for _ in range(100)]
            self.calibration_residual_dps = np.degrees(np.mean(validation, axis=0)).tolist()

    def _packet(self, data):
        p = super().sample(data)
        return Packet(self.mount.T @ p.gyro, self.mount.T @ p.accel, p.angles, p.rates)

    def sample(self, data):
        p = self._packet(data)
        (g, a) = (p.gyro - self.correction, p.accel)
        self.raw_accel_peak = max(self.raw_accel_peak, float(np.max(np.abs(a))))
        self.raw_gyro_peak = max(self.raw_gyro_peak, float(np.max(np.abs(g))))
        if self.variant.startswith('saturate_'):
            low = self.variant == 'saturate_3g_125dps'
            (ar, gr) = ((3 if low else 24) * 9.80665, np.deg2rad(125 if low else 2000))
            self.clipped_accel += int(np.any(np.abs(a) > ar))
            self.clipped_gyro += int(np.any(np.abs(g) > gr))
            (a, g) = (np.clip(a, -ar, ar), np.clip(g, -gr, gr))
        return Packet(g, a, p.angles, p.rates)


