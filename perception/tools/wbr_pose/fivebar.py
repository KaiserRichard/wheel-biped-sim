"""Closed planar five-bar kinematics; coordinates are x forward, z up."""

from dataclasses import dataclass
import math

from sim.models.make_wbr_3d_simple import D_HIP, L_LOW, L_UP


@dataclass(frozen=True)
class LegPose:
    hip_left: tuple[float, float]
    hip_right: tuple[float, float]
    knee_left: tuple[float, float]
    knee_right: tuple[float, float]
    axle: tuple[float, float]
    phi1: float
    phi4: float


def _circle_knee(hip, axle, upper, lower, outward):
    dx, dz = axle[0] - hip[0], axle[1] - hip[1]
    dist = math.hypot(dx, dz)
    if dist < abs(upper - lower) - 1e-12 or dist > upper + lower + 1e-12:
        raise ValueError(f"five-bar unreachable: hip-axle distance {dist:.6g} m")
    if dist < 1e-12:
        raise ValueError("five-bar unreachable: coincident hip and axle")
    a = (upper * upper - lower * lower + dist * dist) / (2 * dist)
    h = math.sqrt(max(0.0, upper * upper - a * a))
    mx, mz = hip[0] + a * dx / dist, hip[1] + a * dz / dist
    candidates = ((mx + h * dz / dist, mz - h * dx / dist),
                  (mx - h * dz / dist, mz + h * dx / dist))
    return min(candidates) if outward < 0 else max(candidates)


def forward_kinematics(l0: float, theta0: float = 0.0) -> LegPose:
    """Return hips, outward knees, axle and hip-link angles for (l0, theta0).

    phi1/phi4 use the conventional signed link orientation from +x (CCW positive).
    The target team firmware convention is at
    ``User/Algorithm/VMC/VMC_calc.c`` (VMC_init / VMC_calc phi1, phi4).
    GitHub DNS was unavailable in this sandbox, so that file could not be read
    and equivalence must be verified before treating signs as firmware-matched.
    """
    if l0 <= 0 or not math.isfinite(l0) or not math.isfinite(theta0):
        raise ValueError("l0 must be positive and inputs finite")
    axle = (l0 * math.sin(theta0), -l0 * math.cos(theta0))
    hip_l, hip_r = (-D_HIP / 2, 0.0), (D_HIP / 2, 0.0)
    knee_l = _circle_knee(hip_l, axle, L_UP, L_LOW, -1)
    knee_r = _circle_knee(hip_r, axle, L_UP, L_LOW, +1)
    phi1 = math.atan2(knee_l[1] - hip_l[1], knee_l[0] - hip_l[0])
    phi4 = math.atan2(knee_r[1] - hip_r[1], knee_r[0] - hip_r[0])
    return LegPose(hip_l, hip_r, knee_l, knee_r, axle, phi1, phi4)


def inverse_kinematics(phi1: float, phi4: float) -> tuple[float, float]:
    """Recover (l0, theta0) from signed upper-link angles (radians).

    Selects the downward, outward-knee assembly branch used by ``forward``.
    """
    if not (math.isfinite(phi1) and math.isfinite(phi4)):
        raise ValueError("angles must be finite")
    h_l = (-D_HIP / 2 + L_UP * math.cos(phi1), L_UP * math.sin(phi1))
    h_r = ( D_HIP / 2 + L_UP * math.cos(phi4), L_UP * math.sin(phi4))
    dx, dz = h_r[0] - h_l[0], h_r[1] - h_l[1]
    dist = math.hypot(dx, dz)
    if dist > 2 * L_LOW + 1e-12 or dist < 1e-12:
        raise ValueError("five-bar unreachable for these hip angles")
    a = dist / 2
    height = math.sqrt(max(0.0, L_LOW * L_LOW - a * a))
    mid = ((h_l[0] + h_r[0]) / 2, (h_l[1] + h_r[1]) / 2)
    # Axle lies below the two knees on the selected physical assembly branch.
    options = ((mid[0] - height * dz / dist, mid[1] + height * dx / dist),
               (mid[0] + height * dz / dist, mid[1] - height * dx / dist))
    axle = min(options, key=lambda p: p[1])
    l0 = math.hypot(axle[0], axle[1])
    theta0 = math.atan2(axle[0], -axle[1])
    # Reject angle pairs on a different knee branch.
    try:
        expected = forward_kinematics(l0, theta0)
    except ValueError as exc:
        raise ValueError("five-bar unreachable for these hip angles") from exc
    if math.dist(expected.knee_left, h_l) > 1e-7 or math.dist(expected.knee_right, h_r) > 1e-7:
        raise ValueError("five-bar angles select a different assembly branch")
    return l0, theta0


forward = forward_kinematics
inverse = inverse_kinematics
