"""Plane-relative obstacle heights and fixed-mount baseline."""
import numpy as np


def height_image(depth, normal, camera_height, *, fx=386.953, fy=386.953,
                 cx=321.034, cy=241.040):
    """Signed height above the plane for each pixel; invalid depth -> NaN."""
    dep = np.asarray(depth, dtype=float)
    rows, cols = np.indices(dep.shape)
    n = np.asarray(normal, dtype=float)
    X = (cols-cx)*dep/fx
    Y = (rows-cy)*dep/fy
    result = n[0]*X+n[1]*Y+n[2]*dep+camera_height
    result[(~np.isfinite(dep)) | (dep <= 0)] = np.nan
    return result


def label_obstacles(depth, normal, camera_height, *, threshold=0.05, max_range=6.0,
                    fx=386.953, fy=386.953, cx=321.034, cy=241.040):
    h = height_image(depth, normal, camera_height, fx=fx, fy=fy, cx=cx, cy=cy)
    return np.isfinite(h) & (np.asarray(depth) <= max_range) & (h > threshold)


def fixed_mount(depth, *, nominal_height=0.40, nominal_pitch=0.0,
                threshold=0.05, max_range=6.0, fx=386.953, fy=386.953,
                cx=321.034, cy=241.040):
    """Labels using nominal pitch-down angle and height, with zero roll."""
    p = np.deg2rad(nominal_pitch)
    n = np.array([0.0, -np.cos(p), -np.sin(p)])
    return label_obstacles(depth, n, nominal_height, threshold=threshold,
                           max_range=max_range, fx=fx, fy=fy, cx=cx, cy=cy)


def angular_height_error(distance_m, pitch_error_rad):
    """Vertical residual from a tilt error over horizontal distance d*tan(delta)."""
    return np.asarray(distance_m) * np.tan(pitch_error_rad)
