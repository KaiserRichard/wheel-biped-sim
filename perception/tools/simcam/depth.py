"""NumPy-only metric depth sanitation and exact pinhole projection helpers."""

import math
import numpy as np


def clipping_distances(znear, zfar, extent):
    """MJCF visual.map clipping values are scaled by model.stat.extent."""
    near, far = znear * extent, zfar * extent
    if not (math.isfinite(near) and math.isfinite(far) and 0 < near < far):
        raise ValueError("clipping distances must satisfy 0 < near < far")
    return near, far


def optical_depth(depth, near, far):
    """Sanitize Renderer.enable_depth_rendering() output; invalid pixels are zero.

    MuJoCo documents depth as distance from the camera plane, versus distance
    from the camera origin: https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera
    Its Renderer undoes the perspective projection to METRIC camera -Z depth:
    https://github.com/google-deepmind/mujoco/blob/3.3.7/python/mujoco/renderer.py
    (Renderer.render, near/far projection inversion). Thus this is already the
    detector's optical +Z, NOT Euclidean ray length and NOT a [0,1] GL buffer.
    Renderer also handles the image flip; do not flip or linearize it again.
    """
    near, far = clipping_distances(near, far, 1.0)
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth must be a 2D metric image")
    # Background rounds to the far plane; exclude its float32 rounding band.
    margin = 8 * np.finfo(np.float32).eps * far
    valid = np.isfinite(depth) & (depth >= near) & (depth < far - margin)
    return np.where(valid, depth, 0.0).astype(np.float32)


def pinhole_frustum(K, width, height, near):
    """Off-axis frustum matching integer-indexed detector pixel centres exactly.

    mjvGLCamera fields: https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjvglcamera
    OpenGL pixel centres are half a pixel inside the viewport, hence the 0.5.
    MuJoCo infers horizontal width from vertical span and viewport aspect ratio,
    which requires square pixels (FX == FY, as in synth.py).
    """
    K = np.asarray(K, dtype=float)
    if (K.shape != (3, 3) or not np.all(np.isfinite(K))
            or K[0, 0] <= 0 or K[1, 1] <= 0
            or not np.isclose(K[0, 0], K[1, 1])
            or not np.allclose(K[[0, 1], [1, 0]], 0)
            or not np.allclose(K[2], (0, 0, 1))):
        raise ValueError("K must be a finite, square-pixel pinhole intrinsic matrix")
    if not (width > 0 and height > 0 and math.isfinite(near) and near > 0):
        raise ValueError("width, height, and near must be positive")
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    left, right = (-0.5 - cx) * near / fx, (width - 0.5 - cx) * near / fx
    bottom, top = -(height - 0.5 - cy) * near / fy, (cy + 0.5) * near / fy
    return dict(frustum_center=(left + right) / 2,
                frustum_bottom=bottom, frustum_top=top)


def stereo_noise(depth, *, sigma_d, fx, baseline=0.050, rng=None, near=0.01, far=6.0):
    """ASSUMED independent Gaussian sigma_Z=Z^2*sigma_d/(f*B), like synth.py."""
    if not (math.isfinite(sigma_d) and sigma_d >= 0 and math.isfinite(fx) and fx > 0
            and math.isfinite(baseline) and baseline > 0):
        raise ValueError("sigma_d must be nonnegative; fx and baseline must be positive")
    depth = optical_depth(depth, near, far)
    rng = np.random.default_rng() if rng is None else rng
    sigma = depth.astype(float)**2 * sigma_d / (fx * baseline)
    return optical_depth(depth + rng.normal(0, sigma), near, far)
