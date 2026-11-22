"""Exact residual pitch errors about the camera; stdlib only, no detector changes."""

import math


def pitch_errors(range_m, height_m, camera_height_m, pitch_error_deg):
    """Return signed (height error, forward error), metres.

    Rotate (forward, up) by residual pitch about the camera, then add the
    assumed-correct camera height. Positive residual rotates forward toward up.
    Range means horizontal forward coordinate, not Euclidean or optical depth.
    """
    values = (range_m, height_m, camera_height_m, pitch_error_deg)
    if not all(math.isfinite(v) for v in values) or range_m < 0:
        raise ValueError("finite inputs and nonnegative forward range required")
    angle = math.radians(pitch_error_deg)
    s, c = math.sin(angle), math.cos(angle)
    up = height_m - camera_height_m
    return range_m * s + up * (c - 1), range_m * (c - 1) - up * s


def markdown_table():
    """Worst absolute errors over +/- the stated angle, H=.30 m and h=.12 m."""
    lines = ["| Pitch error (deg) | Forward (m) | Height error (mm) | Forward error (mm) |",
             "|---|---|---|---|"]
    for angle in (1, 3, 5, 10):
        for distance in (0.5, 1.0, 1.3):
            plus = pitch_errors(distance, .12, .30, angle)
            minus = pitch_errors(distance, .12, .30, -angle)
            height, forward = (1000 * max(abs(a), abs(b)) for a, b in zip(plus, minus))
            lines.append(f"| {angle} | {distance:.1f} | {height:.2f} | {forward:.2f} |")
    return "\n".join(lines)


if __name__ == "__main__":
    print(markdown_table())
