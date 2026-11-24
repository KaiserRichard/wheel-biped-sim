"""CALCULATED pinhole geometry in a vertical plane; distances in metres."""
import math


def vertical_fov(fy, pixels=480):
    if not (math.isfinite(fy) and fy > 0 and math.isfinite(pixels) and pixels > 0):
        raise ValueError("fy and pixels must be finite and positive")
    return math.degrees(2 * math.atan(pixels / (2 * fy)))


def visibility(camera_height, box_height, pitch_down_deg, vfov_deg):
    """Front top-edge visibility interval, excluding occlusion/depth/detector limits.

    Assumes 0 <= box_height < camera_height and a bottom ray below horizon.
    A top edge can remain visible after its front bottom edge leaves the image.
    Infinity for far distance means the upper ray crosses/clears the horizon.
    """
    vals = (camera_height, box_height, pitch_down_deg, vfov_deg)
    if not all(math.isfinite(x) for x in vals):
        raise ValueError("inputs must be finite")
    if not 0 <= box_height < camera_height or not 0 < vfov_deg < 180:
        raise ValueError("require 0 <= box_height < camera_height and 0 < VFOV < 180")
    bottom = pitch_down_deg + vfov_deg / 2
    top = pitch_down_deg - vfov_deg / 2
    if not 0 < bottom < 90 or not -90 < top < 90:
        raise ValueError("require forward-facing rays with bottom below horizon")
    return dict(lowest_ray_deg=bottom,
                floor_near_m=camera_height / math.tan(math.radians(bottom)),
                top_near_m=(camera_height-box_height) / math.tan(math.radians(bottom)),
                top_far_m=((camera_height-box_height) / math.tan(math.radians(top))
                           if top > 0 else math.inf))
