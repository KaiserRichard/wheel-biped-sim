"""Deterministic short camera sequence rendered by tools.ground_plane.synth."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.ground_plane.synth import Box, render, _rotation

WIDTH, HEIGHT = 320, 240
FX = FY = 193.4765
CX, CY = 160.517, 120.520
INTRINSICS = (FX, FY, CX, CY)


def make_sequence(frame_count=16):
    """Move 1.5 m forward at 0.30 m camera height; pitch oscillates ±3°.

    World is (right, forward, up), starts as the first camera optical frame
    levelled by the renderer; returned poses are T_world_camera.
    """
    boxes = [Box((-.28, 2.20, .015), (.32, .35, .03)),
             Box((.28, 3.15, .025), (.38, .42, .05)),
             Box((0.0, 3.40, .06), (.42, .45, .12))]
    frames, labels, heights, poses, pitches = [], [], [], [], []
    for i in range(frame_count):
        frac = i / max(frame_count - 1, 1)
        y = 1.5 * frac
        pitch = 3.0 * np.cos(2 * np.pi * i / 5.0)
        camera_relative_boxes = [Box((b.position[0], b.position[1]-y, b.position[2]), b.size)
                                 for b in boxes]
        depth, lab, h = render(height=.30, pitch=pitch, boxes=camera_relative_boxes,
                               width=WIDTH, height_px=HEIGHT, fx=FX, fy=FY,
                               cx=CX, cy=CY, max_range=5.5)
        T = np.eye(4); T[:3, :3] = _rotation(np.deg2rad(pitch), 0.0)
        T[:3, 3] = (0., y, .30)
        frames.append(depth); labels.append(lab); heights.append(h)
        poses.append(T); pitches.append(pitch)
    return {"frames": frames, "labels": labels, "true_height": heights,
            "poses": poses, "pitch_deg": pitches, "boxes": boxes,
            "intrinsics": INTRINSICS}

