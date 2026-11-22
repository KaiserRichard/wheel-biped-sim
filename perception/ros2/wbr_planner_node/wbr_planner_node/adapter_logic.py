"""ROS-independent parameter and timestamp pairing helpers for the planner adapter."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

POSE_TOLERANCE_NS = 50_000_000
PENDING_POSE_TIMEOUT_S = 0.5


def core_parameters(parameters: dict[str, Any]) -> dict[str, Any]:
    """Drop adapter-only options before constructing PlannerCore."""
    return {key: value for key, value in parameters.items() if key != "max_rate_hz"}


def nearest_pose(poses: Iterable[tuple[int, Any]], stamp_ns: int,
                 tolerance_ns: int = POSE_TOLERANCE_NS):
    poses = tuple(poses)
    if not poses:
        return None
    delta, pose = min(poses, key=lambda item: abs(item[0] - stamp_ns))
    return pose if abs(delta - stamp_ns) <= tolerance_ns else None


def current_arrow_geometry(current_xy, length=0.25):
    """Marker pose translation plus absolute map-frame arrow endpoints."""
    x, y = map(float, current_xy)
    return (0.0, 0.0, 0.0), ((x, y, 0.05), (x + length, y, 0.05))


@dataclass
class PendingFrame:
    message: Any
    stamp_ns: int
    queued_mono: float

    def should_wait(self, pose, now_mono: float,
                    timeout_s: float = PENDING_POSE_TIMEOUT_S) -> bool:
        return pose is None and now_mono - self.queued_mono < timeout_s
