"""Forward-range filtering shared by offline replay paths."""
from __future__ import annotations

from dataclasses import replace


def in_range(frame, max_m, range_hysteresis=0.10, state=None):
    """Keep whole obstacle clusters by nearest face, with live range hysteresis."""
    state = {} if state is None else state
    was_accepted = state.get("in_range", False)
    cutoff = max_m + (range_hysteresis if was_accepted else 0.0)
    inside = [cluster for cluster in frame.clusters if cluster.distance_m <= cutoff]
    state["in_range"] = bool(inside)
    return replace(
        frame,
        clusters=inside,
        nearest=min(inside, key=lambda cluster: cluster.distance_m) if inside else None,
    ), state
