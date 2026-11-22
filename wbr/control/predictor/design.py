"""Design signed S5 gains from a caller-supplied six-state linearization."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

HISTORY = 3
OBSERVATIONS = 6
EFFORTS = 2


def delay_model(a, b):
    """Lift a discrete plant with newest-first commands and three ticks of delay."""
    if a.shape != (OBSERVATIONS, OBSERVATIONS) or b.shape != (OBSERVATIONS, EFFORTS):
        raise ValueError("expected a six-state, two-effort discrete plant")

    size = OBSERVATIONS + HISTORY * EFFORTS
    transition = np.zeros((size, size))
    input_map = np.zeros((size, EFFORTS))
    transition[:OBSERVATIONS, :OBSERVATIONS] = a
    transition[:OBSERVATIONS, -EFFORTS:] = b
    history_tail = slice(OBSERVATIONS + EFFORTS, size)
    transition[history_tail, OBSERVATIONS:-EFFORTS] = np.eye((HISTORY - 1) * EFFORTS)
    input_map[OBSERVATIONS:OBSERVATIONS + EFFORTS] = np.eye(EFFORTS)
    return transition, input_map


def design_gain(a, b, tick_s, state_cost, effort_cost):
    """Return a float32 C-convention gain and its delay-model spectral radius."""
    if not np.isfinite(tick_s) or tick_s <= 0:
        raise ValueError("tick_s must be positive and finite")
    a, b, _, _, _ = cont2discrete(
        (a, b, np.eye(OBSERVATIONS), np.zeros((OBSERVATIONS, EFFORTS))), tick_s
    )
    transition, input_map = delay_model(a, b)
    q = np.diag(np.r_[state_cost, np.zeros(HISTORY * EFFORTS)])
    r = np.diag(effort_cost)
    solution = solve_discrete_are(transition, input_map, q, r)
    gain = -np.linalg.solve(r + input_map.T @ solution @ input_map,
                            input_map.T @ solution @ transition).astype(np.float32)
    radius = float(np.abs(np.linalg.eigvals(transition + input_map @ gain)).max())
    if radius >= 1.0:
        raise ValueError("exported float32 gain did not stabilize the delay model")
    return gain, radius


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path, help="local NPZ with a, b, tick_s, state_cost and effort_cost")
    args = parser.parse_args()
    with np.load(args.model, allow_pickle=False) as model:
        gain, radius = design_gain(model["a"], model["b"], model["tick_s"].item(),
                                   model["state_cost"], model["effort_cost"])
    print(json.dumps({"gain": gain.tolist(), "spectral_radius": radius}, indent=2))


if __name__ == "__main__":
    main()
