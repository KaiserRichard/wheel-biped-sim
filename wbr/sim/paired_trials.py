"""Seeded paired scoring rehearsal using the public primitive simulation."""

import argparse
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from wbr.dynamics import fivebar
from wbr.sim import scenarios


@dataclass(frozen=True)
class Draw:
    seed: int
    push_n: float


def paired_draws(seed, count):
    """Sample one synthetic push and packet-noise seed per paired comparison."""
    if count < 1:
        raise ValueError("at least one draw is required")
    rng = np.random.default_rng(seed)
    packet_seeds = rng.choice(2**32, size=count, replace=False)
    pushes = rng.uniform(0.5, 1.5, size=count)
    return tuple(Draw(int(packet_seed), float(push))
                 for packet_seed, push in zip(packet_seeds, pushes, strict=True))


def terminal_dwell_start(time_s, eligible, dwell_s):
    """Return the start of a continuous eligible suffix of the requested duration."""
    time_s, eligible = np.asarray(time_s), np.asarray(eligible, dtype=bool)
    if time_s.ndim != 1 or eligible.shape != time_s.shape:
        raise ValueError("time and eligibility must be matching vectors")
    if not np.isfinite(dwell_s) or dwell_s < 0:
        raise ValueError("dwell must be finite and nonnegative")
    if not np.isfinite(time_s).all() or np.any(np.diff(time_s) <= 0):
        raise ValueError("time samples must be finite and strictly increasing")
    violations = np.flatnonzero(~eligible)
    start = int(violations[-1]) + 1 if violations.size else 0
    if start < len(time_s) and time_s[-1] - time_s[start] >= dwell_s - 1e-12:
        return float(time_s[start])
    return None


def wilson_interval(passes, count):
    """Return the 95% Wilson interval for a declared population of draws."""
    if not 0 <= passes <= count or count == 0:
        raise ValueError("require 0 <= passes <= count and count > 0")
    z = 1.959963984540054
    rate = passes / count
    denominator = 1 + z**2 / count
    center = (rate + z**2 / (2 * count)) / denominator
    half_width = z * np.sqrt(rate * (1 - rate) / count + z**2 / (4 * count**2)) / denominator
    lower = float(center - half_width) if passes else 0.0
    upper = float(center + half_width) if passes < count else 1.0
    return lower, upper


def primitive_episode(parameters, coefficients, draw, delay_steps):
    """Return a terminal-rest outcome for one short, assumed primitive trial."""
    horizon_s = 1.0
    push_start_s, push_end_s = 0.2, 0.22
    samples = []

    def push_and_sample(time_s, sim, controller):
        torso = sim.m.body("torso").id
        sim.d.xfrc_applied[torso, 0] = draw.push_n if push_start_s <= time_s < push_end_s else 0.0
        step_s = sim.m.opt.timestep
        control_stride = round(scenarios.CTRL_DT / step_s)
        if round(time_s / step_s) % control_stride == 0:
            state = sim.read()
            samples.append((time_s, state["v"], state["phi"], state["roll"], sim.airborne()))

    # The controller reads noisy packets. Score rest from the plant samples.
    _, _, fell, sim = scenarios.run(parameters, coefficients, horizon_s, push_and_sample,
                                    noise=0.25, delay_steps=delay_steps, seed=draw.seed)
    samples = np.asarray(samples, dtype=float).reshape(-1, 5)
    if not np.isfinite(sim.d.qacc).all() or not np.isfinite(samples).all():
        return "numerical_error"
    if fell:
        return "fall"
    time_s, speed, pitch, roll, airborne = samples.T
    if not len(time_s) or time_s[-1] < horizon_s - scenarios.CTRL_DT - 1e-12:
        return "incomplete"
    eligible = ((time_s >= push_end_s) & (np.abs(speed) < 0.02)
                & (np.abs(pitch) < np.deg2rad(5)) & (np.abs(roll) < np.deg2rad(5))
                & (airborne == 0))
    return "pass" if terminal_dwell_start(time_s, eligible, 0.25) is not None else "no_durable_rest"


def run_paired(parameters, coefficients, *, seed, count):
    """Compare zero and one queued command tick using the identical draw in both."""
    pairs = [(primitive_episode(parameters, coefficients, draw, 0),
              primitive_episode(parameters, coefficients, draw, 1))
             for draw in paired_draws(seed, count)]

    def summary(arm):
        counts = Counter(pair[arm] for pair in pairs)
        return {"outcomes": dict(counts), "pass_fraction": counts["pass"] / count,
                "wilson_95": wilson_interval(counts["pass"], count)}

    return {"seed": seed, "pairs": count, "zero_delay": summary(0), "one_tick_delay": summary(1),
            "pass_to_failure": sum(a == "pass" and b != "pass" for a, b in pairs),
            "failure_to_pass": sum(a != "pass" and b == "pass" for a, b in pairs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--count", type=int, default=8)
    args = parser.parse_args()
    parameters = fivebar.load_params(Path("models/design.json"))
    gains = json.loads(Path("wbr/control/gains.json").read_text(encoding="utf-8"))
    print(json.dumps(run_paired(parameters, np.asarray(gains["poly_coef"]),
                                seed=args.seed, count=args.count), indent=2))


if __name__ == "__main__":
    main()
