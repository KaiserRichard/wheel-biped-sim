import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from wbr.dynamics import fivebar
from wbr.sim import paired_trials as trials

ROOT = Path(__file__).resolve().parents[1]


def test_terminal_dwell_requires_rest_through_end():
    time = [0.0, 0.25, 0.5, 0.75, 1.0]
    assert trials.terminal_dwell_start(time, [0, 1, 1, 1, 0], 0.25) is None
    assert trials.terminal_dwell_start(time, [0, 1, 0, 1, 1], 0.25) == 0.75
    assert trials.terminal_dwell_start(time, [1, 1, 1, 0, 1], 0.25) is None
    assert trials.terminal_dwell_start([], [], 0.25) is None
    with pytest.raises(ValueError, match="increasing"):
        trials.terminal_dwell_start([0.0, 0.0], [1, 1], 0.25)


def test_wilson_interval_handles_all_and_no_passes():
    assert trials.wilson_interval(0, 4) == pytest.approx((0.0, 0.4899), abs=1e-4)
    assert trials.wilson_interval(4, 4) == pytest.approx((0.5101, 1.0), abs=1e-4)
    with pytest.raises(ValueError, match="count"):
        trials.wilson_interval(0, 0)


def test_pairs_share_inputs_and_count_disagreements(monkeypatch):
    draws = trials.paired_draws(42, 3)
    assert draws == trials.paired_draws(42, 3)
    assert draws != trials.paired_draws(43, 3)
    episodes = []

    def synthetic_episode(parameters, coefficients, draw, delay):
        episodes.append((draw, delay))
        if draw == draws[0] and delay == 0:
            return "fall"
        if draw == draws[1] and delay == 1:
            return "no_durable_rest"
        return "pass"

    monkeypatch.setattr(trials, "primitive_episode", synthetic_episode)
    report = trials.run_paired(None, None, seed=42, count=3)
    assert episodes == [(draw, delay) for draw in draws for delay in (0, 1)]
    assert report["pass_to_failure"] == report["failure_to_pass"] == 1
    assert report["zero_delay"]["outcomes"] == {"fall": 1, "pass": 2}
    assert report["one_tick_delay"]["outcomes"] == {"no_durable_rest": 1, "pass": 2}


@pytest.mark.parametrize(("true_speed", "packet_speed", "outcome"), [
    (0.03, 0.0, "no_durable_rest"),
    (0.0, 0.03, "pass"),
])
def test_rest_uses_plant_samples(monkeypatch, true_speed, packet_speed, outcome):
    def run(parameters, coefficients, horizon_s, script, **kwargs):
        assert kwargs["noise"] > 0
        sim = SimpleNamespace(
            m=SimpleNamespace(opt=SimpleNamespace(timestep=0.001),
                              body=lambda name: SimpleNamespace(id=0)),
            d=SimpleNamespace(xfrc_applied=np.zeros((1, 6)), qacc=np.zeros(1)),
            read=lambda: {"v": true_speed, "phi": 0.0, "roll": 0.0},
            airborne=lambda: False,
        )
        time_s = np.arange(0, horizon_s, trials.scenarios.CTRL_DT)
        for tick in time_s:
            script(tick, sim, None)
        zeros = np.zeros_like(time_s)
        packet_trace = {"t": time_s, "v": np.full_like(time_s, packet_speed),
                        "phi": zeros, "roll": zeros, "air": zeros}
        return packet_trace, None, False, sim

    monkeypatch.setattr(trials.scenarios, "run", run)
    assert trials.primitive_episode(None, None, trials.Draw(3, 1.0), 0) == outcome


def test_rehearsal_runs_on_public_primitive_model():
    parameters = fivebar.load_params(ROOT / "models/design.json")
    gains = json.loads((ROOT / "wbr/control/gains.json").read_text(encoding="utf-8"))
    coefficients = np.asarray(gains["poly_coef"])
    report = trials.run_paired(parameters, coefficients, seed=42, count=2)
    assert report == trials.run_paired(parameters, coefficients, seed=42, count=2)
    assert report["pairs"] == 2
    for arm in ("zero_delay", "one_tick_delay"):
        assert sum(report[arm]["outcomes"].values()) == 2
        assert set(report[arm]["outcomes"]) <= {"pass", "no_durable_rest", "fall"}
