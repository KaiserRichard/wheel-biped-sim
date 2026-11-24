import json
import sys

import numpy as np
from scipy.linalg import expm

from wbr.control.predictor.design import delay_model, main


def test_delay_model_consumes_oldest_command_and_shifts_history():
    a = 2.0 * np.eye(6)
    b = np.arange(12).reshape(6, 2)
    state = np.arange(6)
    newest, middle, oldest = ([3, 4], [5, 6], [7, 8])
    command = [1, 2]
    transition, input_map = delay_model(a, b)
    actual = transition @ np.r_[state, newest, middle, oldest] + input_map @ command
    np.testing.assert_array_equal(actual, np.r_[a @ state + b @ oldest, command, newest, middle])


def test_cli_design_stabilizes_synthetic_delayed_plant(tmp_path, monkeypatch, capsys):
    a = np.diag([0.4, 0.2, -1.0, -1.5, -2.0, -3.0])
    b = np.eye(6, 2)
    tick = 0.05
    source = tmp_path / "plant.npz"
    np.savez(source, a=a, b=b, tick_s=tick, state_cost=np.ones(6), effort_cost=np.ones(2))
    monkeypatch.setattr(sys, "argv", ["design.py", str(source)])
    main()
    report = json.loads(capsys.readouterr().out)

    ad = expm(a * tick)
    bd = np.linalg.solve(a, (ad - np.eye(6)) @ b)
    transition, input_map = delay_model(ad, bd)
    radius = np.max(np.abs(np.linalg.eigvals(transition + input_map @ report["gain"])))
    np.testing.assert_allclose(report["spectral_radius"], radius, atol=1e-12)
    assert radius < 1.0


def test_c_predictor_contract(run_c_harness):
    run_c_harness("s5_predictor", includes=("wbr/control/predictor", "tests/native"))
