"""Numerical and behavioral contracts; tolerances reflect SI arithmetic, not saved traces."""
import json
from pathlib import Path

import numpy as np
import pytest

from wbr.dynamics import fivebar, model
from wbr.control import controller, lqr
from wbr.control.estimator import Estimator
from wbr.sim.sensors import Packet
from wbr.sim import scenarios

ROOT = Path(__file__).resolve().parents[1]
P = fivebar.load_params(ROOT / "models" / "design.json")


def test_fivebar_ik_fk_round_trip_workspace():
    fb = P["fivebar"]
    for L in np.linspace(fb["leg_height_min"] + .005, fb["leg_height_max"] - .005, 11):
        for alpha in np.linspace(-.22, .22, 7):
            q = fivebar.ik(float(L), float(alpha), P)
            assert np.allclose(fivebar.polar(*q, P), [L, alpha], atol=2e-10)


def test_fivebar_jacobian_matches_independent_finite_difference():
    for L, alpha in ((.14, -.12), (.18, 0), (.24, .15)):
        q = fivebar.ik(L, alpha, P)
        h = 2e-6
        Jfd = np.column_stack([(np.array(fivebar.polar(q[0] + (h if i == 0 else 0), q[1] + (h if i == 1 else 0), P)) - np.array(fivebar.polar(q[0] - (h if i == 0 else 0), q[1] - (h if i == 1 else 0), P))) / (2*h) for i in range(2)])
        assert np.allclose(fivebar.jacobian_analytic(*q, P), Jfd, rtol=2e-5, atol=2e-7)


def test_fivebar_torque_map_virtual_work():
    q = fivebar.ik(.18, .08, P)
    J = fivebar.jacobian_analytic(*q, P)
    wrench = np.array([31., -2.4])
    dq = np.array([.003, -.002])
    assert np.isclose((J.T @ wrench) @ dq, wrench @ (J @ dq), rtol=1e-13, atol=1e-13)


def test_lqr_schedule_closed_loop_stable_over_leg_range():
    data = json.loads((ROOT / "wbr/control/gains.json").read_text())
    coef = np.asarray(data["poly_coef"])
    fb = P["fivebar"]
    for L in np.linspace(fb["leg_height_min"], fb["leg_height_max"], 15):
        A, B, *_ = model.linearize(P, float(L))
        assert np.linalg.eigvals(A - B @ lqr.gain(coef, L)).real.max() < 0


def test_gain_lookup_is_continuous():
    data = json.loads((ROOT / "wbr/control/gains.json").read_text())
    coef = np.asarray(data["poly_coef"])
    lo, hi = data["L"][0], data["L"][-1]
    eps = 1e-7
    assert np.linalg.norm(lqr.gain(coef, lo + eps) - lqr.gain(coef, lo)) < 1e-3
    assert np.linalg.norm(lqr.gain(coef, hi) - lqr.gain(coef, hi - eps)) < 1e-3


@pytest.mark.xfail(reason="gain() currently extrapolates; it has no configured schedule bounds", strict=True)
def test_gain_lookup_rejects_out_of_range():
    data = json.loads((ROOT / "wbr/control/gains.json").read_text())
    with pytest.raises(ValueError):
        lqr.gain(np.asarray(data["poly_coef"]), data["L"][0] - .01)


def test_controller_respects_joint_torque_saturation():
    c = controller.Controller(P, coef=np.zeros((4, 12)))
    q = fivebar.ik(.18, 0, P)
    J = fivebar.jacobian(*q, P)
    leg = {"L": .18, "alpha": 0., "Ld": 0., "alphad": 0., "J": J}
    obs = dict(v=0., phi=.1, phid=0., roll=0., rolld=0., yawd=0., pos=np.zeros(3), legs={"L": leg, "R": leg})
    hips, _ = c.step(obs, .002)
    assert max(np.max(np.abs(t)) for t in hips.values()) <= P["hip_motor"]["peak_torque"] + 1e-9


def test_jump_state_machine_only_uses_legal_transitions():
    c = controller.Controller(P, coef=np.zeros((4, 12)))
    c.start_jump(crouch_time=0)
    allowed = {"crouch": {"crouch", "push"}, "push": {"push", "flight"}, "flight": {"flight", "land"}, "land": {"land", "balance"}, "balance": {"balance"}}
    legs = {s: {"L": .18, "alpha": 0., "Ld": 0., "alphad": 0., "J": np.eye(2)} for s in "LR"}
    obs = dict(v=0., phi=0., phid=0., roll=0., rolld=0., yawd=0., pos=np.zeros(3), legs=legs)
    previous = c.phase
    for i in range(700):
        c.step(obs, .002, air=(i > 240 and i < 300))
        assert c.phase in allowed[previous]
        previous = c.phase


# Positive torque drives the wheels forward, under a forward lean. The stand scenario passing in closed loop is what proves this convention.
def test_pitch_error_drives_wheels_under_the_lean():
    data = json.loads((ROOT / "wbr/control/gains.json").read_text())
    c = controller.Controller(P, coef=np.asarray(data["poly_coef"]))
    q = fivebar.ik(.18, 0, P)
    leg = {"L": .18, "alpha": 0., "Ld": 0., "alphad": 0., "J": fivebar.jacobian(*q, P)}
    obs = dict(v=0., phi=.02, phid=0., roll=0., rolld=0., yawd=0., pos=np.zeros(3), legs={"L": leg, "R": leg})
    _, wheel = c.step(obs, .002)
    assert sum(wheel.values()) > 0


def test_estimator_converges_noiseless_stationary_signal_and_handles_dropouts():
    e = Estimator(P)
    angles = np.array([*fivebar.ik(.18, 0., P), 0.] * 2)
    packet = Packet(np.zeros(3), np.array([0., 0., P["gravity"]]), angles, np.zeros(6))
    for _ in range(500):
        out = e.update([packet, packet], .002)
    assert abs(out["phi"]) < 1e-8 and abs(out["roll"]) < 1e-8
    assert abs(out["v"]) < 1e-6
    assert np.isfinite(out["pos"]).all()
    # Dropout means no packets are delivered; update rejects it explicitly, without corrupting state.
    with pytest.raises(TypeError):
        e.update([], .002)
    assert np.isfinite(e.R).all() and np.isfinite(e.x).all()


@pytest.mark.slow
def test_stand_scenario_from_fresh_instance_passes():
    coef = np.asarray(json.loads((ROOT / "wbr/control/gains.json").read_text())["poly_coef"])
    result, _, _ = scenarios.stand(P, coef)
    assert result["pass"], result


@pytest.mark.slow
def test_drive_scenario_from_fresh_instance_passes():
    coef = np.asarray(json.loads((ROOT / "wbr/control/gains.json").read_text())["poly_coef"])
    result, _, _ = scenarios.drive(P, coef)
    assert result["pass"], result


def test_fall_detection_on_forced_fall():
    def force_fall(t, sim, ctrl):
        if t == 0:
            sim.d.qpos[3:7] = [np.cos(1.1 / 2), 0., np.sin(1.1 / 2), 0.]
            import mujoco
            mujoco.mj_forward(sim.m, sim.d)

    import mujoco
    result, _, fell, _ = scenarios.run(P, np.zeros((4, 12)), .01, force_fall)
    assert fell
