"""New matched CARE/DARE synthesis, independent of any upstream gains."""

import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import cont2discrete

from sim.planar import finite_difference


def eigenvalues(matrix):
    return [[float(v.real), float(v.imag)] for v in np.linalg.eigvals(matrix)]


def controllability(A, B):
    C = np.concatenate([np.linalg.matrix_power(A, i) @ B for i in range(6)], axis=1)
    singular = np.linalg.svd(C, compute_uv=False)
    tolerance = max(C.shape) * np.finfo(float).eps * singular[0]
    return {"rank": int(np.sum(singular > tolerance)), "singular_values": singular.tolist(),
            "rank_tolerance": float(tolerance), "condition_number": float(singular[0] / singular[-1])}


def synthesize(plant):
    cfg = plant.cfg
    eps = cfg["simulation"]["finite_difference_epsilon"]
    dt = cfg["simulation"]["dt_s"]
    Ac, Bc = finite_difference(plant.derivative, eps)
    # Differentiate the ACTUAL held-control mj_step transition, including RK4.
    Ad, Bd = finite_difference(plant.transition, eps)
    Az, Bz, _, _, _ = cont2discrete((Ac, Bc, np.eye(6), np.zeros((6, 2))), dt, method="zoh")
    Q, R = np.diag(cfg["controller"]["Q_diagonal"]), np.diag(cfg["controller"]["R_diagonal"])
    Pc = solve_continuous_are(Ac, Bc, Q, R)
    Kc = np.linalg.solve(R, Bc.T @ Pc)
    Qd, Rd = dt * Q, dt * R
    Pd = solve_discrete_are(Ad, Bd, Qd, Rd)
    Kd = np.linalg.solve(Rd + Bd.T @ Pd @ Bd, Bd.T @ Pd @ Ad)
    rc = Ac.T @ Pc + Pc @ Ac - Pc @ Bc @ Kc + Q
    rd = Ad.T @ Pd @ Ad - Pd - Ad.T @ Pd @ Bd @ Kd + Qd
    K = Kd if cfg["controller"]["solver"] == "DARE" else Kc
    record = {name: value.tolist() for name, value in {
        "A_continuous": Ac, "B_continuous": Bc, "A_discrete_actual": Ad, "B_discrete_actual": Bd,
        "A_discrete_zoh": Az, "B_discrete_zoh": Bz, "Q": Q, "R": R, "Q_discrete": Qd,
        "R_discrete": Rd, "P_CARE": Pc, "P_DARE": Pd, "K_CARE": Kc, "K_DARE": Kd, "K_used": K,
    }.items()}
    record.update({
        "state_order": cfg["controller"]["state_order"], "input_order": cfg["controller"]["input_order"],
        "dt_s": dt, "epsilon": eps, "solver_used": cfg["controller"]["solver"],
        "continuous_open_loop_eigenvalues": eigenvalues(Ac),
        "discrete_open_loop_eigenvalues": eigenvalues(Ad),
        "continuous_CARE_closed_loop_eigenvalues": eigenvalues(Ac - Bc @ Kc),
        "discrete_closed_loop_eigenvalues": eigenvalues(Ad - Bd @ K),
        "continuous_controllability": controllability(Ac, Bc),
        "discrete_controllability": controllability(Ad, Bd),
        "CARE_relative_residual": float(np.linalg.norm(rc) / np.linalg.norm(Q)),
        "DARE_relative_residual": float(np.linalg.norm(rd) / np.linalg.norm(Qd)),
        "actual_vs_zoh_A_max_abs": float(np.max(np.abs(Ad - Az))),
        "actual_vs_zoh_B_max_abs": float(np.max(np.abs(Bd - Bz))),
        "equilibrium_derivative_max_abs": float(np.max(np.abs(plant.derivative(np.zeros(6), np.zeros(2))))),
        "claim": "NEW matched synthesis for ASSUMED plant; gain-parity milestone remains pending."
    })
    if record["continuous_controllability"]["rank"] != 6 or record["discrete_controllability"]["rank"] != 6:
        raise ValueError("Plant not numerically controllable")
    if np.max(np.real(np.linalg.eigvals(Ac))) <= 0:
        raise ValueError("Upright is not open-loop unstable")
    if max(abs(np.linalg.eigvals(Ad - Bd @ K))) >= 1:
        raise ValueError("Sampled closed-loop linearization is unstable")
    return K, record
