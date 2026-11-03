"""Symmetric five-bar leg: kinematics, Jacobian, and lumped leg properties.

Body frame: x forward, z up, origin at the midpoint of the two hips.
q1 swings the front upper link forward, q2 swings the rear upper link backward;
both are zero when the link hangs straight down.
Virtual leg: L is the hip-midpoint-to-axle distance. alpha is the leg's lean in the
body frame, with the axle at -L*(sin(alpha), cos(alpha)).
"""
import json

import numpy as np


def load_params(path):
    with open(path) as f:
        return json.load(f)


def _dims(p):
    fb = p["fivebar"]
    return fb["hip_spacing"], fb["upper_length"], fb["lower_length"]


def knees(q1, q2, p):
    d, l1, _ = _dims(p)
    kf = np.array([d / 2 + l1 * np.sin(q1), -l1 * np.cos(q1)])
    kr = np.array([-d / 2 - l1 * np.sin(q2), -l1 * np.cos(q2)])
    return kf, kr


def fk(q1, q2, p):
    """Axle position (x, z) in the body frame, taking the lower of the two circle intersections."""
    _, _, l2 = _dims(p)
    kf, kr = knees(q1, q2, p)
    c = kr - kf
    dist = np.linalg.norm(c)
    h2 = l2**2 - dist**2 / 4
    if h2 <= 0:
        raise ValueError("five-bar has no assembly at this pose")
    e = c / dist
    down = np.array([e[1], -e[0]])
    if down[1] > 0:
        down = -down
    return (kf + kr) / 2 + np.sqrt(h2) * down


def polar(q1, q2, p):
    foot = fk(q1, q2, p)
    return float(np.hypot(*foot)), float(np.arctan2(-foot[0], -foot[1]))


def ik(L, alpha, p):
    """Upper-link angles for a given virtual leg, knees outward."""
    d, l1, l2 = _dims(p)
    foot = -L * np.array([np.sin(alpha), np.cos(alpha)])
    out = []
    for hip, sign in ((np.array([d / 2, 0.0]), 1.0), (np.array([-d / 2, 0.0]), -1.0)):
        v = foot - hip
        r = np.hypot(*v)
        if r >= l1 + l2 or r <= abs(l1 - l2):
            raise ValueError(f"L={L}, alpha={alpha} is outside the workspace")
        a = np.arccos((l1**2 + r**2 - l2**2) / (2 * l1 * r))
        # Angle of hip-to-foot from straight down, positive outward for this hip.
        to_foot = np.arctan2(sign * v[0], -v[1])
        out.append(to_foot + a)
    return float(out[0]), float(out[1])


def jacobian(q1, q2, p, eps=1e-6):
    """d(L, alpha)/d(q1, q2). Hip torques follow from tau = J.T @ [F, T_alpha]."""
    J = np.zeros((2, 2))
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps
        hi = np.array(polar(q1 + dq[0], q2 + dq[1], p))
        lo = np.array(polar(q1 - dq[0], q2 - dq[1], p))
        J[:, i] = (hi - lo) / (2 * eps)
    return J


def jacobian_analytic(q1, q2, p):
    """Same as jacobian(), by implicit differentiation of the two lower-link length constraints."""
    d, l1, _ = _dims(p)
    kf, kr = knees(q1, q2, p)
    foot = fk(q1, q2, p)
    dkf = l1 * np.array([np.cos(q1), np.sin(q1)])
    dkr = l1 * np.array([-np.cos(q2), np.sin(q2)])
    A = np.vstack([foot - kf, foot - kr])
    dfoot = np.linalg.solve(A, np.diag([(foot - kf) @ dkf, (foot - kr) @ dkr]))
    L = np.hypot(*foot)
    # alpha = atan2(-x, -z), so d(alpha) = (z dx - x dz) / L^2.
    dpolar = np.array([foot / L, [foot[1] / L**2, -foot[0] / L**2]])
    return dpolar @ dfoot


def singularity_margin(q1, q2, p):
    """Smallest singular value of the (L, alpha) Jacobian, with L scaled by 1/L so that units match."""
    L, _ = polar(q1, q2, p)
    J = jacobian(q1, q2, p) * np.array([[1 / L], [1.0]])
    return float(np.linalg.svd(J, compute_uv=False)[-1])


def lumped_leg(L, p):
    """Mass, COM distance from the axle, and inertia about that COM of the pair of legs, with alpha = 0.

    Each leg has two upper and two lower links, modelled as uniform rods.
    The leg axis runs from the axle (height 0) up to the hip midpoint (height L).
    """
    fb = p["fivebar"]
    q1, q2 = ik(L, 0.0, p)
    kf, kr = knees(q1, q2, p)
    foot = fk(q1, q2, p)
    d = fb["hip_spacing"]
    rods = [
        (fb["upper_mass"], np.array([d / 2, 0.0]), kf, fb["upper_length"]),
        (fb["upper_mass"], np.array([-d / 2, 0.0]), kr, fb["upper_length"]),
        (fb["lower_mass"], kf, foot, fb["lower_length"]),
        (fb["lower_mass"], kr, foot, fb["lower_length"]),
    ]
    m = sum(r[0] for r in rods)
    com = sum(r[0] * (r[1] + r[2]) / 2 for r in rods) / m
    inertia = 0.0
    for mi, a, b, li in rods:
        c = (a + b) / 2
        inertia += mi * li**2 / 12 + mi * np.sum((c - com) ** 2)
    lc = float(L - abs(com[1]))
    return {"mass": 2 * m, "lc": lc, "inertia": 2 * float(inertia)}


def workspace(p, n=41):
    """Leg-range table at alpha = 0, with margin and the force per unit hip torque."""
    fb = p["fivebar"]
    out = []
    for L in np.linspace(fb["leg_height_min"], fb["leg_height_max"], n):
        q1, q2 = ik(L, 0.0, p)
        J = jacobian(q1, q2, p)
        out.append(
            {
                "L": float(L),
                "q1": q1,
                "margin": singularity_margin(q1, q2, p),
                # Static: tau = J[0,:] * F, so each hip carries |dL/dq| * F.
                "dL_dq": float(abs(J[0, 0])),
            }
        )
    return out
