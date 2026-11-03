"""ASSUMED LQR weights; computed polynomial schedule."""
import numpy as np
from scipy.linalg import solve_continuous_are
from wbr.dynamics import model
Q_DEFAULT=np.diag([30.,15.,30.,2.,120.,3.])
R_DEFAULT=np.diag([3.,1.])
def design_gains(p, Q=Q_DEFAULT, R=R_DEFAULT, n=15, deg=3, backend=model):
    """LQR over the leg range and a polynomial fit of each gain in L. Returns coefficients and a report."""
    fb = p['fivebar']
    Ls = np.linspace(fb['leg_height_min'], fb['leg_height_max'], n)
    (Ks, worst) = ([], -np.inf)
    for L in Ls:
        (A, B, _, _) = backend.linearize(p, float(L))
        S = solve_continuous_are(A, B, Q, R)
        K = np.linalg.solve(R, B.T @ S)
        Ks.append(K)
        worst = max(worst, float(np.linalg.eigvals(A - B @ K).real.max()))
    Ks = np.array(Ks)
    coef = np.polyfit(Ls, Ks.reshape(n, -1), deg)
    fit = np.array([np.polyval(coef[:, i], Ls) for i in range(12)]).T.reshape(n, 2, 6)
    worst_fit = -np.inf
    for L in np.linspace(Ls[0], Ls[-1], 3 * n):
        (A, B, _, _) = backend.linearize(p, float(L))
        worst_fit = max(worst_fit, float(np.linalg.eigvals(A - B @ gain(coef, L)).real.max()))
    return (coef, dict(L=Ls.tolist(), K=Ks.tolist(), worst_eig=worst, worst_eig_fit=worst_fit, fit_max_err=float(np.abs(fit - Ks).max()), Q=np.diag(Q).tolist(), R=np.diag(R).tolist()))
def gain(coef, L):
    return np.array([np.polyval(coef[:, i], L) for i in range(12)]).reshape(2, 6)
