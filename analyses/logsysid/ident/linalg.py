"""Small dense linear algebra and Nelder-Mead, standard library only."""
import math


def solve(A, b):
    """Gaussian elimination with partial pivoting. A: list of rows."""
    n = len(A)
    M = [list(map(float, A[i])) + [float(b[i])] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-300:
            raise ZeroDivisionError("singular matrix")
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def inv(A):
    n = len(A)
    cols = [solve(A, [1.0 if i == j else 0.0 for i in range(n)]) for j in range(n)]
    return [[cols[j][i] for j in range(n)] for i in range(n)]


def lstsq(X, y, w=None):
    """Weighted least squares. Returns (beta, cov, rss, dof).
    cov is scaled by the residual variance (unit weights) when w is None."""
    p = len(X[0])
    n = len(X)
    w = w or [1.0] * n
    XtX = [[0.0] * p for _ in range(p)]
    Xty = [0.0] * p
    for row, yi, wi in zip(X, y, w):
        for i in range(p):
            ri = row[i] * wi
            Xty[i] += ri * yi
            for j in range(i, p):
                XtX[i][j] += ri * row[j]
    for i in range(p):
        for j in range(i):
            XtX[i][j] = XtX[j][i]
    beta = solve(XtX, Xty)
    rss = sum(wi * (yi - sum(b * x for b, x in zip(beta, row))) ** 2 for row, yi, wi in zip(X, y, w))
    dof = max(1, n - p)
    s2 = rss / dof
    Ci = inv(XtX)
    cov = [[c * s2 for c in r] for r in Ci]
    return beta, cov, rss, dof


def corr_from_cov(cov):
    n = len(cov)
    return [[cov[i][j] / math.sqrt(cov[i][i] * cov[j][j]) for j in range(n)] for i in range(n)]


def nelder_mead(f, x0, step, tol=1e-10, max_iter=2000):
    n = len(x0)
    pts = [list(x0)]
    for i in range(n):
        x = list(x0)
        x[i] += step[i]
        pts.append(x)
    vals = [f(p) for p in pts]
    for _ in range(max_iter):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) <= tol * (abs(vals[0]) + tol):
            break
        c = [sum(p[i] for p in pts[:-1]) / n for i in range(n)]
        xr = [c[i] + (c[i] - pts[-1][i]) for i in range(n)]
        fr = f(xr)
        if fr < vals[0]:
            xe = [c[i] + 2 * (c[i] - pts[-1][i]) for i in range(n)]
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = [c[i] + 0.5 * (pts[-1][i] - c[i]) for i in range(n)]
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                for k in range(1, n + 1):
                    pts[k] = [pts[0][i] + 0.5 * (pts[k][i] - pts[0][i]) for i in range(n)]
                    vals[k] = f(pts[k])
    i = min(range(n + 1), key=lambda i: vals[i])
    return pts[i], vals[i]


def mean(x):
    return sum(x) / len(x)


def std(x):
    m = mean(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / max(1, len(x) - 1))


def percentile(x, q):
    s = sorted(x)
    if not s:
        return float("nan")
    k = (len(s) - 1) * q
    i = int(math.floor(k))
    j = min(i + 1, len(s) - 1)
    return s[i] + (s[j] - s[i]) * (k - i)
