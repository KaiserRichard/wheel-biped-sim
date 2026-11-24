"""IMU bias and noise: overlapping Allan variance and the Studio noise model.

Studio gyro model (SOURCE wbr-studio m13_noise_table.m and sensor_params.m):
  y_k = b_k + sigma_w * n_k,  b_0 ~ N(0, bias_std^2),
  b_k = b_{k-1} + drift * sqrt(tick) * m_k            (rate random walk)
with tick = 1 ms, sigma_w = 0.03 deg/s, bias_std = 0.10 deg/s,
drift = 0.003 deg/s/sqrt(s). Accel: 0.035 m/s^2, 0.020, 0.002.

Allan variance of this model, for averaging time tau:
  sigma_A^2(tau) = N^2 / tau + K^2 tau / 3,
  N = sigma_w * sqrt(tick)   (angle random walk, rad/sqrt(s)),
  K = drift                   (rate random walk, rad/s/sqrt(s)).
A bias-instability term (flat, 2 ln2 / pi * B^2) and a quantisation term
(3 Q^2 / tau^2) are fitted as well when present in real data.
"""
import math
import random

from .linalg import lstsq

D2R = math.pi / 180.0
STUDIO_GYRO = {"sigma_w": 0.03 * D2R, "bias_std": 0.10 * D2R, "drift": 0.003 * D2R}
STUDIO_ACCEL = {"sigma_w": 0.035, "bias_std": 0.020, "drift": 0.002}
# Assumed synthetic gyro-noise setting, 1 kHz reference cadence; not measured here
# (SOURCE h7-advisory-fw docs/topics.md line 166; unbundled protocol reference 3).
BOARD_GYRO_STD = 0.0115


def simulate(n, dt, sigma_w, bias_std, drift, rng, b0=None, tick=1e-3):
    """Studio noise model sampled every dt. sigma_w is the per-sample std at
    the Studio 1 ms tick; for dt != tick it is rescaled so that N is kept
    (white-noise averaging), which is exact for tau >= dt."""
    sw = sigma_w * math.sqrt(tick / dt)
    b = rng.gauss(0.0, bias_std) if b0 is None else b0
    first = b
    out = [0.0] * n
    sd = drift * math.sqrt(dt)
    g = rng.gauss
    for k in range(n):
        if k:
            b += sd * g(0.0, 1.0)
        out[k] = b + sw * g(0.0, 1.0)
    return out, first


def octave_m(n, max_frac=0.2):
    ms, m = [], 1
    while m <= max_frac * n:
        ms.append(m)
        m *= 2
    return ms


def allan(y, dt, ms=None):
    """Overlapping Allan deviation. Returns list of (tau, adev, clusters)."""
    n = len(y)
    theta = [0.0] * (n + 1)
    s = 0.0
    for i, v in enumerate(y):
        s += v * dt
        theta[i + 1] = s
    out = []
    for m in ms or octave_m(n):
        tau = m * dt
        cnt = n + 1 - 2 * m
        if cnt < 2:
            break
        acc = 0.0
        for k in range(cnt):
            d = theta[k + 2 * m] - 2.0 * theta[k + m] + theta[k]
            acc += d * d
        avar = acc / (2.0 * tau * tau * cnt)
        out.append((tau, math.sqrt(avar), n / m))
    return out


def fit(adev, terms=("N", "K"), min_clusters=3.0):
    """Fit sigma^2 = N^2/tau + K^2 tau/3 [+ c_B B^2] [+ 3 Q^2/tau^2] by weighted
    linear least squares on the variance, weights from the cluster count
    (relative variance of sigma^2 ~ 2/(clusters)). Negative terms are dropped."""
    basis = {"N": lambda t: 1.0 / t, "K": lambda t: t / 3.0,
             "B": lambda t: 2.0 * math.log(2) / math.pi, "Q": lambda t: 3.0 / (t * t)}
    pts = [(t, a, c) for t, a, c in adev if c >= min_clusters]
    active = list(terms)
    while active:
        X = [[basis[k](t) for k in active] for t, a, c in pts]
        yv = [a * a for t, a, c in pts]
        w = [c / (2.0 * (a * a) ** 2) for t, a, c in pts]
        beta, cov, _, _ = lstsq(X, yv, w)
        neg = [k for k, b in zip(active, beta) if b <= 0]
        if not neg:
            res = {k: math.sqrt(b) for k, b in zip(active, beta)}
            # 1-sigma relative error of each coefficient (delta method on sqrt)
            res["rel_se"] = {k: 0.5 * math.sqrt(max(cov[i][i], 0.0)) / beta[i] for i, k in enumerate(active)}
            for k in terms:
                res.setdefault(k, 0.0)
            return res
        active = [k for k in active if k not in neg]
    return {k: 0.0 for k in terms}


def bias_error_std(N, K, T):
    """Std of (mean over T) minus the turn-on bias b0 for the Studio model."""
    return math.sqrt(N * N / T + K * K * T / 3.0)


def best_bias_window(N, K):
    """T minimising bias_error_std: sqrt(3) N / K."""
    return math.sqrt(3.0) * N / K if K > 0 else float("inf")


def adev_rel_error(T, tau):
    """Approximate 1-sigma relative error of an overlapping Allan deviation
    estimate at tau from a record of length T (IEEE 952 style, white noise)."""
    n = T / tau
    return 1.0 / math.sqrt(2.0 * max(n - 1.0, 1.0))
