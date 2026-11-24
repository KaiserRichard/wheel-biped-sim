"""PROPOSED clock method: integer epochs, centered weighted least squares.

No synchronization traffic or firmware is implemented. Bounds assume affine
clocks and nonnegative path delays; step detection is limited by those delays.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from functools import total_ordering
import math

import numpy as np

AVAILABLE = "AVAILABLE"
UNAVAILABLE = "UNAVAILABLE"
Session = str | int


class ClockDomainError(ValueError):
    """An operation mixed unrelated clock domains."""


class SessionError(ClockDomainError):
    """An operation mixed different boots/runs of a clock."""


class InvalidClockMap(ValueError):
    """No trustworthy current relation is available."""


class ClockStepError(InvalidClockMap):
    """Clock discontinuity: explicitly bind and fit again."""


def _integer(value: int, name: str) -> None:
    if type(value) is not int:
        raise TypeError(f"{name} must be a Python integer")


def _session(value: Session) -> None:
    if type(value) not in (str, int) or value == "":
        raise TypeError("session must be a nonempty string or integer boot/run ID")


@total_ordering
@dataclass(frozen=True)
class Stamp:
    value_ns: int
    domain: str
    session: Session

    def __post_init__(self) -> None:
        _integer(self.value_ns, "value_ns")
        if not isinstance(self.domain, str) or not self.domain:
            raise TypeError("domain must be a nonempty string")
        _session(self.session)

    def _same_clock(self, other: Stamp) -> None:
        if self.domain != other.domain:
            raise ClockDomainError(f"{self.domain!r} != {other.domain!r}; use ClockMap")
        if self.session != other.session:
            raise SessionError("stamps belong to different clock sessions")

    def __sub__(self, other: Stamp | int) -> int | Stamp:
        if isinstance(other, Stamp):
            self._same_clock(other)
            return self.value_ns - other.value_ns
        _integer(other, "duration_ns")
        return Stamp(self.value_ns - other, self.domain, self.session)

    def __add__(self, duration_ns: int) -> Stamp:
        if isinstance(duration_ns, Stamp):
            self._same_clock(duration_ns)
            raise TypeError("adding two epochs is undefined")
        _integer(duration_ns, "duration_ns")
        return Stamp(self.value_ns + duration_ns, self.domain, self.session)

    __radd__ = __add__

    def __lt__(self, other: Stamp) -> bool:
        if not isinstance(other, Stamp):
            return NotImplemented
        self._same_clock(other)
        return self.value_ns < other.value_ns


@dataclass(frozen=True)
class Exchange:
    """t1/t4 in A; t2/t3 in B. Epochs are acquisition/transport observations."""

    t1: Stamp
    t2: Stamp
    t3: Stamp
    t4: Stamp

    def __post_init__(self) -> None:
        if self.t4 - self.t1 <= 0 or self.t3 - self.t2 < 0:
            raise ValueError("exchange requires t4 > t1 and t3 >= t2")

    @property
    def midpoint_a_ns(self) -> int:
        return (self.t1.value_ns + self.t4.value_ns) // 2

    @property
    def midpoint_b_ns(self) -> int:
        return (self.t2.value_ns + self.t3.value_ns) // 2


@dataclass(frozen=True)
class MappedStamp:
    stamp: Stamp
    uncertainty_ns: int  # Always expressed in the destination clock's ns.
    mapping_revision: int


@dataclass(frozen=True)
class AgeResult:
    status: str
    upper_bound_ns: int | None = None
    mapped_epoch: MappedStamp | None = None
    reason: str | None = None


@dataclass(frozen=True)
class _AffineFit:
    source_ref_ns: int
    target_ref_ns: int
    slope: float
    intercept_ns: float
    residuals_ns: tuple[float, ...]

    def delta(self, value_ns: int) -> float:
        return self.slope * (value_ns - self.source_ref_ns) + self.intercept_ns

    def map_ns(self, value_ns: int) -> int:
        # Never convert the absolute target epoch to float (UTC exceeds 2**53).
        return self.target_ref_ns + round(self.delta(value_ns))


def _fit_affine(source: list[int], target: list[int],
                weights: list[float] | None = None) -> _AffineFit:
    """Numeric fit for explicitly paired domains; epochs remain integer ns."""
    if len(source) < 2 or len(source) != len(target):
        raise ValueError("need at least two matching timestamp pairs")
    source_ref, target_ref = source[0], target[0]
    x = np.array([v - source_ref for v in source], dtype=float)
    y = np.array([v - target_ref for v in target], dtype=float)
    w = np.ones(len(x)) if weights is None else np.array(weights, dtype=float)
    center_x = float(np.average(x, weights=w))
    center_y = float(np.average(y, weights=w))
    dx, dy = x - center_x, y - center_y
    denom = float(np.dot(w * dx, dx))
    if denom <= 0 or not math.isfinite(denom):
        raise ValueError("timestamp pairs have no usable source-time span")
    slope = float(np.dot(w * dx, dy) / denom)
    intercept = center_y - slope * center_x
    if not math.isfinite(slope) or slope <= 0 or not math.isfinite(intercept):
        raise ValueError("clock fit must have a finite positive slope")
    residuals = y - (slope * x + intercept)
    return _AffineFit(source_ref, target_ref, slope, intercept,
                      tuple(float(r) for r in residuals))


def _causal_rate_upper(samples: deque[Exchange]) -> float:
    """Bound true B/A rate using nonnegative forward/reverse delays.

    For every i,j, t3_i - rate*t4_i <= offset <= t2_j - rate*t1_j.
    A finite upper rate needs at least two nonoverlapping A exchanges.
    """
    lower, upper = 0.0, math.inf
    for i in samples:
        for j in samples:
            dx = i.t4.value_ns - j.t1.value_ns
            dy = i.t3.value_ns - j.t2.value_ns
            if dx > 0:
                lower = max(lower, dy / dx)
            elif dx < 0:
                upper = min(upper, dy / dx)
            elif dy > 0:
                raise ValueError("exchange intervals violate causality")
    if not math.isfinite(upper) or upper <= 0 or lower > upper:
        raise ValueError("exchanges do not bound a positive affine clock rate")
    return math.nextafter(upper, math.inf)


class ClockMap:
    """A -> B affine fit; convert() also supports its explicit inverse.

    Parameters/defaults are INFERRED prototype choices, not controller budgets.
    Sessions never auto-switch. bind() is required after an invalidation.
    """

    def __init__(self, domain_a: str, domain_b: str, session_a: Session,
                 session_b: Session, *, window_size: int = 32,
                 min_samples: int = 3, step_threshold_ns: int = 1_000_000):
        Stamp(0, domain_a, session_a)
        Stamp(0, domain_b, session_b)
        if domain_a == domain_b:
            raise ClockDomainError("mapping requires distinct domain labels")
        for name, value in (("window_size", window_size), ("min_samples", min_samples),
                            ("step_threshold_ns", step_threshold_ns)):
            _integer(value, name)
        if not 2 <= min_samples <= window_size or step_threshold_ns < 0:
            raise ValueError("need 2 <= min_samples <= window_size and nonnegative threshold")
        self.domain_a, self.domain_b = domain_a, domain_b
        self.session_a, self.session_b = session_a, session_b
        self.window_size, self.min_samples = window_size, min_samples
        self.step_threshold_ns = step_threshold_ns
        self._samples: deque[Exchange] = deque(maxlen=window_size)
        self._fit: _AffineFit | None = None
        self._bound_ns = 0
        self._rate_upper = 1.0
        self._blocked = False
        self.invalid_reason: str | None = "insufficient exchanges"
        self.mapping_revision = 0

    @property
    def valid(self) -> bool:
        return self._fit is not None and not self._blocked

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    @property
    def slope(self) -> float:
        self._require_valid()
        return self._fit.slope

    @property
    def drift_ppm(self) -> float:
        return (self.slope - 1) * 1_000_000

    @property
    def offset_at_reference_ns(self) -> int:
        self._require_valid()
        return self._fit.map_ns(self._fit.source_ref_ns) - self._fit.source_ref_ns

    def _require_valid(self) -> None:
        if not self.valid:
            raise InvalidClockMap(self.invalid_reason or "no valid map")

    def invalidate(self, reason: str) -> None:
        self._fit = None
        self._samples.clear()
        self._blocked = True
        self.invalid_reason = reason
        self.mapping_revision += 1

    def bind(self, session_a: Session, session_b: Session) -> None:
        """Trusted explicit restart; discard all history even for the same IDs."""
        _session(session_a)
        _session(session_b)
        self.invalidate("insufficient exchanges after bind")
        self.session_a, self.session_b = session_a, session_b
        self._blocked = False

    def _check_stamp(self, stamp: Stamp, domain: str, session: Session) -> None:
        if stamp.domain != domain:
            raise ClockDomainError(f"expected {domain!r}, got {stamp.domain!r}")
        if stamp.session != session:
            self.invalidate("session changed; explicit bind required")
            raise SessionError(self.invalid_reason)

    def add_exchange(self, exchange: Exchange) -> bool:
        for stamp, domain, session in (
            (exchange.t1, self.domain_a, self.session_a),
            (exchange.t4, self.domain_a, self.session_a),
            (exchange.t2, self.domain_b, self.session_b),
            (exchange.t3, self.domain_b, self.session_b),
        ):
            self._check_stamp(stamp, domain, session)
        if self._blocked:
            raise InvalidClockMap(self.invalid_reason)
        if self._samples:
            previous = self._samples[-1]
            if exchange.t1 == previous.t1 and exchange.t4 == previous.t4:
                raise ValueError("duplicate A exchange")
            if exchange.t1 <= previous.t1 or exchange.t4 <= previous.t4:
                self.invalidate("A clock reversal or out-of-order exchange")
                raise ClockStepError(self.invalid_reason)
            if exchange.t2 <= previous.t2 or exchange.t3 <= previous.t3:
                self.invalidate("B clock reversal or out-of-order exchange")
                raise ClockStepError(self.invalid_reason)
        x, y = exchange.midpoint_a_ns, exchange.midpoint_b_ns
        if self.valid:
            # Subtract centered values, never large float epochs.
            residual = (y - self._fit.target_ref_ns) - self._fit.delta(x)
            half_rtt = self._rate_upper * (exchange.t4 - exchange.t1) / 2
            if abs(residual) > (self.step_threshold_ns + self._uncertainty(x) + half_rtt):
                self.invalidate("clock step: residual jump exceeds delay/error bounds")
                raise ClockStepError(self.invalid_reason)
        self._samples.append(exchange)
        if len(self._samples) < self.min_samples:
            return False
        # Prefer low network RTT, retaining temporal spread across the window.
        # Scale is approximately 1 during bootstrap and comes from the old fit later.
        scale = self.slope if self.valid else 1.0
        rtts = [max(1.0, scale * (e.t4 - e.t1) - (e.t3 - e.t2))
                for e in self._samples]
        fastest = min(rtts)
        weights = [(fastest / rtt) ** 2 for rtt in rtts]
        try:
            fit = _fit_affine([e.midpoint_a_ns for e in self._samples],
                              [e.midpoint_b_ns for e in self._samples], weights)
            rate_upper = _causal_rate_upper(self._samples)
            half_rtts = [max(0.0, rate_upper * (e.t4 - e.t1) - (e.t3 - e.t2)) / 2
                         for e in self._samples]
            # Each endpoint bounds the true affine relation by residual + half RTT.
            # Use a causally bounded true rate, rather than assuming the fitted
            # rate is exact. Include midpoint truncation and float roundoff.
            roundoff = 4 * math.ulp(max(abs(fit.delta(e.midpoint_a_ns)) for e in self._samples))
            bound = math.ceil(max(abs(r) + h for r, h in
                                  zip(fit.residuals_ns, half_rtts)) + rate_upper + 2 + roundoff)
        except ValueError as exc:
            self.invalidate(f"unusable affine fit: {exc}")
            raise InvalidClockMap(self.invalid_reason) from exc
        self._fit, self._bound_ns = fit, bound
        self._rate_upper = rate_upper
        self.invalid_reason = None
        self.mapping_revision += 1
        return True

    def _uncertainty(self, value_ns: int) -> int:
        self._require_valid()
        first = self._samples[0].midpoint_a_ns
        last = self._samples[-1].midpoint_a_ns
        outside = max(first - value_ns, value_ns - last, 0)
        # Affine slope error is bounded by the sum of endpoint errors / span.
        growth = (2 * self._bound_ns * outside + last - first - 1) // (last - first)
        return self._bound_ns + growth

    def convert(self, stamp: Stamp) -> MappedStamp:
        """Map an epoch forward/inverse, carrying destination uncertainty/revision."""
        if stamp.domain == self.domain_a:
            self._check_stamp(stamp, self.domain_a, self.session_a)
            self._require_valid()
            value_ns = self._fit.map_ns(stamp.value_ns)
            bound = self._uncertainty(stamp.value_ns)
            target = Stamp(value_ns, self.domain_b, self.session_b)
        elif stamp.domain == self.domain_b:
            self._check_stamp(stamp, self.domain_b, self.session_b)
            self._require_valid()
            delta = ((stamp.value_ns - self._fit.target_ref_ns) -
                     self._fit.intercept_ns) / self.slope
            value_ns = self._fit.source_ref_ns + round(delta)
            # Solve error <= U(x) + rate_error * |inverse_error|. Refuse
            # an inverse when the endpoint error cannot bound a positive rate.
            forward_bound = self._uncertainty(value_ns)
            span = self._samples[-1].midpoint_a_ns - self._samples[0].midpoint_a_ns
            lower_rate = self.slope - 2 * self._bound_ns / span
            if lower_rate <= 0:
                raise InvalidClockMap("inverse uncertainty cannot bound a positive rate")
            bound = math.ceil(forward_bound / lower_rate) + 1
            target = Stamp(value_ns, self.domain_a, self.session_a)
        else:
            raise ClockDomainError("stamp does not belong to either bound domain")
        return MappedStamp(target, bound, self.mapping_revision)


def source_age(now: Stamp, estimate_epoch: Stamp, clock_map: ClockMap | None) -> AgeResult:
    """R2 upper bound: destination now - mapped source epoch + uncertainty.

    Any absent/invalid/wrong-session relation yields UNAVAILABLE. No robot age
    budget or navigation-eligibility decision is encoded here.
    """
    if clock_map is None:
        return AgeResult(UNAVAILABLE, reason="no clock map")
    try:
        # Check now before converting to avoid arithmetic involving another boot.
        if estimate_epoch.domain == clock_map.domain_a:
            clock_map._check_stamp(now, clock_map.domain_b, clock_map.session_b)
        elif estimate_epoch.domain == clock_map.domain_b:
            clock_map._check_stamp(now, clock_map.domain_a, clock_map.session_a)
        else:
            raise ClockDomainError("estimate has an unrelated source domain")
        mapped = clock_map.convert(estimate_epoch)
        age = now - mapped.stamp + mapped.uncertainty_ns
        if age < 0:
            return AgeResult(UNAVAILABLE, reason="source epoch is certainly in the future")
        return AgeResult(AVAILABLE, age, mapped)
    except (InvalidClockMap, ClockDomainError) as exc:
        return AgeResult(UNAVAILABLE, reason=str(exc))


class TickUnwrapper:
    """Ordered uint32 microseconds -> session-scoped integer ns.

    Every observation gap MUST be below 2**31 us. Full missed wraps cannot be
    inferred from raw ticks; reset explicitly after suspension/reboot.
    """

    def __init__(self, domain: str, session: Session):
        Stamp(0, domain, session)
        self.domain, self.session = domain, session
        self._last: int | None = None
        self._unwrapped_us = 0
        self._blocked = False

    def reset(self, session: Session) -> None:
        _session(session)
        self.session = session
        self._last = None
        self._unwrapped_us = 0
        self._blocked = False

    def unwrap(self, ticks_us: int, session: Session) -> Stamp:
        _integer(ticks_us, "ticks_us")
        _session(session)
        if not 0 <= ticks_us < 2**32:
            raise ValueError("ticks_us must be uint32")
        if session != self.session:
            self._blocked = True
            raise SessionError("timer session changed; explicit reset required")
        if self._blocked:
            raise InvalidClockMap("timer unwrap invalid; explicit reset required")
        if self._last is None:
            self._unwrapped_us = ticks_us
        else:
            delta = (ticks_us - self._last) % 2**32
            if delta >= 2**31:
                self._blocked = True
                raise ClockStepError("timer reversal/ambiguous gap; explicit reset required")
            self._unwrapped_us += delta
        self._last = ticks_us
        return Stamp(self._unwrapped_us * 1000, self.domain, self.session)
