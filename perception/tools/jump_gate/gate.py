"""Pure Python research gate based on TILT_JUMP_RESEARCH.md section 3.

All timings/quality limits are ASSUMED, not validated hardware limits.
Status 3 means explicit reset/inactive; None means UNKNOWN, never inactive.
Rates are degrees/second, all times are seconds in one monotonic clock domain.
"""

from dataclasses import dataclass
from enum import Enum
import math

from tools.obstacle_decision.decide import Advisory


ASSUMED_TELEMETRY_MAX_AGE_S = 0.050
ASSUMED_CONTACT_DWELL_S = 0.100
ASSUMED_STABLE_RATE_DEG_S = 5.0  # Strictly below for SETTLE -> STAND only.
ASSUMED_WOBBLE_LIMIT_DEG_S = 60.0  # STAND inhibition above this; not a hardware limit.
ASSUMED_FRESH_PLANE_COUNT = 3
ASSUMED_PLANE_DWELL_S = 0.200
# Additional replay assumption: maximum age of a held depth/advisory observation.
ASSUMED_PLANE_MAX_AGE_S = 0.250
ASSUMED_STAND_QUALITY = 1.0  # Binary accepted geometry/timing, not a probability.
ASSUMED_TIME_EPSILON_S = 1e-9  # Floating-point boundary comparison only.


class Phase(str, Enum):
    STAND = "STAND"
    CROUCH = "CROUCH"
    FLIGHT = "FLIGHT"
    LAND = "LAND"
    SETTLE = "SETTLE"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class Tick:
    t: float
    jump_status: int | None
    airborne_l: bool | None
    airborne_r: bool | None
    telemetry_age_s: float | None
    pitch_rate_deg_s: float | None
    roll_rate_deg_s: float | None
    plane_valid: bool | None
    raw_advisory: Advisory | int | None
    raw_confidence: float | None
    # None means this tick contains a newly acquired observation at t.
    # Supply its original stamp when repeatedly ticking held geometry/advisory.
    plane_stamp_s: float | None = None


@dataclass(frozen=True)
class GateOutput:
    advisory: Advisory
    confidence: float
    phase: Phase
    reason: str


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class JumpGate:
    """Inhibit all non-STAND output, including STOP (no safety-latch authority).

    Temporal history here consists only of post-reset contact/plane evidence.
    A detector's own EMA cannot be reset via this advisory-only interface.
    Callers integrating a detector must reset that external history separately.
    """

    def __init__(self):
        self.phase = Phase.SETTLE
        self._last_t = None
        self._last_plane_stamp = None
        self._reset_history()

    def _reset_history(self):
        self._contact_since = None
        self._planes = []

    def _inhibit(self, phase, reason):
        self.phase = phase
        return GateOutput(Advisory.UNCERTAIN, 0.0, phase, reason)

    def _unknown(self, reason):
        self._reset_history()
        return self._inhibit(Phase.UNKNOWN, reason)

    def update(self, tick: Tick) -> GateOutput:
        """Process one tick; unknown/stale inputs override every phase.

        Missing contact/status/rates cannot establish STAND. Recovery from UNKNOWN
        requires explicit reset status 3 and a complete new SETTLE dwell.
        LAND is observable for at least one tick, even with simultaneous contact.
        """
        t = tick.t
        if not _finite(t):
            return self._unknown("invalid_clock")
        previous_t, self._last_t = self._last_t, t
        if previous_t is not None and t <= previous_t:
            self._last_plane_stamp = None
            return self._unknown("clock_reset_or_duplicate")
        if (type(tick.jump_status) is not int or tick.jump_status not in (0, 1, 2, 3)
                or type(tick.airborne_l) is not bool or type(tick.airborne_r) is not bool):
            return self._unknown("missing_or_invalid_phase_contact")
        age = tick.telemetry_age_s
        if not _finite(age) or age < 0 or age > ASSUMED_TELEMETRY_MAX_AGE_S:
            return self._unknown("stale_or_invalid_telemetry")
        if not all(_finite(rate) for rate in (tick.pitch_rate_deg_s, tick.roll_rate_deg_s)):
            return self._unknown("missing_or_invalid_attitude_rate")

        # Fresh plane timestamps are never rewritten when a display sample is held.
        stamp = t if tick.plane_stamp_s is None else tick.plane_stamp_s
        if not _finite(stamp) or stamp > t:
            return self._unknown("invalid_geometry_clock")
        if self._last_plane_stamp is not None and stamp < self._last_plane_stamp:
            self._last_plane_stamp = stamp
            return self._unknown("geometry_clock_reset")
        new_plane = self._last_plane_stamp is None or stamp > self._last_plane_stamp
        self._last_plane_stamp = stamp

        left, right = tick.airborne_l, tick.airborne_r
        if left and right:
            self._reset_history()
            return self._inhibit(Phase.FLIGHT, "both_airborne")
        if self.phase == Phase.FLIGHT:
            self._reset_history()  # Invalidate pre-impact temporal evidence.
            return self._inhibit(Phase.LAND, "first_contact_after_flight")
        if left != right:
            self._reset_history()
            if self.phase == Phase.LAND:
                return self._inhibit(Phase.LAND, "waiting_for_two_foot_contact")
            return self._unknown("inconsistent_leg_contact")

        # Both feet now contact. Active crouch/thrust inhibits perception.
        if tick.jump_status in (0, 1):
            self._reset_history()
            return self._inhibit(Phase.CROUCH,
                                 "thrust_inhibited" if tick.jump_status == 1 else "crouch_inhibited")
        if tick.jump_status == 2 and self.phase not in (Phase.LAND, Phase.SETTLE):
            return self._unknown("retract_without_contact_history")
        if tick.jump_status == 2:
            self._reset_history()
            return self._inhibit(Phase.SETTLE, "contact_observed_waiting_for_jump_reset")

        # Reset is an abort/recovery cue, never proof of landing or instant STAND.
        if self.phase not in (Phase.SETTLE, Phase.STAND):
            self._reset_history()
            self.phase = Phase.SETTLE
        rate = max(abs(tick.pitch_rate_deg_s), abs(tick.roll_rate_deg_s))
        # Post-jump recovery still requires the strict stability dwell. Once in
        # STAND, ordinary balancing wobble reduces confidence without a reset.
        stable = (rate <= ASSUMED_WOBBLE_LIMIT_DEG_S if self.phase == Phase.STAND
                  else rate < ASSUMED_STABLE_RATE_DEG_S)
        plane_ok = (tick.plane_valid is True
                    and t - stamp <= ASSUMED_PLANE_MAX_AGE_S + ASSUMED_TIME_EPSILON_S)
        try:
            raw = Advisory(tick.raw_advisory)
        except (TypeError, ValueError):
            return self._unknown("unknown_raw_advisory")
        if type(tick.raw_advisory) is bool:
            return self._unknown("unknown_raw_advisory")
        confidence = tick.raw_confidence
        if not _finite(confidence) or not 0 <= confidence <= 1:
            return self._unknown("invalid_raw_confidence")
        if not stable or not plane_ok:
            self._reset_history()
            return self._inhibit(Phase.SETTLE,
                                 "unstable_attitude" if not stable else "invalid_or_stale_plane")

        if self._contact_since is None:
            self._contact_since = t
        if new_plane:
            self._planes.append(stamp)
            # Keep the first observation of the clean dwell and the latest ones.
            # Each was fresh when acquired; a slower valid stream may span >250ms.
            if len(self._planes) > ASSUMED_FRESH_PLANE_COUNT:
                self._planes = [self._planes[0], *self._planes[-(ASSUMED_FRESH_PLANE_COUNT - 1):]]
        # A held plane adds no votes; invalid/stale input clears all evidence.
        ready = (t - self._contact_since + ASSUMED_TIME_EPSILON_S >= ASSUMED_CONTACT_DWELL_S
                 and len(self._planes) >= ASSUMED_FRESH_PLANE_COUNT
                 and self._planes[-1] - self._planes[0] + ASSUMED_TIME_EPSILON_S
                 >= ASSUMED_PLANE_DWELL_S)
        if self.phase != Phase.STAND and not ready:
            return self._inhibit(Phase.SETTLE, "settle_contact_and_fresh_plane_dwell")
        self.phase = Phase.STAND
        if raw == Advisory.UNCERTAIN:
            reason = "raw_uncertain_no_promotion"
            if rate > ASSUMED_STABLE_RATE_DEG_S:
                reason += ";wobble_confidence_scaled"
            return GateOutput(raw, 0.0, self.phase, reason)
        wobble_quality = max(0.0, 1.0 - rate / ASSUMED_WOBBLE_LIMIT_DEG_S)
        reason = ("wobble_confidence_scaled" if rate > ASSUMED_STABLE_RATE_DEG_S
                  else "stand_fresh_raw_advisory")
        return GateOutput(raw, confidence * ASSUMED_STAND_QUALITY * wobble_quality,
                          self.phase, reason)
