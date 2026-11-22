"""Small Python model of the MCU policy for HEARTBEAT + PERCEPTION_HEALTH."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import json
from perception_health import FLAG_AGE_VALID, FLAG_MAPPING_VALID, TRACKING_STATES, U32_MAX

@dataclass
class Event:
    kind: str
    received_us: int
    age_at_send_us: int = 0
    tracking_state: int = 0
    flags: int = 0
    session_id: int = 0
    map_reset_counter: int = 0
    imu_initialized: bool = True
    wire_seq: int = 0

class SupervisorModel:
    def __init__(self, config=None, primary_fresh=False):
        if config is None:
            config = json.loads((Path(__file__).parents[1] / "config.json").read_text())
        self.c = config
        self.state, self.reason = "INIT", "NONE"
        # Supplied by the existing primary supervisor path; this optional
        # message is never allowed to create primary freshness or motion authority.
        self.primary_fresh = bool(primary_fresh)
        self.last_hb = self.last_health = None
        self.last_age = 0
        self.last_age_valid = False
        self.latched = False
        self.session_id = None
        self.last_wire_seq = None
        self.map_reset_counter = None
        self.last_now = None
        self.last_report_fresh = False
        self.healthy_since = None

    def _transition(self, state, reason):
        if self.latched and state != "FAILSAFE": return
        if self.state != state or self.reason != reason:
            self.state, self.reason = state, reason

    def update(self, now_us: int, event: Event | None = None, primary_fresh: bool | None = None):
        if primary_fresh is not None:
            self.primary_fresh = bool(primary_fresh)
        if self.last_now is not None and now_us < self.last_now:
            self.latched = True; self.healthy_since = None
            self._transition("FAILSAFE", "CLOCK_REGRESSION"); return self.state
        self.last_now = now_us
        # Existing deadlines expire before a newly received frame is considered.
        if (self.last_hb is not None and now_us - self.last_hb >= self.c["heartbeat_timeout_us"]
                and not self.latched):
            self.latched = True; self.healthy_since = None
            self._transition("FAILSAFE", "HEARTBEAT_TIMEOUT")
        if self.last_health is not None:
            silence = now_us - self.last_health
            # UINT32_MAX with validity flags clear means unknown, not measured
            # stale age. Receipt/heartbeat watchdogs still expire independently.
            age = (min(0xFFFFFFFFFFFFFFFF, self.last_age + silence + self.c["transit_bound_us"])
                   if self.last_age_valid else None)
            age_timeout = age is not None and age >= self.c["pose_age_timeout_us"]
            if (silence >= self.c["receipt_silence_timeout_us"] or age_timeout) and not self.latched:
                self.latched = True
                self.healthy_since = None
                self._transition("FAILSAFE", "SOURCE_AGE_TIMEOUT" if age_timeout else "RESULT_TIMEOUT")
            elif silence >= self.c["receipt_silence_hold_us"]:
                self._transition("HOLD", "RESULT_SILENCE"); self.healthy_since = None
            elif age is not None and age >= self.c["pose_age_hold_us"]:
                self._transition("HOLD", "SOURCE_AGE_HOLD"); self.healthy_since = None
        if event is not None:
            if not 0 <= event.wire_seq <= 0xFFFF or event.received_us > now_us:
                return self.state
            if event.kind == "heartbeat":
                if self.session_id is None or self.session_id != event.session_id:
                    self.session_id = event.session_id
                    self.last_hb = None; self.last_health = None
                    self.last_age_valid = False
                    self.map_reset_counter = None
                    self.last_report_fresh = False
                    self.healthy_since = None
                    self.last_wire_seq = None
                    if self.state != "INIT" and not self.latched: self._transition("INIT", "SESSION_CHANGE")
            if self.last_wire_seq is not None:
                delta = (event.wire_seq - self.last_wire_seq) & 0xFFFF
                if delta == 0 or delta >= 0x8000:
                    return self.state
            self.last_wire_seq = event.wire_seq
            if event.kind == "heartbeat":
                self.last_hb = event.received_us
            elif event.kind == "health":
                reset_changed = self.map_reset_counter is not None and event.map_reset_counter != self.map_reset_counter
                self.map_reset_counter = event.map_reset_counter
                self.last_health = event.received_us
                self.last_age = min(U32_MAX, event.age_at_send_us)
                self.last_age_valid = (event.flags & (FLAG_AGE_VALID | FLAG_MAPPING_VALID) ==
                                       (FLAG_AGE_VALID | FLAG_MAPPING_VALID))
                self.last_report_fresh = False
                if event.tracking_state == TRACKING_STATES["LOST"]:
                    self.latched = True; self.healthy_since = None
                    self._transition("FAILSAFE", "SOURCE_AGE_TIMEOUT"); return self.state
                if not self.last_age_valid:
                    self._transition("HOLD", "SOURCE_AGE_HOLD"); self.healthy_since = None; return self.state
                conservative_age = self.last_age + self.c["transit_bound_us"]
                if conservative_age >= self.c["pose_age_timeout_us"]:
                    self.latched = True; self.healthy_since = None
                    self._transition("FAILSAFE", "SOURCE_AGE_TIMEOUT"); return self.state
                if conservative_age >= self.c["pose_age_hold_us"] or event.flags & 4 or event.imu_initialized != 1 or reset_changed or event.tracking_state not in (TRACKING_STATES["OK"], TRACKING_STATES["OK_KLT"]):
                    self._transition("HOLD", "SOURCE_AGE_HOLD")
                    self.healthy_since = None; return self.state
                self.last_report_fresh = True
        if self.last_hb is not None and self.last_health is not None:
            silence = max(0, now_us - self.last_health)
            age = (min(0xFFFFFFFFFFFFFFFF, self.last_age + silence + self.c["transit_bound_us"])
                   if self.last_age_valid else None)
            healthy = (self.primary_fresh and self.last_report_fresh and age is not None and age < self.c["pose_age_hold_us"] and
                       silence < self.c["receipt_silence_hold_us"] and
                       now_us - self.last_hb < self.c["heartbeat_timeout_us"])
            if healthy:
                if self.healthy_since is None: self.healthy_since = now_us
                if not self.latched: self._transition("FRESH", "READY")
            else:
                self.healthy_since = None
                if not self.latched and self.state == "FRESH":
                    self._transition("HOLD", "SOURCE_AGE_HOLD" if age is None or age >= self.c["pose_age_hold_us"] else "RESULT_SILENCE")
        return self.state

    def rearm(self, now_us: int):
        """Explicit local action after the configured healthy interval; returns to safe INIT."""
        if not self.latched: return False
        if self.last_now is not None and now_us < self.last_now: return False
        self.update(now_us)
        if (self.healthy_since is None or now_us < self.healthy_since or
                now_us - self.healthy_since < self.c["rearm_healthy_us"]): return False
        self.latched = False; self.last_hb = self.last_health = None
        self.last_age_valid = False
        self.last_report_fresh = False; self.map_reset_counter = None
        self.healthy_since = None; self._transition("INIT", "REARM")
        return True
