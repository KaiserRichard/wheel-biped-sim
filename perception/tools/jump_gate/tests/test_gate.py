from dataclasses import replace
import unittest

from tools.jump_gate.gate import (
    ASSUMED_CONTACT_DWELL_S, ASSUMED_PLANE_DWELL_S, ASSUMED_STABLE_RATE_DEG_S,
    ASSUMED_TELEMETRY_MAX_AGE_S, ASSUMED_WOBBLE_LIMIT_DEG_S, JumpGate, Phase, Tick,
)
from tools.obstacle_decision.decide import Advisory


def clean(t, **changes):
    return replace(Tick(t, 3, False, False, 0.0, 0.0, 0.0, True,
                        Advisory.JUMP_CANDIDATE, 0.8), **changes)


class GateTests(unittest.TestCase):
    def stand(self):
        gate = JumpGate()
        for t in (0.0, 0.1, 0.2):
            result = gate.update(clean(t))
        self.assertEqual(result.phase, Phase.STAND)
        self.assertEqual(result.advisory, Advisory.JUMP_CANDIDATE)
        return gate

    def assert_inhibited(self, output, phase=None):
        self.assertEqual(output.advisory, Advisory.UNCERTAIN)
        self.assertEqual(output.confidence, 0.0)
        if phase:
            self.assertEqual(output.phase, phase)

    def test_startup_requires_all_dwell_conditions_and_preserves_confidence(self):
        gate = JumpGate()
        self.assert_inhibited(gate.update(clean(0)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(ASSUMED_CONTACT_DWELL_S)), Phase.SETTLE)
        result = gate.update(clean(ASSUMED_PLANE_DWELL_S))
        self.assertEqual(result.phase, Phase.STAND)
        self.assertEqual(result.confidence, 0.8)
        self.assertEqual(result.advisory, Advisory.JUMP_CANDIDATE)

    def test_crouch_thrust_flight_land_settle_never_jump(self):
        gate = self.stand()
        samples = (
            (clean(.3, jump_status=0), Phase.CROUCH),
            (clean(.4, jump_status=1), Phase.CROUCH),
            (clean(.5, jump_status=2, airborne_l=True, airborne_r=True), Phase.FLIGHT),
            (clean(.6, jump_status=2, airborne_l=False, airborne_r=True), Phase.LAND),
            (clean(.7), Phase.SETTLE),
            (clean(.8), Phase.SETTLE),
        )
        for tick, expected in samples:
            with self.subTest(t=tick.t):
                self.assert_inhibited(gate.update(tick), expected)
        result = gate.update(clean(.9))
        self.assertEqual(result.advisory, Advisory.JUMP_CANDIDATE)
        self.assertEqual(result.phase, Phase.STAND)

    def test_stale_telemetry_overrides_even_flight_and_restarts_dwell(self):
        gate = self.stand()
        self.assert_inhibited(gate.update(clean(.3, telemetry_age_s=ASSUMED_TELEMETRY_MAX_AGE_S + .001,
                                                airborne_l=True, airborne_r=True)), Phase.UNKNOWN)
        self.assert_inhibited(gate.update(clean(.4)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.5)), Phase.SETTLE)
        self.assertEqual(gate.update(clean(.6)).advisory, Advisory.JUMP_CANDIDATE)

    def test_clean_recovery_to_each_raw_advisory_within_dwell(self):
        for raw in (Advisory.NONE, Advisory.JUMP_CANDIDATE, Advisory.STOP_CANDIDATE):
            with self.subTest(raw=raw):
                gate = self.stand()
                self.assert_inhibited(gate.update(clean(.3, plane_valid=False)))
                for t in (.4, .5):
                    self.assert_inhibited(gate.update(clean(t, raw_advisory=raw)))
                result = gate.update(clean(.6, raw_advisory=raw))
                self.assertEqual(result.advisory, raw)
                self.assertEqual(result.confidence, .8)

    def test_bounce_resets_first_landing_history(self):
        gate = self.stand()
        self.assert_inhibited(gate.update(clean(.3, airborne_l=True, airborne_r=True)), Phase.FLIGHT)
        self.assert_inhibited(gate.update(clean(.4)), Phase.LAND)
        self.assert_inhibited(gate.update(clean(.5)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.6, airborne_l=True, airborne_r=True)), Phase.FLIGHT)
        self.assert_inhibited(gate.update(clean(.7)), Phase.LAND)
        self.assert_inhibited(gate.update(clean(.8)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.9)), Phase.SETTLE)
        self.assertEqual(gate.update(clean(1.0)).phase, Phase.STAND)

    def test_abort_reset_returns_through_settle_not_stand(self):
        gate = self.stand()
        self.assert_inhibited(gate.update(clean(.3, jump_status=0)), Phase.CROUCH)
        self.assert_inhibited(gate.update(clean(.4, jump_status=3)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.5)), Phase.SETTLE)
        self.assertEqual(gate.update(clean(.6)).phase, Phase.STAND)

    def test_status_reset_in_flight_does_not_prove_landing(self):
        gate = self.stand()
        for t in (.3, .4, .5):
            self.assert_inhibited(gate.update(clean(t, jump_status=3, airborne_l=True,
                                                   airborne_r=True)), Phase.FLIGHT)

    def test_retract_is_never_contact_or_inactive_proof(self):
        gate = JumpGate()
        for t in (0, .1, .2, .3):
            self.assert_inhibited(gate.update(clean(t, jump_status=2)))
        self.assertEqual(gate.update(clean(.4)).phase, Phase.SETTLE)
        self.assertEqual(gate.update(clean(.5)).phase, Phase.SETTLE)
        self.assertEqual(gate.update(clean(.6)).phase, Phase.STAND)

    def test_unknown_inputs_never_promote_and_need_new_dwell(self):
        for changes in (
            {"jump_status": None}, {"jump_status": 9}, {"jump_status": True},
            {"airborne_l": None}, {"airborne_l": 1}, {"telemetry_age_s": None},
            {"telemetry_age_s": float("nan")}, {"telemetry_age_s": -.1},
            {"pitch_rate_deg_s": None}, {"roll_rate_deg_s": float("inf")},
            {"airborne_l": True}, {"raw_advisory": None}, {"raw_advisory": True},
            {"raw_confidence": float("nan")}, {"raw_confidence": 1.1},
        ):
            with self.subTest(changes=changes):
                gate = self.stand()
                self.assert_inhibited(gate.update(clean(.3, **changes)), Phase.UNKNOWN)
                self.assert_inhibited(gate.update(clean(.4)), Phase.SETTLE)
                self.assert_inhibited(gate.update(clean(.5)), Phase.SETTLE)
                self.assertEqual(gate.update(clean(.6)).phase, Phase.STAND)

    def test_unknown_raw_advisory_remains_uncertain(self):
        gate = self.stand()
        result = gate.update(clean(.3, raw_advisory=Advisory.UNCERTAIN))
        self.assert_inhibited(result, Phase.STAND)
        self.assertEqual(result.reason, "raw_uncertain_no_promotion")

    def test_clock_reset_drops_temporal_history(self):
        gate = self.stand()
        self.assert_inhibited(gate.update(clean(.1)), Phase.UNKNOWN)
        self.assert_inhibited(gate.update(clean(.2)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.3)), Phase.SETTLE)
        self.assertEqual(gate.update(clean(.4)).phase, Phase.STAND)

    def test_settle_attitude_limit_is_strict_and_checks_both_axes(self):
        for axis in ("pitch_rate_deg_s", "roll_rate_deg_s"):
            for sign in (-1, 1):
                gate = self.stand()
                self.assert_inhibited(gate.update(clean(.3, jump_status=0)), Phase.CROUCH)
                for t in (.4, .5, .6):
                    self.assert_inhibited(gate.update(clean(
                        t, **{axis: sign * ASSUMED_STABLE_RATE_DEG_S})), Phase.SETTLE)
                for t in (.7, .8):
                    self.assert_inhibited(gate.update(clean(t)), Phase.SETTLE)
                self.assertEqual(gate.update(clean(.9)).phase, Phase.STAND)

    def test_stand_wobble_keeps_stop_and_scales_confidence_on_both_axes(self):
        for axis in ("pitch_rate_deg_s", "roll_rate_deg_s"):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    gate = self.stand()
                    for t in (.3, .4, .5):
                        result = gate.update(clean(t, raw_advisory=Advisory.STOP_CANDIDATE,
                                                   **{axis: sign * 15.0}))
                        self.assertEqual(result.phase, Phase.STAND)
                        self.assertEqual(result.advisory, Advisory.STOP_CANDIDATE)
                        self.assertAlmostEqual(result.confidence, .8 * (1 - 15 / 60))
                        self.assertEqual(result.reason, "wobble_confidence_scaled")
                    self.assertEqual(gate.update(clean(.6)).confidence, .8)

    def test_stand_rate_above_wobble_limit_inhibits_and_requires_settle(self):
        for axis in ("pitch_rate_deg_s", "roll_rate_deg_s"):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    gate = self.stand()
                    result = gate.update(clean(.3, raw_advisory=Advisory.STOP_CANDIDATE,
                                               **{axis: sign * 80.0}))
                    self.assert_inhibited(result, Phase.SETTLE)
                    self.assertEqual(result.reason, "unstable_attitude")
                    self.assert_inhibited(gate.update(clean(.4, **{axis: sign * 15.0})),
                                          Phase.SETTLE)
                    for t in (.5, .6):
                        self.assert_inhibited(gate.update(clean(t)), Phase.SETTLE)
                    self.assertEqual(gate.update(clean(.7)).phase, Phase.STAND)

    def test_stand_confidence_uses_max_absolute_rate_and_clamps_at_limit(self):
        for raw in (Advisory.NONE, Advisory.JUMP_CANDIDATE, Advisory.STOP_CANDIDATE):
            with self.subTest(raw=raw):
                gate = self.stand()
                result = gate.update(clean(.3, pitch_rate_deg_s=10, roll_rate_deg_s=-15,
                                           raw_advisory=raw))
                self.assertEqual(result.advisory, raw)
                self.assertAlmostEqual(result.confidence, .6)
                result = gate.update(clean(.4, pitch_rate_deg_s=ASSUMED_WOBBLE_LIMIT_DEG_S,
                                           raw_advisory=raw))
                self.assertEqual(result.phase, Phase.STAND)
                self.assertEqual(result.advisory, raw)
                self.assertEqual(result.confidence, 0)

    def test_wobble_never_promotes_raw_uncertain(self):
        gate = self.stand()
        result = gate.update(clean(.3, pitch_rate_deg_s=15, raw_advisory=Advisory.UNCERTAIN))
        self.assert_inhibited(result, Phase.STAND)
        self.assertIn("wobble_confidence_scaled", result.reason)

    def test_repeated_plane_never_completes_dwell_and_stale_plane_inhibits(self):
        gate = JumpGate()
        for k in range(30):
            self.assert_inhibited(gate.update(clean(k / 100, plane_stamp_s=0)), Phase.SETTLE)
        gate = self.stand()
        result = gate.update(clean(.6, plane_stamp_s=.2))
        self.assert_inhibited(result, Phase.SETTLE)
        self.assertEqual(result.reason, "invalid_or_stale_plane")

    def test_geometry_clock_reset_is_unknown(self):
        gate = self.stand()
        self.assert_inhibited(gate.update(clean(.3, plane_stamp_s=.1)), Phase.UNKNOWN)
        self.assert_inhibited(gate.update(clean(.4, plane_stamp_s=.5)), Phase.UNKNOWN)

    def test_three_fresh_planes_may_span_more_than_the_max_held_age(self):
        gate = JumpGate()
        self.assert_inhibited(gate.update(clean(0)), Phase.SETTLE)
        self.assert_inhibited(gate.update(clean(.16)), Phase.SETTLE)
        self.assertEqual(gate.update(clean(.32)).phase, Phase.STAND)

    def test_stop_history_is_not_republished_during_jump(self):
        gate = self.stand()
        self.assertEqual(gate.update(clean(.3, raw_advisory=Advisory.STOP_CANDIDATE)).advisory,
                         Advisory.STOP_CANDIDATE)
        self.assert_inhibited(gate.update(clean(.4, jump_status=0,
                                                raw_advisory=Advisory.STOP_CANDIDATE)), Phase.CROUCH)


if __name__ == "__main__":
    unittest.main()
