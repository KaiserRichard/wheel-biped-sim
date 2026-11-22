"""Synthetic fixtures only; no hardware timing or real recording validation."""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from clockmap import (
    AVAILABLE, UNAVAILABLE, ClockDomainError, ClockMap, ClockStepError,
    Exchange, InvalidClockMap, SessionError, Stamp, TickUnwrapper, source_age,
)


def exchange(a_ns, *, rate=1.0, offset=5_000_000_000, forward=200_000,
             reverse=200_000, processing=50_000, session_a="pi1", session_b="mcu1"):
    """True B = rate*A + offset; delays/processing here are A-clock ns."""
    a = lambda value: Stamp(value, "pi_steady", session_a)
    b = lambda value: Stamp(round(rate * value) + offset, "mcu_steady", session_b)
    return Exchange(a(a_ns), b(a_ns + forward),
                    b(a_ns + forward + processing),
                    a(a_ns + forward + processing + reverse))


def fitted(**kwargs):
    mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1")
    for i in range(12):
        mapping.add_exchange(exchange(i * 1_000_000_000, **kwargs))
    return mapping


class StampTests(unittest.TestCase):
    def test_same_domain_durations_and_shifts(self):
        stamp = Stamp(100, "steady", "boot")
        self.assertEqual((stamp + 25) - stamp, 25)
        self.assertEqual(stamp - 25, Stamp(75, "steady", "boot"))
        self.assertLess(stamp, stamp + 1)

    def test_cross_domain_arithmetic_and_order_refused(self):
        a, b = Stamp(1, "steady", "boot"), Stamp(1, "utc", "boot")
        for operation in (lambda: a - b, lambda: a + b, lambda: a < b):
            with self.assertRaises(ClockDomainError):
                operation()

    def test_cross_session_refused(self):
        with self.assertRaises(SessionError):
            Stamp(1, "steady", "new") - Stamp(1, "steady", "old")

    def test_float_stamps_durations_and_missing_session_refused(self):
        for value in (1.0, True):
            with self.assertRaises(TypeError):
                Stamp(value, "steady", "boot")
            with self.assertRaises(TypeError):
                Stamp(1, "steady", "boot") + value
        with self.assertRaises(TypeError):
            Stamp(1, "steady", "")


class MappingTests(unittest.TestCase):
    def test_offset_and_drift_recovered(self):
        mapping = fitted(rate=1.000075)
        self.assertAlmostEqual(mapping.drift_ppm, 75, places=4)
        self.assertAlmostEqual(mapping.offset_at_reference_ns, 5_000_000_017, delta=2)
        stamp = Stamp(4_000_000_123, "pi_steady", "pi1")
        mapped = mapping.convert(stamp)
        self.assertLessEqual(abs(mapped.stamp.value_ns -
                                 (round(1.000075 * stamp.value_ns) + 5_000_000_000)), 2)
        self.assertEqual(mapped.stamp.domain, "mcu_steady")
        self.assertEqual(mapped.stamp.session, "mcu1")
        self.assertEqual(mapped.mapping_revision, mapping.mapping_revision)

    def test_inverse_recovers_epoch_and_units(self):
        mapping = fitted(rate=1.0002)
        original = Stamp(4_000_000_005, "pi_steady", "pi1")
        inverse = mapping.convert(mapping.convert(original).stamp)
        self.assertLessEqual(abs(inverse.stamp - original), 1)
        self.assertGreater(inverse.uncertainty_ns, 0)

    def test_constant_asymmetry_bias_is_bounded(self):
        mapping = fitted(forward=900_000, reverse=100_000)
        mapped = mapping.convert(Stamp(6_000_000_000, "pi_steady", "pi1"))
        bias = mapped.stamp.value_ns - 11_000_000_000
        self.assertAlmostEqual(bias, 400_000, delta=2)
        self.assertGreaterEqual(mapped.uncertainty_ns, 500_000)
        self.assertLessEqual(abs(bias), mapped.uncertainty_ns)

    def test_time_varying_asymmetry_and_extrapolation_bounded(self):
        mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1")
        for i in range(12):
            mapping.add_exchange(exchange(i * 1_000_000_000, rate=1.00004,
                                          forward=100_000 + 70_000 * i,
                                          reverse=900_000 - 70_000 * i))
        for a_ns in (-10_000_000_000, 0, 6_000_000_000, 30_000_000_000):
            mapped = mapping.convert(Stamp(a_ns, "pi_steady", "pi1"))
            truth = round(1.00004 * a_ns) + 5_000_000_000
            self.assertLessEqual(abs(mapped.stamp.value_ns - truth), mapped.uncertainty_ns)
            inverse = mapping.convert(Stamp(truth, "mcu_steady", "mcu1"))
            self.assertLessEqual(abs(inverse.stamp.value_ns - a_ns), inverse.uncertainty_ns)

    def test_minimum_rtt_samples_preferred(self):
        mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1")
        for i in range(16):
            mapping.add_exchange(exchange(i * 1_000_000_000, **(
                {"forward": 10_000, "reverse": 10_000} if i % 2 == 0 else
                {"forward": 20_000_000, "reverse": 10_000})))
        mapped = mapping.convert(Stamp(7_000_000_000, "pi_steady", "pi1"))
        self.assertLess(abs(mapped.stamp.value_ns - 12_000_000_000), 100)

    def test_window_bounded_and_revisions_advance(self):
        mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1", window_size=4)
        revisions = []
        for i in range(10):
            mapping.add_exchange(exchange(i * 1_000_000_000))
            if mapping.valid:
                revisions.append(mapping.mapping_revision)
        self.assertEqual(mapping.sample_count, 4)
        self.assertEqual(revisions, sorted(set(revisions)))
        self.assertGreater(revisions[-1], revisions[0])

    def test_insufficient_samples_unavailable(self):
        mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1")
        for i in range(2):
            self.assertFalse(mapping.add_exchange(exchange(i * 1_000_000_000)))
        with self.assertRaises(InvalidClockMap):
            mapping.convert(Stamp(0, "pi_steady", "pi1"))

    def test_session_change_invalidates_until_explicit_bind(self):
        mapping = fitted()
        revision = mapping.mapping_revision
        with self.assertRaises(SessionError):
            mapping.add_exchange(exchange(12_000_000_000, session_b="mcu2"))
        self.assertFalse(mapping.valid)
        self.assertGreater(mapping.mapping_revision, revision)
        with self.assertRaises(InvalidClockMap):
            mapping.add_exchange(exchange(13_000_000_000))
        mapping.bind("pi1", "mcu2")
        for i in range(3):
            mapping.add_exchange(exchange(i * 1_000_000_000, session_b="mcu2"))
        self.assertTrue(mapping.valid)
        self.assertEqual(mapping.convert(Stamp(0, "pi_steady", "pi1")).stamp.session, "mcu2")

    def test_old_session_conversion_invalidates(self):
        mapping = fitted()
        with self.assertRaises(SessionError):
            mapping.convert(Stamp(0, "pi_steady", "pi_old"))
        self.assertFalse(mapping.valid)

    def test_forward_and_backward_steps_invalidate(self):
        for jump in (50_000_000, -50_000_000, -5_000_000_000):
            with self.subTest(jump=jump):
                mapping = fitted()
                with self.assertRaises(ClockStepError):
                    mapping.add_exchange(exchange(12_000_000_000, offset=5_000_000_000 + jump))
                self.assertFalse(mapping.valid)
                self.assertEqual(mapping.sample_count, 0)
                with self.assertRaises(InvalidClockMap):
                    mapping.add_exchange(exchange(13_000_000_000))

    def test_long_rtt_alone_is_not_a_clock_step(self):
        mapping = fitted()
        mapping.add_exchange(exchange(12_000_000_000, forward=60_000_000, reverse=1_000))
        self.assertTrue(mapping.valid)

    def test_source_clock_reversal_invalidates(self):
        mapping = fitted()
        with self.assertRaises(ClockStepError):
            mapping.add_exchange(exchange(1_000_000_000))
        self.assertFalse(mapping.valid)

    def test_duplicate_exchange_refused(self):
        mapping = fitted()
        revision = mapping.mapping_revision
        with self.assertRaises(ValueError):
            mapping.add_exchange(exchange(11_000_000_000))
        self.assertEqual(mapping.mapping_revision, revision)

    def test_large_integer_epochs_preserve_nanoseconds(self):
        base = 1_750_000_000_000_000_003
        mapping = ClockMap("utc", "steady", "run", "boot")
        for i in range(6):
            a_ns = base + i * 1_000_000_000
            a = lambda n: Stamp(n, "utc", "run")
            b = lambda n: Stamp(n - base + 3_000_000_001, "steady", "boot")
            mapping.add_exchange(Exchange(a(a_ns), b(a_ns + 100),
                                           b(a_ns + 200), a(a_ns + 300)))
        result = mapping.convert(Stamp(base + 123, "utc", "run"))
        self.assertEqual(result.stamp.value_ns, 3_000_000_124)


class AgeTests(unittest.TestCase):
    def test_r2_upper_bound_and_frozen_epoch_age(self):
        mapping = fitted()
        source = Stamp(10_000_000_000, "pi_steady", "pi1")
        first = source_age(Stamp(15_050_000_000, "mcu_steady", "mcu1"), source, mapping)
        later = source_age(Stamp(15_150_000_000, "mcu_steady", "mcu1"), source, mapping)
        self.assertEqual(first.status, AVAILABLE)
        self.assertEqual(first.upper_bound_ns, 50_000_000 + first.mapped_epoch.uncertainty_ns)
        self.assertEqual(later.upper_bound_ns - first.upper_bound_ns, 100_000_000)

    def test_no_map_invalid_map_wrong_domains_and_boot_are_unavailable(self):
        now = Stamp(15_000_000_000, "mcu_steady", "mcu1")
        source = Stamp(10_000_000_000, "pi_steady", "pi1")
        self.assertEqual(source_age(now, source, None).status, UNAVAILABLE)
        mapping = fitted()
        self.assertEqual(source_age(Stamp(1, "utc", "run"), source, mapping).status, UNAVAILABLE)
        mapping.invalidate("test fault")
        self.assertEqual(source_age(now, source, mapping).status, UNAVAILABLE)
        mapping = fitted()
        self.assertEqual(source_age(Stamp(1, "mcu_steady", "mcu2"), source, mapping).status, UNAVAILABLE)
        self.assertFalse(mapping.valid)

    def test_certain_future_unavailable(self):
        result = source_age(Stamp(14_000_000_000, "mcu_steady", "mcu1"),
                            Stamp(10_000_000_000, "pi_steady", "pi1"), fitted())
        self.assertEqual(result.status, UNAVAILABLE)


class UnwrapTests(unittest.TestCase):
    def test_wrap_and_equal_ticks(self):
        timer = TickUnwrapper("mcu_steady", "boot")
        first = timer.unwrap(2**32 - 2, "boot")
        second = timer.unwrap(3, "boot")
        self.assertEqual(second - first, 5000)
        self.assertEqual(timer.unwrap(3, "boot"), second)

    def test_reversal_and_half_range_latch(self):
        for next_tick in (99, 100 + 2**31):
            timer = TickUnwrapper("mcu_steady", "boot")
            timer.unwrap(100, "boot")
            with self.assertRaises(ClockStepError):
                timer.unwrap(next_tick, "boot")
            with self.assertRaises(InvalidClockMap):
                timer.unwrap(101, "boot")

    def test_reboot_needs_reset(self):
        timer = TickUnwrapper("mcu_steady", "boot1")
        timer.unwrap(200, "boot1")
        with self.assertRaises(SessionError):
            timer.unwrap(0, "boot2")
        timer.reset("boot2")
        self.assertEqual(timer.unwrap(1, "boot2"), Stamp(1000, "mcu_steady", "boot2"))

    def test_uint32_validation(self):
        for ticks in (-1, 2**32):
            with self.assertRaises(ValueError):
                TickUnwrapper("mcu", "boot").unwrap(ticks, "boot")

    def test_map_fit_across_mcu_wrap(self):
        timer = TickUnwrapper("mcu_steady", "mcu1")
        mapping = ClockMap("pi_steady", "mcu_steady", "pi1", "mcu1")
        base_us = 2**32 - 2_000_000
        for i in range(6):
            a_ns = i * 1_000_000_000
            t2 = timer.unwrap((base_us + i * 1_000_000 + 200) % 2**32, "mcu1")
            t3 = timer.unwrap((base_us + i * 1_000_000 + 250) % 2**32, "mcu1")
            mapping.add_exchange(Exchange(Stamp(a_ns, "pi_steady", "pi1"), t2, t3,
                                           Stamp(a_ns + 450_000, "pi_steady", "pi1")))
        mapped = mapping.convert(Stamp(5_000_000_000, "pi_steady", "pi1"))
        self.assertEqual(mapped.stamp.value_ns, (base_us + 5_000_000) * 1000)


if __name__ == "__main__":
    unittest.main()
