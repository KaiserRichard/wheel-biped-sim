import math
import unittest

from sim.twin3d_cmd.commands import Command, CommandModel, HoldDebounce, rate_limit


class CommandTests(unittest.TestCase):
    def test_limits_units_and_invalid_values(self):
        self.assertEqual(Command(5).v_ref, 0.6)
        self.assertEqual(Command(-5).v_ref, -0.6)
        self.assertEqual(Command(yaw_rate=2).yaw_rate, 2)
        for kw in ({'v_ref': math.nan}, {'v_ref': '1'}, {'v_ref': True},
                   {'yaw_rate': math.inf}, {'jump': 1}, {'src': ''}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                Command(**kw)

    def test_rate_bound_in_both_directions(self):
        self.assertAlmostEqual(rate_limit(0, .6, .1), .06)
        self.assertAlmostEqual(rate_limit(.3, -.6, .1), .24)
        self.assertAlmostEqual(rate_limit(.3, .31, .1), .31)
        self.assertEqual(rate_limit(.3, 0, 0), .3)
        for dt, accel in ((-1, 1), (1, 0), (math.nan, 1)):
            with self.assertRaises(ValueError):
                rate_limit(0, 1, dt, accel)

    def test_watchdog_boundary_and_ramp(self):
        model = CommandModel()
        self.assertTrue(model.tick(0).watchdog)
        for i in range(1, 11):
            model.receive(Command(.3), i*.1)
            out = model.tick(i*.1)
        self.assertAlmostEqual(out.v_ref, .3)
        self.assertFalse(model.tick(1.499).watchdog)
        out = model.tick(1.5)
        self.assertTrue(out.watchdog)
        self.assertGreater(out.v_ref, 0)  # Watchdog brakes with the same slew bound.
        self.assertAlmostEqual(model.tick(2).v_ref, 0)
        model.receive(Command(-.3, src='recovered'), 2.1)
        self.assertFalse(model.tick(2.1).watchdog)

    def test_last_writer_and_one_shot_edge_survives_heartbeat(self):
        model = CommandModel()
        model.receive(Command(.3, 1, True, 'udp'), 0)
        model.receive(Command(-.2, 2, False, 'keyboard'), 0)
        out = model.tick(0)
        self.assertEqual((out.target_v, out.yaw_rate, out.src), (-.2, 2, 'keyboard'))
        self.assertTrue(out.jump)
        self.assertFalse(model.tick(.001).jump)

    def test_active_requests_and_expired_edges_never_queue(self):
        model = CommandModel()
        model.receive(Command(jump=True), 0, active=True)
        self.assertFalse(model.tick(0, True).jump)
        self.assertFalse(model.tick(.1, False).jump)
        self.assertEqual(model.ignored_jumps, 1)
        model.receive(Command(jump=True), .2)
        self.assertFalse(model.tick(.7).jump)
        model.receive(Command(jump=True), .8)
        self.assertFalse(model.tick(.8, active=True).jump)
        self.assertFalse(model.tick(.9).jump)

    def test_duplicate_pending_request_does_not_cancel_first(self):
        model = CommandModel()
        model.receive(Command(jump=True), 0)
        model.receive(Command(jump=True), 0)
        self.assertTrue(model.tick(0).jump)
        self.assertFalse(model.tick(.1).jump)
        self.assertEqual(model.ignored_jumps, 1)

    def test_monotonic_clock_required(self):
        model = CommandModel()
        model.tick(2)
        with self.assertRaises(ValueError):
            model.receive(Command(), 1)
        with self.assertRaises(ValueError):
            model.tick(1)

    def test_source_hold_strictly_greater_than_ten_and_release(self):
        hold = HoldDebounce()
        for _ in range(10):
            self.assertFalse(hold.poll(True))
        self.assertTrue(hold.poll(True))
        for _ in range(20):
            self.assertFalse(hold.poll(True))
        hold.poll(False)
        self.assertFalse(hold.poll(True))

    def test_hold_qualifying_during_jump_is_consumed(self):
        hold = HoldDebounce()
        for _ in range(11):
            self.assertFalse(hold.poll(True, active=True))
        self.assertFalse(hold.poll(True, active=False))
        hold.poll(False)
        for _ in range(10):
            self.assertFalse(hold.poll(True))
        self.assertTrue(hold.poll(True))
