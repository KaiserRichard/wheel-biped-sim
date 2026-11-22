"""Drive the passive-viewer logic with actual physics, without opening a GUI."""

import math
import unittest
from unittest.mock import patch

import mujoco

from sim.twin3d_cmd.teleop_viewer import (
    Keyboard, TeleopTwin, KEY_UP, KEY_DOWN, KEY_JUMP, KEY_RESET, KEY_QUIT,
    KEY_HELP, SETTLE_STABLE_S, overlay,
)


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.twin, self.keyboard = TeleopTwin(), Keyboard()

    def press(self, key):
        self.keyboard.callback(key)
        return self.keyboard.poll(self.twin, float(self.twin.data.time))

    def step(self):
        now = float(self.twin.data.time)
        self.keyboard.poll(self.twin, now)
        return self.twin.step(now)

    def until(self, predicate, seconds=5):
        for _ in range(round(seconds/self.twin.plant.model.opt.timestep)):
            row = self.step()
            if predicate(row):
                return row
        self.fail(f'condition not reached within {seconds}s: {row}')

    def test_velocity_then_two_complete_jumps_without_reset(self):
        self.press(KEY_UP)
        self.assertAlmostEqual(self.twin.commands.command.v_ref, .05)
        self.step()
        self.press(KEY_DOWN)
        self.assertAlmostEqual(self.twin.commands.command.v_ref, 0)
        self.until(lambda row: row['t'] >= .5)
        initial_data = self.twin.data

        for count in (1, 2):
            self.press(KEY_JUMP)
            row = self.step()
            self.assertEqual((row['phase'], row['jump_count']), ('CROUCH', count))
            self.assertTrue(row['jump_edge'])
            self.press(KEY_JUMP)  # Consumed in flight, never queued on re-arm.
            phases = set()
            flight_seen = False
            while row['phase'] != 'STAND':
                phases.add(row['phase'])
                flight_seen |= not (row['contact_left'] or row['contact_right'])
                stable_since = self.twin.stable_since
                row = self.step()
                self.assertLess(row['t'], count*5)
                self.assertFalse(row['jump_locked'])
            self.assertTrue(flight_seen)
            self.assertEqual(phases, {'CROUCH', 'THRUST', 'RETRACT', 'SETTLE'})
            self.assertTrue(row['contact_left'] and row['contact_right'])
            self.assertIsNotNone(stable_since)
            self.assertGreaterEqual(row['t']-stable_since, SETTLE_STABLE_S)
            self.assertEqual(row['jump_count'], count)
            self.assertFalse(self.twin.controller.jump.airborne_seen)
            for _ in range(100):
                self.assertEqual(self.step()['phase'], 'STAND')
        self.assertIs(self.twin.data, initial_data)
        self.assertEqual(self.twin.commands.ignored_jumps, 2)

    def test_unstable_landing_restarts_continuous_simulation_timer(self):
        self.press(KEY_JUMP)
        self.until(lambda row: self.twin.stable_since is not None)
        start = float(self.twin.data.time)
        self.until(lambda row: row['t'] >= start+.3)
        contact = self.twin.plant.contacts(self.twin.data)
        with patch.object(self.twin.plant, 'contacts', return_value={**contact, 'right': False}):
            self.assertEqual(self.step()['phase'], 'SETTLE')
        self.assertIsNone(self.twin.stable_since)
        row = self.until(lambda row: self.twin.stable_since is not None)
        restarted = self.twin.stable_since
        while row['t'] < restarted+SETTLE_STABLE_S-.002:
            # A large wall-time advance cannot shorten the simulation dwell.
            row = self.twin.step(100+float(self.twin.data.time))
            self.assertEqual(row['phase'], 'SETTLE')
        row = self.twin.step(100+float(self.twin.data.time))
        while row['phase'] == 'SETTLE':
            row = self.twin.step(100+float(self.twin.data.time))
            self.assertLess(row['t'], restarted+SETTLE_STABLE_S+.01)
        self.assertEqual(row['phase'], 'STAND')

    def test_fall_latches_jump_lockout_until_home_reset(self):
        twin = self.twin
        twin.data.qpos[twin.plant.q['pitch']] = math.radians(35)
        mujoco.mj_forward(twin.plant.model, twin.data)
        self.assertTrue(self.step()['jump_locked'])
        # Recover physical pose without resetting the failure latch.
        twin.data.qpos[:] = twin.plant.data().qpos
        twin.data.qvel[:] = 0
        mujoco.mj_forward(twin.plant.model, twin.data)
        self.until(lambda row: row['t'] >= .7)
        self.press(KEY_JUMP)
        row = self.step()
        self.assertTrue(row['jump_locked'])
        self.assertEqual(row['jump_count'], 0)
        self.assertTrue(twin.active)
        text = overlay(row, type('Stats', (), dict(accepted=0, rejected=0))())
        self.assertIn('LOCKED: Home to reset', text[1])
        data = twin.data
        self.press(KEY_RESET)
        self.assertIs(twin.data, data)
        self.assertFalse(twin.active)
        self.assertIsNone(twin.stable_since)
        self.assertIsNone(self.keyboard.owned_command)
        self.press(KEY_JUMP)
        self.assertEqual(self.step()['jump_count'], 1)

    def test_viewer_shortcuts_do_not_control_robot_and_help_matches(self):
        for key in (87, 83, 32, 82, 74, 259, 256):  # W/S/Space/R/J/Backspace/Esc
            self.assertFalse(self.press(key))
        row = self.step()
        self.assertEqual(row['jump_count'], 0)
        self.assertEqual(row['target_v'], 0)
        self.assertEqual(row['src'], 'none')
        self.assertTrue(row['watchdog'])
        self.assertGreater(row['t'], 0)  # No reset.
        text = overlay(row, type('Stats', (), dict(accepted=0, rejected=0))())
        self.assertIn(KEY_HELP, text[0])
        self.assertTrue(self.press(KEY_QUIT))
