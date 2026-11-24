import socket
import tempfile
from pathlib import Path
import time
import unittest

from sim.twin3d_cmd.commands import Command
from sim.twin3d_cmd.run_scripted import run, evaluate, scenario_commands, Twin, load_script
from sim.twin3d_cmd.teleop_viewer import Keyboard, overlay
from sim.twin3d_cmd.udp import Listener, Packet, encode


class PhysicsTests(unittest.TestCase):
    def test_scripted_watchdog_tracking_and_stop_actual_physics(self):
        rows = run(7, scenario_commands('watchdog'))
        result = evaluate(rows, 'watchdog')
        self.assertTrue(result['passed'], result)
        self.assertEqual(len(rows), 7000)
        self.assertFalse(evaluate(rows[:3000], 'watchdog')['passed'])

    def test_one_jump_repeated_active_presses_and_physical_landing(self):
        rows = run(3, scenario_commands('jump'))
        result = evaluate(rows, 'jump')
        self.assertTrue(result['passed'], result)

    def test_yaw_logged_but_no_planar_effect(self):
        baseline = run(.02, [dict(at=0, v=.1)])
        yaw = run(.02, [dict(at=0, v=.1, yaw_rate=2)])
        self.assertEqual([r['v'] for r in baseline], [r['v'] for r in yaw])
        self.assertEqual(yaw[-1]['yaw_rate'], 2)

    def test_reset_preserves_passive_viewer_data_reference(self):
        twin = Twin()
        initial = twin.data
        twin.receive(Command(.2, jump=True), 0)
        twin.step(0)
        twin.reset()
        self.assertIs(twin.data, initial)
        self.assertEqual(twin.data.time, 0)
        self.assertFalse(twin.active)
        self.assertEqual(twin.jump_count, 0)
        self.assertTrue(twin.step(0)['watchdog'])

    def test_keyboard_arbitration_heartbeat_reset_and_overlay_no_rendering(self):
        twin, keyboard = Twin(), Keyboard()
        keyboard.callback(265)
        self.assertFalse(keyboard.poll(twin, 0))
        self.assertAlmostEqual(twin.commands.command.v_ref, .05)
        keyboard.poll(twin, .2)
        self.assertEqual(twin.commands.last_received, .2)
        twin.receive(Command(.3, src='udp'), .3)
        keyboard.poll(twin, .5)
        self.assertEqual(twin.commands.command.src, 'udp')
        keyboard.callback(264)  # Down
        keyboard.poll(twin, .6)
        self.assertAlmostEqual(twin.commands.command.v_ref, .25)
        keyboard.callback(257)  # Enter
        keyboard.poll(twin, .6)
        row = twin.step(.6)
        self.assertEqual(row['jump_count'], 1)
        keyboard.callback(268)  # Home
        keyboard.poll(twin, .7)
        self.assertFalse(twin.active)
        text = overlay(twin.step(.7), type('Stats', (), dict(accepted=3, rejected=1))())
        self.assertIn('watchdog', text[0])
        self.assertIn('STALE', text[1])
        keyboard.callback(269)  # End
        self.assertTrue(keyboard.poll(twin, .8))

    def test_script_format_and_duration_validation(self):
        for script in ([dict(at=-1)], [dict(at=0, jump=1)], [dict(at=0, extra=1)]):
            with self.assertRaises(ValueError):
                load_script(script)
        for seconds in (0, -1, float('nan')):
            with self.assertRaises(ValueError):
                run(seconds)

    def test_udp_headless_path_and_csv_with_real_localhost_socket(self):
        try:
            listener = Listener(port=0)
        except PermissionError as error:
            self.skipTest(f'sandbox blocks real localhost UDP bind: {error}')
        # Warm the gain cache before issuing an age-limited packet.
        Twin()
        with listener, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(encode(Packet(0, time.time(), Command(.3, .2, True, 'udp-physics'))),
                          listener.address)
            with tempfile.TemporaryDirectory() as directory:
                out = Path(directory)/'udp.csv'
                rows = run(.65, out=out, listener=listener)
                self.assertEqual(len(out.read_text().splitlines()), len(rows)+1)
        self.assertEqual(listener.accepted, 1)
        self.assertTrue(all(r['src'] == 'udp-physics' for r in rows))
        self.assertEqual(rows[-1]['jump_count'], 1)
        self.assertTrue(rows[-1]['watchdog'])
