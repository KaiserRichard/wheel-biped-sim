import importlib
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from twin_cmd_bridge.core import Forwarder, serialize
from sim.twin3d_cmd.udp import Listener, decode


class BridgeTests(unittest.TestCase):
    def test_import_node_without_ros(self):
        self.assertTrue(callable(importlib.import_module('twin_cmd_bridge.node').main))

    def test_serialization_limits_and_validation(self):
        row = json.loads(serialize(1, 100, 5, 2, True, 'test'))
        self.assertEqual(row, dict(seq=1, t=100, v=.6, yaw_rate=2, jump=True, src='test'))
        packet = decode(serialize(1, 100, -.3, 2, True, 'test'))
        self.assertEqual((packet.seq, packet.command.v_ref, packet.command.jump), (1, -.3, True))
        for args in ((True, 1, 0, 0, False, 'a'), (1, float('nan'), 0, 0, False, 'a'),
                     (1, 1, 0, 0, 1, 'a'), (1, 1, 0, float('inf'), False, 'a')):
            with self.assertRaises(ValueError):
                serialize(*args)

    def test_callbacks_send_once_and_jump_does_not_replay_stale_twist(self):
        clock, sock = Mock(return_value=10.0), Mock()
        bridge = Forwarder('localhost', sock=sock, wall=lambda: 100.0, monotonic=clock, src='test')
        self.assertEqual(json.loads(bridge.twist(.3, 1))['seq'], 0)
        row = json.loads(bridge.jump())
        self.assertEqual((row['seq'], row['v'], row['yaw_rate'], row['jump']), (1, .3, 1, True))
        clock.return_value = 10.5
        row = json.loads(bridge.jump())
        self.assertEqual((row['v'], row['yaw_rate']), (0, 0))
        self.assertEqual(sock.sendto.call_count, 3)
        bridge.close()
        sock.close.assert_called_once()

    def test_actual_localhost_bridge_to_receiver_protocol(self):
        try:
            listener = Listener(port=0)
        except PermissionError as error:
            self.skipTest(f'sandbox blocks real localhost UDP bind: {error}')
        with listener:
            bridge = Forwarder(*listener.address, src='bridge-test')
            try:
                bridge.twist(.3, .2)
                bridge.jump()
                received, deadline = [], time.monotonic()+1  # ASSUMED test timeout, s.
                while len(received) < 2 and time.monotonic() < deadline:
                    received.extend(listener.drain())
                    time.sleep(.001)
            finally:
                bridge.close()
        self.assertEqual(len(received), 2)
        self.assertEqual([p.seq for p in received], [0, 1])
        self.assertEqual(received[0].command.v_ref, .3)
        self.assertTrue(received[1].command.jump)
