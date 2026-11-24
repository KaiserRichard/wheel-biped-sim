import json
import socket
import time
import unittest

from sim.twin3d_cmd.commands import Command, CommandModel
from sim.twin3d_cmd.udp import Packet, encode, decode, Freshness, Listener, main


class UDPTests(unittest.TestCase):
    def listener(self):
        try:
            return Listener(port=0)
        except PermissionError as error:
            self.skipTest(f'sandbox blocks real localhost UDP bind: {error}')

    def test_roundtrip_and_strict_schema(self):
        packet = Packet(1, 100.0, Command(.3, .7, True, 'test'))
        self.assertEqual(decode(encode(packet)), packet)
        valid = json.loads(encode(packet))
        for key, value in (('seq', True), ('seq', -1), ('t', float('nan')),
                           ('v', float('inf')), ('jump', 1), ('src', None)):
            row = dict(valid, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                decode(json.dumps(row).encode())
        for payload in (b'null', b'[]', b'{}', b'\xff', b'{', b' '*4097):
            with self.assertRaises(ValueError):
                decode(payload)

    def test_sequence_timestamp_staleness_and_source_restart(self):
        fresh = Freshness()
        def packet(seq, t=100, src='a'):
            return Packet(seq, t, Command(src=src))
        self.assertTrue(fresh.accept(packet(5), 100))
        self.assertFalse(fresh.accept(packet(5), 100))
        self.assertFalse(fresh.accept(packet(4), 100))
        self.assertFalse(fresh.accept(packet(6, 99.9), 100))
        self.assertFalse(fresh.accept(packet(6, 99), 100))
        self.assertFalse(fresh.accept(packet(6, 101), 100))
        self.assertTrue(fresh.accept(packet(6, 100.1), 100.2))
        self.assertTrue(fresh.accept(packet(0, 100, 'new-boot'), 100.2))

    def test_real_localhost_socket_rejects_without_watchdog_refresh(self):
        now = time.time()
        model = CommandModel()
        model.tick(0)
        with self.listener() as listener, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            packet = Packet(1, now, Command(.3, src='socket'))
            sender.sendto(encode(packet), listener.address)
            sender.sendto(encode(packet), listener.address)  # duplicate
            sender.sendto(encode(Packet(2, now-1, Command(src='socket'))), listener.address)
            sender.sendto(b'bad json', listener.address)
            deadline, received = time.monotonic()+1, []  # ASSUMED test wait limit, s.
            while listener.accepted+listener.rejected < 4 and time.monotonic() < deadline:
                received.extend(listener.drain(now))
                time.sleep(.001)
            self.assertEqual((listener.accepted, listener.rejected), (1, 3))
            for packet in received:
                model.receive(packet.command, 0)
            self.assertFalse(model.tick(.49).watchdog)
            self.assertTrue(model.tick(.5).watchdog)

    def test_sender_cli_one_jump_edge_and_multiple_heartbeats(self):
        with self.listener() as listener:
            main(['send', '--port', str(listener.address[1]), '--v', '.3', '--jump',
                  '--seconds', '.12', '--hz', '50'])
            packets = listener.drain()
        self.assertGreater(len(packets), 1)
        self.assertEqual(sum(p.command.jump for p in packets), 1)
        self.assertTrue(all(p.command.v_ref == .3 for p in packets))
        self.assertEqual(len({p.seq for p in packets}), len(packets))
