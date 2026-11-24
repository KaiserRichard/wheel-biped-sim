"""Runner logic with no pyserial dependency or device access."""
import io
import pathlib
import queue
import sys
import threading
import unittest
from unittest import mock
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import host
import pi_loop


class RunnerTests(unittest.TestCase):
    def test_stat_deltas_wrap_and_interval(self):
        a = pi_loop.parse_stat('STAT ms=1000 sent=4294967294 fed_back=10 rejected=1 rx_drop=0')
        b = pi_loop.parse_stat('STAT ms=3000 sent=2 fed_back=14 rejected=3 rx_drop=6')
        result = pi_loop.stat_delta(a, b)
        self.assertEqual(result['sent_s'], 2)
        self.assertEqual(result['fed_back_s'], 2)
        self.assertEqual(result['rejected_s'], 1)
        self.assertEqual(result['rx_drop_s'], 3)
        with self.assertRaises(ValueError):
            pi_loop.stat_delta(b, b)

    def test_partial_writes_and_failure(self):
        class Port:
            data = b''
            def write(self, data):
                self.data += data[:3]
                return len(data[:3])
        port = Port()
        wire = host.command(1, 6)
        self.assertEqual(pi_loop.write_all(port, wire), len(wire))
        self.assertEqual(port.data, wire)
        port.write = lambda data: 0
        with self.assertRaises(OSError):
            pi_loop.write_all(port, wire)

    def test_one_reader_routes_stat_and_logs_raw(self):
        stop = threading.Event()
        events = queue.Queue()
        class Port:
            def read(self, size):
                stop.set()
                return b'I,1,1,0,0,0,0,0,0,0\nSTAT ms=1000 sent=2\n'
        raw = io.StringIO()
        pi_loop.reader(Port(), stop, events, raw)
        self.assertEqual(events.get_nowait()[1], 'STAT ms=1000 sent=2')
        self.assertIn('rx_hex', raw.getvalue())

    def test_writer_only_zero_commands_at_fixed_rate(self):
        # Fast virtual clock; reader is stubbed, never opens a device.
        clock = [0.0]
        class runner:
            def __init__(self, **kwargs): pass
            def start(self): pass
            def join(self, **kwargs): pass
        class Port:
            data = []
            def write(self, wire):
                self.data.append((clock[0], wire))
                return len(wire)
        port = Port()
        def sleep(dt): clock[0] += max(dt, .00001)
        with mock.patch.object(pi_loop.threading, 'Thread', runner), \
             mock.patch.object(pi_loop.time, 'monotonic', lambda: clock[0]), \
             mock.patch.object(pi_loop.time, 'sleep', sleep), \
             mock.patch('builtins.print'):
            pi_loop.run(port, 100, .1, io.StringIO())
        self.assertEqual(len(port.data), 10)
        for seq, (stamp, wire) in enumerate(port.data):
            self.assertEqual(wire, host.command(seq, host.IDS[seq % 4]))
            self.assertAlmostEqual(stamp, seq / 100, places=4)
