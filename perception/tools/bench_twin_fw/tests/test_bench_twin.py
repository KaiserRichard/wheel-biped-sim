"""No hardware: compile real bench C and verbatim reference function bodies."""
import ctypes as C
import hashlib
import importlib.util
import os
import pathlib
import random
import re
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
import host
sys.path.insert(0, str(REPO))
from tools.imu_view.stream import parse_line, from_lines
REF = pathlib.Path(os.environ.get('DMJUMP_REF', '/tmp/dmjump-ref'))
U8 = C.c_uint8
F = C.c_float
Data = U8 * 8


class Cmd(C.Structure):
    _fields_ = [(k, F) for k in ('p', 'v', 'kp', 'kd', 't')]


class Fb(C.Structure):
    _fields_ = [(k, F) for k in ('p', 'v', 't')] + [(k, U8) for k in ('id', 'state', 'tmos', 'tcoil')]


class Motor(C.Structure):
    _fields_ = [('cmd', Cmd)] + [(k, F) for k in ('p', 'v', 't')] + [('id', U8), ('enabled', U8)]


class Line(C.Structure):
    _fields_ = [('seq', C.c_uint32), ('id', U8), ('special', U8), ('cmd', Cmd)]


def function(source, name):
    """Return exact original signature/body without rewriting the reference."""
    match = re.search(r'^\w+[ \t]+' + name + r'\([^\n]*\)\s*\{', source, re.M)
    if not match:
        raise AssertionError(f'missing reference {name}')
    start = match.start()
    pos = source.index('{', start)
    depth = 1
    end = pos + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class TwinTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = pathlib.Path(cls.tmp.name)
        path = REF / 'DM_Motor/dm4310_drv.c'
        cls.has_ref = path.is_file() and path.with_suffix('.h').is_file()
        if cls.has_ref:
            cls.ref_hash = hashlib.sha256(path.read_bytes()).hexdigest()
            src = path.read_text()
            hdr = (path.with_suffix('.h')).read_text()
            definitions = '\n'.join(re.findall(r'^#define (?:P_|V_|T_|KP_|KD_|MIT_|POS_|SPEED_).*', hdr, re.M))
            types = '\n'.join(re.findall(r'typedef struct\s*\{.*?\}\s*\w+;', hdr, re.S))
            funcs = '\n'.join(function(src, n) for n in ('float_to_uint', 'uint_to_float', 'mit_ctrl', 'mit_ctrl2', 'enable_motor_mode', 'disable_motor_mode', 'dm4310_fbdata', 'dm6215_fbdata'))
            # Stub only the HAL send boundary; function bodies are verbatim from 6855ef8.
            reference = '#include <stdint.h>\n#include <string.h>\ntypedef int hcan_t;\n#define FDCAN_DLC_BYTES_8 8\n' + definitions + '\n' + types + '''
    static uint8_t captured[8];
    static uint32_t captured_id;
    void canx_send_data(hcan_t *h,uint16_t id,uint8_t *d,int n) {
     (void)h; (void)n; captured_id=id; memcpy(captured,d,8);
    }
    ''' + funcs + '''
    uint32_t ref_pack(int wheel,uint16_t id,float p,float v,float kp,float kd,float t,uint8_t *d) {
     if(wheel) mit_ctrl2(0,id,p,v,kp,kd,t); else mit_ctrl(0,id,p,v,kp,kd,t);
     memcpy(d,captured,8); return captured_id;
    }
    void ref_feedback(int wheel,uint8_t *d,float *out) {
     motor_fbpara_t f;
     if(wheel) { Wheel_Motor_t m={0}; dm6215_fbdata(&m,d,8); f=m.para; }
     else { Joint_Motor_t m={0}; dm4310_fbdata(&m,d,8); f=m.para; }
     out[0]=f.pos; out[1]=f.vel; out[2]=f.tor; out[3]=f.id;
     out[4]=f.state; out[5]=f.Tmos; out[6]=f.Tcoil;
    }
    '''
            (tmp / 'ref.c').write_text(reference)
            subprocess.run(['cc', '-shared', '-fPIC', '-O2', str(tmp / 'ref.c'), '-o', str(tmp / 'ref.so')], check=True)
        subprocess.run(['cc', '-shared', '-fPIC', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-I' + str(ROOT / 'firmware'),
                        *(str(ROOT / f) for f in ('firmware/dm.c', 'firmware/twin.c', 'firmware/link_proto.c', 'tests/harness.c')),
                        '-o', str(tmp / 'twin.so')], check=True)
        cls.lib = C.CDLL(str(tmp / 'twin.so'))
        if cls.has_ref:
            cls.ref = C.CDLL(str(tmp / 'ref.so'))
        cls.lib.dm_encode.argtypes = [F, F, F, C.c_uint]; cls.lib.dm_encode.restype = C.c_uint32
        cls.lib.dm_decode.argtypes = [C.c_uint32, F, F, C.c_uint]; cls.lib.dm_decode.restype = F
        if cls.has_ref:
            cls.ref.float_to_uint.argtypes = [F, F, F, C.c_int]
            cls.ref.uint_to_float.argtypes = [C.c_int, F, F, C.c_int]; cls.ref.uint_to_float.restype = F
            cls.ref.ref_pack.argtypes = [C.c_int, C.c_uint16] + [F] * 5 + [C.POINTER(U8)]
            cls.ref.ref_feedback.argtypes = [C.c_int, C.POINTER(U8), C.POINTER(F)]
        cls.lib.twin_format.argtypes = [C.c_char_p, C.c_size_t, C.c_uint32, C.POINTER(Fb)]
        cls.lib.twin_format.restype = C.c_size_t
        cls.lib.dm_motor_step.argtypes = [C.POINTER(Motor), F]
        cls.lib.h_feed.argtypes = [C.c_char_p, C.c_size_t, C.c_uint32]
        cls.lib.h_output.restype = C.c_size_t

    @classmethod
    def tearDownClass(cls):
        # Evidence that reading/extracting/compiling never changed the reference.
        if cls.has_ref:
            assert hashlib.sha256((REF / 'DM_Motor/dm4310_drv.c').read_bytes()).hexdigest() == cls.ref_hash
        cls.tmp.cleanup()

    def setUp(self):
        self.lib.h_init()

    def feed(self, b, now=10):
        self.lib.h_feed(b, len(b), now)

    def output(self):
        b = (U8 * 8192)(); n = self.lib.h_output(b)
        return bytes(b[:n])

    def echo(self):
        out = host.Decoder().feed(self.output())
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0][0], 'echo')
        return out[0][1][1]

    def test_reference_conversions_and_clipping(self):
        if not self.has_ref:
            self.skipTest(f"DMJUMP_REF missing driver .c/.h at {REF}; reference comparison only skipped")
        rng = random.Random(4310)
        for lo, hi, bits in ((-12.5, 12.5, 16), (-30, 30, 12), (-10, 10, 12),
                             (-12, 12, 16), (-45, 45, 12), (-18, 18, 12), (0, 500, 12), (0, 5, 12)):
            for x in [lo, hi, 0] + [rng.uniform(lo, hi) for _ in range(100)]:
                n = self.lib.dm_encode(x, lo, hi, bits)
                self.assertEqual(n, self.ref.float_to_uint(x, lo, hi, bits))
                self.assertEqual(self.lib.dm_decode(n, lo, hi, bits), self.ref.uint_to_float(n, lo, hi, bits))
            self.assertEqual(self.lib.dm_encode(lo - 1000, lo, hi, bits), 0)
            self.assertEqual(self.lib.dm_encode(hi + 1000, lo, hi, bits), (1 << bits) - 1)

    def test_mit_bytes_against_reference_and_roundtrip(self):
        if not self.has_ref:
            self.skipTest(f"DMJUMP_REF missing driver .c/.h at {REF}; reference comparison only skipped")
        rng = random.Random(6215)
        for id_ in host.IDS:
            pm, vm, tm = (12, 45, 18) if id_ in (0, 1) else (12.5, 30, 10)
            ranges = [(-pm, pm), (-vm, vm), (0, 500), (0, 5), (-tm, tm)]
            for j in range(102):
                vals = [r[j] if j < 2 else rng.uniform(*r) for r in ranges]
                c = Cmd(*vals); d = Data(); ref = Data(); decoded = Cmd()
                self.lib.dm_pack(id_, C.byref(c), d)
                rid = self.ref.ref_pack(id_ in (0, 1), id_, *vals, ref)
                self.assertEqual(rid, id_)
                self.assertEqual(bytes(d), bytes(ref))
                self.lib.dm_unpack(id_, d, C.byref(decoded))
                for k, (lo, hi) in zip(('p', 'v', 'kp', 'kd', 't'), ranges):
                    self.assertAlmostEqual(getattr(c, k), getattr(decoded, k), delta=(hi-lo)/(65535 if k=='p' else 4095)+1e-4)
            c = Cmd(999, -999, 999, 999, -999); d = Data(); decoded = Cmd()
            self.lib.dm_pack(id_, C.byref(c), d); self.lib.dm_unpack(id_, d, C.byref(decoded))
            self.assertEqual([getattr(decoded, k) for k in ('p', 'v', 'kp', 'kd', 't')], [pm, -vm, 500, 5, -tm])

    def test_feedback_against_reference(self):
        if not self.has_ref:
            self.skipTest(f"DMJUMP_REF missing driver .c/.h at {REF}; reference comparison only skipped")
        for id_ in host.IDS:
            f = Fb(1.2, -3.4, 2.5, id_, 9, 34, 38); d = Data(); y = Fb(); ref = (F * 7)()
            self.lib.dm_pack_feedback(C.byref(f), d)
            self.lib.dm_unpack_feedback(d, C.byref(y))
            self.ref.ref_feedback(id_ in (0, 1), d, ref)
            self.assertEqual([y.p, y.v, y.t, y.id, y.state, y.tmos, y.tcoil], list(ref))
            self.assertAlmostEqual(f.p, y.p, delta=.0004)
            self.assertAlmostEqual(f.v, y.v, delta=.023)
            self.assertAlmostEqual(f.t, y.t, delta=.009)

    def test_fake_motor_enable_step_disable_and_saturation(self):
        m = Motor(); self.lib.dm_motor_init(C.byref(m), 6)
        d = Data(); c = Cmd(0, 0, 0, 0, 1)
        self.lib.dm_pack(6, C.byref(c), d); self.lib.dm_motor_receive(C.byref(m), d)
        self.lib.dm_motor_step(C.byref(m), .001)
        self.assertEqual((m.p, m.v, m.t), (0, 0, 0))
        self.lib.dm_special(0xFC, d); self.assertEqual(bytes(d), b'\xff'*7+b'\xfc')
        self.lib.dm_motor_receive(C.byref(m), d)
        self.lib.dm_pack(6, C.byref(c), d); self.lib.dm_motor_receive(C.byref(m), d)
        p = v = 0.; torque = m.cmd.t
        for _ in range(100):
            v += .001*(torque/.2-v)/.05; p += .001*v
            self.lib.dm_motor_step(C.byref(m), .001)
        self.assertAlmostEqual(m.v, v, places=5); self.assertAlmostEqual(m.p, p, places=5)
        self.lib.dm_special(0xFD, d); self.assertEqual(bytes(d), b'\xff'*7+b'\xfd')
        self.lib.dm_motor_receive(C.byref(m), d); old_v = m.v
        self.lib.dm_motor_step(C.byref(m), .001)
        self.assertEqual(m.enabled, 0); self.assertEqual(m.t, 0); self.assertLess(m.v, old_v)
        m.enabled = 1; m.cmd = Cmd(12.5, 30, 500, 5, 10)
        for _ in range(2000): self.lib.dm_motor_step(C.byref(m), .001)
        self.assertLessEqual(abs(m.v), 30); self.assertLessEqual(abs(m.p), 12.5); self.assertLessEqual(abs(m.t), 10)

    def test_line_parser_formatter_actual_c(self):
        c = Line()
        self.assertEqual(self.lib.twin_parse(b'C,4294967295,6,1.25,-2,30,0.4,-1.5\n', C.byref(c)), 1)
        self.assertEqual((c.seq, c.id, c.cmd.p, c.cmd.v), (0xFFFFFFFF, 6, 1.25, -2))
        for line in (b'', b'C,0,9,0,0,0,0,0', b'C,4294967296,6,0,0,0,0,0', b'C,1,6,nan,0,0,0,0',
                     b'C,1,6,1e3,0,0,0,0', b'C,1,6,0,0,0,0,0junk', b'C,1,6,', b'E,1,6,0', b'C,1,6,1000000,0,0,0,0'):
            self.assertEqual(self.lib.twin_parse(line, C.byref(c)), 0, line)
        for action, code in [('E', 0xFC), ('D', 0xFD)]:
            self.assertEqual(self.lib.twin_parse(host.command(1, 0, action=action), C.byref(c)), 1)
            self.assertEqual(c.special, code)
        f = Fb(-12.5, -30, -10, 6, 1, 25, 25); b = C.create_string_buffer(80)
        n = self.lib.twin_format(b, 80, 123, C.byref(f))
        self.assertEqual(b.raw[:n], b'M,123,6,-12.500,-30.000,-10.000,25,1\n')
        self.assertEqual(self.lib.twin_format(b, 8, 123, C.byref(f)), 0)

    def test_advisory_echo_leds_stale_rejections_and_sequence_wrap(self):
        for seq, value in enumerate(range(4), 0xFFFFFFFC):
            self.feed(host.advisory(seq, value), 10)
            self.assertEqual(self.lib.h_leds(), 1 << value)
            e = self.echo(); self.assertEqual((e['advisory_seq'], e['result'], e['stale']), (seq, 0, 0))
        self.feed(host.advisory(0, 1), 20); self.assertEqual(self.echo()['result'], 0)
        self.feed(host.advisory(0, 2), 25); self.assertEqual(self.echo()['result'], 3)
        self.assertEqual(self.lib.h_leds(), 2)
        wrong = host.lp.terrain_advisory(999, 1, pi_monotonic_ns=0, frame_stamp_ns=0, advisory=0,
                    plane_valid=0, confidence=0, distance_m=0, height_m=0, width_m=0, plane_pitch_rad=0, plane_height_m=0)
        self.feed(wrong, 30); self.assertEqual(self.echo()['result'], 2)
        self.lib.h_poll(319); self.assertEqual(self.lib.h_leds(), 2)
        self.lib.h_poll(320); self.assertEqual(self.lib.h_leds(), 0)
        e = self.echo(); self.assertEqual((e['stale'], e['ms_since_accepted']), (1, 300))

    def test_corrupt_and_nonfinite_advisory_echo(self):
        good = host.advisory(1)[1:]
        raw = bytearray(host.lp.cobs_decode(good[:-1])); raw[-1] ^= 1
        self.feed(b'\0' + host.lp.cobs_encode(raw) + b'\0')
        self.assertEqual(self.echo()['result'], 1)
        # Valid CRC, invalid finite field must be rejected with FIELDS.
        import struct
        raw = bytearray(host.lp.cobs_decode(good[:-1]))
        struct.pack_into('<f', raw, 34, float('nan'))
        struct.pack_into('<I', raw, len(raw)-4, host.lp.crc32c(raw[:-4]))
        self.feed(b'\0' + host.lp.cobs_encode(raw) + b'\0')
        self.assertEqual(self.echo()['result'], 4)
        self.assertEqual(self.lib.h_leds(), 0)

    def test_uart_mux_chunking_overflow_and_viewer_compatibility(self):
        data = host.command(1, 6) + host.advisory(2, 3) + host.command(3, 0, action='E')
        for b in data: self.feed(bytes([b]))
        self.assertEqual(self.lib.h_commands(), 2); self.assertEqual(self.lib.h_leds(), 8)
        echo = self.output()
        imu = b'I,123,1,-32768,32767,8192,-100,200,-300,400\n'
        mixed = imu + echo + b'M,123,6,0.000,0.000,0.000,25,1\n' + imu
        dec = host.Decoder(); records = []
        for b in mixed: records += dec.feed(bytes([b]))
        self.assertEqual([r[0] for r in records], ['line', 'echo', 'line', 'line'])
        self.assertEqual(len(list(from_lines(mixed.splitlines(keepends=True)))), 2)
        self.assertIsNotNone(parse_line(imu.decode()))
        self.feed(b'C' + b'0' * 200 + b'\n' + host.command(4, 1))
        self.assertEqual(self.lib.h_commands(), 3); self.assertEqual(self.lib.h_errors(), 1)
        self.feed(b'\x04' + b'\xff' * 210 + b'\0' + host.advisory(5, 0))
        self.assertEqual(self.lib.h_leds(), 1)

    def test_cr_lf_and_long_burst_no_loss(self):
        for seq in range(1000):
            terminator = (b'\r', b'\n', b'\r\n')[seq % 3]
            self.feed(host.command(seq, host.IDS[seq % 4]).rstrip(b'\n') + terminator)
        self.assertEqual(self.lib.h_commands(), 1000)
        self.assertEqual(self.lib.h_errors(), 0)

    def test_binary_stall_overflow_and_rx_gap_recover(self):
        self.feed(b'\x04abc', 1)
        self.lib.h_poll(101)
        self.assertEqual(self.lib.h_timeout(), 1)
        self.feed(host.command(1, 6), 102)
        self.assertEqual(self.lib.h_commands(), 1)
        self.feed(b'\x04' + b'x' * 1024, 103)
        self.assertGreater(self.lib.h_binary_overflow(), 0)
        self.lib.h_gap()
        self.feed(host.command(2, 8), 104)
        self.assertEqual(self.lib.h_commands(), 2)
        self.assertGreater(self.lib.h_errors(), 1)

    def test_rx_queue_full_wrap_error_and_gap_marker(self):
        self.lib.h_rx_init()
        for i in range(1023): self.lib.h_rx_push(i % 256, 0)
        self.lib.h_rx_push(42, 0)
        self.assertEqual(self.lib.h_rx_drop(), 1)
        self.assertEqual(self.lib.h_rx_pop(), 0)
        self.lib.h_rx_push(42, 0)
        for i in range(1, 1023): self.assertEqual(self.lib.h_rx_pop(), i % 256)
        self.assertEqual(self.lib.h_rx_pop(), 256 + 42)
        self.assertEqual(self.lib.h_rx_pop(), -1)
        self.lib.h_rx_push(42, 1)
        self.assertEqual(self.lib.h_rx_error(), 1)
        self.lib.h_rx_push(43, 0)
        self.assertEqual(self.lib.h_rx_pop(), 256 + 43)
        for i in range(10000):
            self.lib.h_rx_push(i % 256, 0)
            self.assertEqual(self.lib.h_rx_pop(), i % 256)

    def test_tx_status_reservation_and_maximum_length(self):
        # Repeated ordinary admissions, including wrap, must preserve one STAT.
        for tail in (0, 100, 4095):
            head = tail
            while self.lib.h_tx_fits(head, tail, 64, 0): head = (head + 64) % 4096
            self.assertTrue(self.lib.h_tx_fits(head, tail, 1536, 1))
        source = (ROOT / 'firmware/main.c').read_text()
        stat = source[source.index('puts_("STAT")'):]
        stat = stat[:stat.index("putc_('\\n')")]
        keys = re.findall(r'kvh?\("([^" ]+)"', stat)
        maximum = len('STAT\n') + sum(1 + len(k) + 1 + 10 for k in keys)
        self.assertLessEqual(maximum, 1536)

    def test_bandwidth_bound_uses_actual_formats(self):
        imu = 'I,4294967295,1' + ',-32768' * 7 + '\n'
        self.assertEqual(len(imu), 64)
        self.assertLessEqual(len(host.advisory(1)), 65)
        self.lib.h_poll(100); out = self.output(); self.assertLessEqual(len(out), 51)
        f = Fb(-12.5, -45, -18, 8, 15, 255, 255); b = C.create_string_buffer(80)
        n = self.lib.twin_format(b, 80, 0xFFFFFFFF, C.byref(f)); self.assertLessEqual(n, 48)
        for id_ in host.IDS: self.assertLessEqual(len(host.command(0xFFFFFFFF, id_, *([-999998]*5))), 128)
        budget = host.bandwidth()
        self.assertEqual((budget['tx_bytes_s'], budget['rx_bytes_s']), (12796, 13450))
        self.assertLess(budget['tx_fraction'], .80); self.assertLess(budget['rx_fraction'], .30)

    def test_optional_agv_codec_compiles_only_when_explicitly_selected(self):
        # Compile check only: this does not assert AGV velocity-mode readiness.
        output = pathlib.Path(self.tmp.name) / 'optional_agv.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                        '-DBENCH_OPTIONAL_AGV_CODEC=1', str(ROOT / 'firmware/robstride.c'),
                        '-o', str(output)], check=True)


if __name__ == '__main__':
    unittest.main()
