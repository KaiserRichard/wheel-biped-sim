"""Synthetic captures for the logic-analyzer parser: known delays in, known stages out."""
import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('la_analyze', HERE / 'pi/la_analyze.py')
la = importlib.util.module_from_spec(spec)
spec.loader.exec_module(la)
spec2 = importlib.util.spec_from_file_location('link_bench', HERE / 'pi/link_bench.py')
lb = importlib.util.module_from_spec(spec2)
spec2.loader.exec_module(lb)

BAUD = 461538.0
BIT = 1 / BAUD


def uart_levels(data, t0):
    """(t, level) samples at every bit boundary for 8N1 bytes sent back to back."""
    out, t = [], t0
    for byte in data:
        bits = [0] + [(byte >> i) & 1 for i in range(8)] + [1]
        for b in bits:
            out.append((t, b))
            t += BIT
    return out, t  # t = end of last stop bit


def edges(levels, initial=1):
    out, cur = [], initial
    for t, v in sorted(levels):
        if v != cur:
            out.append((t, v))
            cur = v
    return out


def scenario(n=40):
    health = lb.Bench.__new__(lb.Bench).health(5)
    status = lb.lp.mcu_status(1, 1, sample_mcu_ns=1, last_health_seq=5, last_estimate_seq=0,
                              last_ref_seq=0, faults=0, action_mcu_ns=0, battery_mv=0, fsm=0,
                              armed=0, operator_enable=0, state=5, reason=0, battery_valid=0)
    pi_tx, mcu_tx, pi_mark, mcu_mark = [], [], [], []
    truth = []
    for i in range(n):
        m = 0.010 + i * 0.020
        sw = 40e-6 + (i % 5) * 10e-6
        dec = 3e-6 + (i % 3) * 1e-6
        rep = 2e-6
        lv, end = uart_levels(health, m + sw)
        pi_tx += lv
        mm = end + dec
        lv2, end2 = uart_levels(status, mm + rep)
        mcu_tx += lv2
        pi_mark.append((m, (i + 1) % 2))
        mcu_mark.append((mm, (i + 1) % 2))
        truth.append((sw, dec, rep, end2 - m))
        if i % 4 == 0:  # unrelated 20 Hz heartbeat between packets
            hb, _ = uart_levels(status, m + 0.012)
            mcu_tx += hb
    return {'D0': edges(pi_tx), 'D1': edges(mcu_tx), 'D2': pi_mark, 'D3': mcu_mark}, truth


def write_vcd(path, sig):
    ids = {'D0': '!', 'D1': '"', 'D2': '#', 'D3': '$'}
    events = sorted((round(t * 1e9), ids[n], v) for n, e in sig.items() for t, v in e)
    with open(path, 'w') as f:
        f.write('$timescale 1 ns $end\n$scope module logic $end\n')
        for n, i in ids.items():
            f.write('$var wire 1 %s %s $end\n' % (i, n))
        f.write('$upscope $end\n$enddefinitions $end\n#0 1! 1" 0# 0$\n')
        for t, i, v in events:
            f.write('#%d %d%s\n' % (t, v, i))


def write_csv(path, sig):
    names = ['D0', 'D1', 'D2', 'D3']
    cur = {'D0': 1, 'D1': 1, 'D2': 0, 'D3': 0}
    events = sorted((t, n, v) for n, e in sig.items() for t, v in e)
    with open(path, 'w') as f:
        f.write('Time [s],' + ','.join(names) + '\n0.000000000,1,1,0,0\n')
        for t, n, v in events:
            cur[n] = v
            f.write('%.9f,%s\n' % (t, ','.join(str(cur[x]) for x in names)))


class Analyzer(unittest.TestCase):
    def check(self, res, truth):
        self.assertEqual(res['matched'], len(truth))
        s = res['stages_us']
        tol = 1.5 * BIT * 1e6
        self.assertAlmostEqual(s['pi_tx_sw']['min'], min(t[0] for t in truth) * 1e6, delta=tol)
        self.assertAlmostEqual(s['pi_tx_sw']['max'], max(t[0] for t in truth) * 1e6, delta=tol)
        self.assertAlmostEqual(s['mcu_decode']['p50'], 4.0, delta=tol)
        self.assertAlmostEqual(s['onwire_rtt']['max'], max(t[3] for t in truth) * 1e6, delta=tol)
        self.assertAlmostEqual(s['pi_tx_wire']['p50'], 102 * 10 * BIT * 1e6, delta=tol)
        self.assertAlmostEqual(s['reply_wire']['p50'], 66 * 10 * BIT * 1e6, delta=tol)

    def test_vcd_and_logic2_csv(self):
        sig, truth = scenario()
        with tempfile.TemporaryDirectory() as d:
            write_vcd(os.path.join(d, 'c.vcd'), sig)
            write_csv(os.path.join(d, 'c.csv'), sig)
            for name in ('c.vcd', 'c.csv'):
                cap = os.path.join(d, name)
                s = la.read_logic2_csv(cap) if name.endswith('.csv') else la.read_vcd(cap)
                self.check(la.analyze(s, 'D0', 'D1', 'D2', 'D3', BAUD), truth)

    def test_missing_reply_is_counted_not_paired(self):
        sig, truth = scenario(10)
        sig['D3'] = sig['D3'][:5]  # MCU stopped marking after 5 packets
        sig = {k: (1 if k in ('D0', 'D1') else 0, v) for k, v in sig.items()}
        res = la.analyze(sig, 'D0', 'D1', 'D2', 'D3', BAUD)
        self.assertEqual(res['matched'], 5)
        self.assertEqual(res['unmatched']['no_mcu_mark'], 5)


if __name__ == '__main__':
    unittest.main()
