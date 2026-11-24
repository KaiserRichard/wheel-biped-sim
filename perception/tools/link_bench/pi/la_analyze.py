#!/usr/bin/env python3
"""Logic-analyzer timing for the link bench (runs on the Mac; stdlib only).

Reads a 4-channel capture exported as VCD (sigrok-cli / PulseView) or as a
Saleae Logic 2 digital CSV, finds UART bursts and marker edges, pairs them per
packet, and reports per-stage latencies on the analyzer's single clock:

  pi_tx_sw    Pi marker edge          -> first start bit on Pi TX
  pi_tx_wire  first start bit         -> end of last stop bit (Pi TX burst)
  mcu_decode  end of Pi TX burst      -> MCU marker edge (decode done)
  mcu_reply   MCU marker edge         -> first start bit on MCU TX
  reply_wire  MCU TX burst duration
  onwire_rtt  Pi marker edge          -> end of the MCU reply burst

With --summary (the link_bench summary.json of the same run) it also prints
Pi-clock RTT percentiles minus on-wire RTT percentiles. That difference is an
INFERRED Pi receive-path estimate (kernel UART RX, wake-up, Python decode,
plus the marker ioctl cost). It compares distributions from two clocks and
does not subtract per-packet timestamps across clock domains.

DIAGNOSTIC tooling; not a gate PASS.
"""
import argparse
import csv
import json
import math
import re
import sys

UNITS = {'s': 1.0, 'ms': 1e-3, 'us': 1e-6, 'ns': 1e-9, 'ps': 1e-12, 'fs': 1e-15}


def read_vcd(path):
    """Return {signal_name: (initial_level, [(t_seconds, level), ...])}."""
    with open(path) as f:
        text = f.read()
    m = re.search(r'\$timescale\s+(\d+)\s*([a-z]+)\s*\$end', text)
    scale = int(m.group(1)) * UNITS[m.group(2)] if m else 1e-9
    ids = {}
    for vid, name in re.findall(r'\$var\s+\w+\s+1\s+(\S+)\s+(\S+)(?:\s+\[[^\]]*\])?\s+\$end', text):
        ids[vid] = name
    body = text.split('$enddefinitions', 1)[1].split('$end', 1)[1]
    now, sig = 0.0, {name: [None, []] for name in ids.values()}
    for tok in body.split():
        if tok.startswith('#'):
            now = int(tok[1:]) * scale
        elif tok[0] in '01' and tok[1:] in ids:
            s = sig[ids[tok[1:]]]
            level = int(tok[0])
            if s[0] is None:
                s[0] = level
            elif (s[1][-1][1] if s[1] else s[0]) != level:
                s[1].append((now, level))
    return {k: (v[0] if v[0] is not None else 1, v[1]) for k, v in sig.items()}


def read_logic2_csv(path):
    with open(path, newline='') as f:
        rows = csv.reader(f)
        header = next(rows)
        names = [h.strip() for h in header[1:]]
        sig = {n: [None, []] for n in names}
        for row in rows:
            t = float(row[0])
            for n, v in zip(names, row[1:]):
                level = int(float(v))
                s = sig[n]
                if s[0] is None:
                    s[0] = level
                elif (s[1][-1][1] if s[1] else s[0]) != level:
                    s[1].append((t, level))
    return {k: (v[0] if v[0] is not None else 1, v[1]) for k, v in sig.items()}


def bursts(edges, bit_s, idle_bits=20):
    """UART bursts (start, end) from an idle-high line: a burst starts at a falling
    edge after >= idle_bits of idle and ends one bit after its last rising edge
    (the stop bit of the final 0x00 delimiter)."""
    out, start, last_rise = [], None, None
    for t, level in edges:
        if level == 0 and (start is None or t - last_rise >= idle_bits * bit_s):
            if start is not None:
                out.append((start, last_rise + bit_s))
            start = t
        elif level == 1 and start is not None:
            last_rise = t
    if start is not None and last_rise is not None:
        out.append((start, last_rise + bit_s))
    return out


def first_at_or_after(seq, t, key=lambda x: x):
    lo, hi = 0, len(seq)
    while lo < hi:
        mid = (lo + hi) // 2
        if key(seq[mid]) < t:
            lo = mid + 1
        else:
            hi = mid
    return lo


def percentile(v, p):
    return v[max(1, math.ceil(p / 100.0 * len(v))) - 1] if v else None


def stats(values_s):
    v = sorted(x * 1e6 for x in values_s)
    if not v:
        return {'n': 0}
    return {'n': len(v), 'min': v[0], 'p50': percentile(v, 50), 'p95': percentile(v, 95),
            'p99': percentile(v, 99), 'max': v[-1]}


def analyze(sig, pi_tx, mcu_tx, pi_mark, mcu_mark, baud):
    bit = 1.0 / baud
    for name in (pi_tx, mcu_tx, pi_mark, mcu_mark):
        if name not in sig:
            raise SystemExit('channel %r not in capture; have %s' % (name, sorted(sig)))
    pb = bursts(sig[pi_tx][1], bit)
    mb = bursts(sig[mcu_tx][1], bit)
    pm = [t for t, _ in sig[pi_mark][1]]
    mm = [t for t, _ in sig[mcu_mark][1]]
    stages = {k: [] for k in ('pi_tx_sw', 'pi_tx_wire', 'mcu_decode', 'mcu_reply', 'reply_wire', 'onwire_rtt')}
    unmatched = {'no_pi_tx': 0, 'no_mcu_mark': 0, 'no_reply': 0}
    for i, m in enumerate(pm):
        nxt = pm[i + 1] if i + 1 < len(pm) else float('inf')
        j = first_at_or_after(pb, m, key=lambda b: b[0])
        if j >= len(pb) or pb[j][0] >= min(nxt, m + 5e-3):
            unmatched['no_pi_tx'] += 1
            continue
        b = pb[j]
        # Decode completes around the delimiter's stop bit; allow one bit of slack.
        k = first_at_or_after(mm, b[1] - bit)
        if k >= len(mm) or mm[k] >= min(nxt, b[1] + 5e-3):
            unmatched['no_mcu_mark'] += 1
            continue
        km = mm[k]
        r = first_at_or_after(mb, km, key=lambda x: x[0])
        if r >= len(mb) or mb[r][0] >= km + 2e-3:
            unmatched['no_reply'] += 1
            continue
        reply = mb[r]
        stages['pi_tx_sw'].append(b[0] - m)
        stages['pi_tx_wire'].append(b[1] - b[0])
        stages['mcu_decode'].append(km - b[1])
        stages['mcu_reply'].append(reply[0] - km)
        stages['reply_wire'].append(reply[1] - reply[0])
        stages['onwire_rtt'].append(reply[1] - m)
    return {'baud': baud, 'pi_marks': len(pm), 'mcu_marks': len(mm), 'pi_tx_bursts': len(pb),
            'mcu_tx_bursts': len(mb), 'matched': len(stages['onwire_rtt']), 'unmatched': unmatched,
            'stages_us': {k: stats(v) for k, v in stages.items()}}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('capture', help='.vcd (sigrok/PulseView) or .csv (Logic 2 digital export)')
    ap.add_argument('--pi-tx', default='D0')
    ap.add_argument('--mcu-tx', default='D1')
    ap.add_argument('--pi-mark', default='D2')
    ap.add_argument('--mcu-mark', default='D3')
    ap.add_argument('--baud', type=float, default=461538.0,
                    help='actual line rate; F446 BRR gives 461538 for nominal 460800')
    ap.add_argument('--summary', help='link_bench summary.json of the same run')
    ap.add_argument('--out', help='write the result JSON here')
    args = ap.parse_args()
    sig = read_logic2_csv(args.capture) if args.capture.endswith('.csv') else read_vcd(args.capture)
    res = analyze(sig, args.pi_tx, args.mcu_tx, args.pi_mark, args.mcu_mark, args.baud)
    res['verdict'] = 'OPEN / diagnostic (analyzer timing; not a gate PASS)'
    print('matched %d of %d Pi markers; unmatched %s' % (res['matched'], res['pi_marks'], res['unmatched']))
    print('%-12s %8s %8s %8s %8s %8s' % ('stage (us)', 'min', 'P50', 'P95', 'P99', 'max'))
    for k, s in res['stages_us'].items():
        if s['n']:
            print('%-12s %8.1f %8.1f %8.1f %8.1f %8.1f' % (k, s['min'], s['p50'], s['p95'], s['p99'], s['max']))
    if args.summary:
        with open(args.summary) as f:
            pi = json.load(f).get('rtt', {}).get('rtt_us', {})
        ow = res['stages_us']['onwire_rtt']
        if pi.get('n') and ow.get('n'):
            res['pi_rx_path_estimate_us'] = {p: pi[p] - ow[p] for p in ('p50', 'p95', 'p99')}
            print('Pi-clock RTT minus on-wire RTT (INFERRED Pi RX path), us: P50 %.0f P95 %.0f P99 %.0f' % tuple(
                res['pi_rx_path_estimate_us'][p] for p in ('p50', 'p95', 'p99')))
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(res, f, indent=2)
    print(res['verdict'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
