#!/usr/bin/env python3
"""OPEN: recorder diagnostics only. No hardware access; Python stdlib on Pi."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

RATES = {'Infrared 1': 30, 'Infrared 2': 30, 'Gyro': 200, 'Accel': 100}
BITS = {0: 'under-voltage', 1: 'frequency capped', 2: 'throttled', 3: 'soft temperature limit',
        16: 'under-voltage occurred', 17: 'frequency capped occurred',
        18: 'throttling occurred', 19: 'soft temperature limit occurred'}


def ram_required(seconds):
    value = float(seconds)
    if not math.isfinite(value) or value < 1 or value > 120:
        raise ValueError('SECONDS must be finite and in [1,120]')
    return math.ceil(18_400_000 * value * 1.2 * 1.5)


def summary(analysis, power, recorder_rc, analysis_rc, checksum_ok):
    masks = [int(value, 16) for value in re.findall(r'throttled=0x([0-9a-fA-F]+)', power)]
    combined = 0
    for mask in masks:
        combined |= mask
    labels = [label for bit, label in BITS.items() if combined & (1 << bit)]
    lines = [f'throttled bits seen (before/during/after take, latched bits retained): '
             f'{hex(combined) if masks else "UNKNOWN"}; ' + (', '.join(labels) if labels else 'none decoded')]
    # Conservative diagnostics, not N2/N3 thresholds. Exporter's own audit remains mandatory.
    usable = recorder_rc == 0 and analysis_rc == 0 and checksum_ok and bool(masks) and combined == 0
    usable = usable and 'POWER_COMMAND_FAILED' not in power
    domains = set()
    for stream, hz in RATES.items():
        item = analysis.get(stream, {})
        if not isinstance(item, dict):
            item = {}
        gap, rate = item.get('max_gap_ms'), item.get('rate_hz')
        domain = item.get('domains', [])
        domains.update(domain)
        okay = (item.get('count', 0) >= 2 and isinstance(gap, (float, int)) and math.isfinite(gap)
                and 0 < gap <= 1500 / hz and isinstance(rate, (float, int)) and math.isfinite(rate)
                and 0.95 * hz <= rate <= 1.05 * hz and item.get('nonpositive_steps') == 0
                and len(domain) == 1 and domain[0] in ('Global Time', 'Hardware Clock', 'global_time', 'hardware_clock')
                and (not stream.startswith('Infrared') or item.get('frame_number_skips') == 0))
        usable = usable and okay
        lines.append(f'{stream}: max gap={gap if gap is not None else "UNKNOWN"} ms, rate={rate}, domains={domain}')
    usable = usable and len(domains) == 1
    lines.append(f'recorder_rc={recorder_rc}, analysis_rc={analysis_rc}, sha256 verified={checksum_ok}')
    lines.append('take usable for Kalibr: ' + ('yes' if usable else 'no') + ' (diagnostic)')
    lines.append('OPEN: diagnostic only; capture prerequisites, exposure, board visibility and bag export audit still require review.')
    return '\n'.join(lines), bool(usable)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    ram = sub.add_parser('ram-required')
    ram.add_argument('seconds')
    report = sub.add_parser('summary')
    report.add_argument('analysis', type=Path)
    report.add_argument('power', type=Path)
    report.add_argument('recorder_rc', type=int)
    report.add_argument('analysis_rc', type=int)
    report.add_argument('checksum_ok', choices=('yes', 'no'))
    args = parser.parse_args()
    try:
        if args.action == 'ram-required':
            print(ram_required(args.seconds))
        else:
            try:
                analysis = json.loads(args.analysis.read_text())
                if not isinstance(analysis, dict):
                    analysis = {}
            except (OSError, ValueError):
                analysis = {}
            result, usable = summary(analysis, args.power.read_text(), args.recorder_rc,
                                     args.analysis_rc, args.checksum_ok == 'yes')
            print(result)
            return 0 if usable else 1
    except (OSError, ValueError) as exc:
        print('OPEN: ' + str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
