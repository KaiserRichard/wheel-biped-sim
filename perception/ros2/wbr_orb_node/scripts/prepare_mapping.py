#!/usr/bin/env python3
"""Validate a sender-compatible clockmap offline; never opens a device."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def prepare(data, source_domain, source_session, pi_session):
    # Import only for explicit mapping work; default host tests need no NumPy.
    from tools.clockmap import ClockMap, Exchange, Stamp
    for value in (source_domain, source_session, pi_session):
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,47}', value):
            raise ValueError('invalid clock identity token')
    if (data['camera_domain'] != source_domain or data['camera_session'] != source_session
            or data['pi_domain'] != 'CLOCK_MONOTONIC_RAW' or data['pi_session'] != pi_session
            or data.get('valid') is not True):
        raise ValueError('mapping identity/validity mismatch')
    mapping = ClockMap(source_domain, 'CLOCK_MONOTONIC_RAW', source_session, pi_session)
    for row in data['exchanges_ns']:
        if len(row) != 4 or any(type(value) is not int for value in row):
            raise ValueError('exchange must contain four integer ns epochs')
        t1, t2, t3, t4 = row
        mapping.add_exchange(Exchange(Stamp(t1, source_domain, source_session),
                                     Stamp(t2, 'CLOCK_MONOTONIC_RAW', pi_session),
                                     Stamp(t3, 'CLOCK_MONOTONIC_RAW', pi_session),
                                     Stamp(t4, source_domain, source_session)))
    if not mapping.valid:
        raise ValueError('clockmap is not valid')
    midpoints = [(row[0] + row[3]) // 2 for row in data['exchanges_ns']]
    lower = data.get('valid_source_min_ns', min(midpoints))
    upper = data.get('valid_source_max_ns', max(midpoints))
    if type(lower) is not int or type(upper) is not int or lower < 0 or upper < lower:
        raise ValueError('invalid source validity interval')
    rationale = data.get('validity_rationale', '')
    if (lower < min(midpoints) or upper > max(midpoints)) and not rationale:
        raise ValueError('extrapolation interval requires a validity_rationale in the clockmap config')
    # No independent fit/interpolation implementation: ClockMap also evaluates
    # declared extrapolation endpoints and supplies uncertainty downstream.
    for source in (lower, upper):
        mapping.convert(Stamp(source, source_domain, source_session))
    return {key: data[key] for key in ('valid', 'camera_domain', 'camera_session',
                                      'pi_domain', 'pi_session', 'exchanges_ns')} | {
        'prepared_by': 'wbr_orb_node.prepare_mapping.v1',
        'valid_source_min_ns': lower, 'valid_source_max_ns': upper,
        'validity_rationale': rationale or 'PROPOSED observed exchange midpoint interval only',
        'evidence': 'CALCULATED using tools.clockmap; trusted exchange provenance required'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--source-domain', required=True)
    parser.add_argument('--source-session', required=True)
    parser.add_argument('--pi-session', required=True)
    args = parser.parse_args()
    try:
        raw = args.input.read_bytes()
        result = prepare(json.loads(raw), args.source_domain, args.source_session, args.pi_session)
        result['input_sha256'] = hashlib.sha256(raw).hexdigest()
        with args.output.open('x') as output:
            json.dump(result, output, indent=2)
            output.write('\n')
    except (OSError, ValueError, KeyError, ImportError) as error:
        parser.exit(2, f'mapping BLOCKED: {error}\n')

if __name__ == '__main__':
    main()
