#!/usr/bin/env python3
"""Host-only software regression check, never an N4B/N4C acceptance gate."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

VERSION = '0.1.0'
PACKAGE = Path(__file__).resolve().parents[1]


def utc():
    return datetime.now(timezone.utc).isoformat()


def collect(outdir, probe):
    outdir.mkdir(parents=True, exist_ok=False)
    metadata = {'started_utc': utc(), 'host': {'system': platform.system(),
                'release': platform.release(), 'machine': platform.machine(),
                'python': platform.python_version()}, 'runs': []}
    for name, arguments in [('all', []), ('real', [str(PACKAGE / 'tests/fixtures/real_epochs.txt')]),
                            ('emit', [str(outdir.resolve())])]:
        command = [str(probe.resolve()), name, *arguments]
        try:
            run = subprocess.run(command, capture_output=True, check=False)
            metadata['runs'].append({'name': name, 'command': command, 'returncode': run.returncode,
                                     'missing': False})
            stdout, stderr = run.stdout, run.stderr
        except OSError as error:
            metadata['runs'].append({'name': name, 'command': command, 'returncode': None, 'missing': True})
            stdout, stderr = b'', str(error).encode()
        (outdir / f'{name}.stdout').write_bytes(stdout)
        (outdir / f'{name}.stderr').write_bytes(stderr)
    (outdir / 'execution.json').write_text(json.dumps(metadata, indent=2) + '\n')


def analyze(outdir):
    metadata = json.loads((outdir / 'execution.json').read_text())
    criteria = []
    for run in metadata['runs']:
        log = outdir / f"{run['name']}.stdout"
        if run['missing']:
            status = 'BLOCKED'
        elif run['returncode'] != 0 or f"PASS {run['name']} (" not in log.read_text():
            status = 'FAIL'
        else:
            status = 'PASS'
        criteria.append({'id': run['name'], 'description': 'ROS-free fake-engine ' + run['name'],
                         'measured': run['returncode'], 'unit': 'exit code',
                         'threshold': 'successful probe assertions and scenario completion', 'status': status,
                         'evidence': log.name, 'threshold_source': 'SOURCE-SUPPORTED phase-14 software requirements'})
    if not criteria:
        raise ValueError('saved execution has no scenarios')
    status = 'FAIL' if any(c['status'] == 'FAIL' for c in criteria) else (
             'BLOCKED' if any(c['status'] == 'BLOCKED' for c in criteria) else 'PASS')
    raw = [{'path': p.name, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
           for p in sorted(outdir.iterdir()) if p.is_file() and p.name != 'result.json']
    result = {'check': 'orb_node_software_core', 'tool_version': VERSION, 'status': status,
              'started_utc': metadata['started_utc'], 'ended_utc': utc(), 'host': metadata['host'],
              'criteria': criteria, 'raw_files': raw,
              'notes': ['MEASURED software regression outcome only; no hardware gate PASS.',
                        'Real fixture preserves saved source epochs only; pixels, IMU and poses are synthetic.',
                        'ROS 2 and ORB bindings are UNBUILT and physical behavior UNKNOWN.']}
    (outdir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(f"{'CRITERION':24} STATUS")
    for criterion in criteria:
        print(f"{criterion['id']:24} {criterion['status']}")
    print('software check:', status)
    return {'PASS': 0, 'FAIL': 1, 'BLOCKED': 2}[status]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--outdir', type=Path)
    mode.add_argument('--analyze-only', type=Path)
    parser.add_argument('--probe', type=Path, default=PACKAGE / 'build/core_test')
    args = parser.parse_args()
    try:
        if args.outdir:
            collect(args.outdir, args.probe)
        return analyze(args.analyze_only or args.outdir)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'usage/internal error: {error}', file=sys.stderr)
        return 3

if __name__ == '__main__':
    sys.exit(main())
