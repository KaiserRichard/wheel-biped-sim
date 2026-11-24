#!/usr/bin/env python3
"""Scripted fake; no ORB dependency or hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

mode = sys.argv[1]
if mode == 'mixed':
    mode = ['success', 'crash', 'empty', 'partial', 'reset', 'timeout'][int(sys.argv[2]) - 1]
if mode == 'mutate':
    Path(sys.argv[2]).write_text('changed by synthetic fixture')
print('IMU initialized', flush=True)
print('fake stderr', file=sys.stderr, flush=True)
if mode == 'timeout':
    subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    time.sleep(30)
if mode == 'empty':
    sys.exit(0)
if mode == 'malformed':
    Path('trajectory.txt').write_text('not a pose\n')
    sys.exit(0)
if mode == 'escape':
    Path('trajectory.txt').symlink_to(sys.argv[2])
    sys.exit(0)
points = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
if mode in {'partial', 'crash'}:
    points = points[:2]
if mode == 'reset':
    print('Reseting active map')
    print('Fail to track local map!')
    print('Creation of new map with id: 2')
    print('Map count: 2')
    points = [(0, 0, 0), (2, 0, 0), (100, 0, 0), (100, 1, 0)]
rows, epochs = [], []
for i, (x, y, z) in enumerate(points):
    t = i if mode != 'reset' or i < 2 else i - 2
    if mode == 'backward' and i == 2:
        t = 0
    if mode == 'inverse':
        rows.append(f'{t * 10**9} {-y} {x} {-z} 0.7071067811865476 0 0 -0.7071067811865476')
    else:
        rows.append(f'{t} {x} {y} {z} 0 0 0 1')
    epochs.append({'stamp_ns': t * 10**9, 'clock_domain': 'dataset_clock',
                   'map_id': 'map2' if mode == 'reset' and i >= 2 else 'map1',
                   'reset_counter': 1 if mode == 'reset' and i >= 2 else 0})
Path('trajectory.txt').write_text('\n'.join(rows) + '\n')
Path('epochs.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in epochs))
print('done', flush=True)
if mode == 'crash':
    os._exit(7)
