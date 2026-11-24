#!/usr/bin/env python3
"""Replay lab-day bags under old/new advisory hysteresis and compare live packets.

Example:
  python tools/obstacle_decision/flicker_check.py \
    ./wbr-evidence-archive/lab-day-20261025 ./foxglove-out/FLICKER_REPORT.md
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.obstacle_decision.decide import Advisory, DecisionParams, update
from tools.obstacle_decision.detect import Params, process_frame
from tools.obstacle_decision.range_filter import in_range
from tools.obstacle_decision.replay_bag import _intrinsics, decode_depth
from tools.bag_tools.mcap_recovery import messages as bag_messages, readable_mcap

RUNS = ('2-books46', '3-box13', '4-approach', '4-approach-r2', '4-approach-r3')
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
INFO = '/camera/camera/color/camera_info'
NAMES = {int(item): item.name for item in Advisory}


def _reader_path(bag_dir: Path) -> Path:
    if (bag_dir / 'metadata.yaml').is_file():
        return bag_dir
    files = sorted(bag_dir.glob('*.mcap'))
    if len(files) == 1:
        return files[0]
    raise ValueError(f'expected metadata.yaml or one MCAP file in {bag_dir}')


def _frames(reader_path: Path):
    """Yield (header stamp, depth msg, intrinsics), buffering before first CameraInfo."""
    ts = get_typestore(Stores.ROS2_JAZZY)
    with Reader(reader_path) as reader:
        topics = {conn.topic: conn for conn in reader.connections}
        if DEPTH not in topics or INFO not in topics:
            raise ValueError(f'{reader_path}: missing aligned depth or color CameraInfo')
        pending = []
        K = None
        connections = [topics[DEPTH], topics[INFO]]
        for conn, _, raw in bag_messages(reader, reader_path, connections):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == INFO:
                K = _intrinsics(msg)
                for depth_msg in pending:
                    stamp = _stamp_ns(depth_msg)
                    yield stamp, depth_msg, K
                pending.clear()
            elif K is None:
                pending.append(msg)
            else:
                yield _stamp_ns(msg), msg, K
        if pending:
            raise ValueError(f'{reader_path}: no CameraInfo preceded {len(pending)} depth frames')


def _stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def _replay(bag_dir: Path, max_forward: float, range_hysteresis: float,
            window_start_ns: int, window_end_ns: int):
    params = Params()
    old_params = DecisionParams(height_hysteresis_m=0.0)
    new_params = DecisionParams()
    detector_state = {}
    modes = {
        'old': {'params': old_params, 'range_hysteresis': 0.0, 'range_state': {}, 'decision_state': {}, 'rows': []},
        'new': {'params': new_params, 'range_hysteresis': range_hysteresis, 'range_state': {}, 'decision_state': {}, 'rows': []},
    }
    with readable_mcap(_reader_path(bag_dir)) as (reader_path, recovered_tail_bytes):
        for stamp, msg, K in _frames(reader_path):
            if stamp < window_start_ns or stamp > window_end_ns:
                continue
            frame, detector_state = process_frame(decode_depth(msg), K, params, detector_state)
            for mode in modes.values():
                limited, mode['range_state'] = in_range(
                    frame, max_forward, mode['range_hysteresis'], mode['range_state'])
                decision, mode['decision_state'] = update(
                    limited, mode['decision_state'], mode['params'])
                mode['rows'].append((stamp, int(decision.advisory),
                                     decision.distance_m, decision.height_m))
    for mode in modes.values():
        mode['rows'].sort(key=lambda row: row[0])
    modes['recovered_tail_bytes'] = recovered_tail_bytes
    return modes


def _live_packets(path: Path):
    rows = []
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            rows.append((int(row['frame_stamp_ns']), int(row['advisory'])))
    return rows


def _metrics(rows):
    if not rows:
        return {'transitions': []}
    start = rows[0][0]
    transitions = [
        ((stamp - start) / 1e9, code)
        for (stamp, code, *_), (_, previous, *__) in zip(rows[1:], rows[:-1])
        if code != previous
    ]
    return {'transitions': transitions}


def _fmt_value(value):
    return f'{value:.3f}' if np.isfinite(value) else 'n/a'


def report(archive: Path, output: Path, *, max_forward=1.3, range_hysteresis=0.10):
    results = []
    for run in RUNS:
        run_dir = archive / run
        live_rows = _live_packets(run_dir / 'packets.csv')
        if not live_rows:
            raise ValueError(f'{run_dir / "packets.csv"}: no live packets')
        window_start_ns = min(stamp for stamp, _ in live_rows)
        window_end_ns = max(stamp for stamp, _ in live_rows)
        window_s = (window_end_ns - window_start_ns) / 1e9
        modes = _replay(run_dir / 'camera-bag', max_forward, range_hysteresis,
                        window_start_ns, window_end_ns)
        live_rows = [(stamp, code, float('nan'), float('nan'))
                     for stamp, code in live_rows
                     if window_start_ns <= stamp <= window_end_ns]
        old_rows, new_rows = modes['old']['rows'], modes['new']['rows']
        old_metrics, new_metrics, live_metrics = map(_metrics, (old_rows, new_rows, live_rows))
        results.append((run, modes, live_rows, old_metrics, new_metrics, live_metrics,
                        window_s, window_start_ns))

    lines = [
        '# Lab day 2026-10-25 advisory flicker report', '',
        'Replay frames are limited to each run’s inclusive live packet timestamp range; transition rates use that window length.',
        f'Old replay has zero range/height hysteresis; new replay uses {range_hysteresis:.2f} m range hysteresis and 0.01 m height hysteresis. Approach transition distance/height are the new decision output at the transition frame (metres).', '',
        '| Run | Window (s) | Old transitions / min⁻¹ | New transitions / min⁻¹ | Live transitions / min⁻¹ |',
        '|---|---:|---:|---:|---:|',
    ]
    for run, modes, live_rows, old_m, new_m, live_m, window_s, _ in results:
        rate = window_s / 60.0 if window_s > 0 else float('nan')
        lines.append(
            f'| {run} | {window_s:.2f} | {len(old_m["transitions"])} / {len(old_m["transitions"])/rate:.1f} | '
            f'{len(new_m["transitions"])} / {len(new_m["transitions"])/rate:.1f} | '
            f'{len(live_m["transitions"])} / {len(live_m["transitions"])/rate:.1f} |'
        )
    lines.extend(['', '## New replay transitions', ''])
    for run, modes, live_rows, old_m, new_m, live_m, window_s, window_start_ns in results:
        events = []
        prev = None
        for stamp, code, distance, height in modes['new']['rows']:
            if prev is not None and code != prev:
                events.append((stamp, code, distance, height))
            prev = code
        lines.append(f'### {run} ({len(events)} transitions)')
        if run.startswith('4-approach'):
            lines.extend(f'- {(stamp-window_start_ns)/1e9:.3f}s {NAMES.get(code, code)}: distance {_fmt_value(distance)} m, height {_fmt_value(height)} m'
                         for stamp, code, distance, height in events)
        else:
            lines.extend(f'- {(stamp-window_start_ns)/1e9:.3f}s {NAMES.get(code, code)}'
                         for stamp, code, _, _ in events)
        if not events:
            lines.append('- none')
        lines.append('')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines))
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--max-forward', type=float, default=1.3)
    parser.add_argument('--range-hysteresis', type=float, default=0.10)
    args = parser.parse_args(argv)
    if args.max_forward < 0 or args.range_hysteresis < 0:
        parser.error('--max-forward and --range-hysteresis must be non-negative')
    results = report(args.archive, args.output, max_forward=args.max_forward,
                     range_hysteresis=args.range_hysteresis)
    print(f'wrote {args.output} ({len(results)} runs)')


if __name__ == '__main__':
    main()
