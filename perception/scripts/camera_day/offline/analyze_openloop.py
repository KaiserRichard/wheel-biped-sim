#!/usr/bin/env python3
"""Report contiguous online segments, per-map endpoints and the first pre-loss segment."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

import numpy as np

from align_replays import read_pairs, read_replay, sha256, tracking_diagnostics


def loop_closing_enabled(settings):
    values = re.findall(r'^\s*loopClosing\s*:\s*([^#\n]*)', Path(settings).read_text(), re.M)
    if len(values) > 1 or (values and not re.fullmatch(r'[+-]?[0-9]+', values[0].strip())):
        raise ValueError('loopClosing must be one integer settings key')
    return not values or int(values[0]) != 0  # upstream default is enabled


def log_events(path):
    lines = Path(path).read_text().splitlines()
    return {'loop_detections': sum('*Loop detected' in line for line in lines),
            'merge_detections': sum('*Merge detected' in line for line in lines),
            'completed_merges': sum(line.strip() == 'Merge finished!' for line in lines),
            'completed_resets': sum(line.strip() in ('LM: Active map reset, Done!!!',
                                                     'LM: Map reset, Done!!!') for line in lines),
            'new_map_markers_including_initial': sum(line.startswith('Creation of new map with last KF id: ')
                                                    for line in lines)}


def split_segments(frames, poses, jump_threshold_m, jump_threshold_deg):
    """Partition all frames by map/state; tracked geometry never crosses a break."""
    if (not np.isfinite(jump_threshold_m) or jump_threshold_m <= 0
            or not np.isfinite(jump_threshold_deg) or not 0 < jump_threshold_deg <= 180):
        raise ValueError('jump thresholds must be finite: metres >0, degrees in (0,180]')
    spans, groups, boundaries = [], [], []
    active = None
    for index, row in enumerate(frames):
        key = (row['map_id'], row['tracking_state'], row['pose_published'])
        if index == 0 or key != previous_key:
            spans.append({'map_id': key[0], 'tracking_state': key[1], 'pose_published': key[2],
                          'start_source_ns': row['source_ns'], 'end_source_ns': row['source_ns'], 'frame_count': 0})
        spans[-1]['end_source_ns'] = row['source_ns']
        spans[-1]['frame_count'] += 1
        reasons = []
        if index:
            prev = frames[index - 1]
            if prev['map_id'] != row['map_id']:
                reasons.append('map_change')
            if prev['tracking_state'] != row['tracking_state']:
                reasons.append('state_change')
            if prev['pose_published'] and not row['pose_published']:
                reasons.append('tracking_loss_or_unavailable_pose')
            if not prev['pose_published'] and row['pose_published']:
                reasons.append('tracking_resumed')
            jump = {}
            if prev['pose_published'] and row['pose_published'] and prev['map_id'] == row['map_id']:
                a, b = poses[prev['source_ns']], poses[row['source_ns']]
                distance = float(np.linalg.norm(b[:3] - a[:3]))
                # Quaternion sign does not change orientation. Normalize float runner output.
                dot = abs(float(a[3:] @ b[3:])) / (np.linalg.norm(a[3:]) * np.linalg.norm(b[3:]))
                angle = float(np.degrees(2 * np.arccos(np.clip(dot, 0, 1))))
                if distance > jump_threshold_m or angle > jump_threshold_deg:
                    reasons.append('pose_jump')
                    jump = {'translation_step_m': distance, 'rotation_step_deg': angle}
            if reasons:
                boundaries.append({'from_source_ns': prev['source_ns'], 'to_source_ns': row['source_ns'],
                                   'reasons': reasons, **jump})
        if reasons and active is not None:
            active['end_reason'] = reasons
            active = None
        if row['pose_published']:
            if active is None:
                active = {'segment_id': len(groups) + 1, 'map_id': row['map_id'], 'tracking_state': row['tracking_state'],
                          'start_frame_index': index, 'stamps': [], 'end_reason': ['end_of_record']}
                groups.append(active)
            active['stamps'].append(row['source_ns'])
            active['end_frame_index'] = index
        else:
            active = None
        previous_key = key
    segments = []
    for group in groups:
        stamps = group.pop('stamps')
        points = np.array([poses[stamp][:3] for stamp in stamps])
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        endpoint = float(np.linalg.norm(points[-1] - points[0]))
        path = float(np.sum(steps))
        segments.append({**group, 'start_source_ns': stamps[0], 'end_source_ns': stamps[-1],
                         'tracked_count': len(stamps), 'duration_s': (stamps[-1] - stamps[0]) / 1e9,
                         'start_position_m': points[0].tolist(), 'end_position_m': points[-1].tolist(),
                         'endpoint_displacement_m': endpoint, 'within_segment_path_m': path,
                         'endpoint_percent_path': 100 * endpoint / path if path > 0 else None,
                         'max_within_segment_step_m': float(np.max(steps)) if len(steps) else None,
                         'interpretation': 'segment endpoint displacement; not independently a closed-loop drift metric'})
    return spans, segments, boundaries


def analyze(replay, pairs, settings, log, *, expected_pairs=2215, jump_threshold_m=0.25,
            jump_threshold_deg=45.0, physical_return='unknown'):
    if physical_return not in ('unknown', 'owner-reported'):
        raise ValueError('physical_return must be unknown or owner-reported')
    frames, poses = read_replay(replay)
    schedule = read_pairs(pairs)
    if len(schedule) != expected_pairs or [r['source_ns'] for r in frames] != schedule:
        raise ValueError('frames must cover every expected pair in original order, with no trimming')
    enabled = loop_closing_enabled(settings)
    events = log_events(log)
    spans, segments, boundaries = split_segments(frames, poses, jump_threshold_m, jump_threshold_deg)
    first = segments[0] if segments else None
    full = bool(first and first['start_frame_index'] == 0 and first['end_frame_index'] == len(frames) - 1)
    reasons = []
    if not full:
        reasons.append('first tracked segment does not cover the full physical-start-to-return record')
    if enabled:
        reasons.append('settings do not disable loop closing/place recognition')
    if any(events[k] for k in ('loop_detections', 'merge_detections', 'completed_merges', 'completed_resets')):
        reasons.append('native log contains loop/merge/reset markers')
    if events['new_map_markers_including_initial'] > 1:
        reasons.append('native log contains additional map creation')
    if physical_return != 'owner-reported':
        reasons.append('physical return to the start is not supplied')
    if first is None or first['tracked_count'] < 2 or first['within_segment_path_m'] <= 0:
        reasons.append('insufficient moving tracked poses')
    maps = []
    for map_id in sorted({r['map_id'] for r in frames}):
        rows = [r for r in frames if r['map_id'] == map_id]
        parts = [s for s in segments if s['map_id'] == map_id]
        maps.append({'map_id': map_id, 'frame_count': len(rows),
                     'tracked_count': sum(r['pose_published'] for r in rows),
                     'tracking_state_counts': dict(Counter(str(r['tracking_state']) for r in rows)),
                     'segment_ids': [s['segment_id'] for s in parts],
                     'sum_within_segment_path_m': sum(s['within_segment_path_m'] for s in parts),
                     'interpretation': 'per-map segment diagnostics; no endpoint across separate segments'})
    inputs = [Path(replay) / 'frames.csv', Path(replay) / 'CameraTrajectory.txt', pairs, settings, log]
    return {
        'schema_version': 1, 'measurement': 'online camera-to-world segment endpoints; not ground-truth accuracy',
        'all_expected_pairs_processed': True, 'expected_pairs': expected_pairs,
        **tracking_diagnostics(frames), 'loop_closing_enabled_from_settings': enabled,
        'native_log_events': events, 'physical_return': physical_return,
        'jump_threshold_m': jump_threshold_m, 'jump_threshold_deg': jump_threshold_deg,
        'state_spans': spans, 'tracked_segments': segments, 'boundaries': boundaries, 'per_map': maps,
        'start_segment_before_first_break': first,
        'closed_path_drift_candidate': {
            'eligible': not reasons, 'covers_full_record': full,
            'endpoint_m': first['endpoint_displacement_m'] if not reasons else None,
            'ineligibility_reasons': reasons,
            'meaning': 'conditional open-loop return endpoint only; physical return is unmeasured, local BA remains active'},
        'input_sha256': {str(Path(p).resolve()): sha256(p) for p in inputs},
        'implementation_sha256': {str(p): sha256(p) for p in (Path(__file__).resolve(),
                                  Path(__file__).resolve().with_name('align_replays.py'))},
        'limitations': ['No geometry crosses a loss, map/state change or above-threshold pose jump.',
                        'Jump thresholds are diagnostic assumptions, not validated motion bounds; smaller corrections may remain.',
                        'Segment displacement before a loss/map change is not closed-loop drift without physical start-to-return coverage.',
                        'Coverage conservatively requires the entire recorded interval; no guessed return stamp or trimming.',
                        'Settings identify requested upstream mode; maintainer must verify dispatch exit, runtime provenance and complete log.',
                        'Local mapping/bundle adjustment remains active with loop closing off; online poses are not raw odometry.',
                        'Owner-reported physical alignment and unknown ground truth prevent any ground-truth accuracy claim.'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay', type=Path)
    parser.add_argument('pairs', type=Path)
    parser.add_argument('new_output', type=Path)
    parser.add_argument('--settings', required=True, type=Path)
    parser.add_argument('--log', required=True, type=Path)
    parser.add_argument('--expected-pairs', type=int, default=2215)
    parser.add_argument('--jump-threshold-m', type=float, default=0.25)
    parser.add_argument('--jump-threshold-deg', type=float, default=45.0)
    parser.add_argument('--physical-return', choices=('unknown', 'owner-reported'), default='unknown')
    args = parser.parse_args()
    report = analyze(args.replay, args.pairs, args.settings, args.log, expected_pairs=args.expected_pairs,
                     jump_threshold_m=args.jump_threshold_m, jump_threshold_deg=args.jump_threshold_deg,
                     physical_return=args.physical_return)
    args.new_output.mkdir()
    (args.new_output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = ['# Open-loop segment diagnostics', '', report['measurement'], '',
             f"Processed {report['processed_count']}/{report['expected_pairs']}; tracked {report['tracked_count']}; "
             f"loss transitions {report['loss_transitions']}; map IDs {report['map_ids']}.",
             f"Requested loop closing enabled: {report['loop_closing_enabled_from_settings']}.", '',
             '| Segment | Map | Tracked | Start source ns | End source ns | Endpoint m | Path m | End reason |',
             '|---|---:|---:|---|---|---:|---:|---|']
    for s in report['tracked_segments']:
        lines.append(f"| {s['segment_id']} | {s['map_id']} | {s['tracked_count']} | {s['start_source_ns']} | "
                     f"{s['end_source_ns']} | {s['endpoint_displacement_m']:.9f} | {s['within_segment_path_m']:.9f} | "
                     f"{', '.join(s['end_reason'])} |")
    candidate = report['closed_path_drift_candidate']
    lines += ['', f"First pre-break segment: {report['start_segment_before_first_break']['segment_id'] if report['start_segment_before_first_break'] else 'none'}.",
              f"Conditional closed-path drift candidate eligible: {candidate['eligible']}.",
              f"Conditional return endpoint m: {candidate['endpoint_m']}.",
              *candidate['ineligibility_reasons'], '',
              'Per-map counts, state spans, break stamps, native markers and input hashes: `report.json`.', '',
              *report['limitations'], '']
    (args.new_output / 'report.md').write_text('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
