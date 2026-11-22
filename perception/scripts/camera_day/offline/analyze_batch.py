#!/usr/bin/env python3
"""Summarize online endpoint consistency and measured paced/compute FPS."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('new_analysis', type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('replay_metrics', Path(__file__).resolve().parents[1] / 'replay_metrics.py')
    metrics = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(metrics)
    progress = json.loads((args.batch / 'progress.json').read_text())
    if progress['status'] != 'COMPLETE' or len(progress['results']) != 3:
        raise ValueError('three successful replays required')
    expected_stamps = [int(row['source_ns']) for row in
                       csv.DictReader((args.batch / 'pairs.csv').open())]
    if len(expected_stamps) != 2215:
        raise ValueError('input manifest must contain 2215 pairs')
    args.new_analysis.mkdir()
    results = []
    table = ['| Replay | Endpoint m | Path m | Endpoint % | Tracked / processed | Resets* | Maps* | Merges | Tracked FPS** | Track-compute FPS*** | Loop detected |',
             '|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---|']
    for number in (1, 2, 3):
        run = args.batch / f'replay-{number}'
        log = args.batch / f'replay-{number}.log'
        result = metrics.analyze(f'Replay {number}', run / 'CameraTrajectory.txt',
                                 run / 'frames.csv', log, coverage='complete',
                                 frames_coverage='complete', log_coverage='complete',
                                 same_start_end='yes', expected_processed=2215)
        if result['errors']:
            raise ValueError(result['errors'])
        summary = json.loads((run / 'summary.json').read_text())
        lines = log.read_text().splitlines()
        # Exact native markers distinguish reset requests, new maps and merges.
        event_counts = {'active_map_reset_done': sum(line.strip() == 'LM: Active map reset, Done!!!' for line in lines),
                        'full_map_reset_done': sum(line.strip() == 'LM: Map reset, Done!!!' for line in lines),
                        'atlas_new_maps': sum(line.startswith('Creation of new map with last KF id: ') for line in lines),
                        'stereo_initializations': sum(line.startswith('New Map created with ') for line in lines),
                        'completed_merges': sum(line.strip() == 'Merge finished!' for line in lines)}
        frames = list(csv.DictReader((run / 'frames.csv').open()))
        if len(frames) != 2215 or [int(r['processed']) for r in frames] != list(range(1, 2216)):
            raise ValueError('processed coverage gap')
        attempts = list(csv.DictReader((run / 'attempts.csv').open()))
        if ([int(row['source_ns']) for row in frames] != expected_stamps or
                [int(row['source_ns']) for row in attempts] != expected_stamps):
            raise ValueError('runner source headers differ from input manifest')
        result.update(execution=progress['results'][number-1], measured=summary,
                      native_event_counts=event_counts,
                      map_id_changes=list(csv.DictReader((run / 'map-events.csv').open())),
                      physical_alignment='owner-reported only',
                      clock_scopes={'tracked_fps_paced_span': 'tracked frames / first Track start to last Track end on steady clock; includes pacing and IO gaps',
                                    'tracked_per_track_compute_second': 'tracked frames / sum of all TrackStereo durations; excludes pacing/IO but includes calls that were not tracked',
                                    'source_rate': 'recorded camera clock; not compute throughput'},
                      limitations=['Online map coordinates can jump at resets/merges/loop closure; not pure drift accuracy.',
                                   'Mac Docker platform differs from the Pi; no Pi performance claim.',
                                   'Lossless input and all calls processed do not imply every frame tracked.'])
        g, t = result['geometry'], result['tracking']
        table.append(f"| {number} | {g['endpoint_drift_m']:.6f} | {g['path_length_m']:.3f} | {g['endpoint_drift_percent_path']:.3f} | {summary['tracked']} / {summary['processed']} | {event_counts['active_map_reset_done'] + event_counts['full_map_reset_done']} | {event_counts['atlas_new_maps']} | {event_counts['completed_merges']} | {t['trackedFPS']:.3f} | {summary['tracked_per_track_compute_second']:.3f} | {result['loop']['loop_closure_detected']} |")
        results.append(result)
    table += ['', '*Exact native log markers, including initial map. Current-map ID changes are saved separately and may also reflect merging.',
              '**Measured steady first-start to last-end span, including pacing/IO. ***Tracked / summed TrackStereo compute seconds.',
              'Endpoints/path use online poses and may include map jumps; owner-reported closed path is not ground truth.', '']
    (args.new_analysis / 'results.json').write_text(json.dumps({'results': results}, indent=2) + '\n')
    (args.new_analysis / 'results.md').write_text('\n'.join(table))
    print('\n'.join(table))


if __name__ == '__main__':
    main()
