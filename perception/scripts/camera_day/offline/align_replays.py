#!/usr/bin/env python3
"""Fit replay B positions to A with SE(3), scale fixed at one; agreement, not GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def integer(text, label, minimum=0):
    pattern = r'(0|[1-9][0-9]*)' if minimum >= 0 else r'(0|-?[1-9][0-9]*)'
    if not re.fullmatch(pattern, text or ''):
        raise ValueError(f'{label}: expected decimal integer, got {text!r}')
    value = int(text)
    if not minimum <= value <= 2**63 - 1:
        raise ValueError(f'{label}: integer out of range')
    return value


def source_ns(text):
    """Parse decimal seconds exactly, without passing epoch stamps through float."""
    match = re.fullmatch(r'(0|[1-9][0-9]*)\.([0-9]{1,9})', text)
    if not match:
        raise ValueError(f'malformed trajectory stamp: {text!r}; need decimal seconds <=9 places')
    return integer(str(int(match[1]) * 10**9 + int(match[2].ljust(9, '0'))), 'source_ns')


def read_csv(path, required):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if len(set(fields)) != len(fields) or not set(required).issubset(fields):
            raise ValueError(f'{path}: missing or duplicate CSV columns')
        rows = list(reader)
    if not rows or any(None in row or any(v is None for v in row.values()) for row in rows):
        raise ValueError(f'{path}: empty or malformed CSV rows')
    return rows


def increasing(stamps, label):
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f'{label}: source stamps must be strictly increasing; no sorting/duplicates')


def read_pairs(path):
    rows = read_csv(path, ('source_ns', 'left', 'right'))
    stamps = [integer(row['source_ns'], 'pairs source_ns') for row in rows]
    increasing(stamps, str(path))
    return stamps


def read_replay(directory):
    directory = Path(directory)
    frames = read_csv(directory / 'frames.csv',
                      ('source_ns', 'tracking_state', 'pose_published', 'processed',
                       'application_dropped', 'map_id'))
    for index, row in enumerate(frames, 1):
        for key, minimum in [('source_ns', 0), ('tracking_state', -1), ('processed', 1),
                             ('application_dropped', 0), ('map_id', -1)]:
            row[key] = integer(row[key], key, minimum)
        if row['tracking_state'] not in (-1, 0, 1, 2, 3, 4, 5):
            raise ValueError('unknown tracking_state')
        if row['pose_published'] not in ('true', 'false'):
            raise ValueError('pose_published must be true or false')
        row['pose_published'] = row['pose_published'] == 'true'
        if row['processed'] != index or row['application_dropped'] != 0:
            raise ValueError('non-contiguous processed count or application drops')
        if row['pose_published'] and (row['tracking_state'] != 2 or row['map_id'] < 0):
            raise ValueError('published pose requires tracking state 2 and known map')
    increasing([row['source_ns'] for row in frames], str(directory / 'frames.csv'))
    poses = {}
    for number, line in enumerate((directory / 'CameraTrajectory.txt').read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(f'trajectory line {number}: expected stamp tx ty tz qx qy qz qw')
        stamp = source_ns(parts[0])
        if poses and stamp <= next(reversed(poses)):
            raise ValueError('trajectory source stamps must be strictly increasing')
        values = np.array([float(value) for value in parts[1:]])
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 1e-3:
            raise ValueError(f'trajectory line {number}: nonfinite pose or nonunit quaternion')
        poses[stamp] = values
    if list(poses) != [row['source_ns'] for row in frames if row['pose_published']]:
        raise ValueError('trajectory stamps differ from published frames')
    return frames, poses


def umeyama_se3(source, target):
    """Least-squares target = R @ source + t, column vectors, det(R)=+1.

    Centered covariance is target.T @ source / N. U S Vt = covariance;
    R = U diag(1,1,det(U Vt)) Vt, t = mean(target) - R mean(source).
    Scale is never estimated. Rank two (noncollinear planar) is sufficient;
    reject rank <2 at relative singular-value tolerance 1e-10.
    """
    source, target = np.asarray(source, dtype=float), np.asarray(target, dtype=float)
    if (source.ndim != 2 or source.shape[1:] != (3,) or source.shape != target.shape
            or len(source) < 3 or not np.isfinite(source).all() or not np.isfinite(target).all()):
        raise ValueError('need >=3 finite paired Nx3 positions of equal shape')
    a, b = source - source.mean(axis=0), target - target.mean(axis=0)
    covariance = b.T @ a / len(a)
    spectra = [np.linalg.svd(x, compute_uv=False) for x in (a, b, covariance)]
    ranks = [int(np.count_nonzero(s > s[0] * 1e-10)) for s in spectra]
    if min(ranks) < 2:
        raise ValueError('degenerate trajectory/correspondence: rank <2 (stationary or collinear)')
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = 1 if np.linalg.det(u @ vt) >= 0 else -1
    rotation = u @ correction @ vt
    translation = target.mean(axis=0) - rotation @ source.mean(axis=0)
    return rotation, translation, {'source_rank': ranks[0], 'target_rank': ranks[1],
                                   'covariance_rank': ranks[2],
                                   'covariance_singular_values': singular.tolist(),
                                   'relative_rank_tolerance': 1e-10,
                                   'reflection_corrected': bool(correction[2, 2] < 0)}


def error_stats(errors):
    errors = np.asarray(errors, dtype=float)
    return {'count': len(errors), 'rms_m': float(np.sqrt(np.mean(errors**2))),
            'median_m': float(np.median(errors)), 'p95_m': float(np.percentile(errors, 95)),
            'max_m': float(np.max(errors))}


def tracking_diagnostics(frames):
    return {'processed_count': len(frames),
            'tracked_count': sum(row['pose_published'] for row in frames),
            'map_ids': sorted({row['map_id'] for row in frames}),
            'map_changes': sum(a['map_id'] != b['map_id'] for a, b in zip(frames, frames[1:])),
            'loss_transitions': sum(a['pose_published'] and not b['pose_published']
                                    for a, b in zip(frames, frames[1:]))}


def align(directory_a, directory_b):
    frames_a, poses_a = read_replay(directory_a)
    frames_b, poses_b = read_replay(directory_b)
    shared = [stamp for stamp in poses_a if stamp in poses_b]  # validated input order
    if len(shared) < 3:
        raise ValueError('fewer than three exact shared tracked source stamps')
    target = np.array([poses_a[stamp][:3] for stamp in shared])
    source = np.array([poses_b[stamp][:3] for stamp in shared])
    rotation, translation, diagnostic = umeyama_se3(source, target)
    before = np.linalg.norm(source - target, axis=1)
    after = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    report = {
        'schema_version': 1, 'measurement': 'same-input online camera-position agreement; not ground-truth accuracy',
        'matching': 'exact integer source-nanosecond intersection; no interpolation, sorting or trimming',
        'same_source_stamp_schedule': [r['source_ns'] for r in frames_a] == [r['source_ns'] for r in frames_b],
        'shared_count': len(shared), 'only_a_tracked_count': len(poses_a) - len(shared),
        'only_b_tracked_count': len(poses_b) - len(shared),
        'first_shared_source_ns': shared[0], 'last_shared_source_ns': shared[-1],
        'replay_a': tracking_diagnostics(frames_a), 'replay_b': tracking_diagnostics(frames_b),
        'fit': {'direction': 'B to A: p_A = R_B_to_A @ p_B + t_B_to_A',
                'scale': 1.0, 'scale_fitted': False, 'rotation_matrix': rotation.tolist(),
                'translation_m': translation.tolist(), 'rotation_determinant': float(np.linalg.det(rotation)),
                **diagnostic},
        'before_alignment': error_stats(before), 'after_alignment': error_stats(after),
        'input_sha256': {str(Path(d).resolve() / name): sha256(Path(d) / name)
                         for d in (directory_a, directory_b) for name in ('frames.csv', 'CameraTrajectory.txt')},
        'implementation_sha256': {str(Path(__file__).resolve()): sha256(__file__)},
        'numpy_version': np.__version__,
        'limitations': ['One rigid fit on all shared tracked positions; no orientation-error metric.',
                        'Online poses may include loop corrections or map discontinuities; fitting does not remove these.',
                        'Before-fit errors depend on arbitrary world-frame choice; after-fit errors measure agreement only.',
                        'Fit and residuals use the same samples; no independent ground truth or accuracy claim.'],
    }
    return report, zip(shared, before, after)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay_a', type=Path)
    parser.add_argument('replay_b', type=Path)
    parser.add_argument('new_output', type=Path)
    args = parser.parse_args()
    report, residuals = align(args.replay_a, args.replay_b)
    args.new_output.mkdir()  # refuse overwriting earlier analysis
    (args.new_output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    with (args.new_output / 'residuals.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_ns', 'before_translation_error_m', 'after_translation_error_m'])
        writer.writerows(residuals)
    lines = ['# Rigid replay position agreement', '', report['measurement'], '',
             f"Exact shared tracked stamps: {report['shared_count']}; same source schedule: {report['same_source_stamp_schedule']}.",
             'B to A, scale fixed at 1; RMS is sqrt(mean(squared Euclidean distances)).', '',
             '| Alignment | RMS m | Median m | p95 m | Max m | Count |', '|---|---:|---:|---:|---:|---:|']
    for key in ('before_alignment', 'after_alignment'):
        s = report[key]
        lines.append(f"| {key} | {s['rms_m']:.9f} | {s['median_m']:.9f} | {s['p95_m']:.9f} | {s['max_m']:.9f} | {s['count']} |")
    lines += ['', 'Fitted transform (column vectors), ranks and all input hashes: `report.json`.', '',
              *report['limitations'], '']
    (args.new_output / 'report.md').write_text('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
