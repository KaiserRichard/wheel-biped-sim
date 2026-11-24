#!/usr/bin/env python3
"""Repeat isolated headless engines; preserve failed trials and provenance."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import string
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.frames.convert_trajectory import parse_stamp, read_samples
from tools.frames.frames import (Transform, Sample, convert, base_imu_camera,
                                 sdk_ir1_to_imu)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def git_revision(root):
    """Read HEAD/refs only, including linked worktrees; never invoke Git."""
    try:
        dotgit = root / '.git'
        gitdir = dotgit if dotgit.is_dir() else (root / dotgit.read_text().strip()[8:]).resolve()
        common = gitdir
        if (gitdir / 'commondir').exists():
            common = (gitdir / (gitdir / 'commondir').read_text().strip()).resolve()
        head = (gitdir / 'HEAD').read_text().strip()
        if not head.startswith('ref: '):
            return {'revision': head, 'worktree_state': 'UNKNOWN (not inspected)'}
        ref = head[5:]
        for directory in (gitdir, common):
            if (directory / ref).is_file():
                return {'revision': (directory / ref).read_text().strip(),
                        'ref': ref, 'worktree_state': 'UNKNOWN (not inspected)'}
        for line in (common / 'packed-refs').read_text().splitlines():
            if line.endswith(' ' + ref):
                return {'revision': line.split()[0], 'ref': ref,
                        'worktree_state': 'UNKNOWN (not inspected)'}
    except OSError:
        pass
    return {'revision': None, 'worktree_state': 'UNKNOWN'}


def stamp():
    return {'utc': datetime.now(timezone.utc).isoformat(), 'utc_clock_domain': 'host_utc',
            'monotonic_ns': time.monotonic_ns(), 'monotonic_clock_domain': 'host_monotonic'}


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f'duplicate YAML key: {key}')
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def safe_artifact(value):
    if not isinstance(value, str) or not value:
        raise ValueError('artifact requires a relative filename')
    path = Path(value)
    if path.is_absolute() or '..' in path.parts or path == Path('.'):
        raise ValueError('artifact must stay inside repeat directory')
    if path.parts[0] in {'receipt.json', 'stdout.log', 'stderr.log', 'base_link.jsonl',
                         'evaluation', 'evaluation_estimate.tum', 'evaluation.stdout.log', 'evaluation.stderr.log'}:
        raise ValueError('artifact conflicts with harness output')
    return value


def load_spec(path, output):
    path = path.resolve()
    spec = yaml.load(path.read_text(), Loader=UniqueLoader)
    if not isinstance(spec, dict):
        raise ValueError('spec must be a mapping')
    if type(spec.get('repeats')) is not int or spec['repeats'] < 1:
        raise ValueError('repeats must be a positive integer')
    if spec.get('headless') is not True:
        raise ValueError('headless: true must attest that the supplied engine disables its viewer')
    spec['timeout_s'] = positive(spec.get('timeout_s', 300), 'timeout_s')
    spec['jump_threshold_m'] = positive(spec.get('jump_threshold_m', 1), 'jump_threshold_m')
    if not isinstance(spec.get('clock_domain'), str) or not spec['clock_domain'].strip():
        raise ValueError('trajectory clock_domain is required')
    for key in ('binary', 'vocabulary', 'settings', 'dataset', 'times', 'dataset_manifest'):
        spec[key] = str((path.parent / spec[key]).resolve())
        candidate = Path(spec[key])
        if not (candidate.is_dir() if key == 'dataset' else candidate.is_file()):
            raise ValueError(f'{key} does not exist: {candidate}')
        if output == candidate or output in candidate.parents or candidate in output.parents:
            raise ValueError('output and inputs must not overlap')
    if not os.access(spec['binary'], os.X_OK):
        raise ValueError('binary must be executable')
    if not any(line.split('#', 1)[0].strip() for line in Path(spec['times']).read_text().splitlines()):
        raise ValueError('times file is empty')
    args = spec.get('args')
    if not isinstance(args, list) or not all(isinstance(arg, str) for arg in args):
        raise ValueError('args must be an argv list of strings, never a shell command')
    allowed = {'binary', 'vocabulary', 'settings', 'dataset', 'times', 'run_dir', 'repeat'}
    for arg in args:
        for _, field, fmt, conversion in string.Formatter().parse(arg):
            if field is not None and (field not in allowed or fmt or conversion):
                raise ValueError(f'unsupported template field: {field}')
    traj = spec['trajectory']
    safe_artifact(traj['path'])
    for key, choices in [('time_unit', {'s', 'ns'}),
                         ('sensor_frame', {'camera_optical', 'imu_camera'}),
                         ('pose_direction', {'sensor-to-world', 'world-to-sensor'}),
                         ('quaternion_order', {'xyzw', 'wxyz'})]:
        if traj[key] not in choices:
            raise ValueError(f'unsupported trajectory {key}')
    if traj.get('epochs'):
        safe_artifact(traj['epochs'])
        if traj['epochs'] == traj['path']:
            raise ValueError('epochs and trajectory must differ')
    for extra in spec.get('extra_trajectories', []):
        safe_artifact(extra)
    if spec.get('mount'):
        if not traj.get('epochs'):
            raise ValueError('mount conversion requires a per-pose epochs sidecar')
        spec['mount'] = str((path.parent / spec['mount']).resolve())
        mount_transform(spec['mount'], traj['sensor_frame'])
    if spec.get('ground_truth'):
        gt = spec['ground_truth']
        for key in ('path', 'evaluator'):
            gt[key] = str((path.parent / gt[key]).resolve())
            if not Path(gt[key]).is_file():
                raise ValueError(f'ground_truth {key} does not exist')
        if gt['clock_domain'] != spec['clock_domain']:
            raise ValueError('ground truth clock domain differs; map clocks externally first')
        expected = 'base_link' if spec.get('mount') else traj['sensor_frame']
        if gt['frame'] != expected:
            raise ValueError('ground truth child frame must match evaluated trajectory')
        if not isinstance(gt.get('clock_assumption'), str) or not gt['clock_assumption'].strip():
            raise ValueError('ground truth requires an explicit clock_assumption')
        if gt.get('format', 'tum') not in {'tum', 'orbslam3', 'euroc_gt'}:
            raise ValueError('unsupported reference format')
        if gt.get('time_unit', 's') not in {'s', 'ns'}:
            raise ValueError('unsupported reference time unit')
        if gt.get('align', 'se3') not in {'se3', 'sim3', 'none'}:
            raise ValueError('unsupported alignment')
        positive(gt.get('timeout_s', 60), 'evaluation timeout_s')
    optional = [spec.get('mount')]
    if spec.get('ground_truth'):
        optional += [spec['ground_truth']['path'], spec['ground_truth']['evaluator']]
    for value in optional:
        if value:
            candidate = Path(value)
            if output == candidate or output in candidate.parents or candidate in output.parents:
                raise ValueError('output and optional inputs must not overlap')
    return spec


def mount_transform(path, sensor):
    mount = json.loads(Path(path).read_text())
    def check(record):
        if record['status'] not in {'UNKNOWN', 'PROPOSED', 'MEASURED'} or not record['provenance']:
            raise ValueError('mount/extrinsic requires status and provenance')
    check(mount)
    transform = Transform('base_link', 'camera_optical', mount['T_base_camera'])
    if sensor == 'imu_camera':
        sdk = mount['sdk_ir1_to_imu']
        check(sdk)
        transform = base_imu_camera(transform, sdk_ir1_to_imu(
            sdk['rotation_column_major'], sdk['translation_m']))
    return transform


def contained_file(directory, name):
    path = directory / name
    if not path.is_file():
        raise ValueError(f'missing artifact: {name}')
    if path.is_symlink() or directory.resolve() not in path.resolve().parents:
        raise ValueError(f'artifact escapes repeat directory: {name}')
    return path


def poses(directory, spec):
    traj = spec['trajectory']
    path = contained_file(directory, traj['path'])
    if traj.get('epochs'):
        samples = read_samples(path, contained_file(directory, traj['epochs']),
                               unit=traj['time_unit'], sensor=traj['sensor_frame'],
                               direction=traj['pose_direction'], order=traj['quaternion_order'])
        if any(s.clock_domain != spec['clock_domain'] for s in samples):
            raise ValueError('trajectory epoch clock domain disagrees with spec')
        return samples, True
    # Placeholder epoch is internal only. No claimed map/reset identity is emitted.
    samples = []
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        row = line.split('#', 1)[0].split()
        if not row:
            continue
        if len(row) != 8:
            raise ValueError(f'line {lineno}: expected eight trajectory columns')
        pose = Transform.from_pose('slam_world', traj['sensor_frame'],
                                   [float(x) for x in row[1:4]],
                                   [float(x) for x in row[4:]], order=traj['quaternion_order'])
        if traj['pose_direction'] == 'world-to-sensor':
            pose = Transform(traj['sensor_frame'], 'slam_world', pose.matrix).inverse()
        samples.append(Sample(parse_stamp(row[0], traj['time_unit']), spec['clock_domain'],
                              'UNKNOWN', 0, pose))
    if not samples:
        raise ValueError('empty trajectory')
    return samples, False


def continuity(samples, observed_epochs, threshold):
    length, jumps, segments, resets = 0.0, 0, 1, 0
    order_breaks = 0
    for previous, current in zip(samples, samples[1:]):
        boundary = (current.map_id, current.reset_counter) != (previous.map_id, previous.reset_counter)
        if previous.clock_domain != current.clock_domain:
            raise ValueError('cross-domain arithmetic refused')
        if current.reset_counter != previous.reset_counter:
            resets += 1
        if current.stamp_ns <= previous.stamp_ns:
            order_breaks += 1
            if observed_epochs and not boundary:
                raise ValueError('nonincreasing timestamp inside an observed epoch')
            boundary = True
        if boundary:
            segments += 1
            continue
        distance = float(np.linalg.norm(current.pose.matrix[:3, 3] - previous.pose.matrix[:3, 3]))
        if not math.isfinite(distance):
            raise ValueError('trajectory difference overflows finite metric range')
        length += distance
        if not math.isfinite(length):
            raise ValueError('trajectory path length overflows finite metric range')
        jumps += distance > threshold
    return {'evidence': 'MEASURED (diagnostic calculation)', 'pose_count': len(samples),
            'clock_domain': samples[0].clock_domain, 'path_length_m': length,
            'jump_count': jumps, 'jump_threshold_m': threshold,
            'segment_count': segments if observed_epochs else None,
            'reset_count': resets if observed_epochs else None,
            'diagnostic_timestamp_segment_count': segments,
            'timestamp_order_breaks': order_breaks,
            'epoch_metadata': 'OBSERVED' if observed_epochs else 'UNKNOWN',
            'limitation': 'Unobserved resets or map jumps may contaminate path length; no accuracy claim.'}


def log_counts(text):
    maps = re.findall(r'Creation of new map with id\s*[:=]?\s*(\d+)', text, re.I)
    counts = re.findall(r'(?:Number of maps|Map count)\s*[:=]\s*(\d+)|There are\s+(\d+)\s+maps in the atlas', text, re.I)
    reported = [next(value for value in match if value) for match in counts]
    return {'evidence': 'MEASURED (log phrase counts; not per-frame tracking states)',
            'imu_initialized': len(re.findall(r'IMU initialized', text, re.I)),
            'reset_messages': len(re.findall(r'Reseting|Resetting', text, re.I)),
            'fail_to_track': len(re.findall(r'Fail to track', text, re.I)),
            'map_creation_messages': len(maps), 'observed_map_ids': sorted(set(maps)),
            'distinct_logged_map_count': len(set(maps)),
            'map_count_reported': int(reported[-1]) if reported else None,
            'limitation': 'Log messages can repeat or be absent; not an authoritative reset/state history.'}


def launch(command, directory, timeout_s, prefix=''):
    """POSIX trials own their process group, including after leader exit."""
    start = stamp()
    code, timed_out, error = None, False, None
    env = dict(os.environ)
    for key in ('DISPLAY', 'WAYLAND_DISPLAY'):
        env.pop(key, None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    with (directory / f'{prefix}stdout.log').open('wb') as stdout, (directory / f'{prefix}stderr.log').open('wb') as stderr:
        try:
            process = subprocess.Popen(command, cwd=directory, env=env, stdin=subprocess.DEVNULL,
                                       stdout=stdout, stderr=stderr, start_new_session=True)
            try:
                code = process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                timed_out = True
            finally:
                # A successful leader can leave writers alive. Kill its group
                # before closing logs or reading/hashing any trial artifacts.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if code is None:
                    code = process.wait()
        except OSError as exc:
            error = str(exc)
    end = stamp()
    return {'command': command, 'start': start, 'end': end,
            'wall_time_s': (end['monotonic_ns'] - start['monotonic_ns']) / 1e9,
            'exit_code': code, 'timed_out': timed_out, 'launch_error': error}


def evaluate(directory, samples, spec):
    gt = spec['ground_truth']
    if any((a.map_id, a.reset_counter) != (b.map_id, b.reset_counter) or b.stamp_ns <= a.stamp_ns
           for a, b in zip(samples, samples[1:])):
        raise ValueError('ATE/RPE across reset/segments refused; evaluate aligned epochs separately')
    # Normalize direction, quaternion order, units and optional mount before reuse.
    est = directory / 'evaluation_estimate.tum'
    if est.exists() or est.is_symlink():
        raise ValueError('engine artifact conflicts with normalized evaluation input')
    # Davenport symmetric eigenproblem: stable matrix -> xyzw, including pi rotations.
    rows = []
    for sample in samples:
        r = sample.pose.matrix[:3, :3]
        k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                      [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                      [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                      [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()]]) / 3
        _, vectors = np.linalg.eigh(k)
        q = vectors[:, -1]
        magnitude = abs(sample.stamp_ns)
        seconds = ('-' if sample.stamp_ns < 0 else '') + f'{magnitude // 10**9}.{magnitude % 10**9:09d}'
        rows.append(' '.join([seconds] + [format(float(x), '.17g') for x in [*sample.pose.matrix[:3, 3], *q]]))
    est.write_text('\n'.join(rows) + '\n')
    if (directory / 'evaluation').exists() or (directory / 'evaluation').is_symlink():
        raise ValueError('engine artifact conflicts with evaluation output')
    frame = 'base_link' if spec.get('mount') else spec['trajectory']['sensor_frame']
    command = [sys.executable, gt['evaluator'], 'single', '--est', str(est), '--est-format', 'tum',
               '--est-time-unit', 's', '--ref', gt['path'], '--ref-format', gt.get('format', 'tum'),
               '--ref-time-unit', gt.get('time_unit', 's'), '--est-frame', frame, '--ref-frame', gt['frame'],
               '--clock-assumption', gt['clock_assumption'], '--align', gt.get('align', 'se3'),
               '--t-max-diff', str(gt.get('t_max_diff_s', '0.01')),
               '--rpe-delta', str(gt.get('rpe_delta_s', '1')), '--rpe-unit', 's',
               '--out', str(directory / 'evaluation'), '--no-plots']
    result = launch(command, directory, gt.get('timeout_s', 60), 'evaluation.')
    result['status'] = 'OK' if result['exit_code'] == 0 and not result['timed_out'] else 'FAILED'
    result_path = directory / 'evaluation/result.json'
    if result['status'] == 'OK':
        result['result_sha256'] = sha256(contained_file(directory, 'evaluation/result.json'))
        result['result_path'] = str(result_path)
    return result


def run(spec_path, output):
    output = output.resolve()
    spec = load_spec(spec_path, output)
    if output.exists():
        raise ValueError('output directory must be new')
    inputs = {key: {'path': spec[key], 'sha256': sha256(spec[key])}
              for key in ('binary', 'vocabulary', 'settings', 'times', 'dataset_manifest')}
    inputs['spec'] = {'path': str(spec_path.resolve()), 'sha256': sha256(spec_path)}
    if spec.get('mount'):
        inputs['mount'] = {'path': spec['mount'], 'sha256': sha256(spec['mount'])}
    if spec.get('ground_truth'):
        for key in ('path', 'evaluator'):
            value = spec['ground_truth'][key]
            inputs['ground_truth_' + key] = {'path': value, 'sha256': sha256(value)}
    output.mkdir(parents=True)
    write_json(output / 'resolved_spec.json', spec)
    host = {'hostname': platform.node(), 'platform': platform.platform(),
            'machine': platform.machine(), 'python': platform.python_version(), 'cpu_count': os.cpu_count()}
    revision = git_revision(ROOT)
    input_stamp_count = sum(bool(line.split('#', 1)[0].strip()) for line in Path(spec['times']).read_text().splitlines())
    receipts = []
    for repeat in range(1, spec['repeats'] + 1):
        directory = output / f'repeat_{repeat:03d}'
        directory.mkdir()
        values = {**spec, 'repeat': str(repeat), 'run_dir': str(directory)}
        command = [spec['binary']] + [arg.format_map(values) for arg in spec['args']]
        result = launch(command, directory, spec['timeout_s'])
        status = 'TIMEOUT' if result['timed_out'] else ('CRASH' if result['exit_code'] != 0 else 'OK')
        receipt = {'schema_version': 1, 'repeat': repeat, 'status': status, 'engine': result,
                   'inputs': inputs, 'git': revision, 'host': host, 'artifacts': {}, 'errors': [],
                   'trajectory_conventions': spec['trajectory'],
                   'frame_convention': 'T_A_B maps B into A; metres; slam_world is diagnostic',
                   'headless': 'caller attestation; display environment removed',
                   'dataset_hash_scope': 'manifest bytes only; dataset contents not independently verified'}
        text = '\n'.join((directory / name).read_text(errors='replace') for name in ('stdout.log', 'stderr.log'))
        receipt['tracking_log_counts'] = log_counts(text)
        for name in [spec['trajectory']['path'], spec['trajectory'].get('epochs'),
                     *spec.get('extra_trajectories', []), 'stdout.log', 'stderr.log']:
            if not name:
                continue
            try:
                artifact = contained_file(directory, name)
                receipt['artifacts'][name] = {'sha256': sha256(artifact), 'bytes': artifact.stat().st_size}
            except (OSError, ValueError) as exc:
                receipt['errors'].append(str(exc))
                receipt['analysis_failed'] = True
        receipt['trajectory_valid'] = False
        try:
            samples, epochs = poses(directory, spec)
            receipt['continuity'] = continuity(samples, epochs, spec['jump_threshold_m'])
            receipt['trajectory_valid'] = True
            receipt['coverage_proxy'] = {'input_stamp_count': input_stamp_count,
                                         'output_pose_count': len(samples),
                                         'pose_count_per_input_stamp': len(samples) / input_stamp_count,
                                         'limitation': 'Count ratio only; not temporal coverage or a completeness guarantee.'}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            receipt['errors'].append(str(exc))
            if receipt['status'] == 'OK':
                receipt['status'] = 'NO_TRAJECTORY'
        if receipt['trajectory_valid']:
            try:
                metric_samples = samples
                if spec.get('mount'):
                    transform = mount_transform(spec['mount'], spec['trajectory']['sensor_frame'])
                    converted = convert(samples, transform)
                    base_path = directory / 'base_link.jsonl'
                    if base_path.exists() or base_path.is_symlink():
                        raise ValueError('engine artifact conflicts with base_link output')
                    base_path.write_text(''.join(json.dumps(row, allow_nan=False) + '\n' for row in converted))
                    receipt['artifacts']['base_link.jsonl'] = {'sha256': sha256(base_path), 'bytes': base_path.stat().st_size}
                    metric_samples = [Sample(s.stamp_ns, s.clock_domain, s.map_id, s.reset_counter,
                                             s.pose @ transform.inverse()) for s in samples]
                    receipt['continuity_base_link'] = continuity(metric_samples, True, spec['jump_threshold_m'])
                if spec.get('ground_truth'):
                    receipt['evaluation'] = evaluate(directory, metric_samples, spec)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                receipt['errors'].append(str(exc))
                receipt['analysis_failed'] = True
                if spec.get('ground_truth'):
                    receipt['evaluation'] = {'status': 'FAILED', 'error': str(exc)}
        elif spec.get('ground_truth'):
            receipt['evaluation'] = {'status': 'FAILED', 'error': 'No valid trajectory'}
        # Keep errors in analysis independent from process/trajectory status.
        if spec.get('ground_truth') and receipt.get('evaluation', {}).get('status') != 'OK':
            receipt['analysis_failed'] = True
        receipt['input_changes'] = []
        for key, item in inputs.items():
            try:
                changed = sha256(item['path']) != item['sha256']
            except OSError:
                changed = True
            if changed:
                receipt['input_changes'].append(key)
        write_json(directory / 'receipt.json', receipt)
        receipts.append(receipt)
        write_json(output / 'summary.json', summarize(receipts, spec['repeats']))
    return summarize(receipts, spec['repeats'])


def summarize(receipts, planned):
    counts = {status: sum(r['status'] == status for r in receipts)
              for status in ('OK', 'CRASH', 'NO_TRAJECTORY', 'TIMEOUT')}
    return {'evidence': 'MEASURED (run observations)', 'planned_repeats': planned,
            'completed_repeats': len(receipts), 'denominator': planned, 'status_counts': counts,
            'ok_fraction': counts['OK'] / planned,
            'analysis_failures': sum(bool(r.get('analysis_failed')) for r in receipts),
            'input_change_runs': sum(bool(r['input_changes']) for r in receipts),
            'runs': [{'repeat': r['repeat'], 'status': r['status'],
                      'trajectory_valid': r['trajectory_valid']} for r in receipts]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('spec', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        summary = run(args.spec, args.out)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        parser.exit(2, f'error: {exc}\n')
    print(json.dumps(summary, indent=2))
    return 0 if (summary['status_counts']['OK'] == summary['denominator']
                 and not summary['analysis_failures'] and not summary['input_change_runs']) else 1


if __name__ == '__main__':
    sys.exit(main())
