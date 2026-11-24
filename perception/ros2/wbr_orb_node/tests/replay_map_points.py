#!/usr/bin/env python3
"""Bounded camera-free integration check. Run on Pi with ROS Jazzy sourced."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


DEFAULT_SECONDS = 20.0
DEFAULT_RATE = 0.3
PLAYER_DEADLINE_MARGIN_S = 30
STARTUP_ALLOWANCE_S = 75
CLEANUP_ALLOWANCE_S = 30
VALIDATION_ALLOWANCE_S = 30


def read_bag_duration(metadata_path):
    """Read rosbag2's duration from metadata.yaml without PyYAML."""
    lines = Path(metadata_path).read_text().splitlines()
    duration_indent = None
    duration_lines = []
    for line in lines:
        match = re.match(r'^(\s*)duration\s*:\s*(.*?)\s*(?:#.*)?$', line)
        if match:
            duration_indent = len(match.group(1))
            if match.group(2):
                duration_lines.append(match.group(2))
            continue
        if duration_indent is not None:
            if line.strip() and len(line) - len(line.lstrip()) <= duration_indent:
                break
            duration_lines.append(line.strip())
    if duration_indent is None:
        raise ValueError(f'metadata has no duration field: {metadata_path}')
    duration_text = '\n'.join(duration_lines)
    nanos = re.search(r'(?:^|[\s,{])nanoseconds\s*:\s*(\d+)', duration_text)
    if nanos:
        seconds = int(nanos.group(1)) / 1_000_000_000
    else:
        sec = re.search(r'(?:^|[\s,{])sec\s*:\s*(-?\d+)', duration_text)
        nsec = re.search(r'(?:^|[\s,{])nanosec\s*:\s*(\d+)', duration_text)
        if not sec or not nsec:
            raise ValueError(f'unsupported or incomplete duration in {metadata_path}')
        seconds = int(sec.group(1)) + int(nsec.group(1)) / 1_000_000_000
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError(f'bag duration must be positive, got {seconds}')
    return seconds


def child_deadline_seconds(seconds, rate, margin=PLAYER_DEADLINE_MARGIN_S):
    if seconds <= 0 or rate <= 0 or margin < 0:
        raise ValueError('seconds and rate must be positive and margin nonnegative')
    return math.ceil(seconds / rate + margin)


def build_play_command(bag, seconds, rate):
    return ['ros2', 'bag', 'play', str(bag), '--rate', str(rate),
            '--clock', '40', '--playback-duration', f'{seconds:.9g}',
            '--read-ahead-queue-size', '20', '--disable-keyboard-controls',
            '--topics', '/camera/camera/infra1/image_rect_raw',
            '/camera/camera/infra2/image_rect_raw', '--remap',
            '/camera/camera/infra1/image_rect_raw:=/wbr_map_test/left',
            '/camera/camera/infra2/image_rect_raw:=/wbr_map_test/right']


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ('node', 'bag', 'settings', 'vocabulary', 'output'):
        parser.add_argument('--' + arg, required=True, type=Path)
    parser.add_argument('--domain', type=int, default=187)
    parser.add_argument('--seconds', type=float, default=DEFAULT_SECONDS,
                        help='bag duration in seconds; 0 replays the full bag (default: 20)')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE,
                        help='ros2 bag playback rate (default: 0.3)')
    return parser


def main():
    parser = make_parser()
    args = parser.parse_args()
    for name in ('node', 'bag', 'settings', 'vocabulary', 'output'):
        setattr(args, name, getattr(args, name).resolve())
    if not 1 <= args.domain <= 232:
        parser.error('use a private nonzero ROS domain (1..232)')
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be zero (whole bag) or a positive number')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be a positive number')
    requested_seconds = args.seconds
    if args.seconds == 0:
        try:
            args.seconds = read_bag_duration(args.bag / 'metadata.yaml')
        except (OSError, ValueError) as error:
            parser.error(str(error))
    play_deadline_s = child_deadline_seconds(args.seconds, args.rate)
    args.output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(args.output / 'ros-logs'), OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1', LD_BIND_NOW='1')
    started = time.monotonic()
    # Preserve the original default work window, extending it for long playback.
    # Three owned process groups have at most 10s cleanup each.
    work_budget_s = max(145, STARTUP_ALLOWANCE_S + play_deadline_s)
    deadline = started + work_budget_s
    # Preserve the original 180s default budget; long runs reserve up to 30s
    # each for owned-process cleanup and persisted validation.
    overall_deadline = max(180, work_budget_s + CLEANUP_ALLOWANCE_S + VALIDATION_ALLOWANCE_S) + started
    children, handles = {}, []
    receipt = {'commands': {}, 'pids': {}, 'exits': {}, 'signals': [],
               'domain': args.domain, 'requested_seconds': requested_seconds,
               'bag_seconds': args.seconds, 'rate': args.rate,
               'child_deadline_s': play_deadline_s}

    def save():
        receipt['elapsed_s'] = time.monotonic() - started
        (args.output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')

    def launch(name, command):
        log = (args.output / (name + '.log')).open('x')
        handles.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 cwd=args.output, env=env, start_new_session=True)
        children[name] = child
        receipt['commands'][name], receipt['pids'][name] = command, child.pid
        save()
        return child

    def ready(name, needles, seconds):
        until = min(deadline, time.monotonic() + seconds)
        while time.monotonic() < until:
            if children[name].poll() is not None:
                raise RuntimeError(f'{name} exited {children[name].returncode}')
            log = (args.output / (name + '.log')).read_text(errors='replace')
            if all(needle in log for needle in needles):
                return
            time.sleep(0.2)
        raise TimeoutError(f'{name} readiness timeout')

    def stop(name):
        child = children[name]
        for sig, seconds in ((signal.SIGINT, 7), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
            # Address only our own session/process group, including descendants.
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            receipt['signals'].append([name, sig.name])
            try:
                child.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                continue
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                break
        receipt['exits'][name] = child.poll()

    def terminate(signum, _frame):
        raise RuntimeError(f'supervisor received signal {signum}')

    signal.signal(signal.SIGTERM, terminate)
    try:
        # Store receive-time on this derived bag; source stamps stay in message
        # headers. This lets us wait for subscriptions before publishing /clock.
        launch('record', ['ros2', 'bag', 'record', '--storage', 'mcap',
                         '--output', str(args.output / 'map-bag'), '--topics',
                         '/d435_orb/pose', '/d435_orb/status',
                         '/d435_orb/map_points', '/d435_orb/map_points_info'])
        launch('slam', [str(args.node), '--ros-args', '-p', 'mode:=stereo', '-p', 'use_sim_time:=true',
                        '-p', 'orb_vocabulary:=' + str(args.vocabulary),
                        '-p', 'orb_settings:=' + str(args.settings),
                        '-p', 'image_topic:=/wbr_map_test/left',
                        '-p', 'second_topic:=/wbr_map_test/right'])
        ready('slam', ['viewer disabled'], 55)
        ready('record', ['All requested topics are subscribed'], 20)
        player = launch('play', build_play_command(args.bag, args.seconds, args.rate))
        player_deadline = time.monotonic() + play_deadline_s
        while player.poll() is None:
            if time.monotonic() >= player_deadline:
                raise TimeoutError(f'playback exceeded {play_deadline_s}s child deadline')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{work_budget_s}s work deadline reached')
            if children['slam'].poll() is not None or children['record'].poll() is not None:
                raise RuntimeError('SLAM or recorder exited during replay')
            time.sleep(0.2)
        if player.returncode:
            raise RuntimeError(f'player exited {player.returncode}')
        time.sleep(min(3, max(0, deadline - time.monotonic())))
    except BaseException as error:
        receipt['error'] = repr(error)
    finally:
        for name in ('play', 'slam', 'record'):
            if name in children:
                stop(name)
        remaining = []
        for name, child in children.items():
            try:
                os.killpg(child.pid, 0)
                remaining.append(name)
            except ProcessLookupError:
                pass
        abnormal = {name: code for name, code in receipt['exits'].items() if code != 0}
        if abnormal:
            receipt.setdefault('error', f'abnormal child exits: {abnormal}')
        receipt['remaining_process_groups'] = remaining
        receipt['all_owned_children_exited'] = all(p.poll() is not None for p in children.values())
        save()
        for handle in handles:
            handle.close()
    if receipt.get('error') or remaining or not receipt['all_owned_children_exited']:
        print(json.dumps(receipt, indent=2))
        return 1
    # Writers have stopped: prove the independently retained MCAP is readable.
    checker = Path(__file__).resolve().parents[1] / 'scripts' / 'check_map_points_bag.py'
    remaining_budget = overall_deadline - time.monotonic()
    if remaining_budget <= 0:
        receipt['error'] = 'overall deadline reached before persisted validation'
        save()
        return 1
    with (args.output / 'validation.json').open('x') as output, (args.output / 'validation.log').open('x') as errors:
        try:
            check = subprocess.run([sys.executable, str(checker), str(args.output / 'map-bag'),
                                    '--source-bag', str(args.bag)], stdout=output, stderr=errors,
                                   timeout=min(30, remaining_budget))
            receipt['validation_exit'] = check.returncode
            output.flush()
            if check.returncode == 0:
                validation = json.loads((args.output / 'validation.json').read_text())
                validation['replay'] = {'requested_seconds': requested_seconds,
                                        'bag_seconds': args.seconds, 'rate': args.rate,
                                        'child_deadline_s': play_deadline_s}
                (args.output / 'validation.json').write_text(json.dumps(validation, indent=2) + '\n')
        except subprocess.TimeoutExpired:
            receipt['error'] = 'persisted validation exceeded remaining deadline'
            receipt['validation_exit'] = 124
    save()
    print(json.dumps(receipt, indent=2))
    return receipt['validation_exit']


if __name__ == '__main__':
    raise SystemExit(main())
