#!/usr/bin/env python3
"""Run exactly three fresh paced headless estimators; preserve all logs and exits."""
import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import shutil
import resource
import subprocess
import time


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runner', type=Path)
    parser.add_argument('vocabulary', type=Path)
    parser.add_argument('settings', type=Path)
    parser.add_argument('pairs', type=Path)
    parser.add_argument('new_batch', type=Path)
    parser.add_argument('--pace-rate', type=float, default=1.0)
    args = parser.parse_args()
    root = args.new_batch
    root.mkdir()
    shutil.copy2(args.settings, root / 'stereo.yaml')
    shutil.copy2(args.pairs / 'pairs.csv', root / 'pairs.csv')
    sources = {str(p): sha(p) for p in (args.runner, args.vocabulary, args.settings, args.pairs / 'pairs.csv')}
    receipt = {'status': 'RUNNING', 'platform': 'Intel Mac / isolated Linux x86_64 Docker',
               'pace_rate': args.pace_rate, 'expected_pairs': 2215,
               'input_sha256': sources, 'results': [], 'started_utc': dt.datetime.now(dt.timezone.utc).isoformat()}

    def save():
        (root / 'progress.json').write_text(json.dumps(receipt, indent=2) + '\n')
    save()
    for number in (1, 2, 3):
        run = root / f'replay-{number}'
        command = [str(args.runner), str(args.vocabulary), str(root / 'stereo.yaml'),
                   str(args.pairs), str(run), '2215', str(args.pace_rate)]
        entry = {'number': number, 'command': command, 'started_utc': dt.datetime.now(dt.timezone.utc).isoformat()}
        receipt['active_replay'] = number
        save()
        before_cpu = resource.getrusage(resource.RUSAGE_CHILDREN)
        start = time.monotonic()
        sampled_peak_rss_kib = 0
        with (root / f'replay-{number}.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            while process.poll() is None:
                try:
                    for line in Path(f'/proc/{process.pid}/status').read_text().splitlines():
                        if line.startswith('VmRSS:'):
                            sampled_peak_rss_kib = max(sampled_peak_rss_kib, int(line.split()[1]))
                except FileNotFoundError:
                    pass
                if time.monotonic() - start > 1200:
                    process.kill()
                    process.wait()
                    entry['timeout_s'] = 1200
                    break  # Nonzero killed-process exit is saved below.
                time.sleep(.2)
        wall = time.monotonic() - start
        after_cpu = resource.getrusage(resource.RUSAGE_CHILDREN)
        cpu = after_cpu.ru_utime + after_cpu.ru_stime - before_cpu.ru_utime - before_cpu.ru_stime
        entry.update(exit_code=process.returncode, wall_s=wall, cpu_seconds=cpu,
                     average_process_cpu_percent=100 * cpu / wall,
                     sampled_peak_rss_kib=sampled_peak_rss_kib, rss_sample_period_s=.2)
        result = process
        receipt['results'].append(entry)
        save()
        if result.returncode:
            receipt['status'] = 'FAILED'
            save()
            raise SystemExit(result.returncode)
    receipt['status'] = 'COMPLETE'
    receipt['completed_utc'] = dt.datetime.now(dt.timezone.utc).isoformat()
    save()


if __name__ == '__main__':
    main()
