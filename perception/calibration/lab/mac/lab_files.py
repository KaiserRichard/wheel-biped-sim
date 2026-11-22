#!/usr/bin/env python3
"""OPEN: local file preparation/provenance, without Docker or sensor access."""
import argparse
import csv
import hashlib
import math
import os
from pathlib import Path
import shlex
import sys
from datetime import datetime, timezone

EUROC_HEADER = ['#timestamp [ns]', 'w_x [rad s^-1]', 'w_y [rad s^-1]',
                'w_z [rad s^-1]', 'a_x [m s^-2]', 'a_y [m s^-2]', 'a_z [m s^-2]']


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def prepare_bagcreater(euroc, destination):
    """Hard links stay inside the Docker mount. Preserve decimal integer ns/SI text."""
    euroc, destination = Path(euroc), Path(destination)
    # Validate first; never shift epochs to compensate for bagcreater's slicing.
    rows = []
    with (euroc / 'mav0/imu0/data.csv').open(newline='') as stream:
        reader = csv.reader(stream)
        if next(reader, None) != EUROC_HEADER:
            raise ValueError('unexpected EuRoC IMU header/units/order')
        previous = -1
        for row in reader:
            if len(row) != 7 or not row[0].isascii() or not row[0].isdigit():
                raise ValueError('IMU row must contain integer ns and six SI values')
            stamp = int(row[0])
            if stamp < 1_000_000_000 or stamp <= previous or row[0] != str(stamp):
                raise ValueError('IMU ns must be canonical, increasing and >=1e9 for pinned bagcreater')
            if not all(math.isfinite(float(value)) for value in row[1:]):
                raise ValueError('nonfinite IMU value')
            previous = stamp
            rows.append(row)
    if not rows:
        raise ValueError('empty combined IMU')
    cameras = {}
    for camera in ('cam0', 'cam1'):
        base = euroc / 'mav0' / camera
        with (base / 'data.csv').open(newline='') as stream:
            reader = csv.reader(stream)
            if next(reader, None) != ['#timestamp [ns]', 'filename']:
                raise ValueError('unexpected camera CSV header')
            images, previous = [], -1
            for row in reader:
                if len(row) != 2 or not row[0].isascii() or not row[0].isdigit():
                    raise ValueError('invalid image timestamp')
                stamp = int(row[0])
                if stamp < 1_000_000_000 or stamp <= previous or row[0] != str(stamp):
                    raise ValueError('image ns must be canonical, increasing and >=1e9')
                if row[1] != row[0] + '.png':
                    raise ValueError('image filename must be exact <ns>.png')
                source = base / 'data' / row[1]
                if not source.is_file() or source.is_symlink():
                    raise ValueError('missing image or external image symlink')
                images.append(source)
                previous = stamp
        if not images:
            raise ValueError('empty camera: ' + camera)
        cameras[camera] = images
    destination.mkdir()  # exclusive: a partial earlier run is never overwritten
    with (destination / 'imu0.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        # Kalibr skips this first row; its positional reader is timestamp, omega, alpha.
        writer.writerow(['timestamp [ns]', 'omega_x [rad/s]', 'omega_y [rad/s]',
                         'omega_z [rad/s]', 'alpha_x [m/s^2]', 'alpha_y [m/s^2]', 'alpha_z [m/s^2]'])
        writer.writerows(rows)
    for camera, sources in cameras.items():
        (destination / camera).mkdir()
        for source in sources:
            os.link(source, destination / camera / source.name)


def receipt(path, inputs, command, diagnostic=False):
    """Append exact argv and hashes before execution, including on failed runs."""
    with Path(path).open('a', encoding='utf-8') as stream:
        stream.write('\nUTC: ' + datetime.now(timezone.utc).isoformat() + '\n')
        stream.write('Status OPEN. Diagnostic override: ' + str(diagnostic) + '\n')
        stream.write('Command: ' + shlex.join(command) + '\n')
        for item in inputs:
            item = Path(item).resolve(strict=True)
            paths = sorted(p for p in item.rglob('*') if p.is_file()) if item.is_dir() else [item]
            for source in paths:
                stream.write(f'SHA256 {sha256(source)}  {source}\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    prep = sub.add_parser('prepare')
    prep.add_argument('euroc', type=Path)
    prep.add_argument('destination', type=Path)
    rec = sub.add_parser('receipt')
    rec.add_argument('--file', required=True, type=Path)
    rec.add_argument('--input', action='append', default=[], type=Path)
    rec.add_argument('--diagnostic', action='store_true')
    rec.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        if args.action == 'prepare':
            prepare_bagcreater(args.euroc, args.destination)
        else:
            command = args.command[1:] if args.command[:1] == ['--'] else args.command
            receipt(args.file, args.input, command, args.diagnostic)
    except (OSError, ValueError) as exc:
        print('OPEN: ' + str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
