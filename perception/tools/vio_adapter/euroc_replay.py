#!/usr/bin/env python3
"""Replay retained export into the C++ adapter; fake poses are synthetic only."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import zlib


def decode_png(path):
    """Decode noninterlaced 8-bit grayscale PNG, verify chunk CRCs and all filters."""
    raw = path.read_bytes()
    if raw[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('not PNG')
    pos, compressed, dimensions, ended = 8, bytearray(), None, False
    while pos < len(raw):
        if pos + 12 > len(raw):
            raise ValueError('truncated PNG chunk')
        size = struct.unpack_from('>I', raw, pos)[0]
        tag = raw[pos+4:pos+8]
        data = raw[pos+8:pos+8+size]
        if pos + size + 12 > len(raw):
            raise ValueError('truncated PNG data')
        crc = struct.unpack_from('>I', raw, pos+8+size)[0]
        if zlib.crc32(tag+data) & 0xffffffff != crc:
            raise ValueError('PNG CRC mismatch')
        if tag == b'IHDR':
            if dimensions is not None or len(data) != 13:
                raise ValueError('invalid IHDR')
            w, h, depth, color, comp, filt, interlace = struct.unpack('>IIBBBBB', data)
            if depth != 8 or color != 0 or comp or filt or interlace or not (0 < w <= 8192 and 0 < h <= 8192):
                raise ValueError('requires bounded noninterlaced grayscale Y8 PNG')
            dimensions = w, h
        elif tag == b'IDAT':
            compressed.extend(data)
        elif tag == b'IEND':
            ended = True
            pos += size + 12
            break
        elif tag[0] & 32 == 0:
            raise ValueError('unsupported critical PNG chunk')
        pos += size + 12
    if not ended or pos != len(raw) or dimensions is None:
        raise ValueError('incomplete PNG')
    w, h = dimensions
    payload = zlib.decompress(compressed)
    if len(payload) != h*(w+1):
        raise ValueError('wrong PNG pixel length')
    pixels, previous = bytearray(), bytearray(w)
    for y in range(h):
        filt = payload[y*(w+1)]
        row = bytearray(payload[y*(w+1)+1:(y+1)*(w+1)])
        if filt > 4:
            raise ValueError('unknown PNG filter')
        for x in range(w) if filt else ():
            a, b, c = row[x-1] if x else 0, previous[x], previous[x-1] if x else 0
            predictor = 0
            if filt == 1:
                predictor = a
            elif filt == 2:
                predictor = b
            elif filt == 3:
                predictor = (a+b)//2
            elif filt == 4:
                p = a+b-c
                pa, pb, pc = abs(p-a), abs(p-b), abs(p-c)
                predictor = a if pa <= pb and pa <= pc else b if pb <= pc else c
            row[x] = (row[x]+predictor) & 255
        pixels.extend(row)
        previous = row
    return w, h, pixels


def ordered(values, label):
    if any(a >= b for a, b in zip(values, values[1:])):
        raise ValueError(f'{label}: nonincreasing input; refusing to sort away reversals')


def rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def load_events(root):
    manifest = json.loads((root/'manifest.json').read_text())
    selected = [int(x) for x in (root/'times_ns.txt').read_text().splitlines()]
    ordered(selected, 'times_ns')
    if not selected:
        raise ValueError('empty times_ns')
    domains = manifest['stamp_domains']
    if len(domains) != 1 or not domains[0]:
        raise ValueError('requires one explicit unchanged domain')
    domain = domains[0]
    stamps = {}
    for row in rows(root/'stamps.csv'):
        key = row['stream'], int(row['sdk_ns'])
        if key in stamps:
            raise ValueError('duplicate capture metadata')
        stamps[key] = row['domain']
    events, camera_counts = [], {}
    for cam, kind, stream in [('cam0', b'L', 'ir1'), ('cam1', b'R', 'ir2')]:
        data = rows(root/'mav0'/cam/'data.csv')
        epochs = [int(r['#timestamp [ns]']) for r in data]
        ordered(epochs, cam)
        by_epoch = dict(zip(epochs, data))
        camera_counts[cam] = {'exported': len(data), 'selected': len(selected), 'excluded_by_times': len(data)-len(selected)}
        for ns in selected:
            r = by_epoch[ns]
            filename = r['filename']
            if Path(filename).name != filename:
                raise ValueError('image filename must be a basename')
            event_domain = stamps[(stream, ns)]
            if event_domain != domain:
                raise ValueError('camera domain changed')
            events.append((ns, kind, event_domain, root/'mav0'/cam/'data'/filename))
    raw_counts = {}
    for name, kind in [('accel', b'A'), ('gyro', b'G')]:
        data = rows(root/'mav0'/'imu0'/f'raw_{name}.csv')
        ordered([int(r['#sdk_timestamp [ns]']) for r in data], name)
        raw_counts[name] = len(data)
        for r in data:
            if r['domain'] != domain:
                raise ValueError('IMU domain changed')
            events.append((int(r['#sdk_timestamp [ns]']), kind, r['domain'], tuple(float(r[k]) for k in ['x', 'y', 'z'])))
    # Each source was validated in acquisition order above. Merge only same-domain streams.
    events.sort(key=lambda e: (e[0], e[1]))
    return domain, events, camera_counts, raw_counts


def replay(root, output, executable):
    domain, events, cameras, raw_counts = load_events(root)
    root, output = root.resolve(), output.resolve()
    if output == root or root in output.parents:
        raise ValueError('output must be outside the read-only export')
    output.mkdir(parents=True, exist_ok=False)
    pixels_hash = hashlib.sha256()
    try:
        with (output/'snapshots.jsonl').open('wb') as log, (output/'adapter_counts.json').open('wb') as counts:
            process = subprocess.Popen([str(executable.resolve()), domain], stdin=subprocess.PIPE, stdout=log, stderr=counts)
            try:
                for ns, kind, event_domain, data in events:
                    d = event_domain.encode('ascii')
                    packet = kind + struct.pack('<qH', ns, len(d)) + d
                    if kind in [b'A', b'G']:
                        packet += struct.pack('<ddd', *data)
                    else:
                        w, h, pixels = decode_png(data)
                        pixels_hash.update(kind+struct.pack('<qII', ns, w, h)+pixels)
                        packet += struct.pack('<II', w, h)+pixels
                    process.stdin.write(packet)
                process.stdin.close()
                if process.wait() != 0:
                    raise ValueError('C++ replay failed; see adapter_counts.json')
            except BaseException:
                process.kill()
                process.wait()
                raise
        counts = json.loads((output/'adapter_counts.json').read_text())
        snapshots = [json.loads(line) for line in (output/'snapshots.jsonl').read_text().splitlines()]
        ordered([s['source_ns'] for s in snapshots], 'snapshot source')
        if len(snapshots) != counts['pairs'] or any(s['sequence'] != i+1 for i, s in enumerate(snapshots)):
            raise ValueError('snapshot accounting mismatch')
        report = {'evidence': 'MEASURED real input association; estimates SYNTHETIC',
                  'engine': 'fake', 'input': str(root), 'domain': domain,
                  'camera_counts': cameras, 'raw_counts': raw_counts,
                  'adapter_counts': counts, 'snapshots': len(snapshots),
                  'decoded_y8_sha256': pixels_hash.hexdigest(),
                  'input_sha256': {p: hashlib.sha256((root/p).read_bytes()).hexdigest()
                                   for p in ['manifest.json', 'times_ns.txt', 'stamps.csv',
                                             'mav0/imu0/raw_accel.csv', 'mav0/imu0/raw_gyro.csv',
                                             'mav0/cam0/data.csv', 'mav0/cam1/data.csv']}}
        (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        return report
    except BaseException as e:
        (output/'FAILED.txt').write_text(str(e)+'\n')
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('export', type=Path)
    parser.add_argument('new_output', type=Path)
    parser.add_argument('--executable', type=Path, default=Path(__file__).parent/'build'/'replay_fake')
    args = parser.parse_args()
    print(json.dumps(replay(args.export, args.new_output, args.executable), indent=2))
