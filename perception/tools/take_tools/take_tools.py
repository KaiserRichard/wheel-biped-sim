#!/usr/bin/env python3
"""Inspect, audit and export uncompressed librealsense SQLite/CDR recordings."""
import argparse
import bisect
from collections import defaultdict
from contextlib import closing
import csv
from decimal import Decimal, InvalidOperation, localcontext
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import struct
import sys
import zlib

VERSION = '0.1.0'
UPSTREAM = 'librealsense v2.58.1'
STREAMS = {'ir1': 30, 'ir2': 30, 'gyro': 200, 'accel': 100}
DOMAINS = {'Hardware Clock': 'hardware_clock', 'System Time': 'system_time',
           'Global Time': 'global_time'}
TOPIC = re.compile(r'^/device_\d+/sensor_\d+/(Infrared_[12]|Gyro_0|Accel_0)/(image|imu)/(data|metadata)$')


class Unsupported(ValueError):
    pass


class CDR:
    """Plain XCDR1, alignment relative to the payload after encapsulation."""
    def __init__(self, blob):
        if not isinstance(blob, bytes) or len(blob) < 4 or blob[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('unsupported CDR encapsulation (compressed/XCDR2 data unsupported)')
        if blob[2:4] != b'\x00\x00':
            raise ValueError('unsupported CDR encapsulation options')
        self.data, self.pos = blob[4:], 0
        self.endian = '<' if blob[1] == 1 else '>'

    def take(self, n):
        if n < 0 or self.pos + n > len(self.data):
            raise ValueError('truncated CDR payload')
        result = self.data[self.pos:self.pos+n]
        self.pos += n
        return result

    def number(self, fmt):
        size = struct.calcsize(fmt)
        self.take((-self.pos) % size)
        return struct.unpack(self.endian + fmt, self.take(size))[0]

    def string(self):
        raw = self.take(self.number('I'))
        if not raw or raw[-1] != 0 or b'\x00' in raw[:-1]:
            raise ValueError('invalid CDR string terminator')
        return raw[:-1].decode('utf-8')

    def finish(self):
        if self.pos != len(self.data):
            raise ValueError('unexpected trailing CDR payload')


def decode(blob, kind):
    c = CDR(blob)
    if kind == 'std_msgs/msg/String':
        result = c.string()
    elif kind in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/Imu'):
        sec, ns = c.number('i'), c.number('I')
        if ns >= 1_000_000_000:
            raise ValueError('invalid header nanosec')
        result = {'header_ns': sec * 1_000_000_000 + ns, 'frame_id': c.string()}
        if kind.endswith('/Image'):
            height, width = c.number('I'), c.number('I')
            encoding, big = c.string(), c.number('B')
            step = c.number('I')
            pixels = c.take(c.number('I'))
            if encoding not in ('mono8', '8UC1'):
                raise ValueError(f'unknown image encoding {encoding!r}; expected mono8 or 8UC1 (SDK Y8)')
            if not width or not height or step < width or len(pixels) != step * height or big not in (0, 1):
                raise ValueError('invalid image dimensions, step, endianness or data length')
            result.update(width=width, height=height, encoding=encoding, step=step,
                          pixels=b''.join(pixels[y*step:y*step+width] for y in range(height)))
        else:
            values = [c.number('d') for _ in range(37)]
            if not all(math.isfinite(v) for v in values):
                raise ValueError('nonfinite IMU value')
            result.update(gyro=values[13:16], accel=values[25:28])
    else:
        raise ValueError(f'unsupported selected message type {kind}')
    c.finish()
    return result


def ms_to_ns(text):
    try:
        with localcontext() as ctx:
            ctx.prec = max(50, len(text) + 10)
            value = Decimal(text) * 1_000_000
            if not value.is_finite() or value != value.to_integral_value():
                raise ValueError('SDK milliseconds must represent exact integer nanoseconds')
            stamp = int(value)
            if not 0 <= stamp <= 2**63 - 1:
                raise ValueError('SDK stamp outside nonnegative int64 ns')
            return stamp
    except InvalidOperation as exc:
        raise ValueError('invalid SDK millisecond timestamp') from exc


def metadata(text):
    result = {}
    for entry in text.split(';'):
        if not entry:
            continue
        key, sep, value = entry.partition('=')
        if not sep or key in result:
            raise ValueError('malformed or duplicate metadata key')
        result[key] = value
    for key in ('Frame number', 'timestamp_domain', 'timestamp', 'system_time'):
        if key not in result:
            raise ValueError(f'missing metadata key {key}')
    if not re.fullmatch(r'\d+', result['Frame number']):
        raise ValueError('invalid frame number')
    return dict(metadata=result, frame_number=int(result['Frame number']),
                sdk_ns=ms_to_ns(result['timestamp']), system_ns=ms_to_ns(result['system_time']),
                domain=DOMAINS.get(result['timestamp_domain'], 'UNKNOWN:' + result['timestamp_domain']))


def format_of(path):
    with Path(path).open('rb') as f:
        magic = f.read(16)
    if magic == b'SQLite format 3\x00':
        return 'sqlite'
    if magic.startswith(b'#ROSBAG V2.0'):
        return 'ros1'
    return 'unknown'


def database(path):
    fmt = format_of(path)
    if fmt != 'sqlite':
        raise Unsupported('ROS 1 bag detected but unsupported' if fmt == 'ros1' else 'unknown file magic; unsupported')
    wal = Path(str(Path(path).resolve()) + '-wal')
    if wal.exists() and wal.stat().st_size:
        raise ValueError('nonempty SQLite WAL: finalize/checkpoint recording before reading or hashing')
    con = sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    con.row_factory = sqlite3.Row
    return con


def inspect(path):
    fmt = format_of(path)
    result = {'format': fmt, 'supported': fmt == 'sqlite',
              'timestamp_clock': 'writer timestamp argument; physical clock UNKNOWN'}
    if fmt != 'sqlite':
        result['reason'] = 'ROS 1 bag detected but unsupported' if fmt == 'ros1' else 'unknown file magic'
        return result
    with closing(database(path)) as con:
        result['tables'] = [dict(r) for r in con.execute("SELECT name,sql FROM sqlite_master WHERE type='table' ORDER BY name")]
        names = {r['name'] for r in result['tables']}
        result['topics'] = []
        if {'topics', 'messages'} <= names:
            for topic in con.execute('SELECT * FROM topics ORDER BY id'):
                item = dict(topic)
                stats = con.execute('SELECT COUNT(*) AS message_count, MIN(timestamp) AS first_ns, MAX(timestamp) AS last_ns FROM messages WHERE topic_id=?', (topic['id'],)).fetchone()
                item.update(dict(stats))
                result['topics'].append(item)
            result.update(dict(con.execute('SELECT COUNT(*) AS message_count, MIN(timestamp) AS first_ns, MAX(timestamp) AS last_ns FROM messages').fetchone()))
        else:
            result.update(supported=False, reason='SQLite detected; required topics/messages tables absent')
    return result


def load(path):
    streams = {s: [] for s in STREAMS}
    meta, topic_stream, topic_names = defaultdict(list), {}, {}
    ignored = {}
    devices = set()
    with closing(database(path)) as con:
        topics = list(con.execute('SELECT * FROM topics ORDER BY id'))
        for row in topics:
            match = TOPIC.fullmatch(row['name'])
            if not match:
                # Never silently ignore a frame stream outside the implemented profile.
                if row['name'].endswith('/data') or row['name'].endswith('/metadata'):
                    raise ValueError(f'unsupported frame topic {row["name"]}')
                ignored[row['name']] = con.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?', (row['id'],)).fetchone()[0]
                continue
            stream_name, family, role = match.groups()
            devices.add(row['name'].split('/')[1])
            stream = {'Infrared_1': 'ir1', 'Infrared_2': 'ir2', 'Gyro_0': 'gyro', 'Accel_0': 'accel'}[stream_name]
            expected_family = 'image' if stream.startswith('ir') else 'imu'
            expected_type = 'std_msgs/msg/String' if role == 'metadata' else ('sensor_msgs/msg/Image' if family == 'image' else 'sensor_msgs/msg/Imu')
            if family != expected_family or row['type'] != expected_type or row['serialization_format'] != 'cdr':
                raise ValueError(f'unsupported type/serialization/topic family: {row["name"]}')
            if (stream, role) in topic_names:
                raise ValueError(f'ambiguous multiple {stream} {role} topics')
            topic_names[stream, role] = row['name']
            topic_stream[row['id']] = stream, role, row['type']
        if len(devices) > 1:
            raise ValueError('multiple devices: hardware clock identities cannot be combined')
        orphan = con.execute('SELECT COUNT(*) FROM messages LEFT JOIN topics ON topic_id=topics.id WHERE topics.id IS NULL').fetchone()[0]
        if orphan:
            raise ValueError(f'{orphan} messages reference nonexistent topics')
        for row in con.execute('SELECT * FROM messages ORDER BY id'):
            if row['topic_id'] not in topic_stream:
                continue
            stream, role, kind = topic_stream[row['topic_id']]
            try:
                value = decode(row['data'], kind)
                if role == 'metadata':
                    meta[stream, row['timestamp']].append(metadata(value))
                else:
                    value.update(bag_ns=row['timestamp'], message_id=row['id'], stream=stream)
                    streams[stream].append(value)
            except ValueError as exc:
                raise ValueError(f'message {row["id"]} ({stream} {role}): {exc}') from exc
    issues = []
    for stream, samples in streams.items():
        for sample in samples:
            key = stream, sample['bag_ns']
            candidates = meta.get(key, [])
            if len(candidates) == 1:
                sample.update(candidates[0])
            else:
                sample.update(sdk_ns=None, system_ns=None, frame_number=None, domain='UNKNOWN')
                issues.append(f'{stream} message {sample["message_id"]}: metadata association count {len(candidates)}')
        data_keys = {s['bag_ns'] for s in samples}
        for (name, stamp), values in meta.items():
            if name == stream and stamp not in data_keys:
                issues.append(f'{stream}: {len(values)} orphan metadata at writer stamp {stamp}')
    return streams, issues, ignored


def timing(stamps, nominal_hz, domains=None):
    intervals, backward, duplicates, transitions = [], 0, 0, 0
    for i in range(1, len(stamps)):
        if domains is not None and domains[i] != domains[i-1]:
            transitions += 1
            continue  # No subtraction between clocks.
        if stamps[i] is None or stamps[i-1] is None:
            continue
        dt = stamps[i] - stamps[i-1]
        backward += dt < 0
        duplicates += dt == 0
        if dt > 0:
            intervals.append(dt)
    ordered = sorted(intervals)
    uniform = domains is None or len(set(domains)) == 1
    duration = stamps[-1] - stamps[0] if uniform and stamps and all(t is not None for t in stamps) else None
    return {'count': len(stamps), 'first_ns': stamps[0] if stamps else None,
            'last_ns': stamps[-1] if stamps else None, 'nominal_hz': nominal_hz,
            'rate_hz': (len(stamps)-1)*1e9/duration if duration and duration > 0 and not backward and not duplicates else None,
            'max_interval_ns': max(intervals, default=None),
            'p99_interval_ns': ordered[math.ceil(.99*len(ordered))-1] if ordered else None,
            'gaps_gt_1_5_periods': sum(dt * nominal_hz > 1_500_000_000 for dt in intervals),
            'backward': backward, 'duplicates': duplicates, 'domain_changes': transitions}


def stereo(streams, tolerance_ns=1_000_000):
    left, right = streams['ir1'], streams['ir2']
    lm, rm = defaultdict(list), defaultdict(list)
    for s in left:
        lm[s['frame_number']].append(s)
    for s in right:
        rm[s['frame_number']].append(s)
    paired, skews, failures = 0, [], []
    for number in sorted(set(lm) | set(rm), key=lambda n: -1 if n is None else n):
        a, b = lm[number], rm[number]
        if number is None or len(a) != 1 or len(b) != 1:
            failures.append(f'frame {number}: IR1={len(a)}, IR2={len(b)}; unpaired/ambiguous')
            continue
        a, b = a[0], b[0]
        if a['domain'] != b['domain'] or a['domain'] not in DOMAINS.values():
            failures.append(f'frame {number}: clocks differ or unknown; skew not calculated')
            continue
        skew = b['sdk_ns'] - a['sdk_ns']
        skews.append(skew)
        if abs(skew) > tolerance_ns:
            failures.append(f'frame {number}: stereo skew {skew} ns exceeds {tolerance_ns}')
        else:
            paired += 1
    return {'method': 'unique metadata Frame number, then same-domain SDK stamp skew',
            'tolerance_ns': tolerance_ns, 'valid_pairs': paired,
            'unpaired_or_rejected_ir1': len(left)-paired, 'unpaired_or_rejected_ir2': len(right)-paired,
            'max_abs_skew_ns': max(map(abs, skews), default=None), 'signed_skews_ns': skews,
            'failures': failures}


def audit_streams(streams, issues, ignored):
    result = {'tool_version': VERSION, 'upstream_profile': UPSTREAM, 'streams': {},
              'stamp_used': 'metadata timestamp: SDK milliseconds converted exactly to integer ns',
              'clocks': {'sdk_ns': 'per-frame timestamp_domain metadata',
                         'bag_ns': 'writer timestamp argument; physical clock UNKNOWN',
                         'header_ns': 'writer timestamp argument; physical clock UNKNOWN',
                         'system_ns': 'SDK frame system time (host system clock); not export clock'},
              'ignored_nonframe_topics': ignored, 'rejections': list(issues)}
    all_domains = set()
    for name, samples in streams.items():
        domains = [s['domain'] for s in samples]
        all_domains.update(domains)
        stats = timing([s['sdk_ns'] for s in samples], STREAMS[name], domains)
        stats.update(domains_seen=sorted(set(domains)),
                     domain_change_events=[{'message_id': samples[i]['message_id'],
                                            'from': domains[i-1], 'to': domains[i]}
                                           for i in range(1, len(samples)) if domains[i] != domains[i-1]],
                     writer=timing([s['bag_ns'] for s in samples], STREAMS[name]),
                     header=timing([s['header_ns'] for s in samples], STREAMS[name]),
                     header_bag_mismatches=sum(s['header_ns'] != s['bag_ns'] for s in samples),
                     missing_metadata=sum(s['sdk_ns'] is None for s in samples))
        result['streams'][name] = stats
        if not samples:
            result['rejections'].append(f'{name}: required stream missing')
        if stats['backward'] or stats['duplicates'] or stats['domain_changes']:
            result['rejections'].append(f'{name}: SDK ordering/domain check failed')
        if stats['header_bag_mismatches']:
            result['rejections'].append(f'{name}: header/writer stamp mismatch')
        if stats['writer']['backward'] or stats['writer']['duplicates'] or stats['header']['backward'] or stats['header']['duplicates']:
            result['rejections'].append(f'{name}: writer/header ordering check failed')
    result['domains_seen'] = sorted(all_domains)
    result['single_known_domain'] = len(all_domains) == 1 and all_domains <= set(DOMAINS.values())
    if not result['single_known_domain']:
        result['rejections'].append('all four streams must have one common known SDK domain')
    result['stereo'] = stereo(streams)
    result['rejections'].extend(result['stereo']['failures'])
    result['status'] = 'PASS' if not result['rejections'] else 'FAIL'
    return result


def audit(path):
    return audit_streams(*load(path))


def markdown(result):
    lines = [f'# Take audit: {result["status"]}', '',
             'Stamps: SDK metadata timestamp in integer ns; domains listed below.', '',
             '| Stream | Count | Hz | Max interval ns | p99 ns | Gaps | Backward | Duplicate | Domains / changes |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for name, s in result['streams'].items():
        rate = f'{s["rate_hz"]:.3f}' if s['rate_hz'] is not None else 'unavailable'
        lines.append(f'| {name} | {s["count"]} | {rate} | {s["max_interval_ns"]} | {s["p99_interval_ns"]} | {s["gaps_gt_1_5_periods"]} | {s["backward"]} | {s["duplicates"]} | {", ".join(s["domains_seen"])} / {s["domain_changes"]} |')
    lines += ['', f'Stereo: {result["stereo"]["valid_pairs"]} valid frame-number pairs; max absolute skew {result["stereo"]["max_abs_skew_ns"]} ns.',
              '', 'Bag/header clock: writer argument, physical clock UNKNOWN. System metadata: SDK host system time.',
              'Intervals across SDK domain changes are excluded. No clock mapping is applied.', '',
              'Rejections: ' + ('; '.join(result['rejections']) or 'none'), '']
    return '\n'.join(lines)


def png(width, height, pixels):
    def chunk(name, value):
        return struct.pack('>I', len(value)) + name + value + struct.pack('>I', zlib.crc32(name+value) & 0xffffffff)
    rows = b''.join(b'\x00' + pixels[y*width:(y+1)*width] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


def csv_file(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(header)
        writer.writerows(rows)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def combined_imu(streams, report):
    counts = dict(input_gyro=len(streams['gyro']), input_accel=len(streams['accel']),
                  emitted=0, dropped_prefix=0, dropped_tail=0, dropped_invalid_bracket=0,
                  dropped_clock_or_order=0)
    # Diagnostic mode still never interpolates across a failed domain/order check.
    valid = report['single_known_domain'] and all(
        not report['streams'][s]['backward'] and not report['streams'][s]['duplicates']
        and not report['streams'][s]['domain_changes'] and not report['streams'][s]['missing_metadata']
        for s in ('gyro', 'accel'))
    accel = streams['accel']
    if not valid or len(accel) < 2:
        counts['dropped_clock_or_order'] = len(streams['gyro'])
        return [], counts
    stamps = [s['sdk_ns'] for s in accel]
    rows = []
    for g in streams['gyro']:
        stamp = g['sdk_ns']
        if stamp < stamps[0]:
            counts['dropped_prefix'] += 1
            continue
        if stamp > stamps[-1]:
            counts['dropped_tail'] += 1
            continue
        i = min(max(bisect.bisect_right(stamps, stamp)-1, 0), len(stamps)-2)
        a, b = accel[i], accel[i+1]
        dt = b['sdk_ns'] - a['sdk_ns']
        if dt <= 0 or dt > 15_000_000:
            counts['dropped_invalid_bracket'] += 1
            continue
        alpha = (stamp-a['sdk_ns']) / dt
        # Convex weights preserve finite endpoints without overflowing y-x.
        values = [(1-alpha)*x+alpha*y for x, y in zip(a['accel'], b['accel'])]
        rows.append([stamp, *g['gyro'], *values])
    counts['emitted'] = len(rows)
    return rows, counts


def orb_times(streams, report, imu_rows):
    """Select original, identically named stereo PNGs within combined IMU bounds."""
    counts = dict(input_ir1=len(streams['ir1']), emitted=0, dropped_head=0, dropped_tail=0,
                  dropped_unpaired_or_mismatched=0, dropped_clock_or_order=0,
                  dropped_no_combined_imu=0)
    info = {'path': 'times_ns.txt',
            'method': 'unique frame-number pairs with identical SDK stamps; first_imu_ns < stamp < last_imu_ns',
            'stamp_domain': report['domains_seen'][0] if report['single_known_domain'] else None,
            'first_imu_ns': imu_rows[0][0] if imu_rows else None,
            'last_imu_ns': imu_rows[-1][0] if imu_rows else None,
            'counts': counts,
            'exclusion_reasons': {
                'dropped_head': 'stamp <= first combined-IMU stamp; ORB requires preceding IMU',
                'dropped_tail': 'stamp >= last combined-IMU stamp; ORB requires following IMU',
                'dropped_unpaired_or_mismatched': 'missing/ambiguous frame-number pair or different PNG stamp names',
                'dropped_clock_or_order': 'camera SDK clock, metadata or ordering check failed',
                'dropped_no_combined_imu': 'no combined-IMU interval available'}}
    if not imu_rows:
        counts['dropped_no_combined_imu'] = counts['input_ir1']
        return [], info
    valid = report['single_known_domain'] and all(
        not report['streams'][s]['backward'] and not report['streams'][s]['duplicates']
        and not report['streams'][s]['domain_changes'] and not report['streams'][s]['missing_metadata']
        for s in ('ir1', 'ir2'))
    if not valid:
        counts['dropped_clock_or_order'] = counts['input_ir1']
        return [], info
    lm, rm = defaultdict(list), defaultdict(list)
    for s in streams['ir1']:
        lm[s['frame_number']].append(s)
    for s in streams['ir2']:
        rm[s['frame_number']].append(s)
    stamps = []
    for s in streams['ir1']:
        number, stamp = s['frame_number'], s['sdk_ns']
        if number is None or len(lm[number]) != 1 or len(rm[number]) != 1 or rm[number][0]['sdk_ns'] != stamp:
            counts['dropped_unpaired_or_mismatched'] += 1
        elif stamp <= info['first_imu_ns']:
            counts['dropped_head'] += 1
        elif stamp >= info['last_imu_ns']:
            counts['dropped_tail'] += 1
        else:
            stamps.append(stamp)
    counts['emitted'] = len(stamps)
    return stamps, info


def export_euroc(path, out, allow_diagnostic=False):
    path, out = Path(path), Path(out)
    if out.exists():
        raise ValueError('output directory must be new (no overwrite)')
    if path.resolve().is_relative_to(out.resolve()):
        raise ValueError('input must be outside output directory')
    input_hash = sha256(path)
    streams, issues, ignored = load(path)
    report = audit_streams(streams, issues, ignored)
    if report['rejections'] and not allow_diagnostic:
        raise ValueError('export refused: ' + '; '.join(report['rejections']) + '; use --allow-diagnostic for marked artifacts')
    rows, imu_counts = combined_imu(streams, report)
    times, times_info = orb_times(streams, report, rows)
    if sha256(path) != input_hash:
        raise ValueError('input changed while reading')
    out.mkdir(parents=True)
    camera_counts = {}
    for index, name in enumerate(('ir1', 'ir2')):
        base = out / f'mav0/cam{index}'
        (base / 'data').mkdir(parents=True)
        entries, used, rejected = [], set(), []
        for s in streams[name]:
            stamp = s['sdk_ns']
            if stamp is None or stamp in used:
                rejected.append({'message_id': s['message_id'], 'reason': 'missing/duplicate SDK stamp; cannot create unique filename'})
                continue
            used.add(stamp)
            filename = f'{stamp}.png'
            (base / 'data' / filename).write_bytes(png(s['width'], s['height'], s['pixels']))
            entries.append([stamp, filename])
        csv_file(base / 'data.csv', ['#timestamp [ns]', 'filename'], entries)
        camera_counts[name] = {'input': len(streams[name]), 'exported': len(entries), 'rejected': rejected}
    raw_header = ['#sdk_timestamp [ns]', 'domain', 'writer_timestamp [ns]', 'header_timestamp [ns]',
                  'host_system_timestamp [ns]', 'frame_number', 'x', 'y', 'z', 'message_id']
    for name in ('gyro', 'accel'):
        csv_file(out / f'mav0/imu0/raw_{name}.csv', raw_header,
                 ([s['sdk_ns'], s['domain'], s['bag_ns'], s['header_ns'], s['system_ns'], s['frame_number'],
                   *s[name], s['message_id']] for s in streams[name]))
    # Preserve metadata and all stamp identities for images as well as IMU.
    csv_file(out / 'stamps.csv', ['stream', 'message_id', 'writer_ns', 'header_ns', 'sdk_ns',
                                 'domain', 'system_ns', 'frame_number', 'metadata_json'],
             ([name, s['message_id'], s['bag_ns'], s['header_ns'], s['sdk_ns'], s['domain'],
               s['system_ns'], s['frame_number'], json.dumps(s.get('metadata', {}), sort_keys=True)]
              for name, samples in streams.items() for s in samples))
    csv_file(out / 'mav0/imu0/data.csv', ['#timestamp [ns]', 'w_x [rad s^-1]', 'w_y [rad s^-1]',
             'w_z [rad s^-1]', 'a_x [m s^-2]', 'a_y [m s^-2]', 'a_z [m s^-2]'], rows)
    (out / 'times_ns.txt').write_text(''.join(f'{stamp}\n' for stamp in times), encoding='utf-8')
    (out / 'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    (out / 'audit.md').write_text(markdown(report))
    manifest = {'tool': {'name': 'take_tools', 'version': VERSION, 'source_sha256': sha256(__file__)},
                'evidence': 'SYNTHETIC-TESTED ONLY; real recordings UNKNOWN',
                'input': {'path': str(path.resolve()), 'sha256': input_hash}, 'upstream_profile': UPSTREAM,
                'status': 'DIAGNOSTIC' if report['rejections'] else 'EXPORTED',
                'stamp_used': report['stamp_used'], 'stamp_domains': report['domains_seen'],
                'clock_mapping': 'none', 'cameras': {'cam0': 'IR1', 'cam1': 'IR2'},
                'camera_counts': camera_counts, 'raw_counts': {n: len(streams[n]) for n in ('gyro', 'accel')},
                'stream_counts': {n: len(streams[n]) for n in STREAMS},
                'imu_derivative': {'method': 'accel linear interpolation onto gyro SDK stamps; no extrapolation',
                                   'max_bracket_ns': 15_000_000, 'counts': imu_counts},
                'times_file': times_info,
                'rejections': report['rejections'], 'ignored_nonframe_topics': ignored,
                'output_hashes': {str(p.relative_to(out)): sha256(p) for p in sorted(out.rglob('*')) if p.is_file()},
                'hash_scope': 'all output files except manifest.json (self hash excluded)'}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', action='version', version=VERSION)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('inspect').add_argument('file', type=Path)
    p = sub.add_parser('audit')
    p.add_argument('file', type=Path)
    p.add_argument('--json', type=Path, help='write JSON here; otherwise JSON goes to stdout')
    p.add_argument('--markdown', type=Path, help='write markdown here; otherwise summary goes to stderr')
    p = sub.add_parser('export-euroc')
    p.add_argument('file', type=Path)
    p.add_argument('outdir', type=Path)
    p.add_argument('--allow-diagnostic', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.command == 'inspect':
            result = inspect(args.file)
            print(json.dumps(result, indent=2))
            return 0 if result['supported'] else 3
        if args.command == 'audit':
            destinations = [p.resolve() for p in (args.json, args.markdown) if p is not None]
            if args.file.resolve() in destinations or len(set(destinations)) != len(destinations):
                raise ValueError('audit outputs must be distinct and must not overwrite input')
            result = audit(args.file)
            raw = json.dumps(result, indent=2, allow_nan=False)+'\n'
            if args.json:
                args.json.write_text(raw)
            else:
                print(raw, end='')
            if args.markdown:
                args.markdown.write_text(markdown(result))
            else:
                print(markdown(result), file=sys.stderr)
            return 0 if result['status'] == 'PASS' else 1
        result = export_euroc(args.file, args.outdir, args.allow_diagnostic)
        print(json.dumps(result, indent=2))
        return 0
    except Unsupported as exc:
        print(str(exc), file=sys.stderr)
        return 3
    except (ValueError, OSError, sqlite3.Error, UnicodeError) as exc:
        print(f'take_tools: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
