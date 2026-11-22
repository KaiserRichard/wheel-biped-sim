#!/usr/bin/env python3
"""Offline qualification of one immutable D435i calibration recording (no devices)."""
import argparse
from array import array
import bisect
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import re
import shutil
import sqlite3
import sys
from datetime import datetime, timezone

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
VERSION = '1.0.0'
EXIT = {'PASS': 0, 'FAIL': 1, 'BLOCKED': 2}
CAM_HEADER = ['stream', 'index', 'frame_number', 'sdk_ms', 'domain', 'host_steady_ns',
              'host_utc_ns', 'x', 'y', 'z', 'exposure_us']
ALLAN_HEADER = ['sdk_ts_ms', 'domain', 'frame_number', 'host_steady_ns', 'x', 'y', 'z']


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def utc():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def load_config(path):
    config = json.loads(path.read_text())
    default = json.loads((HERE / 'criteria.json').read_text())
    if config.keys() != default.keys():
        raise ValueError('config must contain exactly the keys in criteria.json')
    for key, item in config.items():
        if not isinstance(item, dict) or not isinstance(item.get('source'), str) or not item['source'].strip() or 'value' not in item:
            raise ValueError(f'{key}: value and source required')
        values = item['value']
        values = list(values.values()) if isinstance(values, dict) else values
        values = values if isinstance(values, list) else [values]
        for value in values:
            if isinstance(value, (float, int)) and (isinstance(value, bool) or not math.isfinite(value) or value < 0):
                raise ValueError(f'{key}: finite nonnegative thresholds required')
        # Keep structural types and dimensions stable for reviewed custom policies.
        template = default[key]['value']
        if isinstance(template, list) and (not isinstance(item['value'], list) or len(template) != len(item['value'])):
            raise ValueError(f'{key}: wrong vector size')
        if isinstance(template, dict) and (not isinstance(item['value'], dict) or template.keys() != item['value'].keys()):
            raise ValueError(f'{key}: wrong stream keys')
        if isinstance(template, (float, int)) and not isinstance(item['value'], (float, int)):
            raise ValueError(f'{key}: numeric threshold required')
        if isinstance(template, str) and not isinstance(item['value'], str):
            raise ValueError(f'{key}: string required')
    for key in ('rates_hz', 'gyro_rms_rad_s', 'accel_std_m_s2', 'camera_duration_s', 'ir_dimensions'):
        value = config[key]['value']
        for n in value.values() if isinstance(value, dict) else value:
            if not isinstance(n, (float, int)) or n <= 0:
                raise ValueError(f'{key}: positive numeric values required')
    for key in ('sharpness_grid_stride', 'min_samples'):
        if not isinstance(config[key]['value'], int) or config[key]['value'] < 1:
            raise ValueError(f'{key}: positive integer required')
    for key in ('card_limit_bytes_s', 'allan_duration_hours', 'gap_periods'):
        if config[key]['value'] <= 0:
            raise ValueError(f'{key}: positive value required')
    if config['allowed_discontinuities']['value'] != 0:
        raise ValueError('timestamp reversals, duplicates, domain transitions and drops cannot be permitted')
    if not set(config['allowed_sdk_domains']['value']) <= {'hardware_clock', 'global_time'}:
        raise ValueError('calibration requires D435i hardware/global SDK domains')
    if config['emitter_enabled_readback']['value'] != 0:
        raise ValueError('calibration requires emitter OFF')
    for key in ('rate_relative_tolerance', 'low_sharpness_fraction_max', 'write_headroom_fraction'):
        if not 0 <= config[key]['value'] < 1:
            raise ValueError(f'{key}: fraction must be in [0,1)')
    if config['camera_duration_s']['value'][0] > config['camera_duration_s']['value'][1]:
        raise ValueError('reversed camera duration bounds')
    return config


class Samples:
    """Compact full-take timestamps; online XYZ moments, counters and host order.

    Decimal subtraction precedes float conversion. Relative SDK nanoseconds retain
    sub-nanosecond CSV precision without converting the epoch to a binary float.
    take_tools.timing supplies rates/order/gaps; no clean segment is substituted.
    """
    def __init__(self):
        self.stamps, self.domains = array('d'), array('i')
        self.anchors, self.codes = {}, {}
        self.previous_frame = self.previous_host = self.previous_utc = None
        self.frame_missing = self.frame_nonincreasing = 0
        self.host_nonpositive = self.utc_nonpositive = 0
        self.mean, self.m2, self.square_mean = [0.] * 3, [0.] * 3, [0.] * 3
        self.first = self.last = None
        self.records = []

    def add(self, row, keep=False):
        stamp = Decimal(row.get('sdk_ms', row.get('sdk_ts_ms', 'NaN')))
        vector = [float(row[a]) for a in 'xyz'] if row.get('x') else None
        if (not stamp.is_finite() or stamp < 0 or stamp*1000000 > 2**63-1
                or (vector is not None and not all(math.isfinite(v) for v in vector))):
            raise ValueError('nonfinite/negative SDK stamp or nonfinite XYZ')
        domain = row['domain'].lower().replace(' ', '_')
        self.codes.setdefault(domain, len(self.codes))
        self.anchors.setdefault(domain, stamp)
        self.stamps.append(float((stamp - self.anchors[domain]) * 1000000))
        self.domains.append(self.codes[domain])
        number, host = int(row['frame_number']), int(row['host_steady_ns'])
        if number < 0 or host < 0:
            raise ValueError('negative frame number or host steady timestamp')
        if self.previous_frame is not None:
            self.frame_missing += max(0, number - self.previous_frame - 1)
            self.frame_nonincreasing += number <= self.previous_frame
            self.host_nonpositive += host <= self.previous_host
        if 'host_utc_ns' in row:
            wall = int(row['host_utc_ns'])
            if self.previous_utc is not None:
                self.utc_nonpositive += wall <= self.previous_utc
            self.previous_utc = wall
        self.previous_frame, self.previous_host = number, host
        self.first = self.first if self.first is not None else stamp
        self.last = stamp
        if vector is not None:
            n = len(self.stamps)
            for i, x in enumerate(vector):
                delta = x - self.mean[i]
                self.mean[i] += delta / n
                self.m2[i] += delta * (x - self.mean[i])
                self.square_mean[i] += (x*x - self.square_mean[i]) / n
                if not all(math.isfinite(v) for v in (self.mean[i], self.m2[i], self.square_mean[i])):
                    raise ValueError('XYZ moments overflow')
        if keep:
            self.records.append({'sdk_ms': stamp, 'domain': domain, 'frame_number': number, 'vector': vector})

    def report(self, tt, hz):
        stats = tt.timing(self.stamps, hz, self.domains)
        stats.update(domains_seen=list(self.codes), frame_missing=self.frame_missing,
                     frame_nonincreasing=self.frame_nonincreasing, host_steady_nonpositive=self.host_nonpositive,
                     host_realtime_nonpositive=self.utc_nonpositive,
                     first_sdk_ms=str(self.first), last_sdk_ms=str(self.last),
                     timestamp_basis='relative SDK ns within each named domain; no clock mapping',
                     span_s=float((self.last-self.first)/1000) if len(self.codes) == 1 and self.first is not None else None)
        n = len(self.stamps)
        stats['rms_xyz'] = [math.sqrt(max(0., v)) for v in self.square_mean] if n else None
        stats['std_xyz'] = [math.sqrt(max(0., v/n)) for v in self.m2] if n else None
        return stats


def read_samples(base, kind, tt, rates):
    names = ('ir1', 'ir2', 'gyro', 'accel') if kind == 'camera-imu' else ('gyro', 'accel')
    samples = {n: Samples() for n in names}
    paths = [('timestamps.csv', CAM_HEADER)] if kind == 'camera-imu' else [(n+'.csv', ALLAN_HEADER) for n in names]
    for filename, header in paths:
        with (base / filename).open(newline='') as f:
            reader = csv.DictReader(f)
            if reader.fieldnames != header:
                raise ValueError(f'{filename}: unexpected CSV header')
            for line, row in enumerate(reader, 2):
                if None in row or any(v is None for v in row.values()):
                    raise ValueError(f'{filename}:{line}: incomplete row')
                if kind == 'camera-imu':
                    label = row['stream']
                    if label in ('Infrared', 'Infrared '+row['index']):
                        name = 'ir'+row['index']
                    else:
                        name = {'Gyro': 'gyro', 'Accel': 'accel'}.get(label)
                    if name not in samples:
                        raise ValueError(f'{filename}:{line}: unexpected stream/index')
                else:
                    name = filename[:-4]
                if name in ('gyro', 'accel') and any(not row[a] for a in 'xyz'):
                    raise ValueError(f'{filename}:{line}: missing XYZ')
                samples[name].add(row, keep=kind == 'camera-imu')
    return samples, {n: s.report(tt, rates[n]) for n, s in samples.items()}


def rate_ok(stats, hz, config):
    value = lambda k: config[k]['value']
    rate, gap = stats.get('rate_hz'), stats.get('max_interval_ns')
    return (stats['count'] >= value('min_samples') and rate is not None and gap is not None
            and abs(rate/hz - 1) <= value('rate_relative_tolerance')
            and gap <= value('gap_periods') * 1e9 / hz
            and stats.get('frame_missing', 0) <= value('allowed_discontinuities')
            and stats.get('frame_nonincreasing', 0) <= value('allowed_discontinuities'))


def sharpness(width, height, pixels, stride):
    """Population variance of four-neighbor Laplacian, stdlib, original Y8."""
    n, mean, m2 = 0, 0., 0.
    for y in range(1, height-1, stride):
        for x in range(1, width-1, stride):
            i = y*width+x
            lap = pixels[i-1]+pixels[i+1]+pixels[i-width]+pixels[i+width]-4*pixels[i]
            n += 1
            delta = lap-mean
            mean += delta/n
            m2 += delta*(lap-mean)
    return m2/n if n else None


def blur_report(streams, config):
    value = lambda k: config[k]['value']
    gyro = streams['gyro']
    # Never sort invalid timestamps into an apparently usable motion sequence.
    domains = {s['domain'] for records in streams.values() for s in records}
    if len(domains) != 1 or not domains <= set(value('allowed_sdk_domains')) or not gyro:
        return {'reason': 'UNKNOWN: common accepted SDK domain and gyro required'}
    if any(a['sdk_ns'] is None or b['sdk_ns'] is None or b['sdk_ns'] <= a['sdk_ns'] for a, b in zip(gyro, gyro[1:])):
        return {'reason': 'UNKNOWN: gyro SDK timeline not strictly increasing'}
    stamps = [s['sdk_ns'] for s in gyro]
    if any(s is None for s in stamps):
        return {'reason': 'UNKNOWN: missing gyro SDK timestamps'}
    result = {}
    for name in ('ir1', 'ir2'):
        moving = low = unmatched = invalid = 0
        minimum = None
        for frame in streams[name]:
            stamp = frame['sdk_ns']
            if stamp is None:
                unmatched += 1
                continue
            index = bisect.bisect_left(stamps, stamp)
            choices = [i for i in (index-1, index) if 0 <= i < len(gyro)]
            nearest = min(choices, key=lambda i: abs(stamps[i]-stamp))
            if abs(stamps[nearest]-stamp) > value('motion_match_max_ns'):
                unmatched += 1
                continue
            if math.hypot(*gyro[nearest]['gyro']) < value('motion_gyro_norm_rad_s'):
                continue
            moving += 1
            score = sharpness(frame['width'], frame['height'], frame['pixels'], value('sharpness_grid_stride'))
            if score is None:
                invalid += 1
                continue
            minimum = score if minimum is None else min(minimum, score)
            low += score < value('sharpness_min')
        result[name] = {'motion_frames': moving, 'low_sharpness_frames': low, 'unmatched_frames': unmatched,
                        'invalid_images': invalid, 'minimum_sharpness': minimum,
                        'low_sharpness_fraction': low/moving if moving and not invalid else None}
    return result


class Check:
    def __init__(self, config, out):
        self.config, self.out = config, out
        self.criteria, self.notes = [], []

    def add(self, name, description, measured, keys, status, unit='', evidence='metrics.json'):
        thresholds = {key: self.config[key]['value'] for key in keys}
        sources = '; '.join(self.config[key]['source'] for key in keys)
        self.criteria.append(dict(id=name, description=description, measured=measured, unit=unit,
                                  threshold=json.dumps(thresholds, sort_keys=True) if thresholds else 'required evidence present and internally consistent',
                                  threshold_source=sources or 'SOURCE-SUPPORTED recorder artifact contract; see README.md',
                                  status=status, evidence=evidence))

    def boolean(self, name, description, measured, keys, okay, **kwargs):
        self.add(name, description, measured, keys, 'PASS' if okay else 'FAIL', **kwargs)


def verify_manifest(path, base, required, tt):
    seen, errors = set(), []
    for line in path.read_text().splitlines():
        match = re.fullmatch(r'([0-9a-fA-F]{64}) [ *](.+)', line)
        if not match:
            errors.append('malformed sha256sum line')
            continue
        digest, name = match.groups()
        target = base / name
        if Path(name).is_absolute() or '..' in Path(name).parts or not target.resolve().is_relative_to(base.resolve()):
            errors.append('unsafe manifest path: ' + name)
            continue
        relative = target.relative_to(base).as_posix()
        if relative in seen:
            errors.append('duplicate manifest entry: ' + name)
        seen.add(relative)
        if not target.is_file() or tt.sha256(target) != digest.lower():
            errors.append('missing or checksum mismatch: ' + name)
    errors.extend('unhashed required file: ' + name for name in required if name not in seen)
    return errors


def metadata(check, kind, base, take, metrics):
    value = lambda k: check.config[k]['value']
    if kind == 'camera-imu':
        path = take / 'record.log'
        if not path.exists():
            check.add('metadata', 'SDK identity, firmware, resolved profile, emitter readback', None, [], 'BLOCKED')
            return
        log = path.read_text()
        identity = re.findall(r'SDK serial=(\S+) firmware=(\S+)', log)
        profiles = re.findall(r'^(Infrared(?: [12])?|Gyro|Accel) index=(\d+) format=(\S+) fps=(\d+)$', log, re.M)
        expected = [('Infrared', '1', 'Y8', str(value('rates_hz')['ir1'])),
                    ('Infrared', '2', 'Y8', str(value('rates_hz')['ir2'])),
                    ('Gyro', '0', 'MOTION_XYZ32F', str(value('rates_hz')['gyro'])),
                    ('Accel', '0', 'MOTION_XYZ32F', str(value('rates_hz')['accel']))]
        resolved = [('Infrared' if p[0] in ('Infrared', 'Infrared '+p[1]) else p[0], *p[1:]) for p in profiles]
        meta = {'sdk_identity_readback': identity, 'resolved_profiles': profiles,
                'emitter_off_readback': 'emitter=0 (read back before streaming)' in log}
        metrics['metadata'] = meta
        known = bool(identity) and bool(profiles) and meta['emitter_off_readback']
        if not known:
            check.add('metadata', 'UNKNOWN: incomplete recorder readback (requested profile is insufficient)', meta,
                      ['sdk_serial', 'firmware', 'rates_hz'], 'FAIL' if identity and identity != [(value('sdk_serial'), value('firmware'))] else 'BLOCKED')
        else:
            check.boolean('metadata', 'MEASURED recorder identity/profile and emitter OFF', meta,
                          ['sdk_serial', 'firmware', 'rates_hz'],
                          identity == [(value('sdk_serial'), value('firmware'))] and sorted(resolved) == sorted(expected))
        exits = re.findall(r'take-end=.* rc=(\d+) interrupted=(\d+)', log)
        okay = bool(exits) and all(x == ('0', '0') for x in exits) and 'recorder error:' not in log
        check.add('recorder_stop', 'Clean recorder finalization receipt', exits or None, [],
                  'PASS' if okay else ('FAIL' if exits or 'recorder error:' in log else 'BLOCKED'))
    else:
        path = base / 'meta.json'
        if not path.exists():
            check.add('metadata', 'UNKNOWN: missing Allan acquisition metadata', None, [], 'BLOCKED')
            return
        meta = json.loads(path.read_text())
        if not isinstance(meta, dict):
            raise ValueError('meta.json must be an object')
        metrics['metadata'] = meta
        resolved = meta.get('profiles_resolved', [])
        if not isinstance(resolved, list) or any(not isinstance(p, dict) for p in resolved):
            raise ValueError('profiles_resolved must be an array of objects')
        expected = {('Gyro', 'MOTION_XYZ32F', value('rates_hz')['gyro']),
                    ('Accel', 'MOTION_XYZ32F', value('rates_hz')['accel'])}
        profiles = {(p.get('stream'), p.get('format'), p.get('fps')) for p in resolved}
        okay = (meta.get('sdk_device_serial') == value('sdk_serial') and meta.get('firmware') == value('firmware')
                and not meta.get('serial_check_bypassed', True) and meta.get('frame') == 'imu_camera'
                and len(resolved) == len(expected) and profiles == expected)
        check.boolean('metadata', 'MEASURED D435i camera IMU identity and IMU-only resolved profile',
                      {k: meta.get(k) for k in ('sdk_device_serial', 'firmware', 'profiles_resolved', 'frame', 'serial_check_bypassed')},
                      ['sdk_serial', 'firmware', 'rates_hz'], okay)
        check.notes.append('SOURCE-SUPPORTED: Allan recorder enables only motion streams; '
                           'this alone does not attest physical emitter OFF.')
        check.boolean('recorder_stop', 'Allan clean finalization; no queue/write/recorder error',
                      {'stop_reason': meta.get('stop_reason'), 'error': meta.get('error'), 'queue_overflow': meta.get('queue_overflow')},
                      [], meta.get('error') == '' and meta.get('stop_reason') in ('duration', 'SIGINT', 'SIGTERM')
                      and not meta.get('queue_overflow', 0))


def evaluate_samples(check, reports, kind):
    value = lambda k: check.config[k]['value']
    accepted = set(value('allowed_sdk_domains'))
    common = {d for r in reports.values() for d in r['domains_seen']}
    check.boolean('sdk_domain', 'One common accepted D435i SDK clock; no mapping applied', sorted(common),
                  ['allowed_sdk_domains'], len(common) == 1 and common <= accepted)
    durations = {}
    for name, stats in reports.items():
        check.boolean('rate_drops.'+name, 'Full-take SDK rate, maximum gap and frame-number continuity', stats,
                      ['rates_hz', 'rate_relative_tolerance', 'gap_periods', 'allowed_discontinuities', 'min_samples'],
                      rate_ok(stats, value('rates_hz')[name], check.config))
        bad = {k: stats[k] for k in ('backward', 'duplicates', 'domain_changes', 'host_steady_nonpositive', 'host_realtime_nonpositive')}
        check.boolean('timestamp.'+name, 'Strictly increasing per-stream SDK, host steady and host realtime timelines',
                      bad, ['allowed_discontinuities'], all(n <= value('allowed_discontinuities') for n in bad.values()))
        durations[name] = stats['span_s']
    if any(r['span_s'] is None or r['backward'] or r['duplicates'] or r['domain_changes'] for r in reports.values()):
        check.add('duration', 'UNKNOWN: duration requires valid per-stream SDK timelines', durations,
                  ['camera_duration_s' if kind == 'camera-imu' else 'allan_duration_hours'], 'BLOCKED', unit='s')
    else:
        bounds = value('camera_duration_s') if kind == 'camera-imu' else [value('allan_duration_hours')*3600, math.inf]
        check.boolean('duration', 'Each required stream spans the configured duration in its SDK domain', durations,
                      ['camera_duration_s' if kind == 'camera-imu' else 'allan_duration_hours'],
                      all(bounds[0] <= span <= bounds[1] for span in durations.values()), unit='s')
    if kind == 'camera-imu':
        for name, metric, key, unit in [('gyro', 'rms_xyz', 'gyro_rms_rad_s', 'rad/s'),
                                         ('accel', 'std_xyz', 'accel_std_m_s2', 'm/s^2')]:
            numbers = reports[name][metric]
            check.add('excitation.'+name, 'CALCULATED per-axis '+metric+' (PROPOSED minimums)',
                      dict(zip('xyz', numbers)) if numbers else None, [key],
                      'BLOCKED' if numbers is None else ('PASS' if all(x >= t for x, t in zip(numbers, value(key))) else 'FAIL'), unit=unit)


def camera_bag(check, base, samples, metrics, tt):
    path = base / 'stereo_imu.db3'
    if not path.is_file():
        check.add('bag_audit', 'UNKNOWN: SDK recording absent; cannot decode IR or verify stereo', None, [], 'BLOCKED')
        blocked_camera_dependencies(check)
        return
    try:
        streams, issues, ignored = tt.load(path)
        audit = tt.audit_streams(streams, issues, ignored)
        # Supply the single configured tolerance rather than the upstream default.
        pairing = tt.stereo(streams, check.config['stereo_skew_ns']['value'])
        metrics['bag_audit'], metrics['stereo'] = audit, pairing
        # Audit's fixed stereo limit is displayed, not used as an independent threshold.
        nonstereo = [r for r in audit['rejections'] if r not in audit['stereo']['failures']]
        domains = set(audit['domains_seen'])
        okay = not nonstereo and domains <= set(check.config['allowed_sdk_domains']['value'])
        # Also test the separately recorded SDK host-system timeline, never subtract it from SDK/header stamps.
        system_stats = {}
        for name, records in streams.items():
            system_stats[name] = tt.timing([s['system_ns'] for s in records], check.config['rates_hz']['value'][name])
            okay = okay and not system_stats[name]['backward'] and not system_stats[name]['duplicates']
        metrics['bag_host_system_timing'] = system_stats
        check.boolean('bag_audit', 'take_tools SDK, header, writer and SDK host-system timestamp sanity',
                      {'nonstereo_rejections': nonstereo, 'system_timing': system_stats}, ['allowed_sdk_domains'], okay)
        check.boolean('stereo_pairing', 'Unique matching frame numbers in one SDK domain, bounded skew', pairing,
                      ['stereo_skew_ns'], not pairing['failures'] and bool(pairing['valid_pairs']))
        mismatch = {}
        for name, records in streams.items():
            callback = samples[name].records if samples else []
            bad = abs(len(callback)-len(records))
            for a, b in zip(callback, records):
                bad += (a['frame_number'] != b['frame_number'] or a['domain'] != b['domain']
                        or b['sdk_ns'] is None or abs(a['sdk_ms']*1000000 - b['sdk_ns']) > check.config['audit_stamp_tolerance_ns']['value'])
            mismatch[name] = bad
        check.boolean('callback_bag_agreement', 'Callback and saved bag SDK stamps/domains/frame numbers/counts agree',
                      mismatch, ['audit_stamp_tolerance_ns'], not any(mismatch.values()))
        dimensions = sorted({(s['width'], s['height']) for n in ('ir1', 'ir2') for s in streams[n]})
        check.boolean('ir_dimensions', 'Decoded Y8 IR image dimensions match calibration profile', dimensions,
                      ['ir_dimensions'], dimensions == [tuple(check.config['ir_dimensions']['value'])])
        metrics['blur'] = blur_report(streams, check.config)
        for name in ('ir1', 'ir2'):
            report = metrics['blur'].get(name)
            fraction = report.get('low_sharpness_fraction') if report else None
            status = 'BLOCKED' if fraction is None or report['unmatched_frames'] else (
                'PASS' if fraction <= check.config['low_sharpness_fraction_max']['value'] else 'FAIL')
            check.add('motion_blur.'+name, 'CALCULATED low-sharpness fraction during same-clock gyro motion',
                      report or metrics['blur'], ['sharpness_min', 'sharpness_grid_stride', 'motion_gyro_norm_rad_s',
                                               'motion_match_max_ns', 'low_sharpness_fraction_max'], status, unit='fraction')
    except (ValueError, OSError, sqlite3.Error) as exc:
        metrics['bag_error'] = str(exc)
        status = 'BLOCKED' if isinstance(exc, tt.Unsupported) else 'FAIL'
        check.add('bag_audit', 'SDK take decoder: '+str(exc), None, [], status)
        blocked_camera_dependencies(check)


def blocked_camera_dependencies(check):
    existing = {c['id'] for c in check.criteria}
    for name in ('stereo_pairing', 'callback_bag_agreement', 'ir_dimensions', 'motion_blur.ir1', 'motion_blur.ir2'):
        if name not in existing:
            check.add(name, 'UNKNOWN: unavailable decoded SDK recording', None, [], 'BLOCKED')


def storage(check, base, take, reports, metrics):
    value = lambda k: check.config[k]['value']
    size = sum(p.stat().st_size for p in take.rglob('*') if p.is_file())
    spans = [s['span_s'] for s in reports.values()]
    valid = bool(spans) and all(s is not None and s > 0 for s in spans) and all(
        not r['backward'] and not r['duplicates'] and not r['domain_changes'] for r in reports.values())
    # This is bytes per SDK acquisition span, never a measured disk-write timer.
    rate = size/min(spans) if valid else None
    report = {'payload_bytes': size, 'required_average_bytes_s': rate,
              'basis': 'CALCULATED all saved take bytes / shortest per-stream SDK span; peak/fsync throughput UNKNOWN',
              'historical_card_limit_bytes_s': value('card_limit_bytes_s'),
              'headroom_fraction': 1-rate/value('card_limit_bytes_s') if rate is not None else None}
    limit = value('card_limit_bytes_s') * (1-value('write_headroom_fraction'))
    check.add('storage.write_headroom', 'WARN if take average write demand lacks historical card headroom',
              report, ['card_limit_bytes_s', 'write_headroom_fraction'],
              'BLOCKED' if rate is None else ('PASS' if rate <= limit else 'FAIL'), unit='bytes/s')
    if rate is not None and rate > limit:
        check.notes.append('WARN: average take write demand exceeds historical microSD headroom. '
                           'RAM acquisition may avoid live card writes; copy time and a replacement medium need separate qualification.')
    log = (take / 'record.log').read_text() if (take / 'record.log').exists() else ''
    free = re.findall(r'storage_free=(\d+)', log)
    free_after = None
    if (take/'storage.json').is_file():
        receipt = json.loads((take/'storage.json').read_text())
        if receipt.get('clock') != 'linux_clock_realtime' or not receipt.get('observed_utc') or not receipt.get('source'):
            raise ValueError('storage.json requires clock=linux_clock_realtime, observed_utc and source')
        free_after = receipt.get('free_bytes_after')
        if isinstance(free_after, bool) or not isinstance(free_after, int) or free_after < 0:
            raise ValueError('storage.json free_bytes_after must be a nonnegative integer')
        report['receipt'] = receipt
    elif free:
        free_after = int(free[-1]) - size
        report['free_estimate_note'] = 'INFERRED pre-capture free minus payload; actual free after and concurrent writes UNKNOWN'
    report['free_bytes_after_or_estimate'] = free_after
    required = value('free_reserve_bytes') + value('free_take_copies')*size
    report['required_free_bytes'] = required
    check.add('storage.free_space', 'Saved capture-volume capacity receipt for OS reserve and further copies',
              report, ['free_reserve_bytes', 'free_take_copies'],
              'BLOCKED' if free_after is None or 'free_estimate_note' in report else ('PASS' if free_after >= required else 'FAIL'), unit='bytes')
    if free_after is not None and free_after < required:
        check.notes.append('WARN: saved capture-volume free space or its pre-capture estimate is below the configured reserve.')
    metrics['storage'] = report


def run(take, out, kind, config_path):
    started = utc()
    config = load_config(config_path)
    take, out = take.resolve(), out.resolve()
    if not take.is_dir():
        raise ValueError('take directory does not exist')
    if out == take or out.is_relative_to(take) or take.is_relative_to(out):
        raise ValueError('output and take must be disjoint')
    if out.exists():
        raise ValueError('output directory must be new')
    base = take/'take' if kind == 'camera-imu' and (take/'take').is_dir() else take
    # No symlinks in evidence: prevents silent hashing/copying outside one take.
    if any(p.is_symlink() for p in take.rglob('*')):
        raise ValueError('take must not contain symlinks')
    out.mkdir(parents=True)
    check = Check(config, out)
    metrics = {'evidence_label': 'CALCULATED from saved files; capture provenance is not independently attested',
               'kind': kind, 'clock_mapping': 'none', 'input_directory': str(take)}
    write_json(out/'config.json', config)
    try:
        tt = module('calib_take_tools', ROOT/'tools/take_tools/take_tools.py')
        diag = module('calib_diagnostics', ROOT/'calibration/lab/pi/take_diagnostics.py')
    except (OSError, ImportError) as exc:
        (out/'dependency-error.txt').write_text(str(exc)+'\n')
        check.add('dependencies', 'UNKNOWN: required existing repository tool unavailable', None, [], 'BLOCKED',
                  evidence='dependency-error.txt')
        raw = []
        for path in sorted(out.iterdir()):
            with path.open('rb') as f:
                raw.append({'path': path.name, 'sha256': hashlib.file_digest(f, 'sha256').hexdigest()})
        write_json(out/'result.json', dict(check='calib_record', tool_version=VERSION, status='BLOCKED',
                   started_utc=started, ended_utc=utc(), host={'hostname': platform.node(), 'os': platform.platform()},
                   criteria=check.criteria, raw_files=raw, notes=['UNKNOWN: no recording qualification performed.']))
        print('BLOCKED dependencies: '+str(exc)+'\nOverall: BLOCKED')
        return 2
    inventory = [{'path': str(p.relative_to(take)), 'size_bytes': p.stat().st_size, 'sha256': tt.sha256(p)}
                 for p in sorted(take.rglob('*')) if p.is_file()]
    write_json(out/'input-manifest.json', {'source_directory': str(take), 'files': inventory,
                                         'scope': 'all saved take files before analysis; SHA-256 is identity, not authenticity'})
    (out/'raw').mkdir()
    # Preserve command outputs verbatim; recordings themselves stay at their immutable source paths.
    for p in sorted(take.rglob('*')):
        if p.is_file() and (p.suffix in ('.log', '.json', '.txt') or p.name == 'SHA256SUMS'):
            dest = out/'raw'/p.relative_to(take)
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, dest)
    required = ['stereo_imu.db3', 'timestamps.csv', 'sdk.log'] if kind == 'camera-imu' else ['gyro.csv', 'accel.csv', 'meta.json', 'sdk.log']
    missing = [n for n in required if not (base/n).is_file() or not (base/n).stat().st_size]
    check.boolean('completeness', 'Expected nonempty recorder files present', {'missing_or_empty': missing}, [], not missing,
                  evidence='input-manifest.json')
    manifest = take/'ram-sha256.txt' if kind == 'camera-imu' and (take/'ram-sha256.txt').exists() else base/'SHA256SUMS'
    if manifest.exists():
        try:
            errors = verify_manifest(manifest, base, required, tt)
        except (OSError, UnicodeError) as exc:
            errors = ['unreadable capture manifest: '+str(exc)]
        check.boolean('capture_checksums', 'Verify original recorder payload SHA-256 manifest and coverage', errors, [], not errors,
                      evidence='input-manifest.json')
    else:
        check.add('capture_checksums', 'UNKNOWN: original capture manifest absent; new analysis inventory cannot attest original copy',
                  None, [], 'BLOCKED', evidence='input-manifest.json')
    reports, samples = {}, None
    try:
        samples, reports = read_samples(base, kind, tt, config['rates_hz']['value'])
        metrics['streams'] = reports
        evaluate_samples(check, reports, kind)
    except (ValueError, OSError, InvalidOperation) as exc:
        metrics['sample_error'] = str(exc)
        check.add('samples', 'Saved sample audit: '+str(exc), None, [], 'BLOCKED' if isinstance(exc, FileNotFoundError) else 'FAIL')
        names = ('ir1', 'ir2', 'gyro', 'accel') if kind == 'camera-imu' else ('gyro', 'accel')
        for name in names:
            check.add('rate_drops.'+name, 'UNKNOWN: sample audit unavailable', None, [], 'BLOCKED')
            check.add('timestamp.'+name, 'UNKNOWN: sample audit unavailable', None, [], 'BLOCKED')
        check.add('duration', 'UNKNOWN: sample audit unavailable', None, [], 'BLOCKED')
        if kind == 'camera-imu':
            for name in ('gyro', 'accel'):
                check.add('excitation.'+name, 'UNKNOWN: sample audit unavailable', None, [], 'BLOCKED')
    try:
        metadata(check, kind, base, take, metrics)
    except (ValueError, TypeError, AttributeError) as exc:
        check.add('metadata', 'Malformed saved metadata: '+str(exc), None, [], 'FAIL')
    if kind == 'allan':
        meta = metrics.get('metadata', {})
        expected_counts = [reports[n]['count'] for n in ('gyro', 'accel')] if reports else None
        written = [meta.get(n, {}).get('sample_count') if isinstance(meta.get(n, {}), dict) else None for n in ('gyro', 'accel')]
        counts = {'csv_counts': expected_counts, 'callback_samples': meta.get('callback_samples'),
                  'written_samples': written}
        missing = expected_counts is None or counts['callback_samples'] is None or None in counts['written_samples']
        check.add('recorder_counts', 'Allan CSV, callback and writer sample counts agree', counts, [],
                  'BLOCKED' if missing else ('PASS' if expected_counts == counts['callback_samples'] == counts['written_samples'] else 'FAIL'))
        receipt_path = take/'emitter.json'
        if not receipt_path.is_file():
            check.add('emitter_state', 'UNKNOWN: IMU-only profile is not an emitter OFF readback', None,
                      ['emitter_enabled_readback'], 'BLOCKED')
        else:
            try:
                receipt = json.loads(receipt_path.read_text())
                if (not isinstance(receipt, dict) or not receipt.get('source') or not receipt.get('observed_utc')
                        or receipt.get('clock') != 'linux_clock_realtime' or 'enabled_readback' not in receipt):
                    raise ValueError('emitter.json requires enabled_readback, source, observed_utc, clock=linux_clock_realtime')
                metrics['emitter'] = receipt
                check.boolean('emitter_state', 'Saved actual emitter OFF readback for this SDK unit', receipt,
                              ['emitter_enabled_readback', 'sdk_serial'],
                              type(receipt['enabled_readback']) in (int, float)
                              and receipt['enabled_readback'] == config['emitter_enabled_readback']['value']
                              and receipt.get('sdk_device_serial') == config['sdk_serial']['value'])
            except (ValueError, TypeError) as exc:
                check.add('emitter_state', 'Malformed emitter receipt: '+str(exc), None, [], 'FAIL')
    if kind == 'camera-imu':
        camera_bag(check, base, samples, metrics, tt)
        check.add('target_detection', 'UNKNOWN: existing take_tools/take_diagnostics expose no AprilGrid detection counts',
                  None, [], 'BLOCKED')
        # Reuse existing conservative diagnostics as supplementary output, never the policy authority.
        legacy = {}
        for name, stats in reports.items():
            legacy[{'ir1': 'Infrared 1', 'ir2': 'Infrared 2', 'gyro': 'Gyro', 'accel': 'Accel'}[name]] = {
                'count': stats['count'], 'rate_hz': stats['rate_hz'],
                'max_gap_ms': stats['max_interval_ns']/1e6 if stats['max_interval_ns'] is not None else None,
                'domains': stats['domains_seen'], 'nonpositive_steps': stats['backward']+stats['duplicates'],
                'frame_number_skips': stats['frame_missing']}
        power = (take/'power-1hz.log').read_text() if (take/'power-1hz.log').exists() else ''
        text, _ = diag.summary(legacy, power, 0 if any(c['id'] == 'recorder_stop' and c['status'] == 'PASS' for c in check.criteria) else 1,
                               0 if reports else 1, any(c['id'] == 'capture_checksums' and c['status'] == 'PASS' for c in check.criteria))
        (out/'take_diagnostics.txt').write_text(text+'\n')
    try:
        storage(check, base, take, reports, metrics)
    except (ValueError, TypeError) as exc:
        check.add('storage', 'Malformed storage evidence: '+str(exc), None, [], 'FAIL')
    # Rehash to detect changes during the analysis, including file creation/deletion.
    final_inventory = [{'path': str(p.relative_to(take)), 'size_bytes': p.stat().st_size, 'sha256': tt.sha256(p)}
                       for p in sorted(take.rglob('*')) if p.is_file()]
    write_json(out/'input-manifest-after.json', {'source_directory': str(take), 'files': final_inventory})
    check.boolean('input_unchanged', 'Source file set and SHA-256 unchanged during analysis', final_inventory == inventory, [],
                  final_inventory == inventory, evidence='input-manifest.json')
    write_json(out/'metrics.json', metrics)
    write_json(out/'tool-provenance.json', {str(p.relative_to(ROOT)): tt.sha256(p) for p in
               (Path(__file__), ROOT/'tools/take_tools/take_tools.py', ROOT/'calibration/lab/pi/take_diagnostics.py')})
    write_json(out/'invocation.json', {'argv': sys.argv, 'kind': kind, 'source_directory': str(take),
                                     'config_path': str(config_path.resolve()), 'mode': 'saved-file analysis only'})
    status = 'FAIL' if any(c['status'] == 'FAIL' for c in check.criteria) else (
        'BLOCKED' if any(c['status'] == 'BLOCKED' for c in check.criteria) else 'PASS')
    lines = ['Status  Criterion                       Detail']
    for item in check.criteria:
        name, measured = item['id'], item['measured']
        detail = item['description']
        if name.startswith('excitation.') and measured is not None:
            detail += ': ' + ', '.join(f'{axis}={number:.6g}' for axis, number in measured.items()) + ' '+item['unit']
        elif name.startswith('rate_drops.') and measured is not None:
            detail = f'count={measured["count"]}, SDK rate={measured["rate_hz"]} Hz, max gap={measured["max_interval_ns"]} ns'
        elif name == 'duration' and isinstance(measured, dict):
            detail += ': ' + ', '.join(f'{n}={span} s' for n, span in measured.items())
        elif name.startswith('motion_blur.') and isinstance(measured, dict):
            detail += ': ' + str(measured)
        elif name == 'storage.write_headroom':
            detail += ': ' + str(measured['required_average_bytes_s'])+' bytes/s'
        elif name == 'storage.free_space':
            detail += ': '+str(measured['free_bytes_after_or_estimate'])+' bytes (measured or INFERRED; see metrics)'
        lines.append(f'{item["status"]:7} {name:30} {detail}')
    table = '\n'.join(lines)
    (out/'summary.txt').write_text(table+'\nOverall: '+status+'\n')
    result = dict(check='calib_record', tool_version=VERSION, status=status, started_utc=started, ended_utc=utc(),
                  host={'hostname': platform.node(), 'os': platform.platform(), 'python': platform.python_version(),
                        'role': 'analysis host; capture host UNKNOWN unless saved receipt identifies it'},
                  criteria=check.criteria,
                  raw_files=[{'path': str(p.relative_to(out)), 'sha256': tt.sha256(p)} for p in sorted(out.rglob('*')) if p.is_file()],
                  notes=['Fixture success is tooling verification, never a physical gate PASS.',
                         'MEASURED means recorded observation; this offline tool cannot attest hardware prerequisites.',
                         'D435i imu_camera only; no STM32 imu_body samples or cross-clock subtraction.',
                         'Original large recordings are referenced by input-manifest.json and are required to reanalyze.'] + check.notes)
    write_json(out/'result.json', result)
    print(table+'\nOverall: '+status)
    return EXIT[status]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', action='version', version=VERSION)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--take-dir', type=Path, help='saved take/session directory; never starts capture')
    source.add_argument('--analyze-only', type=Path, help='re-evaluate saved raw take directory (same offline checks)')
    parser.add_argument('--kind', choices=('camera-imu', 'allan'), required=True)
    parser.add_argument('--outdir', type=Path, required=True, help='new directory outside the take')
    parser.add_argument('--config', type=Path, default=HERE/'criteria.json')
    try:
        args = parser.parse_args(argv)
        return run(args.take_dir or args.analyze_only, args.outdir, args.kind, args.config)
    except SystemExit as exc:
        return 0 if exc.code == 0 else 3
    except (OSError, ValueError, TypeError, KeyError, ImportError, sqlite3.Error) as exc:
        print('calib_record: '+str(exc), file=sys.stderr)
        return 3


if __name__ == '__main__':
    sys.exit(main())
