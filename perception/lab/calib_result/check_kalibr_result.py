#!/usr/bin/env python3
"""Offline acceptance gate for Kalibr camera-IMU and stereo-check results."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import shutil
import socket
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
LAB = HERE.parent.parent / 'calibration' / 'lab'
DEFAULT_FACTORY = LAB / 'config/camchain_factory_serial-alias-6.yaml'
DEFAULT_IMU = LAB / 'config/imu_d435i_provisional.yaml'
DEFAULT_THRESHOLDS = HERE / 'thresholds.yaml'
SUMMARY_PATH = LAB / 'mac/summarize_kalibr.py'


def load_summary():
    spec = importlib.util.spec_from_file_location('kalibr_summary', SUMMARY_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SUMMARY = load_summary()


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds').replace('+00:00', 'Z')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def yaml_read(path):
    with Path(path).open(encoding='utf-8') as stream:
        return yaml.load(stream, Loader=SUMMARY.UniqueLoader)


def finite_number(value, label):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(label + ' must be finite')
    return number


def threshold_config(path):
    cfg = yaml_read(path)
    if not isinstance(cfg, dict) or not isinstance(cfg.get('thresholds'), dict):
        raise ValueError('threshold config must contain thresholds mapping')
    result = {}
    for name, item in cfg['thresholds'].items():
        if not isinstance(item, dict) or 'value' not in item or not item.get('source'):
            raise ValueError('threshold ' + str(name) + ' needs value and source')
        result[name] = (finite_number(item['value'], name), str(item['source']))
    return result


class Gate:
    def __init__(self, limits):
        self.limits = limits
        self.criteria = []
        self.diffs = []
        self.blocked = []

    def add(self, key, description, measured, unit, threshold, passed, evidence, source=None):
        lim, default_source = self.limits[threshold]
        self.criteria.append({
            'id': key, 'description': description,
            'measured': measured, 'unit': unit,
            'threshold': f'<= {lim:g} {unit}',
            'status': 'PASS' if passed else 'FAIL',
            'evidence': evidence,
            'threshold_source': source or default_source,
        })

    def missing(self, message):
        self.blocked.append(message)


def compare_metric(g, key, desc, value, unit, threshold, evidence):
    lim = g.limits[threshold][0]
    g.add(key, desc, value, unit, threshold, abs(value) <= lim, evidence)


def compare_run_imu(run_imu, provisional_imu, g, evidence):
    run = yaml_read(run_imu)
    ref = yaml_read(provisional_imu)
    keys = ('rostopic', 'update_rate', 'gyroscope_noise_density', 'accelerometer_noise_density',
            'gyroscope_random_walk', 'accelerometer_random_walk')
    mismatches = []
    for key in keys:
        if key not in run or key not in ref or run[key] != ref[key]:
            mismatches.append(key)
    g.criteria.append({
        'id': 'imu_noise_provenance',
        'description': 'Run IMU configuration values equal the provisional D435i IMU YAML.',
        'measured': {'equal': not mismatches, 'mismatched_fields': mismatches},
        'unit': 'exact YAML values', 'threshold': 'all listed provenance fields equal',
        'status': 'PASS' if not mismatches else 'FAIL', 'evidence': evidence,
        'threshold_source': 'calibration/lab/config/imu_d435i_provisional.yaml; required by phase-18 provenance criterion.'})


def check(inputs, output, thresholds_path):
    started = utc_now()
    output.mkdir(parents=True, exist_ok=True)
    thresholds_path = Path(thresholds_path)
    policy_available = thresholds_path.is_file()
    policy_error = None
    try:
        limits = threshold_config(thresholds_path) if policy_available else {}
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        limits = {}
        policy_error = 'Threshold policy could not be evaluated: ' + str(exc)
    g = Gate(limits)
    policy_copy = output / 'input-thresholds.yaml'
    if policy_available and thresholds_path.resolve() != policy_copy.resolve():
        shutil.copyfile(thresholds_path, policy_copy)
    if not policy_available:
        g.missing('Required threshold policy unavailable: ' + str(thresholds_path))
    elif policy_error:
        g.missing(policy_error)
    expected = ('camimu_chain', 'camimu_results', 'stereo_chain', 'stereo_results', 'run_imu', 'provisional_imu', 'factory')
    sources = {k: Path(v) for k, v in inputs.items() if v is not None}
    for key in expected:
        if key not in sources:
            g.missing('Required input argument unavailable: ' + key)
    missing = [f'{key}: {path}' for key, path in sources.items() if not path.is_file()]
    for item in missing:
        g.missing('Required input unavailable: ' + item)
    raw = ([{'path': policy_copy.name, 'sha256': sha256(policy_copy)}]
           if policy_copy.is_file() else [])
    copied = {}
    for key, src in sources.items():
        if src.is_file():
            dest = output / ('input-' + key + src.suffix)
            shutil.copyfile(src, dest)
            copied[key] = dest
            raw.append({'path': dest.name, 'sha256': sha256(dest)})
    if not g.blocked:
        try:
            camchain = SUMMARY.load_chain(copied['camimu_chain'])
            stereo = SUMMARY.load_chain(copied['stereo_chain'])
            factory = SUMMARY.load_chain(copied['factory'])
            cam_text = copied['camimu_results'].read_text(encoding='utf-8')
            stereo_text = copied['stereo_results'].read_text(encoding='utf-8')
            cam_stats = SUMMARY.parse_reprojection(cam_text, 'camimu')
            stereo_stats = SUMMARY.parse_reprojection(stereo_text, 'stereo')
            # Reuse the canonical transform validators, direction and factory values.
            imu_to_cam = SUMMARY.transform(camchain['cam0']['T_cam_imu'])
            cam1_imu = SUMMARY.transform(camchain['cam1']['T_cam_imu'])
            cam1_cam0 = SUMMARY.transform(camchain['cam1']['T_cn_cnm1'])
            closure_limit = g.limits['chain_closure_abs'][0]
            if not np.allclose(cam1_imu, cam1_cam0 @ imu_to_cam, atol=closure_limit, rtol=0):
                raise ValueError('camera-IMU transforms do not close the stereo chain')
            cam_to_imu = SUMMARY.inverse_transform(imu_to_cam)
            factory_t = SUMMARY.FACTORY_T_BC
            trans_delta = float(np.linalg.norm(cam_to_imu[:3, 3] - factory_t))
            rot_delta = SUMMARY.rotation_angle_deg(cam_to_imu[:3, :3])
            g.diffs.append(('cam0 camera->IMU translation (m)', repr(cam_to_imu[:3, 3].tolist()),
                            repr(factory_t.tolist()), repr((cam_to_imu[:3, 3] - factory_t).tolist()) +
                            f'; norm={trans_delta*1000:.6f} mm; rotation delta={rot_delta:.6f} deg'))
            compare_metric(g, 'imu_translation_delta', 'Camera-to-IMU translation delta versus factory CAD reference.', trans_delta, 'm', 'imu_translation_delta_m', copied['camimu_chain'].name)
            compare_metric(g, 'imu_rotation_delta', 'Camera-to-IMU rotation angle versus identity factory reference.', rot_delta, 'deg', 'imu_rotation_delta_deg', copied['camimu_chain'].name)

            for camera in ('cam0', 'cam1'):
                if camera not in cam_stats or camera not in stereo_stats:
                    g.missing(f'{camera} reprojection statistics absent or unrecognized in result text')
                    continue
                for field in ('mean', 'std'):
                    value = cam_stats[camera][field]
                    compare_metric(g, f'{camera}_camimu_reprojection_{field}', f'{camera} camera-IMU reprojection {field}.', value, 'px', 'reprojection_' + field + '_px', copied['camimu_results'].name)
                    # Stereo summary parses signed per-axis means and population standard deviations;
                    # bound the magnitude of each component to avoid cancellation.
                    sv = stereo_stats[camera][field]
                    for axis, component in zip(('x', 'y'), sv):
                        compare_metric(g, f'{camera}_stereo_reprojection_{field}_{axis}', f'{camera} stereo reprojection {field} {axis}-component.', abs(component), 'px', 'reprojection_' + field + '_px', copied['stereo_results'].name)

            residual_patterns = {
                'gyro': (r'Gyroscope error \(imu0\) \[rad/s\]:\s*mean\s+([^,]+),\s*median\s+([^,]+),\s*std\x3a\s*([^\s]+)', 'rad/s', 'gyro_residual_mean_rad_s', 'gyro_residual_std_rad_s'),
                'accel': (r'Accelerometer error \(imu0\) \[m/s\^2\]:\s*mean\s+([^,]+),\s*median\s+([^,]+),\s*std\x3a\s*([^\s]+)', 'm/s^2', 'accel_residual_mean_m_s2', 'accel_residual_std_m_s2'),
            }
            import re
            for label, (pattern, unit, mean_limit, std_limit) in residual_patterns.items():
                matches = re.findall(pattern, cam_text)
                if len(matches) != 1:
                    g.missing(f'{label} residual summary must occur exactly once in cam-IMU results')
                    continue
                mean = abs(finite_number(matches[0][0], label + ' mean'))
                std = finite_number(matches[0][2], label + ' std')
                if std < 0:
                    raise ValueError(label + ' residual std is negative')
                compare_metric(g, label + '_residual_mean', f'{label} residual mean magnitude.', mean, unit, mean_limit, copied['camimu_results'].name)
                compare_metric(g, label + '_residual_std', f'{label} residual standard deviation.', std, unit, std_limit, copied['camimu_results'].name)

            for camera in ('cam0', 'cam1'):
                shift = abs(finite_number(camchain[camera]['timeshift_cam_imu'], camera + ' timeshift'))
                limit, source = g.limits['abs_timeshift_s']
                g.criteria.append({'id': camera + '_timeshift',
                                   'description': f'{camera} camera-to-IMU time-shift magnitude; t_imu = t_cam + shift.',
                                   'measured': shift, 'unit': 's', 'threshold': f'< {limit:g} s',
                                   'status': 'PASS' if shift < limit else 'FAIL',
                                   'evidence': copied['camimu_chain'].name, 'threshold_source': source})

            for camera in ('cam0', 'cam1'):
                fitted, reference = stereo[camera], factory[camera]
                model_ok = fitted.get('camera_model') == 'pinhole' and fitted.get('distortion_model') == 'radtan'
                g.criteria.append({'id': camera + '_distortion_model',
                                   'description': f'{camera} fitted distortion model matches the factory pinhole-radtan stream.',
                                   'measured': {'camera_model': fitted.get('camera_model'), 'distortion_model': fitted.get('distortion_model')},
                                   'unit': 'model', 'threshold': 'pinhole / radtan',
                                   'status': 'PASS' if model_ok else 'FAIL', 'evidence': copied['stereo_chain'].name,
                                   'threshold_source': 'phase-18 operating-stream compatibility requirement and factory camchain.'})
                if not model_ok:
                    continue
                kval = SUMMARY.vector(fitted['intrinsics'], 4)
                kref = SUMMARY.vector(reference['intrinsics'], 4)
                for name, val, refval in zip(('fx', 'fy', 'cx', 'cy'), kval, kref):
                    delta = abs(float(val - refval))
                    compare_metric(g, f'{camera}_{name}_drift', f'{camera} {name} absolute drift versus factory.', delta, 'px', 'intrinsics_delta_px', copied['stereo_chain'].name)
                    g.diffs.append((f'{camera}.{name} (px)', f'{val:.9f}', f'{refval:.9f}', f'{val-refval:+.9f}'))
                dval = SUMMARY.vector(fitted['distortion_coeffs'], 4)
                dref = SUMMARY.vector(reference['distortion_coeffs'], 4)
                for name, val, refval in zip(('k1', 'k2', 'p1', 'p2'), dval, dref):
                    delta = abs(float(val - refval))
                    compare_metric(g, f'{camera}_{name}_distortion_delta', f'{camera} {name} distortion drift versus factory.', delta, 'coefficient', 'distortion_delta', copied['stereo_chain'].name)
                    compare_metric(g, f'{camera}_{name}_distortion_sanity', f'{camera} {name} absolute distortion coefficient sanity.', abs(float(val)), 'coefficient', 'distortion_abs', copied['stereo_chain'].name)
                    g.diffs.append((f'{camera}.{name} distortion', f'{val:.9g}', f'{refval:.9g}', f'{val-refval:+.9g}'))

            bfit = float(np.linalg.norm(SUMMARY.transform(stereo['cam1']['T_cn_cnm1'])[:3, 3]))
            bref = float(np.linalg.norm(SUMMARY.transform(factory['cam1']['T_cn_cnm1'])[:3, 3]))
            bdelta = abs(bfit - bref)
            compare_metric(g, 'stereo_baseline_delta', 'Fitted stereo baseline norm delta versus factory camchain baseline.', bdelta, 'm', 'stereo_baseline_delta_m', copied['stereo_chain'].name)
            g.diffs.append(('stereo baseline norm (mm)', f'{bfit*1000:.9f}', f'{bref*1000:.9f}', f'{(bfit-bref)*1000:+.6f}'))
            compare_run_imu(copied['run_imu'], copied['provisional_imu'], g, copied['run_imu'].name)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            g.missing('Could not evaluate required Kalibr input: ' + str(exc))
    status = 'FAIL' if any(c['status'] == 'FAIL' for c in g.criteria) else ('BLOCKED' if g.blocked else 'PASS')
    if status == 'BLOCKED':
        for item in g.blocked:
            g.criteria.append({'id': 'input_available', 'description': item, 'measured': None, 'unit': '',
                               'threshold': 'all required files and parseable summaries present', 'status': 'BLOCKED',
                               'evidence': 'result.json', 'threshold_source': 'phase-18 input requirement'})
    table = ['| Quantity | Fit | Factory | Delta / notes |', '|---|---:|---:|---|']
    table.extend('| ' + ' | '.join(row) + ' |' for row in g.diffs)
    if not g.diffs:
        table.append('| unavailable | Required inputs missing or invalid | — | — |')
    (output / 'diff.txt').write_text('Diff table versus factory camchain\n' + '\n'.join(table) + '\n', encoding='utf-8')
    host = {'hostname': socket.gethostname(), 'platform': platform.platform(), 'python': platform.python_version()}
    result = {'check': 'kalibr_result_acceptance', 'tool_version': '1.0', 'status': status,
              'started_utc': started, 'ended_utc': utc_now(), 'host': host,
              'criteria': g.criteria, 'raw_files': raw,
              'notes': ['Synthetic fixture results only establish parser behavior, not calibration validity.',
                        'CALCULATED: transform deltas, intrinsic drift, distortion drift, and stereo baseline delta are computed against saved factory references.',
                        'SOURCE-SUPPORTED: stereo factory values come from the archived SDK readback camchain.',
                        'ASSUMED: camera-to-IMU extrinsic limits are the documented 5 mm / 2 degree review bounds.',
                        'PROPOSED: locally selected reprojection, IMU residual, time-shift, intrinsic, and distortion thresholds require real D435i review.',
                        'UNKNOWN: physical calibration accuracy, time-shift repeatability, target metrology, and ORB-SLAM3 behavior.',
                        'Status PASS means criteria satisfy configured policy; it does not prove ORB-SLAM3 performance or physical calibration accuracy.',
                        'See diff.txt for factory comparison.', *g.blocked]}
    (output / 'result.json').write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    lines = [f"Kalibr result acceptance: {status} ({len(g.criteria)} criteria)"]
    for criterion in g.criteria:
        measured = criterion['measured']
        if isinstance(measured, float):
            measured = f"{measured:.6g} {criterion['unit']}"
        lines.append(f"{criterion['status']:7} {criterion['id']}: {measured}")
    table = '\n'.join(lines) + '\n'
    (output / 'check.log').write_text(table, encoding='utf-8')
    print(table, end='')
    return {'PASS': 0, 'FAIL': 1, 'BLOCKED': 2}[status]


class ContractArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(3, f'{self.prog}: error: {message}\n')


def main(argv=None):
    parser = ContractArgumentParser(description=__doc__)
    parser.add_argument('--camimu-chain', type=Path)
    parser.add_argument('--camimu-results', type=Path)
    parser.add_argument('--stereo-chain', type=Path)
    parser.add_argument('--stereo-results', type=Path)
    parser.add_argument('--run-imu', type=Path)
    parser.add_argument('--provisional-imu', type=Path, default=DEFAULT_IMU)
    parser.add_argument('--factory', type=Path, default=DEFAULT_FACTORY)
    parser.add_argument('--thresholds', type=Path, default=DEFAULT_THRESHOLDS)
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--analyze-only', type=Path, metavar='SAVED_DIR', help='re-evaluate raw input snapshots from a prior output directory')
    args = parser.parse_args(argv)
    keys = ('camimu_chain', 'camimu_results', 'stereo_chain', 'stereo_results', 'run_imu', 'provisional_imu', 'factory')
    if args.analyze_only:
        names = {'camimu_chain': 'input-camimu_chain.yaml', 'camimu_results': 'input-camimu_results.txt',
                 'stereo_chain': 'input-stereo_chain.yaml', 'stereo_results': 'input-stereo_results.txt',
                 'run_imu': 'input-run_imu.yaml', 'provisional_imu': 'input-provisional_imu.yaml', 'factory': 'input-factory.yaml'}
        inputs = {key: args.analyze_only / names[key] for key in keys}
        thresholds = args.analyze_only / 'input-thresholds.yaml'
    else:
        inputs = dict(zip(keys, (args.camimu_chain, args.camimu_results, args.stereo_chain,
                                 args.stereo_results, args.run_imu, args.provisional_imu, args.factory)))
        thresholds = args.thresholds
        if thresholds.is_file():
            # Include the exact evaluated policy in the saved evidence bundle.
            args.outdir.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(thresholds, args.outdir / 'input-thresholds.yaml')
    try:
        return check(inputs, args.outdir, thresholds)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print('Internal error: ' + str(exc), file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
