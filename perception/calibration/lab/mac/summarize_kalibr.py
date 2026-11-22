#!/usr/bin/env python3
"""OPEN: Kalibr summaries. Dependencies: stdlib, NumPy, PyYAML only."""
import argparse
from decimal import Decimal, ROUND_HALF_EVEN
import math
from pathlib import Path
import re
import sys

import numpy as np
import yaml

FACTORY_T_BC = np.array([-0.00552, 0.00510, 0.01174])  # m, task rounded CAD reference
NUMBER = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'


class UniqueLoader(yaml.SafeLoader):
    """Avoid silently accepting a second conflicting camera/calibration field."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError('duplicate YAML key: ' + str(key))
        result[key] = loader.construct_object(value_node)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load_chain(path):
    with Path(path).open() as stream:
        chain = yaml.load(stream, Loader=UniqueLoader)
    if not isinstance(chain, dict) or not all(isinstance(chain.get(c), dict) for c in ('cam0', 'cam1')):
        raise ValueError('expected two-camera Kalibr YAML: cam0/cam1')
    return chain


def transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('transform must be finite 4x4')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError('invalid homogeneous bottom row')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError('rotation is not proper orthonormal (tolerance 1e-6)')
    return matrix


def inverse_transform(value):
    matrix = transform(value)
    inverse = np.eye(4)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return inverse


def camera_to_imu_ns(camera_ns, shift_seconds):
    """t_imu = t_cam + shift; preserve integer ns (half-even rounding)."""
    shift = Decimal(str(shift_seconds))
    if not shift.is_finite():
        raise ValueError('nonfinite time shift')
    return int(camera_ns) + int((shift * Decimal(1_000_000_000)).to_integral_value(rounding=ROUND_HALF_EVEN))


def rotation_angle_deg(rotation):
    cosine = (np.trace(rotation) - 1) / 2
    return math.degrees(math.acos(float(np.clip(cosine, -1, 1))))


def threshold(value, limit):
    if not math.isfinite(value):
        raise ValueError('nonfinite comparison value')
    return ('within' if value <= limit else 'outside') + ' ASSUMED threshold'


def parse_reprojection(text, mode):
    """Source: IccUtil.printErrorStatistics and CameraUtils.printParameters.

    IMU-camera: population statistics of 2D residual norms, explicitly [px].
    Stereo: per-axis population mean/std; radial RMS from both second moments.
    Never use the normalized residual block or substitute mean for RMS.
    """
    stats = {}
    camera = None
    for line in text.splitlines():
        if mode == 'camimu':
            match = re.match(r'\s*Reprojection error \((cam\d+)\) \[px\]:\s*mean (' + NUMBER +
                             r'),\s*median (' + NUMBER + r'),\s*std\x3a\s*(' + NUMBER + r')\s*$', line)
            if match:
                camera, mean, median, std = match.groups()
                mean, median, std = map(float, (mean, median, std))
                if min(mean, median, std) < 0:
                    raise ValueError('negative norm residual statistic')
                entry = dict(mean=mean, median=median, std=std, rms=math.hypot(mean, std))
            else:
                continue
        else:
            heading = re.match(r'\s*(cam\d+) \([^\n]+\):\s*$', line)
            if heading:
                camera = heading.group(1)
                continue
            match = re.match(r'\s*reprojection error\x3a\s*\[(' + NUMBER + r'),\s*(' + NUMBER +
                             r')\]\s*\+-\s*\[(' + NUMBER + r'),\s*(' + NUMBER + r')\]\s*$', line)
            if not match or camera is None:
                continue
            mx, my, sx, sy = map(float, match.groups())
            if sx < 0 or sy < 0:
                raise ValueError('negative axis residual std')
            entry = dict(mean=[mx, my], std=[sx, sy], rms=math.sqrt(mx*mx + my*my + sx*sx + sy*sy))
        if camera in stats:
            raise ValueError('duplicate pixel residual statistics for ' + camera)
        if not math.isfinite(entry['rms']):
            raise ValueError('nonfinite residual RMS')
        stats[camera] = entry
    return stats


def residual_lines(text, mode):
    stats = parse_reprojection(text, mode)
    output = []
    for camera in ('cam0', 'cam1'):
        if camera not in stats:
            output.append(camera + ' pixel reprojection mean/std/RMS: UNKNOWN (missing/unrecognized/no corners); ASSUMED RMS <=0.5 px unevaluated')
        else:
            item = stats[camera]
            label = '2D norm' if mode == 'camimu' else 'X/Y components'
            output.append(f"{camera} reprojection {label}: mean={item['mean']}, std={item['std']} px; "
                          f"CALCULATED radial RMS={item['rms']:.6f} px: {threshold(item['rms'], 0.5)} (<=0.5 px)")
    return output


def camimu_summary(chain, text):
    output = ['OPEN: camera-IMU calibration diagnostics; no gate qualification.']
    for camera in ('cam0', 'cam1'):
        matrix = transform(chain[camera]['T_cam_imu'])
        inverse = inverse_transform(matrix)
        shift = float(chain[camera]['timeshift_cam_imu'])
        if not math.isfinite(shift):
            raise ValueError('nonfinite timeshift')
        output.append(f'{camera} T_cam_imu (IMU -> camera), metres\x3a\n' + np.array2string(matrix, precision=9))
        inverse_label = 'ORB IMU.T_b_c1' if camera == 'cam0' else 'camera2 -> IMU'
        output.append(f'{camera} inverse ({inverse_label}), metres\x3a\n' + np.array2string(inverse, precision=9))
        if camera == 'cam0':
            delta = inverse[:3, 3] - FACTORY_T_BC
            distance = float(np.linalg.norm(delta))
            angle = rotation_angle_deg(inverse[:3, :3])
            output.append('Factory/CAD reference T_b_c1.t = [-0.00552,+0.00510,+0.01174] m (MEASURED readback; INFERRED CAD)')
            output.append(f'CALCULATED translation difference vector={delta.tolist()} m; norm={distance:.9f} m ({distance*1000:.6f} mm): '
                          f'{threshold(distance, 0.005)} (<=5 mm)')
            output.append(f'CALCULATED rotation angle vs identity={angle:.6f} deg: {threshold(angle, 2.0)} (<=2 deg)')
        output.append(f'{camera} timeshift_cam_imu={shift*1000:.9f} ms; t_imu = t_cam + shift. '
                      'ASSUMED shift spread <=1 ms over >=3 takes: UNKNOWN from one take.')
    if not np.allclose(transform(chain['cam1']['T_cam_imu']),
                       transform(chain['cam1']['T_cn_cnm1']) @ transform(chain['cam0']['T_cam_imu']), atol=1e-5, rtol=0):
        raise ValueError('camera-IMU transforms do not close the stereo chain')
    output.extend(residual_lines(text, 'camimu'))
    output.append('Noise remains ASSUMED unless the supplied IMU YAML has independent Allan evidence. ORB YAML does not apply the time shift.')
    return '\n'.join(output) + '\n'


def vector(value, length):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError('invalid camera vector')
    return result


def stereo_summary(chain, factory, text):
    output = ['OPEN: stereo validation only; fitted values never replace MEASURED factory settings.']
    for camera in ('cam0', 'cam1'):
        fitted, reference = chain[camera], factory[camera]
        if fitted['camera_model'] != 'pinhole' or fitted['distortion_model'] != 'radtan':
            raise ValueError('expected pinhole-radtan fit')
        k, k0 = vector(fitted['intrinsics'], 4), vector(reference['intrinsics'], 4)
        for name, value, original in zip(('fx', 'fy', 'cx', 'cy'), k, k0):
            output.append(f'{camera} {name}: fitted={value:.9f}, factory={original:.9f}, delta={value-original:+.9f} px; threshold unspecified (OPEN)')
        d, d0 = vector(fitted['distortion_coeffs'], 4), vector(reference['distortion_coeffs'], 4)
        output.append(f'{camera} radtan [k1,k2,p1,p2]: fitted={d.tolist()}, factory={d0.tolist()}, delta={(d-d0).tolist()}; threshold unspecified (OPEN)')
    fitted = transform(chain['cam1']['T_cn_cnm1'])
    original = transform(factory['cam1']['T_cn_cnm1'])
    baseline, baseline0 = float(np.linalg.norm(fitted[:3, 3])), float(np.linalg.norm(original[:3, 3]))
    difference = abs(baseline - baseline0)
    output.append('cam0 -> cam1 fitted T_cn_cnm1, metres\x3a\n' + np.array2string(fitted, precision=9))
    output.append(f'CALCULATED baseline norm: fitted={baseline*1000:.6f} mm, factory={baseline0*1000:.6f} mm, '
                  f'absolute delta={difference*1000:.6f} mm: {threshold(difference, 0.001)} (<=1 mm)')
    output.extend(residual_lines(text, 'stereo'))
    output.append('Review target dimensions, corner coverage/outlier removal and systematic errors in PDF; scalar summaries cannot establish optical accuracy.')
    return '\n'.join(output) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('camimu', 'stereo'))
    parser.add_argument('--chain', required=True, type=Path)
    parser.add_argument('--results', required=True, type=Path)
    parser.add_argument('--factory', type=Path, default=Path(__file__).resolve().parents[1] / 'config/camchain_factory_serial-alias-6.yaml')
    args = parser.parse_args()
    try:
        chain, text = load_chain(args.chain), args.results.read_text()
        summary = camimu_summary(chain, text) if args.mode == 'camimu' else stereo_summary(chain, load_chain(args.factory), text)
        print(summary, end='')
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print('OPEN: summary failed: ' + str(exc), file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
