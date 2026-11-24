"""ROS-free computation and message specifications for live depth advisory."""
from dataclasses import dataclass, fields
import importlib
import math
from pathlib import Path
import sys
import time

import numpy as np


@dataclass
class CoreParams:
    stride: int = 2
    min_obstacle_height: float = 0.02
    corridor_half_width: float = 0.25
    min_forward: float = 0.30
    max_forward: float = 3.0
    cell_size: float = 0.02
    min_cells: int = 3
    plane_max_age: int = 3
    plane_gate_deg: float = 30.0
    jump_min_h: float = 0.03
    jump_max_h: float = 0.12
    confirm_n: int = 3
    confirm_m: int = 5
    ema_alpha: float = 0.4
    near_m: float = 1.5
    hysteresis_frames: int = 2
    max_rate_hz: float = 10.0
    publish_obstacle_points: bool = True
    point_stride: int = 8
    corridor_length_m: float = 2.0


class FrameRateLimiter:
    """Monotonic gate; rejected frames are dropped, never queued."""
    def __init__(self, max_rate_hz):
        self.period = 0.0 if max_rate_hz <= 0 else 1.0 / float(max_rate_hz)
        self.last_time = None

    def accept(self, now):
        if self.last_time is not None and now - self.last_time < self.period:
            return False
        self.last_time = now
        return True


def depth_to_metres(depth, encoding):
    """Normalize ROS depth encodings; RealSense 16UC1 is millimetres, zero invalid."""
    arr = np.asarray(depth)
    enc = encoding.upper()
    if enc == '16UC1':
        out = arr.astype(np.float32) * 0.001
        out[arr == 0] = 0.0
        return out
    if enc == '32FC1':
        out = arr.astype(np.float32, copy=True)
        out[~np.isfinite(out) | (out <= 0)] = 0.0
        return out
    raise ValueError(f'unsupported depth encoding: {encoding}')


class AdvisoryCore:
    def __init__(self, algo_path, params=None):
        self.params = params or CoreParams()
        self.algo_path = str(Path(algo_path).expanduser().resolve())
        if not Path(self.algo_path, 'tools', 'obstacle_decision').is_dir():
            raise ValueError(f"algo_path must contain tools/obstacle_decision: {self.algo_path}")
        if self.algo_path not in sys.path:
            sys.path.insert(0, self.algo_path)
        self.detect = importlib.import_module('tools.obstacle_decision.detect')
        self.decide = importlib.import_module('tools.obstacle_decision.decide')
        self.codec = importlib.import_module('tools.obstacle_decision.advisory_codec')
        self.detect_state = {}
        self.decision_state = {}
        self.range_state = {}
        self.sequence = 0
        self.advisory_sequence = 0

    def process(self, depth_m, K, *, frame_stamp_ns=0, frame_id='camera_depth_optical_frame',
                 now_monotonic=None, range_limit_m=None, range_hysteresis=0.10):
        p = self.params
        started = time.perf_counter()
        dp = self.detect.Params(**{f.name: getattr(p, f.name) for f in fields(self.detect.Params)
                                  if hasattr(p, f.name)})
        frame, self.detect_state = self.detect.process_frame(depth_m, K, dp, self.detect_state)
        if range_limit_m is not None:
            from tools.obstacle_decision.range_filter import in_range
            frame, self.range_state = in_range(frame, range_limit_m, range_hysteresis, self.range_state)
        decision_params = self.decide.DecisionParams(**{
            f.name: getattr(p, f.name) for f in fields(self.decide.DecisionParams)
            if hasattr(p, f.name)})
        decision, self.decision_state = self.decide.update(
            frame, self.decision_state, decision_params)
        processing_ms = (time.perf_counter() - started) * 1000.0
        advisory = {
            'advisory': decision.advisory.name,
            'advisory_code': int(decision.advisory),
            'confidence': float(decision.confidence),
            'distance_m': _finite_or_none(decision.distance_m),
            'height_m': _finite_or_none(decision.height_m),
            'width_m': _finite_or_none(decision.width_m),
            'plane_valid': bool(frame.plane_valid),
            'plane_height_m': _finite_or_none(frame.plane_height_m),
            'plane_pitch_rad': _finite_or_none(frame.plane_pitch_rad),
            'frame_stamp_ns': int(frame_stamp_ns),
            'frame_id': frame_id,
            'processing_ms': processing_ms,
        }
        points = _obstacle_points(depth_m, K, p.point_stride) if p.publish_obstacle_points else np.empty((0, 3), np.float32)
        markers = marker_specs(advisory, p.corridor_half_width, p.corridor_length_m, frame_id)
        mono_ns = int((time.monotonic() if now_monotonic is None else now_monotonic) * 1e9)
        # The proposed wire record requires finite floats; zero encodes unavailable fields.
        wire = self.codec.encode(
            0, self.sequence, pi_monotonic_ns=mono_ns, frame_stamp_ns=int(frame_stamp_ns),
            advisory=int(decision.advisory), confidence=float(decision.confidence),
            distance_m=_finite_or_zero(decision.distance_m), height_m=_finite_or_zero(decision.height_m),
            width_m=_finite_or_zero(decision.width_m), plane_valid=frame.plane_valid,
            plane_pitch_rad=_finite_or_zero(frame.plane_pitch_rad),
            plane_height_m=_finite_or_zero(frame.plane_height_m))
        sequence = self.advisory_sequence
        self.advisory_sequence += 1
        self.sequence = (self.sequence + 1) & 0xffffffff
        return {'advisory': advisory, 'markers': markers, 'points': points,
                'advisory_bytes': list(wire), 'sequence': sequence,
                'frame_result': frame}


def pack_terrain_advisory(advisory, sequence, age_s):
    """Pack the fixed Float64MultiArray layout for the STM32 advisory topic."""
    code = int(advisory['advisory_code'])
    if code < 0 or code > 3:
        raise ValueError(f'advisory code must be in [0, 3], got {code}')
    confidence = min(1.0, max(0.0, _finite_or_zero(advisory.get('confidence'))))
    return [
        float(code),
        _finite_or_zero(advisory.get('distance_m')),
        _finite_or_zero(advisory.get('height_m')),
        _finite_or_zero(advisory.get('width_m')),
        confidence,
        1.0 if advisory.get('plane_valid', False) else 0.0,
        float(sequence),
        max(0.0, _finite_or_zero(age_s)),
    ]


def marker_specs(advisory, corridor_half_width, corridor_length_m, frame_id):
    """Plain marker dictionaries; positions and points are optical-frame metres."""
    valid = advisory['plane_valid'] and advisory['plane_height_m'] is not None
    h = advisory['plane_height_m'] if valid else 0.30
    pitch = advisory['plane_pitch_rad'] if advisory['plane_pitch_rad'] is not None else 0.0
    roll = 0.0
    n = np.array([math.sin(roll), -math.cos(pitch)*math.cos(roll), -math.sin(pitch)*math.cos(roll)])
    # Ground plane n·p+h=0, with forward Z and lateral -X.
    def floor_point(forward, lateral):
        x = -lateral
        z = float(forward)
        y = -(h + n[0]*x + n[2]*z) / n[1] if abs(n[1]) > 1e-5 else 0.0
        return [float(x), float(y), float(z)]
    half = float(corridor_half_width); length = float(corridor_length_m)
    a, b = floor_point(.45, -half), floor_point(length, -half)
    c, d = floor_point(length, half), floor_point(.45, half)
    plane = [a, b, c, a, c, d]
    distance = advisory['distance_m'] or 1.0
    height = advisory['height_m'] or 0.0
    width = max(advisory['width_m'] or 0.1, 0.05)
    center_z = max(float(distance), .1)
    ground_y = floor_point(center_z, 0.)[1]
    obstacle = {'center': [0.0, ground_y-height/2, center_z], 'scale': [width, max(height, .01), .04]}
    state = advisory['advisory']
    color = {'NONE': [0.1, 0.8, 0.2, 0.8], 'JUMP_CANDIDATE': [1.0, .65, 0.0, .9],
             'STOP_CANDIDATE': [1.0, 0.0, 0.0, .9], 'UNCERTAIN': [.9, .7, .1, .9]}.get(state, [.8,.8,.8,1.])
    return [
        {'ns': 'ground', 'id': 0, 'type': 'TRIANGLE_LIST', 'frame_id': frame_id,
         'points': plane, 'scale': [1., 1., 1.],
         'color': [.1,.8,.3,.25] if valid else [.5,.5,.5,.12], 'text': ''},
        {'ns': 'obstacle', 'id': 1, 'type': 'CUBE', 'frame_id': frame_id,
         **obstacle, 'scale': obstacle['scale'], 'color': color, 'text': ''},
        {'ns': 'obstacle_height', 'id': 2, 'type': 'TEXT_VIEW_FACING', 'frame_id': frame_id,
         'position': [0., ground_y-height-.06, center_z], 'scale': [.0,.0,.10], 'color': color,
         'text': f'{height:.3f} m' if height else 'height unavailable'},
        {'ns': 'corridor', 'id': 3, 'type': 'LINE_LIST', 'frame_id': frame_id,
         'points': [floor_point(.45,-half), floor_point(length,-half),
                    floor_point(.45,half), floor_point(length,half)],
         'scale': [.018,0.,0.], 'color': [.1,.6,1.,.9], 'text': ''},
    ]


def _obstacle_points(depth, K, stride):
    dep = np.asarray(depth, dtype=np.float32)
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    rows, cols = np.mgrid[0:dep.shape[0]:max(1,stride), 0:dep.shape[1]:max(1,stride)]
    z = dep[::max(1,stride), ::max(1,stride)]
    good = np.isfinite(z) & (z > 0)
    return np.column_stack(((cols[good]-cx)*z[good]/fx, (rows[good]-cy)*z[good]/fy, z[good])).astype(np.float32)


def _finite_or_none(value):
    return float(value) if value is not None and math.isfinite(float(value)) else None


def _finite_or_zero(value):
    value = _finite_or_none(value)
    return 0.0 if value is None else value
