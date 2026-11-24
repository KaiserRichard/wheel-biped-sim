"""Validate the shared JSON format using only the standard library."""
import argparse
import json
import math
from pathlib import Path

from tools.obstacle_decision.decide import Advisory

SCENARIOS = Path(__file__).with_name('scenarios')


def _keys(obj, required, optional=()):
    if not isinstance(obj, dict):
        raise ValueError('expected an object')
    missing = set(required) - obj.keys()
    extra = obj.keys() - set(required) - set(optional)
    if missing or extra:
        raise ValueError(f'missing keys {sorted(missing)}, unknown keys {sorted(extra)}')


def _number(value, name, minimum=None, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if minimum is not None and (value < minimum or (strict and value == minimum)):
        raise ValueError(f'{name} must be {">" if strict else ">="} {minimum}')
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name} must be nonempty text')


def validate(data):
    """Raise ValueError on schema/sanity failure; return the original object."""
    _keys(data, ('name', 'description', 'status', 'robot', 'obstacles'), ('expected',))
    for key in ('name', 'description', 'status'):
        _text(data[key], key)
    if not data['status'].startswith('ASSUMED'):
        raise ValueError('status must be ASSUMED')
    robot = data['robot']
    _keys(robot, ('start_x', 'speed_m_s', 'duration_s', 'l0', 'jump_at_s'))
    _number(robot['start_x'], 'start_x')
    _number(robot['speed_m_s'], 'speed_m_s', 0)
    duration = _number(robot['duration_s'], 'duration_s', 0, True)
    _number(robot['l0'], 'l0', 0, True)
    if robot['jump_at_s'] is not None:
        jump = _number(robot['jump_at_s'], 'jump_at_s', 0)
        if jump >= duration:
            raise ValueError('jump_at_s must be before duration_s')
    if not isinstance(data['obstacles'], list):
        raise ValueError('obstacles must be a list')
    fields = {'box': ('x_face', 'y', 'depth', 'width', 'height', 'yaw_deg'),
              'step': ('x_face', 'height'),
              'ramp': ('x_start', 'length', 'rise', 'width')}
    for ob in data['obstacles']:
        if not isinstance(ob, dict) or ob.get('type') not in fields:
            raise ValueError('obstacle type must be box, step or ramp')
        _keys(ob, ('type', *fields[ob['type']]))
        for key in fields[ob['type']]:
            _number(ob[key], key, 0 if key in ('depth', 'width', 'height', 'length', 'rise') else None,
                    key in ('depth', 'width', 'height', 'length', 'rise'))
        front = ob.get('x_face', ob.get('x_start'))
        if front <= robot['start_x']:
            raise ValueError('obstacle must start in front of robot axle')
    expected = data.get('expected', [])
    if not isinstance(expected, list):
        raise ValueError('expected must be a list')
    end = 0.
    for win in expected:
        _keys(win, ('t_from', 't_to', 'advisory', 'note'))
        a = _number(win['t_from'], 't_from', 0)
        b = _number(win['t_to'], 't_to', 0)
        if not (end <= a < b <= duration):
            raise ValueError('expected windows must be ordered, nonoverlapping and inside duration')
        if not isinstance(win['advisory'], str) or win['advisory'] not in Advisory.__members__:
            raise ValueError('invalid advisory name')
        _text(win['note'], 'note')
        end = b
    return data


def load(path):
    return validate(json.loads(Path(path).read_text(encoding='utf-8')))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='*', type=Path)
    args = parser.parse_args(argv)
    paths = args.paths or sorted(SCENARIOS.glob('*.json'))
    failed = not paths
    for path in paths:
        try:
            load(path)
            print(f'PASS {path.name}')
        except (OSError, ValueError, TypeError) as exc:
            print(f'FAIL {path}: {exc}')
            failed = True
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
