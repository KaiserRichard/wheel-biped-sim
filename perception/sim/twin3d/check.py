"""Evaluate saved CSV evidence, independently of MuJoCo and rendering."""

import argparse
import csv
import json
import math
from pathlib import Path


def evaluate(rows):
    """ASSUMED acceptance tolerances; requested 30-degree and 8/10 limits fixed."""
    if len(rows) < 2:
        return dict(passed=False, error='empty or incomplete log')
    try:
        scenario = rows[0]['scenario']
        numeric = ('t', 'x', 'pitch', 'vx', 'base_ground', 'bad_box', 'wheel_box',
                   'contact_left', 'contact_right', 'flight')
        for row in rows:
            if row['scenario'] != scenario or not all(math.isfinite(float(row[k])) for k in numeric):
                raise ValueError('mixed scenarios or nonfinite values')
        times = [float(r['t']) for r in rows]
        if abs(times[0]) > 1e-8 or any(b <= a or b-a > 0.011 for a, b in zip(times, times[1:])):
            raise ValueError('missing start, duplicate/backward time or >11ms log gap')
        peak = max(abs(float(r['pitch'])) for r in rows)
        upright = peak < math.radians(30)
        base_clear = not any(int(r['base_ground']) for r in rows)
        result = dict(scenario=scenario, seed=int(rows[0]['seed']), duration_s=times[-1],
                      peak_pitch_deg=math.degrees(peak), base_clear=base_clear)
        gate_rows = [r for r in rows if r.get('raw_advisory')]
        if gate_rows:
            inhibited = [r for r in gate_rows if r['phase'] in ('CROUCH', 'THRUST')]
            covered = {r['phase'] for r in inhibited}
            suppressed = all(r.get('gated_advisory') == 'UNCERTAIN' and float(r['gated_confidence']) == 0 for r in inhibited)
            result['gate_passed'] = suppressed and (scenario != 'jump_box' or covered == {'CROUCH', 'THRUST'})
            result['gate_samples'] = len(inhibited)
        else:
            result['gate_passed'] = None
        perception_requested = any(int(r.get('perception', 0)) for r in rows)
        if perception_requested and len(gate_rows) != len(rows):
            result['gate_passed'] = False
        result['physics_warnings'] = max(int(r.get('physics_warning', 0)) for r in rows)
        valid = upright and base_clear and result['gate_passed'] is not False and result['physics_warnings'] == 0
        if scenario == 'stand':
            result['passed'] = valid and times[-1] >= 30-1e-6
        elif scenario == 'drive':
            start = float(rows[0]['x'])
            crossing = next((r for r in rows if float(r['x'])-start >= 1.0), None)
            speed_error = abs(float(rows[-1]['vx'])-0.3)
            result.update(distance_m=float(rows[-1]['x'])-start,
                          one_m_time_s=float(crossing['t']) if crossing else None,
                          one_m_speed_m_s=float(crossing['vx']) if crossing else None,
                          final_speed_error_m_s=speed_error)
            result['passed'] = valid and crossing is not None and speed_error <= 0.03
        elif scenario == 'jump_box':
            face, depth, height = (float(rows[0][k]) for k in ('box_face', 'box_depth', 'box_height'))
            # Full wheel footprint is conservative: bottom must exceed box top
            # whenever axle +/- radius overlaps the fixture, for BOTH wheels.
            R_WHEEL = 0.0603  # SOURCE-SUPPORTED observer radius, same as model.
            clearance, traversed = [], []
            for side in ('left', 'right'):
                overlap = [r for r in rows if face-R_WHEEL <= float(r['wheel_'+side+'_x']) <= face+depth+R_WHEEL]
                clearance.extend(float(r['wheel_'+side+'_bottom'])-height for r in overlap)
                traversed.append(float(rows[-1]['wheel_'+side+'_x']) > face+depth+R_WHEEL)
            air = [i for i, r in enumerate(rows) if int(r['flight'])]
            landing = None
            if air:
                # Last airborne interval prevents counting a brief touchdown
                # before a bounce as the definitive landing.
                landing = next((r for r in rows[air[-1]+1:]
                                if int(r['contact_left']) and int(r['contact_right'])), None)
            after = ([r for r in rows if float(r['t']) >= float(landing['t'])+1.0]
                     if landing else [])
            landed_upright = bool(after) and all(abs(float(r['pitch'])) < math.radians(30) for r in after)
            box_clear = not any(int(r['bad_box']) or int(r['wheel_box']) for r in rows)
            phases = {r['phase'] for r in rows}
            result.update(min_wheel_clearance_m=min(clearance) if clearance else None,
                          traversed=all(traversed), no_box_contact=box_clear,
                          landing_s=float(landing['t']) if landing else None,
                          upright_after_one_s=landed_upright)
            result['passed'] = (valid and all(traversed) and bool(clearance) and min(clearance) > 0
                                and box_clear and landed_upright
                                and {'CROUCH', 'THRUST', 'RETRACT', 'SETTLE'} <= phases)
        else:
            raise ValueError('unknown scenario')
        return result
    except (KeyError, ValueError, TypeError, OverflowError) as exc:
        return dict(passed=False, error=str(exc))


def check(path):
    with Path(path).open(newline='') as stream:
        return evaluate(list(csv.DictReader(stream)))


def aggregate(results):
    jumps = [r for r in results if r.get('scenario') == 'jump_box']
    seeds = {r.get('seed') for r in jumps}
    jump_ok = len(jumps) == 10 and len(seeds) == 10 and sum(r['passed'] for r in jumps) >= 8
    stands = [r for r in results if r.get('scenario') == 'stand']
    drives = [r for r in results if r.get('scenario') == 'drive']
    return dict(passed=bool(stands and drives) and all(r['passed'] for r in stands+drives) and jump_ok
                and not any('error' in r for r in results),
                jump_passes=sum(r['passed'] for r in jumps), jump_runs=len(jumps),
                unique_jump_seeds=len(seeds), jump_criterion_passed=jump_ok, runs=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('paths', nargs='+', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--require-perception', action='store_true', help='fail if any input lacks passing gate evidence')
    args = parser.parse_args()
    paths = [p for path in args.paths for p in (sorted(path.glob('*.csv')) if path.is_dir() else [path])]
    result = aggregate([dict(path=str(p), **check(p)) for p in paths])
    if args.require_perception:
        result['passed'] &= bool(result['runs']) and all(r.get('gate_passed') is True for r in result['runs'])
    output = json.dumps(result, indent=2)
    print(output)
    if args.out:
        args.out.write_text(output+'\n')
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
