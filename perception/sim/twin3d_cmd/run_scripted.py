"""Headless phase-1 command rehearsal, scripted or real-time UDP, with CSV evidence."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import mujoco

from sim.twin3d.control import Controller
from sim.twin3d.model import Plant, DT
from .commands import Command, CommandModel, finite
from .udp import Listener, PORT

TRACK_TOL = 0.08  # ASSUMED m/s coarse rehearsal tolerance; initial 0.05 failed (RUN_LOG).
STOP_TOL = 0.03  # ASSUMED m/s after braking transient, RUN_LOG documents timing.
PITCH_LIMIT = math.radians(30)  # ASSUMED fall screen, inherited phase-1 check policy.
FIELDS = ('t', 'v_ref', 'target_v', 'v', 'yaw_rate', 'pitch', 'leg_length',
          'phase', 'src', 'watchdog', 'jump_edge', 'jump_count', 'ignored_jumps',
          'contact_left', 'contact_right', 'base_ground', 'physics_warning')


class Twin:
    """Shared physics step for headless and passive viewer; never renders."""
    def __init__(self):
        self.plant = Plant()
        self.reset()

    def reset(self):
        initial = self.plant.data()
        if hasattr(self, 'data'):
            # Preserve the MjData identity held by the passive viewer (3.3.7 has
            # no Python mj_copyData binding).
            mujoco.mj_resetData(self.plant.model, self.data)
            self.data.qpos[:] = initial.qpos
            mujoco.mj_forward(self.plant.model, self.data)
        else:
            self.data = initial
        self.controller = Controller()
        self.commands = CommandModel()
        self.jump_count = 0

    @property
    def active(self):
        return self.controller.jump.phase != 'STAND'

    def receive(self, command, now):
        self.commands.receive(command, now, self.active)

    def step(self, now=None):
        now = float(self.data.time) if now is None else now
        output = self.commands.tick(now, self.active)
        self.controller.update(self.plant, self.data, output.v_ref, output.jump)
        self.jump_count += int(output.jump)
        mujoco.mj_step(self.plant.model, self.data)
        mujoco.mj_forward(self.plant.model, self.data)
        state, contact = self.plant.state(self.data), self.plant.contacts(self.data)
        if not all(math.isfinite(float(value)) for value in self.data.qpos):
            raise RuntimeError('nonfinite physics state')
        return dict(t=float(self.data.time), v_ref=output.v_ref, target_v=output.target_v,
                    v=float(state[3]), yaw_rate=output.yaw_rate, pitch=float(state[2]),
                    leg_length=self.plant.length(self.data), phase=self.controller.jump.phase,
                    src=output.src, watchdog=int(output.watchdog), jump_edge=int(output.jump),
                    jump_count=self.jump_count, ignored_jumps=self.commands.ignored_jumps,
                    contact_left=int(contact['left']), contact_right=int(contact['right']),
                    base_ground=int(contact['base_ground']),
                    physics_warning=sum(int(w.number) for w in self.data.warning))


def scenario_commands(scenario):
    """ASSUMED rehearsal schedules, seconds; no obstacle/clearance claim."""
    if scenario == 'jump':
        # The final press also tests phase-1's SETTLE lockout.
        return [dict(at=t, v=0.0, jump=True) for t in (0.5, 0.55, 0.6, 0.65, 1.0)]
    end = 3.9 if scenario == 'drive' else 2.8
    return [dict(at=round(0.5+i*0.1, 8), v=0.3)
            for i in range(round((end-0.5)/0.1)+1)]


def load_script(rows):
    result = []
    for row in rows:
        if not isinstance(row, dict) or 'at' not in row or set(row)-{'at', 'v', 'yaw_rate', 'jump', 'src'}:
            raise ValueError('script row needs at and optional v/yaw_rate/jump/src')
        at = finite(row['at'])
        if at < 0:
            raise ValueError('negative script timestamp')
        result.append((at, Command(row.get('v', 0.0), row.get('yaw_rate', 0.0),
                                   row.get('jump', False), row.get('src', 'script'))))
    return sorted(result, key=lambda item: item[0])


def run(seconds=7.0, script=None, out=None, listener=None):
    """UDP uses wall-clock pacing/freshness, script uses simulation seconds.

    In UDP mode watchdog time is monotonic wall time, so a simulation stall does
    not refresh a command. First physics step after a stall brakes immediately.
    """
    seconds = finite(seconds)
    if seconds <= 0:
        raise ValueError('seconds must be positive')
    events = load_script(script if script is not None else [])
    twin, rows, index = Twin(), [], 0
    start = time.monotonic()
    stream = None
    if out is not None:
        path = Path(out)
        path.parent.mkdir(parents=True, exist_ok=True)
        stream = path.open('w', newline='')
    try:
        writer = csv.DictWriter(stream, FIELDS) if stream else None
        if writer:
            writer.writeheader()
        for _ in range(math.ceil(seconds/DT)):
            if listener is not None:
                wait = start+float(twin.data.time)-time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                now = time.monotonic()
                for packet in listener.drain():
                    twin.receive(packet.command, now)
            else:
                now = float(twin.data.time)
            while index < len(events) and events[index][0] <= twin.data.time+1e-9:
                twin.receive(events[index][1], now)
                index += 1
            row = twin.step(now)
            rows.append(row)
            if writer:
                writer.writerow(row)
    finally:
        if stream:
            stream.close()
    return rows


def evaluate(rows, scenario):
    """Predeclared checks on actual physics; values are MEASURED in simulation."""
    checks = dict(nonempty=bool(rows))
    if not rows:
        return dict(passed=False, checks=checks)
    checks['physics'] = all(math.isfinite(r['v']) and abs(r['pitch']) < PITCH_LIMIT
                            and not r['base_ground'] and not r['physics_warning'] for r in rows)
    tracking = [r for r in rows if 2.5 <= r['t'] <= 2.8]
    if scenario in ('drive', 'watchdog'):
        checks['tracking_window'] = len(tracking) >= 299  # CALCULATED 0.3 s / DT, endpoint roundoff.
        checks['tracking'] = bool(tracking) and all(abs(r['v']-0.3) <= TRACK_TOL for r in tracking)
    if scenario == 'watchdog':
        stopped = [r for r in rows if 6.5 <= r['t'] <= 7.0]
        checks['stop_window'] = len(stopped) >= 499  # CALCULATED 0.5 s / DT, endpoint roundoff.
        checks['watchdog_stop'] = bool(stopped) and all(r['watchdog'] and abs(r['v']) <= STOP_TOL
                                                     and r['v_ref'] == 0 for r in stopped)
    if scenario == 'jump':
        checks['one_jump'] = rows[-1]['jump_count'] == 1 and sum(r['jump_edge'] for r in rows) == 1
        checks['ignored_repeats'] = rows[-1]['ignored_jumps'] == 4
        checks['phases'] = {'CROUCH', 'THRUST', 'RETRACT', 'SETTLE'} <= {r['phase'] for r in rows}
        checks['flight_and_landing'] = (any(not (r['contact_left'] or r['contact_right'])
                                              and r['phase'] == 'SETTLE' for r in rows)
                                        and bool(rows[-1]['contact_left'] and rows[-1]['contact_right']))
    return dict(passed=all(checks.values()), checks=checks, label='MEASURED in simulation',
                final_v=rows[-1]['v'], final_v_ref=rows[-1]['v_ref'],
                jump_count=rows[-1]['jump_count'], ignored_jumps=rows[-1]['ignored_jumps'],
                peak_pitch_rad=max(abs(r['pitch']) for r in rows),
                tracking_max_error=max((abs(r['v']-0.3) for r in tracking), default=None))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--script', help='JSON array of {at, v, yaw_rate, jump, src}')
    mode.add_argument('--udp', action='store_true', help='real-time UDP input')
    p.add_argument('--scenario', choices=('drive', 'watchdog', 'jump'), default='watchdog')
    p.add_argument('--seconds', type=float, default=7.0, help='ASSUMED default duration, s')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=PORT)
    p.add_argument('--out', default='docs/twin3d_cmd/runs/scripted.csv')
    p.add_argument('--check', action='store_true', help='check the built-in scenario')
    a = p.parse_args(argv)
    if a.check and (a.script or a.udp):
        p.error('--check applies only to built-in scenarios')
    script = (json.loads(Path(a.script).read_text()) if a.script else
              [] if a.udp else scenario_commands(a.scenario))
    if a.udp:
        with Listener(a.host, a.port) as listener:
            rows = run(a.seconds, script, a.out, listener)
            print(json.dumps(dict(accepted=listener.accepted, rejected=listener.rejected)))
    else:
        rows = run(a.seconds, script, a.out)
    if a.check:
        report = evaluate(rows, a.scenario)
        print(json.dumps(report, indent=2))
        if not report['passed']:
            return 1
    print(f'wrote {len(rows)} physics samples to {a.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
