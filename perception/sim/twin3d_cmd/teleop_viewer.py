"""Run with mjpython -m sim.twin3d_cmd.teleop_viewer (graphics host only)."""

import argparse
import math
from queue import Empty, SimpleQueue
import time

from .commands import Command, MAX_SPEED
from .run_scripted import Twin, PITCH_LIMIT
from .udp import Listener, PORT

KEY_STEP = 0.05  # ASSUMED m/s per key press.
HEARTBEAT_S = 0.1  # ASSUMED local keyboard heartbeat, seconds.
DRAW_S = 1/60  # ASSUMED maximum viewer sync frequency, Hz=60.
# GLFW codes; checked against MuJoCo 3.3.7 simulate UI and visual flag
# shortcuts (see README.md). J, Backspace and Escape are already bound there.
KEY_UP, KEY_DOWN, KEY_JUMP, KEY_RESET, KEY_QUIT = 265, 264, 257, 268, 269
KEY_HELP = 'Up/Down: velocity | Enter: jump\nHome: reset + unlock | End: quit'
SETTLE_STABLE_S = 0.5  # ASSUMED continuous simulation seconds, never wall time.


class TeleopTwin(Twin):
    """Viewer landing re-arm policy; leave the scripted phase-1 baseline unchanged."""

    def reset(self):
        super().reset()
        self.jump_locked = False
        self.stable_since = None

    @property
    def active(self):
        return self.jump_locked or super().active

    def step(self, now=None):
        row = super().step(now)
        self.jump_locked |= bool(row['base_ground'] or row['physics_warning']
                                 or abs(row['pitch']) >= PITCH_LIMIT)
        q, data = self.plant.q, self.data
        # ASSUMED landing envelope: both wheels down, upright and quiet in
        # pitch, vertical motion and each leg. Forward rolling is allowed.
        stable = (not self.jump_locked and row['phase'] == 'SETTLE'
                  and row['contact_left'] and row['contact_right']
                  and abs(row['pitch']) < math.radians(5)
                  and abs(data.qvel[q['pitch']]) < 0.2
                  and abs(data.qvel[q['base_z']]) < 0.05
                  and all(abs(data.qpos[q['length_'+s]]-self.controller.jump.stand) < 0.01
                          and abs(data.qvel[q['length_'+s]]) < 0.05
                          for s in ('left', 'right')))
        if not stable:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = row['t']
        elif row['t']-self.stable_since >= SETTLE_STABLE_S:
            jump = self.controller.jump
            # Fresh per-jump counters/airborne flag; retain balance references,
            # command ownership, physics time and cumulative jump count.
            self.controller.jump = type(jump)(stand=jump.stand)
            self.stable_since = None
            row['phase'] = 'STAND'
        row['jump_locked'] = self.jump_locked
        return row


class Keyboard:
    """Callbacks enqueue events only; simulation thread owns all physics state."""
    def __init__(self):
        self.events = SimpleQueue()
        self.v = 0.0
        self.last_heartbeat = float('-inf')
        self.owned_command = None

    def callback(self, keycode):
        self.events.put(keycode)

    def poll(self, twin, now):
        quit_requested = False
        while True:
            try:
                key = self.events.get_nowait()
            except Empty:
                break
            # SOURCE-SUPPORTED GLFW codes used by MuJoCo's passive key callback.
            if key == KEY_QUIT:
                quit_requested = True
            elif key == KEY_RESET:
                twin.reset()
                self.v = 0.0
                self.last_heartbeat = float('-inf')
                self.owned_command = None
            elif key in (KEY_UP, KEY_DOWN, KEY_JUMP):
                # Adopt latest remote target before applying a local increment.
                self.v = twin.commands.command.v_ref
                if key != KEY_JUMP:
                    self.v += KEY_STEP if key == KEY_UP else -KEY_STEP
                self.v = max(-MAX_SPEED, min(MAX_SPEED, self.v))
                self.owned_command = Command(self.v, 0.0, key == KEY_JUMP, 'keyboard')
                twin.receive(self.owned_command, now)
                self.last_heartbeat = now
        # Keep an intentional keyboard setpoint alive, but never reclaim control
        # from UDP without another key press. UDP loss therefore trips watchdog.
        if twin.commands.command is self.owned_command and now-self.last_heartbeat >= HEARTBEAT_S:
            self.owned_command = Command(self.v, src='keyboard')
            twin.receive(self.owned_command, now)
            self.last_heartbeat = now
        return quit_requested


def overlay(row, listener):
    left = ('ASSUMED simulation; planar phase-1\nv_ref / target [m/s]\nmeasured v [m/s]\n'
            'pitch [deg]\nleg length [m]\njump phase\njump readiness\ncommand source\nwatchdog\n'
            'yaw [rad/s] IGNORED\nUDP accepted / rejected\n'
            f'{KEY_HELP}\nLanding: auto re-arm after 0.5 s stable')
    readiness = ('LOCKED: Home to reset' if row.get('jump_locked', False) else
                 'ready' if row['phase'] == 'STAND' else 'wait for stable landing')
    right = (f"\n{row['v_ref']:+.2f} / {row['target_v']:+.2f}\n{row['v']:+.3f}\n"
             f"{math.degrees(row['pitch']):+.2f}\n{row['leg_length']:.3f}\n{row['phase']}\n"
             f"{readiness}\n{row['src']}\n{'STALE: ramp to zero' if row['watchdog'] else 'fresh'}\n"
             f"{row['yaw_rate']:+.2f}\n{listener.accepted} / {listener.rejected}")
    return left, right


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='localhost', help='use localhost for Pi UDP')
    p.add_argument('--port', type=int, default=PORT)
    a = p.parse_args(argv)
    # Lazy import: headless tests can exercise keyboard/overlay with no viewer.
    import mujoco
    import mujoco.viewer

    twin, keyboard = TeleopTwin(), Keyboard()
    with Listener(a.host, a.port) as listener:
        with mujoco.viewer.launch_passive(twin.plant.model, twin.data,
                                         key_callback=keyboard.callback,
                                         show_left_ui=False, show_right_ui=False) as viewer:
            viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 90, -15, 0.8  # ASSUMED view.
            last_draw = float('-inf')
            next_step = time.monotonic()
            while viewer.is_running():
                now = time.monotonic()
                if next_step > now:
                    time.sleep(next_step-now)
                    now = time.monotonic()
                with viewer.lock():
                    for packet in listener.drain():
                        twin.receive(packet.command, now)
                    # Stable precedence on one tick: drain UDP, then local keys.
                    quit_requested = keyboard.poll(twin, now)
                    row = twin.step(now)
                    viewer.cam.lookat[:] = twin.data.xpos[twin.plant.model.body('base').id]
                if quit_requested:
                    break
                if now-last_draw >= DRAW_S:
                    left, right = overlay(row, listener)
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right))
                    viewer.sync()
                    last_draw = now
                # Wall-clock overrun slows simulation instead of an unbounded
                # catch-up loop; watchdog still uses wall time during overruns.
                next_step = max(next_step+twin.plant.model.opt.timestep, time.monotonic())


if __name__ == '__main__':
    main()
