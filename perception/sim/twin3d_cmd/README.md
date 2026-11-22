# C1-phase-16: teleop viewer controls and repeated jumps

This README describes the current teleop viewer and supersedes the viewer key,
camera and landing instructions in `../../docs/twin3d_cmd/README.md`. That shared
document is outside this task's explicit `sim/twin3d_cmd/` edit boundary.
The scripted runner retains its existing latched `SETTLE` behavior.

Run on a graphics host (macOS requires `mjpython`):

```sh
./.venv/bin/mjpython -m sim.twin3d_cmd.teleop_viewer
```

| Key | Action |
| --- | --- |
| Up / Down | Increase / decrease velocity by 0.05 m/s per press, clamped to ±0.6 m/s |
| Enter / Return | Request one jump; requests during a jump or lockout are consumed |
| Home | Reset physics, controller, commands, jump counts and failure lockout |
| End | Quit; closing the window also quits |

On compact Mac keyboards, Home and End are normally Fn-Left and Fn-Right.
Keyboard velocity remains latched with a 0.1 s heartbeat while it owns the
command; a new UDP command takes ownership until the next local command key.
Reset preserves the `MjData` object held by the viewer and UDP sequence history.
The initial camera distance is 0.8 m, azimuth 90°, elevation −15°; its look-at
point follows the base every simulation tick. These are assumed view settings.

## Checked key bindings

The installed environment reported MuJoCo **3.3.7**, using:

```sh
./.venv/bin/python -c \
  'import inspect, mujoco, mujoco.viewer; print(mujoco.__version__); print(inspect.getfile(mujoco.viewer))'
```

The wheel contains Python `viewer.py` and a compiled `_simulate` extension,
so the matching upstream 3.3.7 C++ sources were inspected:

- [Python callback adapter](https://github.com/google-deepmind/mujoco/blob/3.3.7/python/mujoco/simulate.cc): built-in processing precedes the Python callback; it cannot consume the event first.
- [Simulate UI and event switch](https://github.com/google-deepmind/mujoco/blob/3.3.7/simulate/simulate.cc) and [UI definitions](https://github.com/google-deepmind/mujoco/blob/3.3.7/simulate/simulate.h): Backspace resets; Escape changes camera; Space is reserved for play/pause (guarded out in passive mode in this version).
- [Visual/render flag shortcuts](https://github.com/google-deepmind/mujoco/blob/3.3.7/src/engine/engine_vis_init.c): W = wireframe, S = shadows, R = reflection, J = joint visualization, even when panels are hidden.
- [GLFW adapter](https://github.com/google-deepmind/mujoco/blob/3.3.7/simulate/glfw_adapter.cc): key codes pass through unchanged; 3.3.7 accepts press events, not OS repeat events.

Up (265), Down (264), Enter (257), Home (268) and End (269) have no viewer
shortcuts in these definitions. W/S/Space/R/J/Backspace/Escape are ignored by
the robot's handler; they can still perform MuJoCo actions. Controls assume
the default hidden panels, without an active viewer text-edit field. Recheck
these sources when upgrading MuJoCo.

## Landing and failure behavior

`TeleopTwin` applies a viewer-only landing policy after each actual physics step.
In `SETTLE`, both wheels must contact the ground continuously for **0.5 simulation
seconds**, with pitch below 5°, pitch rate below 0.2 rad/s, vertical base speed
below 0.05 m/s, and both leg lengths within 0.01 m of standing length with rates
below 0.05 m/s. These are **ASSUMED simulation thresholds**, not hardware limits.
Any interruption restarts the timer; wall-clock stalls cannot complete it.
Forward rolling is allowed. Once stable, a fresh jump state returns to `STAND`
without resetting physics, balance references, command ownership or jump counts.
The next Enter starts another cycle; ignored earlier requests never replay.

Base-floor contact, pitch reaching 30° or any physics warning latches a failure
lockout until Home resets it. The overlay shows readiness or `LOCKED: Home to
reset`. Recovery of the physical pose alone cannot clear the lockout.

The macOS line `error messaging the mach port for IMKCFRunLoopWakeUpReliable`
is a harmless input-method diagnostic reported by the owner; ignore it.

## Headless validation

```sh
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  ./.venv/bin/python \
  tools/test_support/run_unittest.py sim/twin3d_cmd
```

`tests/test_teleop_viewer.py` drives callbacks and actual MuJoCo physics for
velocity up/down, two full jumps and landings without reset, consumed repeat
requests, interrupted stability, simulation-time dwell, fall lockout/reset,
ignored built-in keys and overlay help. No viewer window or renderer is launched.
An initial 8 s headless run completed two jumps, re-arming at 2.221 s and 3.942 s,
with no base-floor contact or physics warnings. GUI key delivery and camera
appearance remain unverified in this environment. No commit was made.

Validation on 2026-10-30: the four new teleop tests passed; full discovery found
24 tests in four files, with **21 passing and three localhost UDP tests skipped**
because sandbox socket binding returned `Operation not permitted`. Markdown
lint reported zero issues. This task changed only `teleop_viewer.py`, its
keyboard tests and this local README; no files outside `sim/twin3d_cmd/` changed.
