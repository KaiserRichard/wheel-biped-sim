"""Command arbitration, slew limiting, freshness and PS2-style hold adapter.

Call receive/tick from one simulation thread. Times are receiver-monotonic seconds
(simulation time for scripted runs). Numerical policy is ASSUMED unless marked.
"""

from dataclasses import dataclass
import math

MAX_SPEED = 0.6  # ASSUMED m/s, symmetric command clamp.
MAX_ACCEL = 0.6  # ASSUMED m/s^2, applies to acceleration and watchdog braking.
WATCHDOG_S = 0.5  # ASSUMED seconds since last accepted command.
HOLD_POLLS = 10  # SOURCE-SUPPORTED: fire only after >10 held R1 polls.
POLL_S = 0.01  # SOURCE-SUPPORTED nominal PS2 polling, not a guaranteed clock.


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('expected finite number')
    return float(value)


@dataclass(frozen=True)
class Command:
    v_ref: float = 0.0  # m/s; receive clamps to MAX_SPEED.
    yaw_rate: float = 0.0  # rad/s; retained for logging, ignored by planar phase-1.
    jump: bool = False  # A one-shot request edge, never a held key level.
    src: str = 'script'

    def __post_init__(self):
        object.__setattr__(self, 'v_ref', max(-MAX_SPEED, min(MAX_SPEED, finite(self.v_ref))))
        object.__setattr__(self, 'yaw_rate', finite(self.yaw_rate))
        if type(self.jump) is not bool:
            raise ValueError('jump must be boolean')
        if not isinstance(self.src, str) or not 1 <= len(self.src) <= 64:
            raise ValueError('src must contain 1..64 characters (ASSUMED)')


def rate_limit(current, target, dt, acceleration=MAX_ACCEL):
    """CALCULATED bounded change, |delta v| <= acceleration * dt."""
    current, target, dt, acceleration = map(finite, (current, target, dt, acceleration))
    if dt < 0 or acceleration <= 0:
        raise ValueError('dt must be nonnegative and acceleration positive')
    return current + max(-acceleration*dt, min(acceleration*dt, target-current))


class HoldDebounce:
    """SOURCE-SUPPORTED >N polls; ASSUMED require release after each request.

    Poll once per real input processing pass, not once per physics tick. A hold
    that qualifies during lockout is consumed, so it cannot trigger on unlock.
    """
    def __init__(self, polls=HOLD_POLLS):
        if type(polls) is not int or polls < 0:
            raise ValueError('polls must be a nonnegative integer')
        self.threshold = polls
        self.count = 0
        self.fired = False

    def poll(self, held, active=False):
        if not held:
            self.count, self.fired = 0, False
            return False
        self.count = min(self.count+1, self.threshold+1)
        if self.count > self.threshold and not self.fired:
            self.fired = True
            return not active
        return False


@dataclass(frozen=True)
class Output:
    v_ref: float
    target_v: float
    yaw_rate: float
    jump: bool
    src: str
    watchdog: bool


class CommandModel:
    """Last accepted writer wins velocity/yaw; jumps are never queued in flight.

    Velocity/yaw use last-writer-wins; a request edge is consumed once at tick.
    The phase-1 caller supplies active=(phase != 'STAND'), including latched SETTLE.
    """
    def __init__(self, watchdog_s=WATCHDOG_S, acceleration=MAX_ACCEL):
        self.watchdog_s, self.acceleration = map(finite, (watchdog_s, acceleration))
        if self.watchdog_s <= 0 or self.acceleration <= 0:
            raise ValueError('watchdog and acceleration must be positive')
        self.command = Command(src='none')
        self.last_received = None
        self.last_tick = None
        self.v_ref = 0.0
        self.pending_jump = False
        self.pending_at = None
        self.ignored_jumps = 0

    def receive(self, command, now, active=False):
        now = finite(now)
        if self.last_tick is not None and now < self.last_tick:
            raise ValueError('receive time predates last tick')
        if self.last_received is not None and now < self.last_received:
            raise ValueError('receive clock moved backwards')
        self.command, self.last_received = command, now
        self.ignored_jumps += int(command.jump and (active or self.pending_jump))
        if command.jump and not active and not self.pending_jump:
            self.pending_jump, self.pending_at = True, now

    def tick(self, now, active=False):
        now = finite(now)
        if self.last_tick is not None and now < self.last_tick:
            raise ValueError('tick clock moved backwards')
        if self.last_received is not None and now < self.last_received:
            raise ValueError('tick predates received command')
        dt = 0.0 if self.last_tick is None else now-self.last_tick
        self.last_tick = now
        stale = self.last_received is None or now >= self.last_received+self.watchdog_s
        target = 0.0 if stale else self.command.v_ref
        self.v_ref = rate_limit(self.v_ref, target, dt, self.acceleration)
        trigger = (self.pending_jump and not active and not stale
                   and now < self.pending_at+self.watchdog_s)
        self.pending_jump = False
        return Output(self.v_ref, target, self.command.yaw_rate, trigger,
                      self.command.src, stale)
