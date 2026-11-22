"""Recorded advisories plus explicitly SYNTHETIC phase/contact/quality inputs.

No MuJoCo import or renderer is used by timeline construction or CSV export.
"""

from bisect import bisect_right
import csv
from dataclasses import dataclass
import math
from pathlib import Path

from tools.obstacle_decision.decide import Advisory
from tools.scene_twin.replay import Replay, bag_start, read_packets
from .gate import ASSUMED_TIME_EPSILON_S, GateOutput, JumpGate, Tick


DEFAULT_PACKETS = Path.home() / "wbr-evidence-archive/lab-day-20261025/3-box13/packets.csv"
ASSUMED_TICK_HZ = 100
ASSUMED_RAW_CONFIDENCE = 1.0  # packets.csv does NOT contain confidence.
ASSUMED_TELEMETRY_AGE_S = 0.0
ASSUMED_RATE_DEG_S = 0.0
ASSUMED_PLANE_VALID = True  # packets.csv does NOT contain plane_valid.
ASSUMED_DEFAULT_JUMP_AT_S = 20.0


@dataclass(frozen=True)
class SyntheticPose:
    phase: str
    leg_length_m: float
    vertical_offset_m: float
    jump_status: int
    airborne_l: bool
    airborne_r: bool


def synthetic_pose(t, jump_at):
    """Reuse wbr_pose exactly; --jump-at denotes CROUCH onset, not leading STAND.

    LAND has a synthetic first-foot contact and SETTLE has two-foot contact.
    This illustrates phase mapping; it does not simulate estimated normal force.
    """
    from tools.wbr_pose.jump_demo import STAND_S, state_at
    local_t = 0.0 if t < jump_at else STAND_S + t - jump_at + ASSUMED_TIME_EPSILON_S
    phase, length, z = state_at(local_t)
    status, left, right = {
        "STAND": (3, False, False),
        "CROUCH": (0, False, False),
        "PUSH": (1, False, False),
        "FLIGHT": (2, True, True),
        "LAND": (2, False, True),
        "SETTLE": (3, False, False),
    }[phase]
    return SyntheticPose(phase, length, z, status, left, right)


@dataclass(frozen=True)
class TimelineRow:
    packet_index: int
    frame_stamp_ns: int
    inputs: Tick
    output: GateOutput
    pose: SyntheticPose


class Timeline:
    def __init__(self, replay, rows, jump_at):
        self.replay, self.rows, self.jump_at = replay, rows, jump_at
        self.times = [row.inputs.t for row in rows]

    def at(self, t):
        return self.rows[max(0, bisect_right(self.times, t) - 1)]


def build_timeline(replay, jump_at=ASSUMED_DEFAULT_JUMP_AT_S):
    if not math.isfinite(jump_at) or not 0 <= jump_at < replay.duration:
        raise ValueError("--jump-at must be finite, nonnegative, and before replay end")
    # Merge source acquisition events with synthetic telemetry ticks. Repeated
    # display samples retain their original plane stamp and never add plane votes.
    packet_times = [(p.stamp - replay.stamps[0]) / 1e9 for p in replay.packets]
    times = set(packet_times)
    times.update(k / ASSUMED_TICK_HZ for k in range(int(replay.duration * ASSUMED_TICK_HZ) + 1))
    gate, rows = JumpGate(), []
    for t in sorted(times):
        i = bisect_right(packet_times, t) - 1
        p = replay.packets[i]
        pose = synthetic_pose(t, jump_at)
        inputs = Tick(t, pose.jump_status, pose.airborne_l, pose.airborne_r,
                      ASSUMED_TELEMETRY_AGE_S, ASSUMED_RATE_DEG_S, ASSUMED_RATE_DEG_S,
                      ASSUMED_PLANE_VALID, Advisory(p.advisory), ASSUMED_RAW_CONFIDENCE,
                      plane_stamp_s=packet_times[i])
        rows.append(TimelineRow(i, p.stamp, inputs, gate.update(inputs), pose))
    return Timeline(replay, rows, jump_at)


def load_timeline(path=DEFAULT_PACKETS, jump_at=ASSUMED_DEFAULT_JUMP_AT_S):
    path = Path(path).expanduser()
    # 3-box13 is a stationary run: detector jitter must not drive the robot.
    replay = Replay(read_packets(path), bag_start(path.parent), motion="static")
    return build_timeline(replay, jump_at)


CSV_FIELDS = (
    "t_s", "frame_stamp_ns", "packet_index", "raw_advisory", "raw_confidence_assumed",
    "gated_advisory", "gated_confidence", "gate_phase", "reason", "synthetic_phase",
    "jump_status_synthetic", "airborne_l_synthetic", "airborne_r_synthetic",
    "telemetry_age_s_synthetic", "pitch_rate_deg_s_synthetic", "roll_rate_deg_s_synthetic",
    "plane_valid_assumed", "plane_stamp_s", "observation_age_s", "leg_length_m_synthetic",
    "vertical_offset_m_synthetic", "phase_source",
)


def export_csv(timeline, path):
    """One row per gate tick, not per rendered frame; preserve integer source ns."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        for row in timeline.rows:
            tick, out, pose = row.inputs, row.output, row.pose
            writer.writerow((
                tick.t, row.frame_stamp_ns, row.packet_index, Advisory(tick.raw_advisory).name,
                tick.raw_confidence, out.advisory.name, out.confidence, out.phase.value,
                out.reason, pose.phase, tick.jump_status, int(tick.airborne_l), int(tick.airborne_r),
                tick.telemetry_age_s, tick.pitch_rate_deg_s, tick.roll_rate_deg_s,
                int(tick.plane_valid), tick.plane_stamp_s, tick.t - tick.plane_stamp_s,
                pose.leg_length_m, pose.vertical_offset_m, "SYNTHETIC",
            ))
    return path
