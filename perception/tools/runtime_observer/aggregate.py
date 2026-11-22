"""Offline observer CSV summaries. ClockMap is an optional explicit input."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path
import re
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.runtime_observer.stats import UNAVAILABLE, distribution

FIELDS = ("host_ns", "host_domain", "host_session", "frame_id", "event",
          "source_ns", "source_domain", "source_session", "reason", "depth")
LIFECYCLE = ("frame_received", "queued", "track_start", "track_end", "snapshot_published")
KINDS = (*LIFECYCLE, "dropped", "queue_depth")
STAGES = {"receive_to_queue_ns": ("frame_received", "queued"),
          "queue_wait_ns": ("queued", "track_start"),
          "track_duration_ns": ("track_start", "track_end"),
          "publish_delay_ns": ("track_end", "snapshot_published"),
          "receive_to_publish_ns": ("frame_received", "snapshot_published")}


def integer(text, *, minimum=None, maximum=None):
    if not isinstance(text, str) or not re.fullmatch(r"-?[0-9]+", text):
        raise ValueError(f"expected integer, got {text!r}")
    value = int(text)
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError("integer out of range")
    return value


def token(text):
    if not isinstance(text, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,47}", text):
        raise ValueError("expected a 1..47 character clock/session/reason token")
    return text


@dataclass(frozen=True)
class Event:
    host_ns: int
    host_session: str
    frame_id: int
    kind: str
    source: tuple[int, str, str] | None = None
    reason: str | None = None
    depth: int | None = None

    def __post_init__(self):
        if type(self.host_ns) is not int or not 0 <= self.host_ns < 2**63:
            raise ValueError("host_ns must be a nonnegative int64")
        token(self.host_session)
        if type(self.frame_id) is not int or not 0 <= self.frame_id < 2**64:
            raise ValueError("frame_id must be a uint64")
        if self.kind not in KINDS:
            raise ValueError("unknown event")
        if self.source is not None:
            ns, domain, session = self.source
            if type(ns) is not int or not -(2**63) <= ns < 2**63:
                raise ValueError("source epoch must be int64")
            token(domain)
            token(session)
        if self.kind == "dropped":
            token(self.reason)
        elif self.reason is not None:
            raise ValueError("reason is only valid on dropped")
        if self.kind == "queue_depth":
            if type(self.depth) is not int or not 0 <= self.depth < 2**63:
                raise ValueError("queue depth must be a nonnegative int64")
        elif self.depth is not None:
            raise ValueError("depth is only valid on queue_depth")


def read_csv(text):
    lines = text.splitlines()
    losses = None
    if lines and lines[0].startswith("# observer_loss "):
        match = re.fullmatch(r"# observer_loss overwritten=([0-9]+) contention=([0-9]+) "
                             r"invalid=([0-9]+) clock_errors=([0-9]+)", lines.pop(0))
        if match is None:
            raise ValueError("malformed observer loss metadata")
        losses = dict(zip(("overwritten", "contention", "invalid", "clock_errors"), map(int, match.groups())))
    reader = csv.DictReader(io.StringIO("\n".join(lines)))
    if reader.fieldnames != list(FIELDS):
        raise ValueError("observer CSV header does not match schema")
    events = []
    for row in reader:
        if None in row or any(value is None for value in row.values()):
            raise ValueError("malformed observer CSV row")
        if row["host_domain"] != "host_steady":
            raise ValueError("observer host domain must be host_steady")
        source_values = [row[key] for key in ("source_ns", "source_domain", "source_session")]
        if any(source_values) and not all(source_values):
            raise ValueError("partial source clock identity")
        source = (integer(source_values[0]), *source_values[1:]) if all(source_values) else None
        events.append(Event(integer(row["host_ns"]), row["host_session"],
                            integer(row["frame_id"]), row["event"], source,
                            row["reason"] or None,
                            integer(row["depth"]) if row["depth"] else None))
    return events, losses


def load_clockmap(path):
    """Reconstruct tools.clockmap.ClockMap from explicit four-stamp exchanges.

    JSON: domain_a, domain_b, session_a, session_b, exchanges_ns:[[t1,t2,t3,t4],...].
    Sessions use strings to match the CSV. This is never an arrival-time fit.
    """
    from tools.clockmap import ClockMap, Exchange, Stamp
    data = json.loads(Path(path).read_text())
    for field in ("domain_a", "domain_b", "session_a", "session_b"):
        token(data[field])
    mapping = ClockMap(data["domain_a"], data["domain_b"],
                       data["session_a"], data["session_b"])
    for row in data["exchanges_ns"]:
        if len(row) != 4 or any(type(value) is not int for value in row):
            raise ValueError("clockmap exchanges require four integer ns")
        mapping.add_exchange(Exchange(
            Stamp(row[0], data["domain_a"], data["session_a"]),
            Stamp(row[1], data["domain_b"], data["session_b"]),
            Stamp(row[2], data["domain_b"], data["session_b"]),
            Stamp(row[3], data["domain_a"], data["session_a"])))
    return mapping


def source_age(event, source, mapping):
    if source is None:
        return None, "no source epoch"
    ns, domain, session = source
    if domain == "host_steady":
        if session != event.host_session:
            return None, "host session mismatch"
        age = event.host_ns - ns
        return (None, "source epoch in future") if age < 0 else ((age, age, 0, None), None)
    if mapping is None:
        return None, "no clock map"
    from tools.clockmap import Stamp, source_age as mapped_age
    age = mapped_age(Stamp(event.host_ns, "host_steady", event.host_session),
                     Stamp(ns, domain, session), mapping)
    if age.status == UNAVAILABLE:
        return None, age.reason
    mapped = age.mapped_epoch
    return (event.host_ns - mapped.stamp.value_ns, age.upper_bound_ns,
            mapped.uncertainty_ns, mapped.mapping_revision), None


def queue_summary(points):
    result = distribution([depth for _, depth in points])
    if not points:
        return {**result, "slope_depth_per_s": UNAVAILABLE, "time_weighted_mean": UNAVAILABLE}
    result["sample_mean"] = sum(depth for _, depth in points) / len(points)
    # Center integer epochs BEFORE converting differences to seconds.
    xs = [(ns - points[0][0]) / 1e9 for ns, _ in points]
    ys = [depth for _, depth in points]
    mx, my = sum(xs) / len(xs), result["sample_mean"]
    denom = sum((x - mx)**2 for x in xs)
    result["slope_depth_per_s"] = (sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
                                    if denom else UNAVAILABLE)
    span = points[-1][0] - points[0][0]
    result["observed_span_ns"] = span
    result["time_weighted_mean"] = (sum(depth * (next_ns - ns) for (ns, depth), (next_ns, _)
                                        in zip(points, points[1:])) / span if span else UNAVAILABLE)
    return result


def summarize(events, mapping=None, losses=None):
    sessions = defaultdict(list)
    for event in events:
        sessions[event.host_session].append(event)
    reports = {}
    for session, rows in sessions.items():
        if any(b.host_ns < a.host_ns for a, b in zip(rows, rows[1:])):
            raise ValueError("host clock reversal/out-of-order events; input is never sorted")
        frames, queue, counts = defaultdict(dict), [], Counter()
        for event in rows:
            counts[event.kind] += 1
            if event.kind == "queue_depth":
                queue.append((event.host_ns, event.depth))
                continue
            frame = frames[event.frame_id]
            if event.kind in frame:
                raise ValueError("duplicate lifecycle event/frame ID")
            frame[event.kind] = event
        stage_values, missing = defaultdict(list), Counter()
        age_values, upper_values, uncertainty_values = defaultdict(list), defaultdict(list), defaultdict(list)
        host_ages, missing_host_ages = defaultdict(list), Counter()
        unavailable, revisions, drops = defaultdict(Counter), set(), Counter()
        incomplete_frames = 0
        for frame in frames.values():
            ordered = [frame[kind].host_ns for kind in LIFECYCLE if kind in frame]
            if ordered != sorted(ordered):
                raise ValueError("frame stage order is inconsistent")
            if "dropped" in frame:
                if "snapshot_published" in frame or frame["dropped"].host_ns < max(ordered, default=0):
                    raise ValueError("drop conflicts with frame lifecycle")
                drops[frame["dropped"].reason] += 1
            elif "snapshot_published" not in frame:
                incomplete_frames += 1
            sources = {event.source for event in frame.values() if event.source is not None}
            if len(sources) > 1:
                raise ValueError("source identity/epoch changed within a frame")
            source = next(iter(sources), None)
            for stage, (start, end) in STAGES.items():
                if start in frame and end in frame:
                    stage_values[stage].append(frame[end].host_ns - frame[start].host_ns)
                else:
                    missing[stage] += 1
            for kind in LIFECYCLE:
                if kind not in frame:
                    continue
                if "frame_received" in frame:
                    host_ages[kind].append(frame[kind].host_ns - frame["frame_received"].host_ns)
                else:
                    missing_host_ages[kind] += 1
                age, reason = source_age(frame[kind], source, mapping)
                if age is None:
                    unavailable[kind][reason] += 1
                else:
                    point, upper, uncertainty, revision = age
                    age_values[kind].append(point)
                    upper_values[kind].append(upper)
                    uncertainty_values[kind].append(uncertainty)
                    if revision is not None:
                        revisions.add(revision)
        span = rows[-1].host_ns - rows[0].host_ns
        rates = {kind: counts[kind] * 1e9 / span if span else UNAVAILABLE
                 for kind in ("frame_received", "track_end", "snapshot_published", "dropped")}
        reports[session] = {
            "host_domain": "host_steady", "observed_span_ns": span,
            "event_counts": dict(counts), "frame_count": len(frames),
            "frames_without_terminal_event": incomplete_frames,
            "stage_durations_ns": {stage: {**distribution(stage_values[stage]),
                                          "missing_pairs": missing[stage]} for stage in STAGES},
            "host_ages_since_received_ns": {kind: {**distribution(host_ages[kind]),
                                                   "missing_received": missing_host_ages[kind]} for kind in LIFECYCLE},
            "source_ages_ns": {kind: {"point": distribution(age_values[kind]),
                                     "upper_bound": distribution(upper_values[kind]),
                                     "uncertainty": distribution(uncertainty_values[kind]),
                                     "unavailable_reasons": dict(unavailable[kind])} for kind in LIFECYCLE},
            "mapping_revisions": sorted(revisions), "queue_depth": queue_summary(queue),
            "drops_by_reason": dict(drops), "throughput_per_s": rates,
        }
    lost = sum(losses.values()) if losses is not None else None
    return {"evidence": "MEASURED input diagnostics; physical timing qualification UNKNOWN",
            "quantiles": "nearest rank", "observer_loss": losses if losses is not None else UNAVAILABLE,
            "trace_completeness": "COMPLETE" if lost == 0 else "INCOMPLETE" if lost else "UNKNOWN",
            "throughput_denominator": "each host session's first-to-last retained event span",
            "sessions": reports}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    parser.add_argument("--clockmap", type=Path)
    args = parser.parse_args(argv)
    try:
        raw = args.events.read_bytes()
        events, losses = read_csv(raw.decode("utf-8"))
        mapping = load_clockmap(args.clockmap) if args.clockmap else None
        report = summarize(events, mapping, losses)
        report["input"] = {"path": str(args.events), "sha256": hashlib.sha256(raw).hexdigest()}
        if args.clockmap:
            report["clockmap_input"] = {"path": str(args.clockmap),
                                       "sha256": hashlib.sha256(args.clockmap.read_bytes()).hexdigest()}
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError, KeyError, ImportError) as exc:
        print(f"observer: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
