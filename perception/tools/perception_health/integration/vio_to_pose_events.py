#!/usr/bin/env python3
"""Convert VIO snapshot JSONL to phase-7 sender JSONL, preserving unknown health."""
from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT))
from tools.perception_health.py.perception_health import TRACKING_STATES

CONFIG = json.loads((HERE / "config.json").read_text())
COLLAPSED = {
    "STARTING": ("SYSTEM_NOT_READY", {"SYSTEM_NOT_READY", "NO_IMAGES_YET"}),
    "INITIALIZING": ("NOT_INITIALIZED", {"NOT_INITIALIZED"}),
    "TRACKING": ("OK", {"OK", "OK_KLT"}),
    "LOST": ("LOST", {"RECENTLY_LOST", "LOST"}),
    "FAULT": ("SYSTEM_NOT_READY", set()),
}


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def load_mapping(path):
    """Validate explicit exchanges using the existing ClockMap, never a new fit."""
    if path is None:
        return None
    mapping = json.loads(Path(path).read_text())
    if not isinstance(mapping, dict):
        raise ValueError("clockmap input must be an object")
    if mapping.get("valid") is not True:
        raise ValueError("clockmap must explicitly declare valid=true")
    for key in ("snapshot_domain", "camera_domain", "pi_domain", "camera_session", "pi_session"):
        if not isinstance(mapping.get(key), str) or not mapping[key]:
            raise ValueError(f"clockmap requires nonempty {key}")
    if mapping["pi_domain"] != "CLOCK_MONOTONIC_RAW":
        raise ValueError("phase-7 samples Linux CLOCK_MONOTONIC_RAW; a steady/realtime label is insufficient")
    from tools.clockmap.clockmap import ClockMap, Exchange, Stamp
    opts = {k: v for k, v in CONFIG["clockmap"].items() if k != "source"}
    cmap = ClockMap(mapping["camera_domain"], mapping["pi_domain"],
                    mapping["camera_session"], mapping["pi_session"], **opts)
    for row in mapping["exchanges_ns"]:
        if not isinstance(row, list) or len(row) != 4:
            raise ValueError("each exchange needs [t1_camera,t2_pi_raw,t3_pi_raw,t4_camera]")
        for value in row:
            integer(value, "exchange ns")
        a, b, c, d = row
        cmap.add_exchange(Exchange(Stamp(a, mapping["camera_domain"], mapping["camera_session"]),
                                  Stamp(b, mapping["pi_domain"], mapping["pi_session"]),
                                  Stamp(c, mapping["pi_domain"], mapping["pi_session"]),
                                  Stamp(d, mapping["camera_domain"], mapping["camera_session"])))
    if not cmap.valid:
        raise ValueError("insufficient usable clockmap exchanges")
    return mapping


class Converter:
    def __init__(self, snapshot_session, mapping=None):
        if not snapshot_session:
            raise ValueError("snapshot_session must identify this producer lifetime")
        self.session = snapshot_session
        self.mapping = mapping
        self.previous_source = None
        self.domain = None
        self.previous_map = None
        self.local_changes = 0
        self.previous_counter = None

    def convert(self, snapshot):
        if not isinstance(snapshot, dict):
            raise ValueError("snapshot must be a JSON object")
        source = snapshot.get("source_ns")
        if source is not None:
            integer(source, "source_ns")
        domain = snapshot.get("domain")
        reasons = []
        valid_mapping = self.mapping is not None
        if source is None or not isinstance(domain, str) or not domain:
            valid_mapping = False
            reasons.append("source epoch/domain UNAVAILABLE")
        if self.domain is not None and domain != self.domain:
            raise ValueError("snapshot domain changed; start a new converter/session")
        if source is not None and self.previous_source is not None and source < self.previous_source:
            raise ValueError("source epoch regressed; start a new converter/session")
        self.domain = domain
        if source is not None:
            self.previous_source = source
        if self.mapping and (domain != self.mapping["snapshot_domain"] or
                             self.session != self.mapping["camera_session"]):
            valid_mapping = False
            reasons.append("clockmap snapshot domain/session mismatch")

        state = snapshot.get("state")
        if state in COLLAPSED:
            tracking, possible = COLLAPSED[state]
        elif state in TRACKING_STATES:
            tracking, possible = state, {state}
        else:
            tracking, possible = "SYSTEM_NOT_READY", set()
            reasons.append("unknown adapter state")
        raw = snapshot.get("orb_tracking_state")
        exact = (state not in COLLAPSED and state in TRACKING_STATES) or len(possible) == 1
        if raw is not None:
            if raw not in TRACKING_STATES or raw not in possible:
                raise ValueError("orb_tracking_state must be an exact name consistent with adapter state")
            tracking, exact = raw, True
        if not exact:
            reasons.append("exact ORB tracking state UNAVAILABLE (adapter collapsed it)")

        reset = snapshot.get("reset_counter")
        reset_source = "reset_counter"
        if reset is None:
            reset = snapshot.get("epoch_id")
            reset_source = "epoch_id"
        if reset is None and snapshot.get("map_id") is not None:
            current = integer(snapshot["map_id"], "map_id")
            if self.previous_map is not None and current != self.previous_map:
                self.local_changes += 1
            self.previous_map = current
            reset = self.local_changes
            reset_source = "observed map_id changes (local baseline; hidden resets UNKNOWN)"
            reasons.append("full reset history UNAVAILABLE; only local map changes observed")
        if reset is None:
            reset_source = "UNAVAILABLE"
            reasons.append("map reset counter UNAVAILABLE")
        else:
            integer(reset, "reset counter")
            if reset_source in ("reset_counter", "epoch_id"):
                if self.previous_counter is not None and reset < self.previous_counter:
                    raise ValueError("adapter lifetime counter regressed; new session required")
                self.previous_counter = reset

        imu = snapshot.get("imu_initialized")
        if type(imu) is not bool:
            imu = False
            reasons.append("D435i camera IMU initialization UNAVAILABLE")
        features = snapshot.get("tracked_features")
        if features is None:
            reasons.append("tracked feature count UNAVAILABLE")
        else:
            integer(features, "tracked_features")
        if snapshot.get("valid") is not True or snapshot.get("reasons", 0) != 0:
            reasons.append("adapter reports invalid pose/reason flags")
        mapping = copy.deepcopy(self.mapping) if valid_mapping else {
            "valid": False, "reason": "explicit bound clockmap unavailable or mismatched"}
        # Missing source is a sentinel only. Invalid mapping forbids age arithmetic.
        event = {"pose_source_d435i_ns": source if source is not None else 0,
                 "mapping": mapping, "tracking_state": tracking,
                 "imu_initialized": imu, "degraded": bool(reasons),
                 "integration": {"snapshot_session": self.session,
                                 "source_domain": domain, "source_available": source is not None,
                                 "tracking_exact": exact, "map_reset_source": reset_source,
                                 "unavailable_or_degraded": reasons,
                                 "evidence": snapshot.get("evidence", "UNKNOWN")}}
        if reset is not None:
            event["map_reset_counter"] = reset % (1 << 16)
        if features is not None:
            event["tracked_features"] = min(features, (1 << 16) - 1)
        return event


class ArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        self.print_usage(sys.stderr)
        self.exit(3, f"{self.prog}: error: {message}\n")


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="snapshot JSONL; default stdin")
    parser.add_argument("--clockmap", type=Path, help="explicit session-bound four-stamp exchanges JSON")
    parser.add_argument("--snapshot-session", required=True, help="producer lifetime/camera clock session ID")
    args = parser.parse_args()
    source = None
    try:
        converter = Converter(args.snapshot_session, load_mapping(args.clockmap))
        source = args.input.open() if args.input else sys.stdin
        for line_number, line in enumerate(source, 1):
            if line.strip():
                try:
                    event = converter.convert(json.loads(line))
                except (TypeError, ValueError, KeyError) as exc:
                    raise ValueError(f"line {line_number}: {exc}") from exc
                print(json.dumps(event, separators=(",", ":")), flush=True)
        return 0
    except (OSError, TypeError, ValueError, KeyError, ImportError) as exc:
        print(f"conversion error: {exc}", file=sys.stderr)
        return 3
    finally:
        if source is not None and source is not sys.stdin:
            source.close()


if __name__ == "__main__":
    sys.exit(main())
