#!/usr/bin/env python3
"""Generate C parity vectors by executing the reviewed Python supervisor model."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "py"))
from supervisor_model import Event, SupervisorModel  # noqa: E402

CONFIG = json.loads((ROOT / "config.json").read_text())
U32 = 0xFFFFFFFF

def scenario(name, actions):
    model = SupervisorModel(CONFIG, primary_fresh=True)
    rows = []
    for now, kind, seq, session, received, age, tracking, flags, imu, reset in actions:
        ev = None if kind in ("-", "crc_error") else Event(
            kind=kind, received_us=received, age_at_send_us=age, tracking_state=tracking,
            flags=flags, session_id=session, map_reset_counter=reset,
            imu_initialized=bool(imu), wire_seq=seq)
        state = model.update(now, ev)
        # Parity surface is the optional module's recommendation: INIT/FRESH -> NONE.
        action = {"INIT": "NONE", "FRESH": "NONE", "HOLD": "HOLD", "FAILSAFE": "FAILSAFE"}[state]
        rows.append((now & U32, kind, seq, session, received & U32, age, tracking, flags, imu, reset,
                     int(action == "NONE"), action, "NONE" if action == "NONE" else model.reason))
    return name, rows

def good(t, kind, seq, session=42, age=1000, flags=3, tracking=2, imu=1, reset=0):
    return (t, kind, seq, session, t, age, tracking, flags, imu, reset)

def saved_input_excerpt():
    # This saved fixture has MEASURED D435i input association but SYNTHETIC fake
    # engine estimates; only the recorded estimator-state labels are reused.
    fixture = ROOT / "integration/tests/fixtures/real_excerpt.jsonl"
    tracking = {"INITIALIZING": 1, "TRACKING": 2, "LOST": 4}
    rows = [good(0,"heartbeat",1)]
    for index, line in enumerate(fixture.read_text().splitlines(), 1):
        item = json.loads(line)
        rows.append(good(index * 10, "health", index + 1, age=U32, flags=0,
                         tracking=tracking[item["state"]], reset=item["reset_counter"]))
    return scenario("saved_input_excerpt_synthetic_estimates", rows)

def main(path):
    c = CONFIG
    scenarios = [
        scenario("baseline_gap_duplicate_crc", [good(1000,"heartbeat",1), good(1100,"health",3),
            (1150,"crc_error",0,0,1150,0,0,0,0,0), good(1200,"health",3), good(1300,"health",7)]),
        scenario("silence_hold_timeout", [good(0,"heartbeat",1), good(100,"health",2),
            (200100,"-",0,0,0,0,0,0,0,0), (500100,"-",0,0,0,0,0,0,0,0)]),
        scenario("age_hold_unavailable_lost", [good(0,"heartbeat",1), good(10,"health",2,age=98000),
            good(20,"health",3,age=U32,flags=0), good(30,"health",4,age=0,tracking=4)]),
        scenario("valid_saturated_age", [good(0,"heartbeat",1), good(10,"health",2,age=U32,flags=3)]),
        scenario("saturation_and_wrap", [good(0xFFFFFF00,"heartbeat",1), good(0xFFFFFF10,"health",2,age=U32,flags=0),
            (0x100000100,"-",0,0,0,0,0,0,0,0)]),
        saved_input_excerpt(),
    ]
    with open(path, "w", encoding="ascii") as out:
        out.write("C %d %d %d %d %d %d\n" % (c["pose_age_hold_us"], c["pose_age_timeout_us"],
            c["receipt_silence_hold_us"], c["receipt_silence_timeout_us"], c["heartbeat_timeout_us"],
            c["transit_bound_us"]))
        for name, rows in scenarios:
            out.write("S " + name + "\n")
            for row in rows:
                now, kind, seq, session, received, age, tracking, flags, imu, reset, primary, action, reason = row
                out.write(f"V {now} {kind} {seq} {session} {received} {age} {tracking} {flags} {imu} {reset} {primary} {action} {reason}\n")

if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "vectors.generated.txt")
