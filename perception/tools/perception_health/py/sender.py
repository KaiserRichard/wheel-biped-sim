#!/usr/bin/env python3
"""JSONL pose-event sender for UART v3. Never uses camera/ROS APIs itself."""
from __future__ import annotations
import argparse, json, os, secrets, sys, threading, time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path: sys.path.insert(0, str(ROOT))
try:
    from .perception_health import *
except ImportError:
    from perception_health import *

def monotonic_raw_ns():
    clock_id = getattr(time, "CLOCK_MONOTONIC_RAW", time.CLOCK_MONOTONIC)
    return time.clock_gettime_ns(clock_id)

def json_true(event, key):
    """Accept JSON booleans only; malformed truthy values must not claim health."""
    return event.get(key) is True

def map_source_time(source_ns, mapping):
    """Map camera epoch with the existing qualified ClockMap implementation."""
    from tools.clockmap.clockmap import ClockMap, Exchange, Stamp
    domain_a=mapping["camera_domain"]; domain_b=mapping["pi_domain"]
    if domain_b != "CLOCK_MONOTONIC_RAW":
        raise ValueError("Pi mapping domain must be CLOCK_MONOTONIC_RAW to match linux_send_ns")
    session_a=mapping["camera_session"]; session_b=mapping["pi_session"]
    cmap=ClockMap(domain_a,domain_b,session_a,session_b,min_samples=3)
    for t1,t2,t3,t4 in mapping["exchanges_ns"]:
        cmap.add_exchange(Exchange(Stamp(int(t1),domain_a,session_a),Stamp(int(t2),domain_b,session_b),
                                   Stamp(int(t3),domain_b,session_b),Stamp(int(t4),domain_a,session_a)))
    if not cmap.valid: raise ValueError("clock mapping does not have enough valid exchanges")
    mapped=cmap.convert(Stamp(int(source_ns),domain_a,session_a))
    if mapped.stamp.domain != domain_b or mapped.stamp.session != session_b:
        raise ValueError("clock map returned unexpected Pi clock identity")
    return mapped.stamp.value_ns, mapped.uncertainty_ns

class Port:
    def __init__(self, path, baud):
        try:
            import serial
        except ImportError:
            import termios
            self.fd = os.open(path, os.O_WRONLY | os.O_NOCTTY)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] |= termios.CLOCAL | termios.CREAD | termios.CS8
            rate = getattr(termios, f"B{baud}")
            attrs[4] = attrs[5] = rate
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            self.ser = None
        else:
            self.ser = serial.Serial(path, baudrate=baud, timeout=0, write_timeout=1)
            self.fd = None
    def write(self, frame):
        if self.ser: self.ser.write(frame)
        else:
            view = memoryview(frame)
            while view:
                view = view[os.write(self.fd, view):]
    def close(self):
        if self.ser: self.ser.close()
        elif self.fd is not None: os.close(self.fd)

def process(args):
    port = Port(args.device, args.baud)
    session = args.session_id if args.session_id is not None else secrets.randbits(64)
    seq = 0
    lock = threading.Lock()
    stop = threading.Event()
    def send_hb():
        nonlocal seq
        while not stop.wait(args.heartbeat_ms / 1000):
            with lock:
                port.write(encode_heartbeat(seq, monotonic_raw_ns(), session)); seq = (seq + 1) & 0xFFFF
    with lock:
        port.write(encode_heartbeat(seq,monotonic_raw_ns(),session)); seq=(seq+1)&0xFFFF
    thread = threading.Thread(target=send_hb, daemon=True); thread.start()
    source = open(args.input, encoding="utf-8") if args.input else sys.stdin
    try:
        for line_num, line in enumerate(source, 1):
            if not line.strip(): continue
            event = json.loads(line)
            source_ns = int(event["pose_source_d435i_ns"])
            mapping = event.get("mapping")
            mapping_valid = bool(mapping and json_true(mapping, "valid"))
            pi_source_ns = uncertainty_ns = None
            if mapping_valid:
                try:
                    pi_source_ns, uncertainty_ns = map_source_time(source_ns,mapping)
                except (KeyError,ValueError,ImportError) as exc:
                    print(f"line {line_num}: mapping unavailable: {exc}",file=sys.stderr)
                    mapping_valid=False
            else:
                mapping_valid=False
            state = event["tracking_state"]
            state_id = TRACKING_STATES[state] if isinstance(state, str) else int(state)
            with lock:
                # Take the Pi raw-monotonic timestamp at the write boundary, after
                # lock contention and all mapping work, so reported age is conservative.
                send_ns = monotonic_raw_ns()
                if mapping_valid:
                    try: age=min(U32_MAX,saturating_age_us(send_ns,pi_source_ns)+(uncertainty_ns+999)//1000)
                    except ValueError:
                        mapping_valid=False; age=U32_MAX
                        print(f"line {line_num}: mapped source time is in the future; age unavailable",file=sys.stderr)
                else: age=U32_MAX
                flags = (FLAG_AGE_VALID | FLAG_MAPPING_VALID) if mapping_valid else 0
                if event.get("degraded", False) is not False: flags |= FLAG_DEGRADED
                payload = PerceptionHealth(send_ns, age, state_id, int(event.get("map_reset_counter", 0)) & 0xFFFF,
                    int(json_true(event, "imu_initialized")), int(event.get("tracked_features", 0)), flags)
                port.write(encode_health(seq, payload)); seq = (seq + 1) & 0xFFFF
            print(json.dumps({"line": line_num, "seq": (seq - 1) & 0xFFFF, "pose_age_us": age,
                              "mapping_valid": mapping_valid, "status": "SENT"}), file=sys.stderr)
    finally:
        stop.set(); thread.join(); port.close()
        if source is not sys.stdin: source.close()

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", required=True, help="UART tty")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--input", help="JSONL file; defaults to stdin")
    p.add_argument("--session-id", type=lambda x: int(x, 0), help="reproducible testing only; otherwise random per process")
    p.add_argument("--heartbeat-ms", type=int, default=None)
    a = p.parse_args()
    if a.heartbeat_ms is None:
        a.heartbeat_ms = json.loads((Path(__file__).parents[1] / "config.json").read_text())["heartbeat_period_ms"]
    if a.heartbeat_ms <= 0: p.error("--heartbeat-ms must be positive")
    process(a)
if __name__ == "__main__": main()
