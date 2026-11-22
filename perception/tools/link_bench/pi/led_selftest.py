#!/usr/bin/env python3
"""LED self-test: send each advisory for a few seconds so the operator can check the wiring.

NONE -> green, JUMP_CANDIDATE -> yellow, STOP_CANDIDATE -> red, UNCERTAIN -> yellow blinking
(+ D9 LED if fitted), then silence -> red blinking 2 Hz (stale). No camera needed.
"""
import argparse, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.obstacle_decision import advisory_codec
import importlib.util
spec = importlib.util.spec_from_file_location('link_bench', Path(__file__).with_name('link_bench.py'))
lb = importlib.util.module_from_spec(spec); spec.loader.exec_module(lb)

STEPS = [(0, 'GREEN (NONE)'), (1, 'YELLOW (JUMP)'), (2, 'RED (STOP)'), (3, 'YELLOW BLINKING (UNCERTAIN)')]

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='/dev/ttyAMA0'); p.add_argument('--baud', type=int, default=460800)
    p.add_argument('--seconds', type=float, default=4.0)
    a = p.parse_args()
    link = lb.Link(a.port, a.baud); link.dropping = False; link.synced = True
    seq = (time.monotonic_ns() // 1_000_000) & 0xffffffff; echoes = 0
    for adv, label in STEPS:
        print(f'now: {label}', flush=True); end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            seq = (seq + 1) & 0xffffffff
            link.send(advisory_codec.encode(lb.BENCH_SESSION, seq, pi_monotonic_ns=time.monotonic_ns(), frame_stamp_ns=0,
                      advisory=adv, confidence=1.0, distance_m=1.0, height_m=0.05, width_m=0.25, plane_valid=True,
                      plane_pitch_rad=0.17, plane_height_m=0.29))
            echoes += sum(1 for _, r in link.poll(0.066) if r.get('type') == 5)
    print('now: nothing sent -> RED BLINKING (stale)', flush=True)
    print(f'echoes from the Nucleo: {echoes} (0 means the UART wires are wrong)')

if __name__ == '__main__':
    main()
