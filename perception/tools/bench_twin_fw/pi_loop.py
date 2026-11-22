#!/usr/bin/env python3
"""Isolated bench runner: one reader, absolute-deadline writer, zero targets/gains.

No E/D, enable, torque, gain or advisory options. Requires pyserial to run.
"""
import argparse
import json
import math
import queue
import threading
import time

import host

DROP_KEYS = ('lost', 'rx_drop', 'rx_error', 'uart_drop', 'tx_byte_drop',
             'parse_error', 'rx_gap', 'mux_timeout', 'text_overflow',
             'binary_overflow', 'echo_encode_drop',
             'm_format_drop', 'c1_ovr', 'c2_ovr', 'fb_busy')


def parse_stat(line):
    if not line.startswith('STAT '):
        return None
    return {k: int(v, 0) for k, v in (x.split('=', 1) for x in line.split()[1:])}


def stat_delta(previous, current):
    elapsed = ((current['ms'] - previous['ms']) & 0xFFFFFFFF) / 1000
    if elapsed <= 0 or elapsed > 60:
        raise ValueError('STAT clock reset/stall; baseline discarded')
    keys = ('sent', 'fed_back', 'rejected') + DROP_KEYS
    return dict(seconds=elapsed, **{k + '_s': ((current[k] - previous[k]) & 0xFFFFFFFF) / elapsed
                for k in keys if k in current and k in previous})


def write_all(port, data):
    offset = 0
    while offset < len(data):
        n = port.write(data[offset:])
        if n is None or n <= 0:
            raise OSError('serial write made no progress')
        offset += n
    return offset


def reader(port, stop, events, raw):
    decoder = host.Decoder()
    try:
        while not stop.is_set():
            data = port.read(4096)
            if not data:
                continue
            raw.write(json.dumps(dict(monotonic_ns=time.monotonic_ns(), rx_hex=data.hex())) + '\n')
            raw.flush()
            for kind, record in decoder.feed(data):
                if kind == 'line' and (record.startswith('STAT ') or record.startswith('BOOT')):
                    events.put((time.monotonic(), record))
            if decoder.errors:
                raise ValueError(f'host decoder errors={decoder.errors}')
    except Exception as exc:
        events.put((time.monotonic(), exc))


def run(port, rate, duration, raw):
    stop = threading.Event()
    events = queue.Queue()
    runner = threading.Thread(target=reader, args=(port, stop, events, raw), daemon=True)
    runner.start()
    previous = None
    last_stat = start = time.monotonic()
    seq = written = late = 0
    slot = 0
    total_slots = math.ceil(rate * duration)
    next_write = start
    end = start + duration
    try:
        # Drain two further STAT intervals after the last write.
        while time.monotonic() < end + 2.2:
            now = time.monotonic()
            if slot < total_slots and now < end and now >= next_write:
                wire = host.command(seq, host.IDS[seq % len(host.IDS)])
                late += int(now - next_write > 1 / rate)
                n = write_all(port, wire)
                print(json.dumps(dict(event='write', monotonic_ns=time.monotonic_ns(),
                                      seq=seq, bytes=n, wire=wire.decode().strip())), flush=True)
                written += 1
                seq = (seq + 1) & 0xFFFFFFFF
                slot += 1
                next_write = start + slot / rate
                # Skip expired slots, never issue catch-up mailbox bursts.
                while slot < total_slots and next_write < time.monotonic():
                    slot += 1
                    next_write = start + slot / rate
                    late += 1
            while True:
                try:
                    stamp, record = events.get_nowait()
                except queue.Empty:
                    break
                if isinstance(record, Exception):
                    raise record
                if record.startswith('BOOT'):
                    raise RuntimeError('MCU BOOT/reset during measurement; restart runner')
                current = parse_stat(record)
                last_stat = stamp
                if previous is not None:
                    print(json.dumps(dict(event='stat_delta', **stat_delta(previous, current))), flush=True)
                else:
                    print(json.dumps(dict(event='stat_baseline', **current)), flush=True)
                previous = current
            if now - last_stat > 3:
                raise TimeoutError('no STAT for 3 seconds while reader is active')
            time.sleep(max(0, min(.005, next_write - time.monotonic())) if now < end else .005)
    finally:
        stop.set()
        runner.join(timeout=1)
        print(json.dumps(dict(event='writer_total', complete_lines=written, deadline_misses=late)), flush=True)
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('--rate', type=float, default=100, help='aggregate C-lines/s')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--raw-log', default='pi_loop_rx.jsonl')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.rate, args.seconds)):
        parser.error('rate and seconds must be finite and positive')
    import serial
    with open(args.raw_log, 'x') as raw, serial.Serial(
            args.port, baudrate=460800, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=.05, write_timeout=.5, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        run(port, args.rate, args.seconds, raw)


if __name__ == '__main__':
    main()
