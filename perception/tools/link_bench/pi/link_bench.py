#!/usr/bin/env python3
"""Pi-side bench test for the CANDIDATE Pi<->MCU link (tools/link_proto).

DIAGNOSTIC only. Talks to the Nucleo-F446RE bench firmware (../firmware) over
a serial port and measures, with every timestamp on the Pi's CLOCK_MONOTONIC:

  preflight  MCU->Pi direction alone: heartbeat McuStatus records heard
  rtt        round trip: PerceptionHealth out, matching McuStatus back
             (P50/P95/P99), round-trip loss, CRC/COBS errors per direction
  faults     corrupted CRC, duplicate sequence and wrong session must each be
             rejected by the MCU (REJECTED + reason) and recover on the next
             valid packet
  timeout    Pi goes silent for 100 ms (MCU should stay linked) and 300 ms
             (MCU should declare NO_LINK at its 150 ms rule)

MCU-side durations are computed only from MCU timestamps (sample/action
epochs in one McuStatus stream); Pi-side durations only from Pi timestamps.
The two clock domains are never subtracted from each other.

The health records are deliberately *ineligible* (no tracking, no clock
mapping): they exercise link liveness without a Pi->MCU clock mapping, which
does not exist yet. Standard library only; needs link_proto.py next to this
file or at ../../link_proto/py/.
"""
import argparse
import csv
import fcntl
import glob
import json
import math
import os
import platform
import select
import struct
import sys
import termios
import time

HERE = os.path.dirname(os.path.abspath(__file__))
for candidate in (HERE, os.path.join(HERE, '..', '..', 'link_proto', 'py')):
    if os.path.exists(os.path.join(candidate, 'link_proto.py')):
        sys.path.insert(0, candidate)
        break
import link_proto as lp  # noqa: E402

# Must match firmware/bench.h (test-only provisioning, not a real handshake).
BENCH_SESSION = 0x5742524C494E4B31
BENCH_MAPPING_REVISION = 0xF446
BENCH_TRANSFORM_REVISION = 0xF446

STATES = ['NO_LINK', 'LINK_OK_NO_ESTIMATE', 'ESTIMATE_VALID', 'STALE',
          'REJECTED', 'RESET_PENDING_REENABLE']
REASONS = ['NONE', 'WIRE', 'SESSION', 'SEQUENCE', 'FIELDS', 'AGE', 'RESET',
           'DISABLED', 'ASSOCIATION', 'SILENCE', 'LEASE']
NO_LINK, REJECTED, PENDING = 0, 4, 5
LINKED_STATES = (1, 2, 5)  # health-only bench without enable: expect 5
HEALTH_WIRE, STATUS_WIRE = 102, 66  # max wire bytes incl. delimiter (SPEC)


def now_ns():
    return time.monotonic_ns()


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or code - 1 > len(data) - i:
            return None
        block = data[i:i + code - 1]
        if 0 in block:
            return None
        out += block
        i += code - 1
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)


def decode(frame):
    """Decoded record dict, or None on any COBS/CRC/layout error."""
    raw = cobs_decode(frame)
    if raw is None or len(raw) < 20:
        return None
    version, kind, length, session, seq = struct.unpack_from('<BBHQI', raw)
    if version != lp.VERSION or len(raw) != 20 + length:
        return None
    if struct.unpack_from('<I', raw, len(raw) - 4)[0] != lp.crc32c(raw[:-4]):
        return None
    rec = {'type': kind, 'session': session, 'seq': seq}
    if kind == lp.MCU_STATUS and length == 44:
        (rec['sample_mcu_ns'], rec['last_health_seq'], rec['last_estimate_seq'],
         rec['last_ref_seq'], rec['faults'], rec['action_mcu_ns'], _battery,
         _fsm, _armed, _enable, rec['state'], rec['reason'],
         _valid) = struct.unpack_from('<QIIIIQHBBBBBB', raw, 16)
        if rec['state'] >= len(STATES) or rec['reason'] >= len(REASONS):
            return None
        return rec
    if kind == lp.HEALTH and length == 80:  # loopback mode reads its own records
        rec['pi_send_ns'] = struct.unpack_from('<Q', raw, 16 + 48)[0]
        return rec
    if kind in (lp.TERRAIN_ADVISORY, lp.ADVISORY_ECHO):
        try:
            header, body = lp.unpack(frame + b'\0')
        except ValueError:
            return None
        rec.update(body)
        return rec
    return None


def percentile(sorted_values, p):
    """Nearest-rank percentile."""
    if not sorted_values:
        return None
    k = max(1, math.ceil(p / 100.0 * len(sorted_values)))
    return sorted_values[k - 1]


def stats_us(values_ns):
    v = sorted(x / 1000.0 for x in values_ns)
    if not v:
        return {'n': 0}
    return {'n': len(v), 'min': v[0], 'p50': percentile(v, 50),
            'p95': percentile(v, 95), 'p99': percentile(v, 99), 'max': v[-1],
            'mean': sum(v) / len(v)}


class GpioMarker:
    """Toggle one GPIO line per sent packet through the Linux GPIO character
    device (uAPI v2, stdlib ioctl only), for a logic-analyzer timing channel.
    Pi 5 header GPIOs live on the RP1 chip, labelled 'pinctrl-rp1'; the line
    offset equals the BCM GPIO number. Struct sizes/ioctl numbers checked
    against linux/gpio.h (request 592 B, values 16 B, chipinfo 68 B)."""
    GET_CHIPINFO = 0x8044B401
    GET_LINE = 0xC250B407
    SET_VALUES = 0xC010B40F
    FLAG_OUTPUT = 1 << 3

    def __init__(self, line, chip_label='pinctrl-rp1'):
        self.fd = None
        for path in sorted(glob.glob('/dev/gpiochip*')):
            try:
                cfd = os.open(path, os.O_RDWR)
            except OSError:
                continue
            try:
                info = bytearray(68)
                fcntl.ioctl(cfd, self.GET_CHIPINFO, info)
                if info[32:64].split(b'\0')[0].decode() != chip_label:
                    continue
                req = bytearray(592)
                struct.pack_into('<I', req, 0, line)              # offsets[0]
                req[256:256 + 14] = b'wbr-link-bench'             # consumer
                struct.pack_into('<Q', req, 288, self.FLAG_OUTPUT)  # config.flags
                struct.pack_into('<I', req, 560, 1)                # num_lines
                fcntl.ioctl(cfd, self.GET_LINE, req)
                self.fd = struct.unpack_from('<i', req, 588)[0]
                self.path = path
                break
            finally:
                os.close(cfd)
        if self.fd is None:
            raise OSError('no GPIO chip labelled %r (or no permission; try sudo)' % chip_label)
        self.level = 0
        self.set(0)

    def set(self, level):
        fcntl.ioctl(self.fd, self.SET_VALUES, struct.pack('<QQ', level & 1, 1))
        self.level = level & 1

    def toggle(self):
        self.set(self.level ^ 1)


class Link:
    def __init__(self, port, baud):
        speed = getattr(termios, 'B%d' % baud, None)
        # macOS omits high-speed constants in Python's termios module. PTYs do
        # not serialize at a physical baud rate; use the nearest exposed speed.
        if speed is None and platform.system() == 'Darwin' and baud == 460800:
            speed = getattr(termios, 'B230400', None)
        if speed is None:
            raise SystemExit('baud %d not supported by termios' % baud)
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                                        # iflag: raw, no XON/XOFF
        attrs[1] = 0                                        # oflag
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, no flow control
        attrs[3] = 0                                        # lflag: no echo/canonical
        attrs[4] = attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = bytearray()
        # Opened mid-stream: discard bytes up to the first delimiter (COBS resync),
        # without counting that partial frame as a wire error.
        self.dropping = True
        self.synced = False
        self.rx_frames = 0
        self.rx_wire_errors = 0
        self.mcu_boot_ids = []
        self.marker = None

    def send(self, data, mark=True):
        t = now_ns()
        if mark and self.marker is not None:
            self.marker.toggle()  # edge follows t by the ioctl cost; see analyzer notes
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.05)
        return t

    def poll(self, timeout_s):
        """Read what arrives within timeout; return [(t_rx_ns, record)]."""
        out = []
        ready, _, _ = select.select([self.fd], [], [], max(0.0, timeout_s))
        if not ready:
            return out
        t = now_ns()
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            return out
        for b in chunk:
            if b == 0:
                if not self.synced:
                    self.synced = True
                elif self.dropping or self.buf:
                    rec = None if self.dropping else decode(bytes(self.buf))
                    if rec is None:
                        self.rx_wire_errors += 1
                    else:
                        self.rx_frames += 1
                        if rec['type'] == lp.MCU_STATUS and (
                                not self.mcu_boot_ids or self.mcu_boot_ids[-1] != rec['session']):
                            self.mcu_boot_ids.append(rec['session'])
                        out.append((t, rec))
                self.buf.clear()
                self.dropping = False
            elif not self.dropping:
                self.buf.append(b)
                if len(self.buf) > 194:
                    self.dropping = True
                    self.buf.clear()
        return out


class Bench:
    def __init__(self, link, rate_hz, loopback):
        self.link = link
        self.period_ns = int(1e9 / rate_hz)
        self.loopback = loopback
        # Start above any sequence an earlier run in this Pi boot could have used.
        self.seq = (now_ns() // 1_000_000) & 0xFFFFFFFF
        self.last_action = None
        self.rejections = {}
        self.statuses = []

    def health(self, seq, session=BENCH_SESSION):
        return lp.perception_health(
            session, seq, estimate_seq=0, map_epoch=0,
            transform_revision=BENCH_TRANSFORM_REVISION,
            mapping_revision=BENCH_MAPPING_REVISION, reset_counter=0,
            tracking=0, estimator_id=1, source_domain=2, flags=0, frame_id=2,
            child_frame_id=1, quality=0, reasons=0, source_ns=now_ns(),
            mapped_mcu_ns=0, pi_send_ns=now_ns(), age_at_send_ns=0,
            uncertainty_ns=0)

    def next_packet(self):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return self.seq, self.health(self.seq)

    def track(self, records):
        """Book-keep every MCU status; count each new rejection once (new action epoch)."""
        for t, rec in records:
            if rec['type'] != lp.MCU_STATUS:
                continue
            self.statuses.append((t, rec))
            if rec['state'] == REJECTED and rec['action_mcu_ns'] != self.last_action:
                name = REASONS[rec['reason']]
                self.rejections[name] = self.rejections.get(name, 0) + 1
            self.last_action = rec['action_mcu_ns']
        return records

    def listen(self, seconds):
        end = now_ns() + int(seconds * 1e9)
        got = []
        while now_ns() < end:
            got += self.track(self.link.poll((end - now_ns()) / 1e9))
        return got

    def wait_for(self, predicate, seconds):
        end = now_ns() + int(seconds * 1e9)
        while now_ns() < end:
            for t, rec in self.track(self.link.poll((end - now_ns()) / 1e9)):
                if predicate(rec):
                    return t, rec
        return None, None

    def reply_matches(self, seq):
        if self.loopback:
            return lambda r: r['type'] == lp.HEALTH and r['seq'] == seq
        return lambda r: r['type'] == lp.MCU_STATUS and r['last_health_seq'] == seq

    def exchange(self, timeout_s=0.1):
        seq, pkt = self.next_packet()
        t_send = self.link.send(pkt)
        t_rx, rec = self.wait_for(self.reply_matches(seq), timeout_s)
        return seq, t_send, t_rx, rec

    # ---------------------------------------------------------------- phases
    def preflight(self, seconds=1.0):
        recs = [r for _, r in self.listen(seconds) if r['type'] == lp.MCU_STATUS]
        return {'seconds': seconds, 'status_records': len(recs),
                'states': sorted({STATES[r['state']] for r in recs}),
                'mcu_boot_ids': ['%016x' % b for b in self.link.mcu_boot_ids],
                'expect': 'about 20 status records/s in NO_LINK before the Pi sends'}

    def rtt(self, duration_s, rows):
        pending = {}  # seq -> t_send
        rtts, late, states = [], 0, {}
        start = now_ns()
        next_send = start
        end = start + int(duration_s * 1e9)
        while True:
            t = now_ns()
            if t >= next_send and t < end:
                seq, pkt = self.next_packet()
                pending[seq] = self.link.send(pkt)
                next_send += self.period_ns
            # A reply later than 200 ms counts as lost.
            for seq in [s for s, ts in pending.items() if t - ts > 200_000_000]:
                rows.append([seq, pending.pop(seq), '', '', '', ''])
            if t >= end and not pending:
                break
            if t >= end + 300_000_000:
                break
            wait = (min(next_send, end) - now_ns()) / 1e9 if t < end else 0.01
            for t_rx, rec in self.track(self.link.poll(max(wait, 0.0))):
                key = rec['seq'] if self.loopback else rec.get('last_health_seq')
                ok = (rec['type'] == lp.HEALTH) if self.loopback else (rec['type'] == lp.MCU_STATUS)
                if ok and key in pending:
                    t_send = pending.pop(key)
                    rtts.append(t_rx - t_send)
                    late += (t_rx - t_send) > self.period_ns
                    state = 'LOOPBACK' if self.loopback else STATES[rec['state']]
                    states[state] = states.get(state, 0) + 1
                    rows.append([key, t_send, t_rx, (t_rx - t_send) / 1000.0, state,
                                 rec.get('sample_mcu_ns', '')])
        lost = sum(1 for r in rows if r[2] == '')
        sent = len(rtts) + lost
        return {'rate_hz': 1e9 / self.period_ns, 'duration_s': duration_s,
                'sent': sent, 'replied': len(rtts), 'lost': lost,
                'loss_ratio': lost / sent if sent else None,
                'replies_later_than_period': late, 'rtt_us': stats_us(rtts),
                'reply_states': states}

    def faults(self, trials, rows):
        result = {}
        kinds = [('crc', 'WIRE'), ('duplicate', 'SEQUENCE'), ('session', 'SESSION')]
        for kind, expect in kinds:
            ok = 0
            for i in range(trials):
                seq, _, _, rec = self.exchange()
                linked_before = rec is not None and rec['state'] in LINKED_STATES
                if kind == 'crc':
                    bad = bytearray(self.health(seq + 1))
                    j = len(bad) // 2
                    bad[j] ^= 0x80 if bad[j] ^ 0x80 else 0x40
                elif kind == 'duplicate':
                    bad = self.health(seq)
                else:
                    bad = self.health(seq + 1, BENCH_SESSION ^ 1)
                self.link.send(bytes(bad))
                _, rej = self.wait_for(lambda r: r['type'] == lp.MCU_STATUS and r['state'] == REJECTED, 0.1)
                seen = REASONS[rej['reason']] if rej else None
                _, _, _, rec2 = self.exchange()
                recovered = rec2 is not None and rec2['state'] in LINKED_STATES
                passed = linked_before and seen == expect and recovered
                ok += passed
                rows.append([kind, i, linked_before, expect, seen, recovered, passed])
            result[kind] = {'trials': trials, 'expected_reason': expect,
                            'observed_as_expected': ok}
        return result

    def timeout(self, gaps_ms, repeats, rows):
        out = {}
        for gap in gaps_ms:
            trips = []
            for i in range(repeats):
                # Re-establish a steady link for 0.5 s at the normal rate.
                last = None
                for _ in range(max(1, int(0.5e9 / self.period_ns))):
                    seq, t_send, t_rx, rec = self.exchange(timeout_s=self.period_ns / 1e9)
                    if rec is not None:
                        last = (seq, t_send, rec)
                    rest = (t_send + self.period_ns - now_ns()) / 1e9
                    if rest > 0:
                        self.listen(rest)
                if last is None:
                    rows.append([gap, i, 'no reply before gap', '', '', '', ''])
                    continue
                seq, t_last_send, last_rec = last
                # Silence: send nothing for `gap` ms after the last health packet.
                t_drop, drop = None, None
                end = t_last_send + gap * 1_000_000
                while now_ns() < end:
                    for t, rec in self.track(self.link.poll((end - now_ns()) / 1e9)):
                        if drop is None and rec['type'] == lp.MCU_STATUS and rec['state'] == NO_LINK:
                            t_drop, drop = t, rec
                seq2, t_res, t_back, back = self.exchange()
                tripped = drop is not None
                pi_ms = (t_drop - t_last_send) / 1e6 if tripped else None
                mcu_ms = ((drop['action_mcu_ns'] - last_rec['sample_mcu_ns']) / 1e6
                          if tripped and drop['session'] == last_rec['session'] else None)
                recovered = back is not None and back['state'] in LINKED_STATES
                trips.append((tripped, pi_ms, mcu_ms, recovered))
                rows.append([gap, i, tripped, pi_ms, mcu_ms, recovered,
                             (t_back - t_res) / 1e6 if back else None])
            pi = sorted(x[1] for x in trips if x[1] is not None)
            mcu = sorted(x[2] for x in trips if x[2] is not None)
            out['%d_ms' % gap] = {
                'repeats': repeats, 'no_link_declared': sum(x[0] for x in trips),
                'recovered_after_resume': sum(x[3] for x in trips),
                'pi_detect_ms_last_send_to_no_link_status': {
                    'min': pi[0] if pi else None, 'max': pi[-1] if pi else None},
                'mcu_silence_ms_accept_to_action': {
                    'min': mcu[0] if mcu else None, 'max': mcu[-1] if mcu else None},
                'expect': ('stay linked' if gap < 150 else
                           'NO_LINK; MCU silence about 150 ms; Pi detect about 150 ms + one status')}
        return out


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', required=True, help='/dev/ttyAMA0 (GPIO14/15) or /dev/ttyACM0 (Nucleo USB)')
    ap.add_argument('--baud', type=int, default=460800)
    ap.add_argument('--out', required=True, help='output directory (created)')
    ap.add_argument('--rate', type=float, default=50.0, help='health send rate for rtt, Hz')
    ap.add_argument('--duration', type=float, default=60.0, help='rtt phase length, s')
    ap.add_argument('--fault-trials', type=int, default=10)
    ap.add_argument('--timeout-repeats', type=int, default=5)
    ap.add_argument('--phases', default='preflight,rtt,faults,timeout')
    ap.add_argument('--marker-gpio', type=int, default=None,
                    help='BCM GPIO toggled at every health send (logic analyzer), e.g. 17 = header pin 11')
    ap.add_argument('--loopback', action='store_true',
                    help='no MCU: Pi TX jumpered to Pi RX; runs rtt only')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    phases = ['rtt'] if args.loopback else [p.strip() for p in args.phases.split(',') if p.strip()]
    link = Link(args.port, args.baud)
    if args.marker_gpio is not None:
        link.marker = GpioMarker(args.marker_gpio)
    bench = Bench(link, args.rate, args.loopback)
    floor_ms = (HEALTH_WIRE + (HEALTH_WIRE if args.loopback else STATUS_WIRE)) * 10 * 1000.0 / args.baud
    summary = {
        'verdict': 'OPEN / diagnostic (bench link test; not a gate PASS)',
        'mode': 'loopback (Pi TX->RX jumper)' if args.loopback else 'Pi <-> MCU bench firmware',
        'port': args.port, 'baud': args.baud, 'host': platform.node(),
        'kernel': platform.release(), 'python': platform.python_version(),
        'clock': 'all Pi times CLOCK_MONOTONIC (time.monotonic_ns); MCU durations from MCU stamps only',
        'started_monotonic_ns': now_ns(), 'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'serialization_floor_ms': round(floor_ms, 3),
        'marker_gpio': args.marker_gpio,
    }
    print('link_bench: %s on %s @ %d baud -> %s' % (summary['mode'], args.port, args.baud, args.out))
    if 'preflight' in phases:
        summary['preflight'] = bench.preflight()
        print('preflight: %d status records, states %s' % (
            summary['preflight']['status_records'], summary['preflight']['states']))
        if summary['preflight']['status_records'] == 0:
            print('  nothing heard from the MCU: check MCU TX (PA9) -> Pi RX (GPIO15), GND, port, baud, firmware')
    if 'rtt' in phases or 'faults' in phases or 'timeout' in phases:
        if not args.loopback:
            seq, _, _, rec = bench.exchange(0.2)
            if rec is None and bench.rejections.get('SEQUENCE'):
                print('MCU rejects our sequence numbers: press the Nucleo black RESET button and rerun')
                summary['aborted'] = 'SEQUENCE rejection at start (stale MCU high-water mark)'
            elif rec is None:
                print('no reply to the first health packet: check Pi TX (GPIO14) -> MCU RX (PA10)')
    if 'aborted' not in summary:
        if 'rtt' in phases:
            rows = []
            before = dict(bench.rejections)
            summary['rtt'] = bench.rtt(args.duration, rows)
            summary['rtt']['mcu_rejections_during_rtt'] = {
                k: v - before.get(k, 0) for k, v in bench.rejections.items() if v - before.get(k, 0)}
            write_csv(os.path.join(args.out, 'rtt.csv'),
                      ['seq', 't_send_pi_ns', 't_reply_pi_ns', 'rtt_us', 'mcu_state', 'mcu_sample_ns'], rows)
            r = summary['rtt']
            s = r['rtt_us']
            print('rtt: sent %d replied %d lost %d; RTT us P50 %s P95 %s P99 %s max %s' % (
                r['sent'], r['replied'], r['lost'], fmt(s.get('p50')), fmt(s.get('p95')),
                fmt(s.get('p99')), fmt(s.get('max'))))
        if 'faults' in phases and not args.loopback:
            rows = []
            summary['faults'] = bench.faults(args.fault_trials, rows)
            write_csv(os.path.join(args.out, 'faults.csv'),
                      ['kind', 'trial', 'linked_before', 'expected_reason', 'observed_reason',
                       'recovered', 'as_expected'], rows)
            print('faults: ' + ', '.join('%s %d/%d' % (k, v['observed_as_expected'], v['trials'])
                                         for k, v in summary['faults'].items()))
        if 'timeout' in phases and not args.loopback:
            rows = []
            summary['timeout'] = bench.timeout([100, 300], args.timeout_repeats, rows)
            write_csv(os.path.join(args.out, 'timeout.csv'),
                      ['gap_ms', 'trial', 'no_link_declared', 'pi_detect_ms', 'mcu_silence_ms',
                       'recovered', 'resume_rtt_ms'], rows)
            for k, v in summary['timeout'].items():
                print('timeout %s: NO_LINK %d/%d, recovered %d/%d, Pi detect ms %s, MCU silence ms %s' % (
                    k, v['no_link_declared'], v['repeats'], v['recovered_after_resume'], v['repeats'],
                    v['pi_detect_ms_last_send_to_no_link_status'],
                    v['mcu_silence_ms_accept_to_action']))
    summary['mcu_rejections_seen'] = bench.rejections
    summary['pi_rx_frames'] = link.rx_frames
    summary['pi_rx_wire_errors'] = link.rx_wire_errors
    summary['mcu_boot_ids'] = ['%016x' % b for b in link.mcu_boot_ids]
    summary['mcu_reboots_seen'] = max(0, len(link.mcu_boot_ids) - 1)
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print('MCU rejections %s; Pi RX wire errors %d; MCU boot IDs %s' % (
        bench.rejections, link.rx_wire_errors, summary['mcu_boot_ids']))
    print(summary['verdict'])
    return 2 if 'aborted' in summary else 0


def fmt(x):
    return '-' if x is None else '%.0f' % x


if __name__ == '__main__':
    sys.exit(main())
