"""JSON UDP input and sender: python -m sim.twin3d_cmd.udp send --v 0.3."""

import argparse
from dataclasses import dataclass
import json
import socket
import time

from .commands import Command, finite

PORT = 9876  # ASSUMED UDP port.
MAX_AGE = 0.5  # ASSUMED seconds, sender wall time must be synchronized.
FUTURE_SKEW = 0.1  # ASSUMED allowed future wall-clock skew, s.
MAX_PACKET = 4096  # ASSUMED byte limit, oversized datagrams are rejected.
DRAIN_LIMIT = 256  # ASSUMED per-tick work cap to avoid starving physics.


@dataclass(frozen=True)
class Packet:
    seq: int
    t: float
    command: Command


def decode(payload):
    try:
        if len(payload) > MAX_PACKET:
            raise ValueError('oversized packet')
        row = json.loads(payload)
        if not isinstance(row, dict) or set(row) != {'seq', 't', 'v', 'yaw_rate', 'jump', 'src'}:
            raise ValueError('expected exact protocol fields')
        if type(row['seq']) is not int or row['seq'] < 0:
            raise ValueError('seq must be nonnegative integer')
        return Packet(row['seq'], finite(row['t']),
                      Command(row['v'], row['yaw_rate'], row['jump'], row['src']))
    except (TypeError, UnicodeError, json.JSONDecodeError, OverflowError) as error:
        raise ValueError('invalid UDP command') from error


def encode(packet):
    c = packet.command
    payload = json.dumps(dict(seq=packet.seq, t=packet.t, v=c.v_ref,
                              yaw_rate=c.yaw_rate, jump=c.jump, src=c.src),
                         allow_nan=False, separators=(',', ':')).encode('utf-8')
    decode(payload)  # Apply exactly the same validation on transmit.
    return payload


class Freshness:
    """Sequence and wall-clock checks per src; rejected packets never refresh WD.

    A sender restart needs a new src or continued sequence. No automatic sequence
    reset on silence. t must also be nondecreasing for each accepted source.
    """
    def __init__(self, max_age=MAX_AGE, future_skew=FUTURE_SKEW):
        self.max_age, self.future_skew = finite(max_age), finite(future_skew)
        if self.max_age <= 0 or self.future_skew < 0:
            raise ValueError('invalid freshness policy')
        self.last = {}

    def accept(self, packet, now):
        age = finite(now)-packet.t
        old = self.last.get(packet.command.src)
        if age > self.max_age or age < -self.future_skew:
            return False
        if old and (packet.seq <= old[0] or packet.t < old[1]):
            return False
        self.last[packet.command.src] = packet.seq, packet.t
        return True


class Listener:
    def __init__(self, host='localhost', port=PORT, freshness=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((host, port))
            self.sock.setblocking(False)
        except BaseException:
            self.sock.close()
            raise
        self.address = self.sock.getsockname()
        self.freshness = freshness if freshness is not None else Freshness()
        self.accepted = self.rejected = 0

    def drain(self, now=None):
        """Accepted packets in socket receive order (last-writer-wins)."""
        packets = []
        for _ in range(DRAIN_LIMIT):
            try:
                payload, _ = self.sock.recvfrom(MAX_PACKET+1)
            except BlockingIOError:
                break
            try:
                packet = decode(payload)
                if not self.freshness.accept(packet, time.time() if now is None else now):
                    raise ValueError('sequence or timestamp rejected')
            except ValueError:
                self.rejected += 1
                continue
            self.accepted += 1
            packets.append(packet)
        return packets

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='action', required=True)
    send = sub.add_parser('send')
    send.add_argument('--host', default='localhost')
    send.add_argument('--port', type=int, default=PORT)
    send.add_argument('--v', type=float, default=0.0)
    send.add_argument('--yaw-rate', type=float, default=0.0)
    send.add_argument('--jump', action='store_true')
    send.add_argument('--src', default='mac-cli')
    send.add_argument('--seconds', type=float, default=0.0, help='ASSUMED default one packet')
    send.add_argument('--hz', type=float, default=20.0, help='ASSUMED heartbeat Hz when --seconds > 0')
    a = p.parse_args(argv)
    if finite(a.seconds) < 0 or finite(a.hz) <= 0:
        p.error('seconds must be nonnegative; hz must be positive')
    deadline, seq, first = time.monotonic()+a.seconds, time.time_ns(), True
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        while True:
            c = Command(a.v, a.yaw_rate, a.jump and first, a.src)
            sock.sendto(encode(Packet(seq, time.time(), c)), (a.host, a.port))
            seq, first = seq+1, False
            if time.monotonic()+1/a.hz > deadline:
                break
            time.sleep(1/a.hz)


if __name__ == '__main__':
    main()
