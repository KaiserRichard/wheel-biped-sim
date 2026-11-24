"""Standalone stdlib forwarding logic; importable without ROS or MuJoCo."""

import json
import math
import socket
import time

MAX_SPEED = 0.6  # ASSUMED m/s, same clamp as sim.twin3d_cmd.commands.
DEFAULT_PORT = 9876  # ASSUMED UDP port.


def number(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('expected finite number')
    return float(value)


def serialize(seq, stamp, v, yaw_rate, jump, src):
    if type(seq) is not int or seq < 0 or type(jump) is not bool:
        raise ValueError('invalid sequence or jump edge')
    if not isinstance(src, str) or not 1 <= len(src) <= 64:
        raise ValueError('src must contain 1..64 characters (ASSUMED)')
    return json.dumps(dict(seq=seq, t=number(stamp),
                           v=max(-MAX_SPEED, min(MAX_SPEED, number(v))),
                           yaw_rate=number(yaw_rate), jump=jump, src=src),
                      allow_nan=False, separators=(',', ':')).encode('utf-8')


class Forwarder:
    """Every ROS callback sends exactly one packet; no velocity replay timer.

    A jump callback carries the latest velocity only if that Twist is locally
    fresh. No Twist for 0.5 s -> jump packet uses v=0, not an expired setpoint.
    """
    def __init__(self, host, port=DEFAULT_PORT, src=None, sock=None,
                 wall=time.time, monotonic=time.monotonic):
        if not host or not 1 <= port <= 65535:  # SOURCE-SUPPORTED IP port range.
            raise ValueError('host and valid port required')
        self.address = host, port
        self.wall, self.monotonic = wall, monotonic
        self.src = src if src is not None else f'agv-ros-{time.time_ns()}'
        self.seq = 0
        self.v = self.yaw_rate = 0.0
        self.last_twist = None
        self.sock = sock if sock is not None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _send(self, v, yaw_rate, jump):
        payload = serialize(self.seq, self.wall(), v, yaw_rate, jump, self.src)
        self.sock.sendto(payload, self.address)
        self.seq += 1
        return payload

    def twist(self, v, yaw_rate):
        v, yaw_rate = number(v), number(yaw_rate)
        result = self._send(v, yaw_rate, False)
        self.v, self.yaw_rate, self.last_twist = v, yaw_rate, self.monotonic()
        return result

    def jump(self):
        fresh = self.last_twist is not None and self.monotonic()-self.last_twist < 0.5  # ASSUMED s.
        return self._send(self.v if fresh else 0.0, self.yaw_rate if fresh else 0.0, True)

    def close(self):
        self.sock.close()
