#!/usr/bin/env python3
"""Forward /terrain_advisory messages to the bench Nucleo link."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.obstacle_decision import advisory_codec
from tools.link_proto.py import link_proto as lp

HERE = Path(__file__).resolve().parent
spec_path = HERE / 'link_bench.py'
import importlib.util
spec = importlib.util.spec_from_file_location('link_bench', spec_path)
lb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lb)


def message_to_packet(data, session=lb.BENCH_SESSION, monotonic_ns=None):
    """Validate the eight-value ROS payload and encode a TerrainAdvisory.

    The topic has no camera timestamp or plane geometry fields, so these are
    zeroed. Sequence is carried through as the wire sequence.
    """
    if len(data) != 8:
        raise ValueError('expected exactly 8 values')
    try:
        values = [float(value) for value in data]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError('values must be numeric') from exc
    if not all(math.isfinite(value) for value in values):
        raise ValueError('all values must be finite')
    code, distance, height, width, confidence, plane_valid, sequence, _age_s = values
    if code < 0 or code > 3 or not code.is_integer():
        raise ValueError('code must be an integer in [0, 3]')
    if plane_valid not in (0.0, 1.0):
        raise ValueError('plane_valid must be 0 or 1')
    if not sequence.is_integer() or sequence < 0 or sequence > 0xFFFFFFFF:
        raise ValueError('sequence must be an integer in uint32 range')
    if distance < 0 or height < 0 or width < 0 or not 0 <= confidence <= 1:
        raise ValueError('distance, height, width, or confidence out of range')
    stamp = time.monotonic_ns() if monotonic_ns is None else int(monotonic_ns)
    packet = advisory_codec.encode(
        session, int(sequence), pi_monotonic_ns=stamp, frame_stamp_ns=0,
        advisory=int(code), confidence=confidence, distance_m=distance,
        height_m=height, width_m=width, plane_valid=int(plane_valid),
        plane_pitch_rad=0.0, plane_height_m=0.0)
    return int(sequence), packet


class Bridge:
    def __init__(self, link):
        self.link = link
        self.received = self.forwarded = self.dropped = self.echoed = 0
        self.pending = {}
        self.rtts_ns = []
        self.seq_base = (time.monotonic_ns() // 1_000_000) & 0x7FFFFFFF
        self.echo_results = {}

    def receive(self, data, logger):
        self.received += 1
        try:
            # The MCU only accepts a wire sequence newer than the last one it saw (also from
            # earlier runs), so offset the topic counter by this run's start time in ms.
            topic_seq = int(float(data[6])) if len(data) == 8 and math.isfinite(float(data[6])) else 0
            data = list(data); data[6] = float((self.seq_base + topic_seq) & 0xFFFFFFFF)
            sequence, packet = message_to_packet(data)
        except (ValueError, TypeError) as exc:
            self.dropped += 1
            logger.warning(f'dropped invalid /terrain_advisory: {exc}')
            return
        sent_ns = self.link.send(packet, mark=False)
        self.pending[sequence] = sent_ns
        self.forwarded += 1

    def poll_echoes(self):
        for received_ns, record in self.link.poll(0):
            if record.get('type') != lp.ADVISORY_ECHO:
                continue
            result = record.get('result', 0)
            self.echo_results[result] = self.echo_results.get(result, 0) + 1
            sent_ns = self.pending.pop(record.get('advisory_seq'), None)
            if sent_ns is not None:
                self.echoed += 1
                self.rtts_ns.append(received_ns - sent_ns)

    def drain_echoes(self, seconds=.15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.poll_echoes()
            time.sleep(min(.01, max(0.0, deadline - time.monotonic())))

    def write_summary(self, path):
        rtts_us = sorted(ns / 1000.0 for ns in self.rtts_ns)
        def percentile(p):
            return None if not rtts_us else rtts_us[max(0, math.ceil(p * len(rtts_us)) - 1)]
        summary = {
            'echo_result_codes': {str(k): v for k, v in sorted(self.echo_results.items())},
            'received': self.received, 'forwarded': self.forwarded,
            'dropped': self.dropped, 'echoed': self.echoed,
            'rtt_us': {'n': len(rtts_us), 'p50': percentile(.50), 'p99': percentile(.99)},
        }
        Path(path).write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument('--baud', type=int, default=460800)
    parser.add_argument('--summary', default='summary.json')
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
    from std_msgs.msg import Float64MultiArray

    link = lb.Link(args.port, args.baud)
    # The link layer normally discards through the first delimiter after open.
    # We own the startup boundary here, before any advisory is transmitted.
    link.dropping = False
    link.synced = True
    bridge = Bridge(link)
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                     reliability=QoSReliabilityPolicy.BEST_EFFORT)

    class TopicBridge(Node):
        def __init__(self):
            super().__init__('topic_to_nucleo')
            self.create_subscription(Float64MultiArray, '/terrain_advisory',
                                     self.on_message, qos)
            self.create_timer(.002, self.poll_echoes)
        def on_message(self, msg):
            bridge.receive(msg.data, self.get_logger())
        def poll_echoes(self):
            bridge.poll_echoes()

    rclpy.init()
    node = TopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.drain_echoes()
        bridge.write_summary(args.summary)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(link.fd)


if __name__ == '__main__':
    main()
