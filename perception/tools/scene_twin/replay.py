"""Pure Python replay logic. MEASURED means detector output, not ground truth."""
from bisect import bisect_right
from collections import deque
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import re
from statistics import median

# ASSUMED camera at body front, +x from wheel axle (metres).
CAMERA_OFFSET = 0.15
# ASSUMED drawing dimensions (metres), not obstacle measurements.
BOX_DEPTH, BOX_WIDTH, FALLBACK_HEIGHT, FALLBACK_DISTANCE = 0.20, 0.40, 0.01, 1.0
NAMES = ('NONE', 'JUMP_CANDIDATE', 'STOP_CANDIDATE', 'UNCERTAIN')
# ASSUMED display RGBA palette; advisory integer mapping is the packet protocol.
COLORS = ((0.15, 0.75, 0.25, 1), (0.15, 0.35, 1, 1),
          (0.95, 0.15, 0.12, 1), (0.5, 0.5, 0.5, 1))


def positive(value):
    return math.isfinite(value) and value > 0


@dataclass(frozen=True)
class Packet:
    seq: int
    advisory: int
    height: float
    distance: float
    stamp: int  # MEASURED frame_stamp_ns, never converted through float.


def read_packets(path):
    rows = []
    with Path(path).open(newline='') as f:
        for line, r in enumerate(csv.DictReader(f), 2):
            try:
                p = Packet(int(r['seq']), int(r['advisory']), float(r['height_m']),
                           float(r['distance_m']), int(r['frame_stamp_ns']))
                if p.advisory not in range(4) or p.stamp <= 0:
                    raise ValueError('invalid advisory or stamp')
                if rows and p.stamp <= rows[-1].stamp:
                    raise ValueError('frame stamps must increase strictly')
                rows.append(p)
            except (KeyError, ValueError) as e:
                raise ValueError(f'{path}:{line}: {e}') from e
    if not rows:
        raise ValueError(f'{path}: no packets')
    return rows


def bag_start(run):
    """MEASURED earliest rosbag log time from its read-only metadata."""
    path = Path(run) / 'camera-bag/metadata.yaml'
    if not path.exists():
        return None  # UNKNOWN bag origin; replay labels its packet-relative fallback.
    text = path.read_text()
    match = re.search(r'starting_time\x3a\s*nanoseconds_since_epoch\x3a\s*(\d+)', text)
    if not match:
        raise ValueError('camera-bag metadata has no starting_time')
    return int(match[1])


class Replay:
    def __init__(self, packets, bag_start_ns, motion='approach', window=3):
        if motion not in ('static', 'approach') or window < 1:
            raise ValueError('invalid motion or median window')
        if not packets:
            raise ValueError('no packets')
        self.packets, self.motion = packets, motion
        self.bag_origin_known = bag_start_ns is not None
        self.bag_start_ns = bag_start_ns if self.bag_origin_known else packets[0].stamp
        self.stamps = [p.stamp for p in packets]
        heights = [p.height for p in packets if p.advisory in (1, 2) and positive(p.height)]
        self.height = median(heights) if heights else FALLBACK_HEIGHT
        self.height_status = 'INFERRED detector median' if heights else 'ASSUMED placeholder'
        self.anchor = next((i for i, p in enumerate(packets) if positive(p.distance)), None)
        self.anchor_distance = packets[self.anchor].distance if self.anchor is not None else FALLBACK_DISTANCE
        # INFERRED box face: robot axle x=0 at first valid distance frame.
        self.face = self.anchor_distance + CAMERA_OFFSET
        self.distances, self.x = [], []
        history = deque(maxlen=window)  # ASSUMED trailing 3 valid samples by default.
        held = self.anchor_distance  # ASSUMED backfill until first valid distance.
        for p in packets:
            if positive(p.distance):
                history.append(p.distance)
                held = median(history)
            # Invalid rows HOLD the previous smoothed value; do not advance filter.
            self.distances.append(held)
            self.x.append(self.face - held - CAMERA_OFFSET if motion == 'approach' else 0.0)

    @property
    def duration(self):
        return (self.stamps[-1] - self.stamps[0]) / 1e9

    def index_at(self, t):
        # Integer offset preserves nanosecond epoch precision.
        return max(0, min(len(self.stamps)-1, bisect_right(self.stamps, self.stamps[0] + round(t*1e9))-1))

    def text(self, i):
        p = self.packets[i]
        t = (p.stamp-self.stamps[0])/1e9
        b = (p.stamp-self.bag_start_ns)/1e9
        valid = 'valid' if positive(p.distance) else 'HOLD / initial backfill'
        origin = 'bag-relative' if self.bag_origin_known else 'ASSUMED origin (bag UNKNOWN)'
        return (f't={t:.3f}s  {origin}={b:+.3f}s  {NAMES[p.advisory]}\n'
                f'frame_stamp_ns={p.stamp}  seq={p.seq}\n'
                f'MEASURED detector: height={p.height:.4f}m  distance={p.distance:.4f}m ({valid})\n'
                f'INFERRED camera gap={self.face-self.x[i]-CAMERA_OFFSET:.4f}m; motion={self.motion}\n'
                f'box height={self.height:.4f}m ({self.height_status})\n'
                'ASSUMED camera/body pose; pitch=roll=yaw=0; no IMU; visual replay only')
