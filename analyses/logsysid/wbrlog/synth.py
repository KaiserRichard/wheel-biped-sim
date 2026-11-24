"""Write a log exactly as the planned firmware would, from Studio-side signals.

Used by the round-trip tests and the rehearsal. The firmware timing follows
DM-jump (SOURCE chassisR_task.c lines 79-84): one 3 ms cycle per side, three
sequential MIT sends 1 ms apart; the feedback frame comes back after a
round-trip time; the IMU runs at 1 kHz on its own data-ready interrupt.
"""
import math
import random
import copy

from . import codec, dm, spec

_AX = ("x", "y", "z")


def _i16(x):
    return max(-32768, min(32767, int(round(x))))


class LogWriter:
    def __init__(self):
        self.chunks = []
        self.seq = 0
        self.records = []

    def add(self, name, t_us, fields):
        rec = codec.Record(name, int(t_us) & 0xFFFFFFFF, fields, 0)
        self.records.append((int(t_us), len(self.records), rec))

    def _ordered(self):
        # Records leave the ring buffer in MCU-time order; seq counts that order.
        self.records.sort(key=lambda p: (p[0], p[1]))
        for k, (_, _, r) in enumerate(self.records):
            r.seq = k & 0xFFFF
        return [r for _, _, r in self.records]

    def to_bytes(self):
        return b"".join(codec.encode(r) for r in self._ordered())

    def to_text(self):
        return "\n".join(codec.to_text_line(r) for r in self._ordered()) + "\n"


def synth_log(duration_s, meta, sig, t0_us=1_000_000, imu_phase_us=137,
              rtt_us=250, cycle_us=3000, send_gap_us=1000, jitter_us=0, seed=1,
              est=None, ctrl=None, events=()):
    """Return (LogWriter, info).

    sig: callables of Studio time t (s):
      sig['gyro_S'](t)->3, sig['accel_S'](t)->3,
      sig['q'](axis,t), sig['dq'](axis,t), sig['tau_fb'](axis,t), sig['applied'](axis,t)
    est(t)->dict of EST_FIELDS, ctrl(t)->dict of CTRL_FIELDS (optional).
    MCU time = t0_us + 1e6*t. A truth info dict is returned for the tests.
    """
    rng = random.Random(seed)
    meta = copy.deepcopy(meta)
    meta["measurement_domain"] = "synthetic"
    meta["provenance"] = "measurement_domain=synthetic; wbrlog.synth"
    w = LogWriter()
    imu = meta["imu"]
    gq = imu["gyro_range_dps"] / 32768.0 * math.pi / 180.0
    aq = imu["accel_range_g"] / 32768.0 * 9.80665
    w.add("META", t0_us - 10, {"json": meta})
    end_us = t0_us + int(duration_s * 1e6)
    for t_ev, code, label in events:
        w.add("EVENT", t0_us + int(round(t_ev * 1e6)),
              {"code": code, "arg": 0, "label": label.encode("ascii")[:12].ljust(12, b"\0")})

    # IMU at 1 kHz
    k = 0
    imu_times = []
    t_us = t0_us + imu_phase_us
    while t_us < end_us:
        t = (t_us - t0_us) * 1e-6
        g, a = sig["gyro_S"](t), sig["accel_S"](t)
        w.add("IMU", t_us, {"gx": _i16(g[0] / gq), "gy": _i16(g[1] / gq), "gz": _i16(g[2] / gq),
                            "ax": _i16(a[0] / aq), "ay": _i16(a[1] / aq), "az": _i16(a[2] / aq),
                            "imu_seq": k & 0xFFFF})
        imu_times.append(t_us)
        k += 1
        t_us = t0_us + imu_phase_us + k * 1000
    # 1 Hz SYNC and temperature
    s = 0
    while t0_us + s * 1_000_000 < end_us:
        w.add("SYNC", t0_us + s * 1_000_000, {"sync_count": s, "source": 0, "host_time_ns": 0})
        w.add("IMU_TEMP", t0_us + s * 1_000_000 + 1, {"temp_raw": 40})
        s += 1

    by_bus = {}
    for m in meta["motors"]:
        by_bus.setdefault(m["bus"], []).append(m)
    cycle = 0
    tx_log = []
    import bisect
    while True:
        c_us = t0_us + cycle * cycle_us
        if c_us >= end_us:
            break
        t = (c_us - t0_us) * 1e-6
        # CTRL / EST / TIMING at the start of the cycle
        i = bisect.bisect_right(imu_times, c_us) - 1
        if i < 0:
            # DM-jump waits for INS.ins_flag before the first control cycle
            # (chassisR_task.c line 62); no CTRL, CAN or TIMING until then.
            cycle += 1
            continue
        imu_used = imu_times[i]
        if ctrl is not None:
            f = dict(ctrl(t))
            f.update({"ctrl_tick": cycle, "imu_t_us_used": imu_used & 0xFFFFFFFF,
                      "imu_seq_used": max(i, 0) & 0xFFFF, "flags": 0})
            w.add("CTRL", c_us, f)
        if est is not None:
            w.add("EST", c_us + 2, est(t))
        w.add("TIMING", c_us + 3, {"loop_period_us": cycle_us, "exec_us": 180,
                                   "log_drop_count": 0, "can_tx_err": 0})
        for bus, motors in by_bus.items():
            for j, m in enumerate(motors):
                j_us = rng.randint(0, jitter_us) if jitter_us else 0  # TX never precedes the compute
                tx_us = c_us + 50 + j * send_gap_us + j_us
                ttx = (tx_us - t0_us) * 1e-6
                ax, sg = m["axis"], m["sign"]
                data = dm.pack_mit_cmd(0, 0, 0, 0, sg * sig["applied"](ax, ttx), m["pmax"], m["vmax"], m["tmax"])
                w.add("CAN_TX", tx_us, {"bus": bus, "flags": 0, "can_id": m["can_id"], "dlc": 8, "data": data})
                tx_log.append((ax, tx_us))
                rx_us = tx_us + rtt_us
                trx = (rx_us - t0_us) * 1e-6
                pos_raw = sg * (sig["q"](ax, trx) - m["offset_rad"])
                fb = dm.pack_feedback(m["can_id"], 1, pos_raw, sg * sig["dq"](ax, trx),
                                      sg * sig["tau_fb"](ax, trx), 35, 40, m["pmax"], m["vmax"], m["tmax"])
                w.add("CAN_RX", rx_us, {"bus": bus, "flags": 0, "can_id": 0x00, "dlc": 8, "data": fb})
        cycle += 1
    info = {"t0_us": t0_us, "tx_log": tx_log, "imu_times": imu_times}
    return w, info
