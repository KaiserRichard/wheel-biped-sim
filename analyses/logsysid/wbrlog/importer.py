"""Import a WBR log (binary or text) into Studio-named channels in SI units."""
import bisect
import math
import os

from . import codec, dm, spec

G0 = 9.80665
D2R = math.pi / 180.0
_AX = ("x", "y", "z")
_X_NAMES = ("axle_x", "axle_v", "leg_minus_pitch", "leg_minus_pitch_rate", "pitch", "pitch_rate")


def _channel_registry():
    """name -> (unit, Studio source, real-log source). One row per Studio signal.

    Studio sources: M13 packet wire (m13_sensor_block.m, 20 values), the
    controller output y (motion_control_eval.m, 23 values), the estimator
    diagnostic (m13_packet_update.m, 11 values) and the scorer truth columns
    (m13_score_prefix.m). 'none' marks a side with no counterpart.
    """
    reg = {}
    for i, a in enumerate(_AX):
        reg[f"imu.gyro_S.{a}"] = ("rad/s", f"wire[{2+i}] gyro_S", "IMU.g" + a + " * range")
        reg[f"imu.accel_S.{a}"] = ("m/s^2", f"wire[{5+i}] specific force S", "IMU.a" + a + " * range")
    for i, ax in enumerate(spec.STUDIO_AXES):
        reg[f"enc.q.{ax}"] = ("rad, wrapped [-pi,pi)", f"wire[{9+i}]", "CAN_RX pos * sign + offset, wrapped")
        reg[f"enc.dq.{ax}"] = ("rad/s", f"wire[{15+i}]", "CAN_RX vel * sign")
    for i, ax in enumerate(spec.STUDIO_AXES):
        reg[f"ctrl.applied.{ax}"] = ("N*m", f"y[{1+i}] applied", "CAN_TX MIT torque * sign")
        reg[f"ctrl.request.{ax}"] = ("N*m", f"y[{7+i}] request", "CTRL tau_req_* / wheel_req_* * sign")
    for i, n in enumerate(_X_NAMES):
        unit = ("m", "m/s", "rad", "rad/s", "rad", "rad/s")[i]
        reg[f"ctrl.x.{n}"] = (unit, f"y[{13+i}] x", "CTRL native, LOG_SPEC 5.3")
    reg["ctrl.L.L"] = ("m", "y[19] L(1)", "CTRL L0_L")
    reg["ctrl.L.R"] = ("m", "y[20] L(2)", "CTRL L0_R")
    reg["ctrl.phi"] = ("rad", "y[21] phi", "CTRL pitch * pitch_sign")
    reg["ctrl.roll"] = ("rad", "y[22] roll", "CTRL roll * roll_sign")
    reg["ctrl.yaw"] = ("rad", "y[23] yaw", "CTRL yaw_total * yaw_sign, wrapped")
    for i, n in enumerate(("ready", "pitch", "roll", "yaw", "body_speed")):
        reg[f"est.{n}"] = (("1", "rad", "rad", "rad", "m/s")[i], f"diag[{1+i}]", f"EST.{n}")
    reg["est.L.L"] = ("m", "diag[6]", "EST.L_L")
    reg["est.L.R"] = ("m", "diag[7]", "EST.L_R")
    for i, a in enumerate(_AX):
        reg[f"est.gbias.{a}"] = ("rad/s", f"diag[{8+i}]", f"EST.gbias_{a}")
    reg["est.accel_bias"] = ("m/s^2", "diag[11] x(2)", "EST.accel_bias")
    # Studio-only (truth) and real-only channels.
    reg["truth.pitch"] = ("rad", "scorer truth", "none")
    reg["truth.speed"] = ("m/s", "scorer truth", "none")
    reg["truth.L.L"] = ("m", "scorer truth", "none")
    reg["truth.L.R"] = ("m", "scorer truth", "none")
    for ax in spec.STUDIO_AXES:
        reg[f"motor.tau_fb.{ax}"] = ("N*m", "none (M09 lag output is internal)", "CAN_RX tau * sign")
        reg[f"motor.t_mos.{ax}"] = ("degC", "none", "CAN_RX D6")
        reg[f"motor.t_rotor.{ax}"] = ("degC", "none", "CAN_RX D7")
        reg[f"motor.err.{ax}"] = ("code", "none", "CAN_RX D0 >> 4")
    reg["imu.temp"] = ("degC", "none", "IMU_TEMP")
    reg["ctrl.imu_age"] = ("s", "estimator_processing_ticks*tick (1 ms)", "CTRL t - imu_t_us_used")
    reg["timing.loop_period"] = ("s", "Ts = 0.002 (control_params.m)", "TIMING")
    reg["timing.exec"] = ("s", "none", "TIMING")
    reg["timing.log_drops"] = ("count", "none", "TIMING")
    return reg


CHANNELS = _channel_registry()


class Series:
    __slots__ = ("t", "y")

    def __init__(self, t=None, y=None):
        self.t = t if t is not None else []
        self.y = y if y is not None else []

    def append(self, t, y):
        self.t.append(t)
        self.y.append(y)

    def __len__(self):
        return len(self.t)


class Dataset:
    def __init__(self, meta=None):
        self.meta = meta or {}
        self.ch = {}
        self.stats = {}
        self.events = []

    def add(self, name, t, y):
        s = self.ch.get(name)
        if s is None:
            s = self.ch[name] = Series()
        s.append(t, y)

    def names(self):
        return sorted(self.ch)

    def sort(self):
        for s in self.ch.values():
            if any(s.t[i] > s.t[i + 1] for i in range(len(s.t) - 1)):
                pairs = sorted(zip(s.t, s.y))
                s.t = [p[0] for p in pairs]
                s.y = [p[1] for p in pairs]


def wrap_pi(a):
    """Map to [-pi, pi), the Studio encoder_wrap range."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def unwrap_us(stamps):
    """u32 microsecond stamps -> monotonic-ish seconds. Small reorderings allowed."""
    out, offset, last = [], 0, None
    for s in stamps:
        if last is not None:
            d = s - last
            if d < -(1 << 31):
                offset += 1 << 32
            elif d > (1 << 31):
                offset -= 1 << 32
        last = s
        out.append((s + offset) * 1e-6)
    return out


def read_records(path_or_bytes):
    if isinstance(path_or_bytes, (bytes, bytearray)):
        data = bytes(path_or_bytes)
    else:
        with open(path_or_bytes, "rb") as f:
            data = f.read()
    if spec.MAGIC in data:
        return codec.decode(data)
    return codec.from_text_lines(data.decode("ascii", "replace").splitlines())


def import_log(path_or_bytes, meta_override=None, t0="first"):
    """Return a Dataset. t0='first' puts t=0 at the first record; 'none' keeps MCU time."""
    if t0 not in ("first", "none"):
        raise ValueError("t0 must be 'first' or 'none'")
    records, stats = read_records(path_or_bytes)
    meta = None
    for r in records:
        if r.name == "META":
            meta = r.fields["json"]
            break
    if meta_override is not None:
        meta = meta_override
    if meta is None:
        meta = spec.default_meta()
        stats_note = "no META record; default_meta() used (placeholders)"
    else:
        stats_note = "META override" if meta_override is not None else "META from log"
    if not isinstance(meta, dict) or meta.get("format_version") != spec.FORMAT_VERSION:
        raise ValueError("unsupported or missing META format_version")
    ds = Dataset(meta)
    ds.stats = stats.as_dict()
    ds.stats["meta_source"] = stats_note

    times = unwrap_us([r.t_us for r in records])
    origin = times[0] if (records and t0 == "first") else 0.0
    ds.stats["t_origin_mcu_s"] = origin

    imu = meta["imu"]
    gscale = imu["gyro_range_dps"] / 32768.0 * D2R
    ascale = imu["accel_range_g"] / 32768.0 * G0
    R = imu["R_BS"]
    tx_map, rx_map = {}, {}
    for m in meta["motors"]:
        tx_map[(m["bus"], m["can_id"])] = m
        rx_map[(m["bus"], m["can_id"] & 0xF)] = m
    cm = meta["ctrl_map"]
    mirror = -1.0 if cm.get("mirror_left", True) else 1.0
    hip_order = cm.get("hip_order", ["R_hip3", "R_hip4", "L_hip3", "L_hip4"])
    sign_of = {m["axis"]: m["sign"] for m in meta["motors"]}

    for r, tt in zip(records, times):
        t = tt - origin
        f = r.fields
        n = r.name
        if n in ("CAN_TX", "CAN_RX") and (f["dlc"] != 8 or f["flags"] != 0):
            ds.stats["unsupported_can"] = ds.stats.get("unsupported_can", 0) + 1
            continue
        if n == "IMU":
            g = (f["gx"] * gscale, f["gy"] * gscale, f["gz"] * gscale)
            a = (f["ax"] * ascale, f["ay"] * ascale, f["az"] * ascale)
            for i, ax in enumerate(_AX):
                ds.add(f"imu.gyro_S.{ax}", t, g[i])
                ds.add(f"imu.accel_S.{ax}", t, a[i])
                ds.add(f"imu.gyro_B.{ax}", t, sum(R[i][k] * g[k] for k in range(3)))
                ds.add(f"imu.accel_B.{ax}", t, sum(R[i][k] * a[k] for k in range(3)))
            ds.add("imu.seq", t, f["imu_seq"])
        elif n == "IMU_TEMP":
            ds.add("imu.temp", t, f["temp_raw"] * imu["temp_scale"] + imu["temp_offset_c"])
        elif n == "CAN_TX":
            if f["data"][:7] == b"\xff" * 7 and f["data"][7] in (0xFC, 0xFD, 0xFE):
                ds.stats["motor_mode_commands"] = ds.stats.get("motor_mode_commands", 0) + 1
                continue
            m = tx_map.get((f["bus"], f["can_id"]))
            if m is None:
                ds.stats["unmapped_tx"] = ds.stats.get("unmapped_tx", 0) + 1
                continue
            c = dm.unpack_mit_cmd(f["data"], m["pmax"], m["vmax"], m["tmax"])
            ds.add(f"ctrl.applied.{m['axis']}", t, m["sign"] * c["t"])
            ds.add(f"cmd.kp.{m['axis']}", t, c["kp"])
            ds.add(f"cmd.kd.{m['axis']}", t, c["kd"])
            ds.add(f"cmd.v.{m['axis']}", t, m["sign"] * c["v"])
        elif n == "CAN_RX":
            m = rx_map.get((f["bus"], f["data"][0] & 0xF))
            if m is None:
                ds.stats["unmapped_rx"] = ds.stats.get("unmapped_rx", 0) + 1
                continue
            fb = dm.unpack_feedback(f["data"], m["pmax"], m["vmax"], m["tmax"])
            ax, s = m["axis"], m["sign"]
            q = s * fb["pos"] + m["offset_rad"]
            ds.add(f"enc.q.{ax}", t, wrap_pi(q))
            ds.add(f"enc.q_cont.{ax}", t, q)
            ds.add(f"enc.dq.{ax}", t, s * fb["vel"])
            ds.add(f"motor.tau_fb.{ax}", t, s * fb["tau"])
            ds.add(f"motor.t_mos.{ax}", t, fb["t_mos"])
            ds.add(f"motor.t_rotor.{ax}", t, fb["t_rotor"])
            ds.add(f"motor.err.{ax}", t, fb["err"])
        elif n == "CTRL":
            _ctrl_to_studio(ds, t, f, cm, mirror, hip_order, sign_of)
            ds.add("ctrl.imu_t_used_us", t, f["imu_t_us_used"])
            ds.add("ctrl.tick", t, f["ctrl_tick"])
        elif n == "EST":
            ps, rs, ys = cm["pitch_sign"], cm["roll_sign"], cm["yaw_sign"]
            ds.add("est.ready", t, f["ready"])
            ds.add("est.pitch", t, ps * f["pitch"])
            ds.add("est.roll", t, rs * f["roll"])
            ds.add("est.yaw", t, wrap_pi(ys * f["yaw"]) if math.isfinite(f["yaw"]) else f["yaw"])
            ds.add("est.body_speed", t, cm["x_sign"] * f["body_speed"])
            ds.add("est.L.L", t, f["L_L"])
            ds.add("est.L.R", t, f["L_R"])
            for ax in _AX:
                ds.add(f"est.gbias.{ax}", t, f[f"gbias_{ax}"])
            ds.add("est.accel_bias", t, f["accel_bias"])
        elif n == "TIMING":
            ds.add("timing.loop_period", t, f["loop_period_us"] * 1e-6)
            ds.add("timing.exec", t, f["exec_us"] * 1e-6)
            ds.add("timing.log_drops", t, f["log_drop_count"])
            ds.add("timing.can_tx_err", t, f["can_tx_err"])
        elif n == "SYNC":
            ds.add("sync.count", t, f["sync_count"])
            ds.add("sync.host_time_ns", t, f["host_time_ns"])
        elif n == "EVENT":
            ds.events.append((t, f["code"], f["arg"], f["label"].rstrip(b"\0").decode("ascii", "replace")))
    _imu_age(ds, records, times, origin)
    ds.sort()
    return ds


def _imu_age(ds, records, times, origin):
    """ctrl.imu_age = CTRL time - time of the IMU sample it used (both MCU clock)."""
    for r, tt in zip(records, times):
        if r.name != "CTRL" or r.fields["imu_t_us_used"] == 0:
            continue  # 0 = no IMU sample used yet (LOG_SPEC 3.6)
        # Modular subtraction remains correct after multiple full wraps,
        # even when the referenced IMU record was dropped from the log.
        age = ((r.t_us - r.fields["imu_t_us_used"]) & 0xFFFFFFFF) * 1e-6
        ds.add("ctrl.imu_age", tt - origin, age)


def _ctrl_to_studio(ds, t, f, cm, mirror, hip_order, sign_of):
    ps = cm["pitch_sign"]
    asg = cm["alpha_sign"]
    xs = cm["x_sign"]
    phi = ps * f["pitch"]
    prate = ps * f["pitch_gyro"]
    alpha_c = asg * 0.5 * (f["alpha_R"] + mirror * f["alpha_L"]) + cm.get("alpha_offset_rad", 0.0)
    dalpha_c = asg * 0.5 * (f["d_alpha_R"] + mirror * f["d_alpha_L"])
    x = (xs * f["x_filter"], xs * f["v_filter"], alpha_c - phi, dalpha_c - prate, phi, prate)
    for name, v in zip(_X_NAMES, x):
        ds.add(f"ctrl.x.{name}", t, v)
    ds.add("ctrl.L.L", t, f["L0_L"])
    ds.add("ctrl.L.R", t, f["L0_R"])
    ds.add("ctrl.phi", t, phi)
    ds.add("ctrl.roll", t, cm["roll_sign"] * f["roll"])
    ds.add("ctrl.yaw", t, wrap_pi(cm["yaw_sign"] * f["yaw_total"]))
    req = dict(zip(hip_order, (f["tau_req_R0"], f["tau_req_R1"], f["tau_req_L0"], f["tau_req_L1"])))
    req["R_wheel"] = f["wheel_req_R"]
    req["L_wheel"] = f["wheel_req_L"]
    for ax, v in req.items():
        ds.add(f"ctrl.request.{ax}", t, sign_of.get(ax, 1) * v)
    for k in ("F0_R", "F0_L", "Tp_R", "Tp_L", "leg_set", "turn_T", "roll_out", "theta_R", "theta_L"):
        ds.add(f"native.{k}", t, f[k])


# ------------------------------------------------------------- export
def interp(series, tq, method="linear"):
    """Sample a Series at times tq. 'zoh' = last value at or before tq (causal)."""
    t, y = series.t, series.y
    if method not in ("linear", "zoh", "angle"):
        raise ValueError("method must be linear, zoh or angle")
    if not t:
        return [float("nan") for _ in tq]
    out = []
    n = len(t)
    for q in tq:
        i = bisect.bisect_right(t, q)
        if method == "zoh":
            out.append(y[i - 1] if i > 0 else float("nan"))
            continue
        if i == 0:
            out.append(y[0] if q == t[0] else float("nan"))
        elif i == n:
            out.append(y[-1] if q == t[-1] else float("nan"))
        else:
            t0, t1 = t[i - 1], t[i]
            w = (q - t0) / (t1 - t0) if t1 > t0 else 0.0
            if q == t[i - 1]:
                out.append(y[i - 1])
            elif method == "angle":
                out.append(wrap_pi(y[i - 1] + w * wrap_pi(y[i] - y[i - 1])))
            else:
                out.append(y[i - 1] + w * (y[i] - y[i - 1]))
    return out


def write_wide_csv(ds, path, names, grid, method="zoh", header_meta=None):
    cols = [interp(ds.ch[n], grid, method) for n in names]
    with open(path, "w", newline="") as f:
        for k, v in (header_meta or {}).items():
            f.write(f"# {k}: {v}\n")
        f.write("# resample: " + method + "\n")
        f.write("t_s," + ",".join(names) + "\n")
        for i, tq in enumerate(grid):
            f.write(repr(tq) + "," + ",".join(repr(c[i]) for c in cols) + "\n")


def write_long_csv(ds, path):
    """All channels at their native timestamps: t_s,channel,value."""
    with open(path, "w", newline="") as f:
        f.write("t_s,channel,value\n")
        for n in ds.names():
            s = ds.ch[n]
            for t, y in zip(s.t, s.y):
                f.write(f"{t!r},{n},{y!r}\n")


def file_sha256(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()
