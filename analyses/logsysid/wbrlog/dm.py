"""Damiao (DM) MIT-mode CAN frames.

Command layout P16|V12|Kp12|Kd12|T12 (SOURCE h7-advisory-fw docs/porting.md
line 71, "Classic MIT-Cheetah ... packing"; DM-jump calls mit_ctrl with
p=v=kp=kd=0 and a torque, chassisR_task.c lines 79-84).

Feedback layout (INFERRED from the public Damiao manual, not in the local
sources): D0 = ID (low nibble) | ERR << 4, D1-D2 = POS (16 bit),
D3 + D4[7:4] = VEL (12 bit), D4[3:0] + D5 = T (12 bit), D6 = T_MOS (degC),
D7 = T_Rotor (degC). Check against one captured frame on robot day (R0).
The ranges PMAX/VMAX/TMAX are set inside each motor and must be logged in META.
"""

KP_MAX, KD_MAX = 500.0, 5.0  # DM MIT gain ranges (INFERRED)


def float_to_uint(x, lo, hi, bits):
    x = min(max(x, lo), hi)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def uint_to_float(u, lo, hi, bits):
    return u * (hi - lo) / ((1 << bits) - 1) + lo


def lsb(lo, hi, bits):
    return (hi - lo) / ((1 << bits) - 1)


def pack_mit_cmd(p, v, kp, kd, t, pmax, vmax, tmax):
    pu = float_to_uint(p, -pmax, pmax, 16)
    vu = float_to_uint(v, -vmax, vmax, 12)
    kpu = float_to_uint(kp, 0.0, KP_MAX, 12)
    kdu = float_to_uint(kd, 0.0, KD_MAX, 12)
    tu = float_to_uint(t, -tmax, tmax, 12)
    return bytes([
        pu >> 8, pu & 0xFF,
        vu >> 4, ((vu & 0xF) << 4) | (kpu >> 8),
        kpu & 0xFF, kdu >> 4,
        ((kdu & 0xF) << 4) | (tu >> 8), tu & 0xFF,
    ])


def unpack_mit_cmd(d, pmax, vmax, tmax):
    pu = (d[0] << 8) | d[1]
    vu = (d[2] << 4) | (d[3] >> 4)
    kpu = ((d[3] & 0xF) << 8) | d[4]
    kdu = (d[5] << 4) | (d[6] >> 4)
    tu = ((d[6] & 0xF) << 8) | d[7]
    return {
        "p": uint_to_float(pu, -pmax, pmax, 16),
        "v": uint_to_float(vu, -vmax, vmax, 12),
        "kp": uint_to_float(kpu, 0.0, KP_MAX, 12),
        "kd": uint_to_float(kdu, 0.0, KD_MAX, 12),
        "t": uint_to_float(tu, -tmax, tmax, 12),
    }


def pack_feedback(motor_id, err, pos, vel, tau, t_mos, t_rotor, pmax, vmax, tmax):
    pu = float_to_uint(pos, -pmax, pmax, 16)
    vu = float_to_uint(vel, -vmax, vmax, 12)
    tu = float_to_uint(tau, -tmax, tmax, 12)
    return bytes([
        (motor_id & 0xF) | ((err & 0xF) << 4),
        pu >> 8, pu & 0xFF,
        vu >> 4, ((vu & 0xF) << 4) | (tu >> 8), tu & 0xFF,
        int(round(t_mos)) & 0xFF, int(round(t_rotor)) & 0xFF,
    ])


def unpack_feedback(d, pmax, vmax, tmax):
    pu = (d[1] << 8) | d[2]
    vu = (d[3] << 4) | (d[4] >> 4)
    tu = ((d[4] & 0xF) << 8) | d[5]
    return {
        "id": d[0] & 0xF,
        "err": d[0] >> 4,
        "pos": uint_to_float(pu, -pmax, pmax, 16),
        "vel": uint_to_float(vu, -vmax, vmax, 12),
        "tau": uint_to_float(tu, -tmax, tmax, 12),
        "t_mos": float(d[6]),
        "t_rotor": float(d[7]),
    }
