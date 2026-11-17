"""Binary and text (CSV) codec for the WBR log format v1."""
import json
import struct

from . import spec


def _crc_table():
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _crc_table()


def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
    Check value for b'123456789' is 0x29B1."""
    t = _CRC_TABLE
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ t[((crc >> 8) ^ byte) & 0xFF]
    return crc


class Record:
    __slots__ = ("name", "seq", "t_us", "fields")

    def __init__(self, name, t_us, fields, seq=0):
        self.name, self.t_us, self.fields, self.seq = name, t_us, fields, seq

    def __repr__(self):
        return f"Record({self.name}, t_us={self.t_us}, seq={self.seq})"


def _payload(name, fields):
    type_id, fmt, names = spec.RECORDS[name]
    if name == "META":
        return json.dumps(fields["json"], separators=(",", ":")).encode("utf-8")
    if name == "CTRL":
        vals = [fields[k] for k in spec.CTRL_HEAD_FIELDS] + [fields[k] for k in spec.CTRL_FIELDS]
        return fmt.pack(*vals)
    if name == "EST":
        return fmt.pack(*[fields[k] for k in spec.EST_FIELDS])
    return fmt.pack(*[fields[k] for k in names])


def encode(record):
    type_id = spec.RECORDS[record.name][0]
    payload = _payload(record.name, record.fields)
    if len(payload) > spec.MAX_PAYLOAD:
        raise ValueError("payload too large")
    head = spec.FRAME_HEAD.pack(spec.MAGIC, type_id, len(payload),
                                record.seq & 0xFFFF, record.t_us & 0xFFFFFFFF)
    crc = crc16_ccitt(head[2:] + payload)
    return head + payload + struct.pack("<H", crc)


def _unpack(name, payload):
    type_id, fmt, names = spec.RECORDS[name]
    if name == "META":
        return {"json": json.loads(payload.decode("utf-8"))}
    if len(payload) != fmt.size:
        raise ValueError("bad payload size")
    vals = fmt.unpack(payload)
    if name == "CTRL":
        keys = spec.CTRL_HEAD_FIELDS + spec.CTRL_FIELDS
    elif name == "EST":
        keys = spec.EST_FIELDS
    else:
        keys = names
    return dict(zip(keys, vals))


class DecodeStats:
    def __init__(self):
        self.records = 0
        self.crc_errors = 0
        self.skipped_bytes = 0
        self.unknown_types = 0
        self.seq_gaps = 0          # frames lost, from the global sequence number
        self.malformed_records = 0
        self.truncated_tails = 0

    def as_dict(self):
        return dict(self.__dict__)


def decode(data, stats=None):
    """Decode a byte string. Resynchronises on the magic after any error."""
    stats = stats or DecodeStats()
    out = []
    i, n = 0, len(data)
    last_seq = None
    hs = spec.FRAME_HEAD.size
    while i + spec.FRAME_OVERHEAD <= n:
        if data[i:i + 2] != spec.MAGIC:
            j = data.find(spec.MAGIC, i + 1)
            stats.skipped_bytes += (j if j >= 0 else n) - i
            if j < 0:
                i = n
                break
            i = j
            continue
        _, type_id, length, seq, t_us = spec.FRAME_HEAD.unpack_from(data, i)
        end = i + hs + length + 2
        if length > spec.MAX_PAYLOAD or end > n:
            if end > n and length <= spec.MAX_PAYLOAD:
                # A corrupted length can hide later complete frames. Search
                # again; a true truncated tail has no later valid frame.
                if data.find(spec.MAGIC, i + 2) < 0:
                    stats.truncated_tails += 1
                    break
            stats.skipped_bytes += 1
            i += 1
            continue
        crc_rx = struct.unpack_from("<H", data, end - 2)[0]
        if crc16_ccitt(data[i + 2:end - 2]) != crc_rx:
            stats.crc_errors += 1
            stats.skipped_bytes += 1
            i += 1
            continue
        if last_seq is not None:
            stats.seq_gaps += (seq - last_seq - 1) & 0xFFFF
        last_seq = seq
        name = spec.TYPE_TO_NAME.get(type_id)
        if name is None:
            stats.unknown_types += 1
            i = end
            continue
        try:
            fields = _unpack(name, data[i + hs:end - 2])
        except (ValueError, struct.error, json.JSONDecodeError):
            stats.malformed_records += 1
            i = end
            continue
        out.append(Record(name, t_us, fields, seq))
        stats.records += 1
        i = end
    if i < n:
        stats.skipped_bytes += n - i if data[i:i + 2] != spec.MAGIC else 0
    return out, stats


# ------------------------------------------------------------ text form
# One line per record:  NAME,t_us,seq,<fields in spec order>*HH
# HH = XOR of all bytes between the start of the line and '*' (NMEA style).
# bytes fields are hex; META carries JSON with commas escaped as hex.

def _xor(s):
    x = 0
    for c in s.encode("ascii"):
        x ^= c
    return x


def _field_list(name, fields):
    if name == "META":
        return [json.dumps(fields["json"], separators=(",", ":")).encode().hex()]
    if name == "CTRL":
        keys = spec.CTRL_HEAD_FIELDS + spec.CTRL_FIELDS
    elif name == "EST":
        keys = spec.EST_FIELDS
    else:
        keys = spec.RECORDS[name][2]
    vals = []
    for k in keys:
        v = fields[k]
        if isinstance(v, (bytes, bytearray)):
            vals.append(bytes(v).hex())
        elif isinstance(v, float):
            vals.append(repr(struct.unpack("<f", struct.pack("<f", v))[0]))
        else:
            vals.append(str(v))
    return vals


def to_text_line(record):
    body = ",".join([record.name, str(record.t_us), str(record.seq)] + _field_list(record.name, record.fields))
    return f"{body}*{_xor(body):02X}"


def from_text_lines(lines, stats=None):
    stats = stats or DecodeStats()
    out = []
    last_seq = None
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "*" not in line:
            stats.crc_errors += 1
            continue
        body, ck = line.rsplit("*", 1)
        try:
            if int(ck, 16) != _xor(body):
                stats.crc_errors += 1
                continue
        except ValueError:
            stats.crc_errors += 1
            continue
        parts = body.split(",")
        name = parts[0]
        if name not in spec.RECORDS:
            stats.unknown_types += 1
            continue
        try:
            record = _parse_text_parts(parts)
        except (ValueError, IndexError, KeyError, TypeError, struct.error, OverflowError):
            stats.malformed_records += 1
            continue
        if last_seq is not None:
            stats.seq_gaps += (record.seq - last_seq - 1) & 0xFFFF
        last_seq = record.seq
        out.append(record)
        stats.records += 1
    return out, stats


def _parse_text_parts(parts):
    name = parts[0]
    t_us, seq, vals = int(parts[1]), int(parts[2]), parts[3:]
    if not 0 <= t_us < 2**32 or not 0 <= seq < 2**16:
        raise ValueError("timestamp or sequence out of range")
    if name == "META":
        if len(vals) != 1:
            raise ValueError("META requires one JSON field")
        fields = {"json": json.loads(bytes.fromhex(vals[0]).decode())}
    else:
        if name == "CTRL":
            keys = spec.CTRL_HEAD_FIELDS + spec.CTRL_FIELDS
            conv = [int] * 4 + [float] * 32
        elif name == "EST":
            keys, conv = spec.EST_FIELDS, [float] * 12
        else:
            keys = spec.RECORDS[name][2]
            fmt = spec.RECORDS[name][1].format.lstrip("<")
            conv = []
            for ch in _expand(fmt):
                conv.append(bytes.fromhex if ch == "s" else (float if ch in "fd" else int))
        if len(vals) != len(keys):
            raise ValueError("wrong field count")
        fields = {k: c(v) for k, c, v in zip(keys, conv, vals)}
    for key, width in (("data", 8), ("label", 12)):
        if key in fields and len(fields[key]) != width:
            raise ValueError("wrong byte field width")
    # Apply the same numeric bounds and float32 conversion as binary.
    fields = _unpack(name, _payload(name, fields))
    return Record(name, t_us, fields, seq)


def _expand(fmt):
    """'6hH' -> ['h']*6+['H']; '12s' -> ['s']."""
    out, num = [], ""
    for ch in fmt:
        if ch.isdigit():
            num += ch
            continue
        count = int(num) if num else 1
        out.extend([ch] if ch == "s" else [ch] * count)
        num = ""
    return out
