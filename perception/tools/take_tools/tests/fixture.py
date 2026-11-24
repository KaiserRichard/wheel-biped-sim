"""Independent synthetic serializer; schema/layout references are in ../README.md."""
from decimal import Decimal
import sqlite3
import struct


class Writer:
    def __init__(self, endian='<'):
        self.endian = endian
        self.payload = bytearray()

    def number(self, fmt, value):
        size = struct.calcsize(fmt)
        self.payload.extend(b'\0' * (-len(self.payload) % size))
        self.payload.extend(struct.pack(self.endian + fmt, value))

    def string(self, value):
        raw = value.encode() + b'\0'
        self.number('I', len(raw))
        self.payload.extend(raw)

    def blob(self):
        return (b'\0\1\0\0' if self.endian == '<' else b'\0\0\0\0') + self.payload


def string(value, endian='<'):
    w = Writer(endian)
    w.string(value)
    return w.blob()


def header(w, stamp, frame):
    w.number('i', stamp // 1_000_000_000)
    w.number('I', stamp % 1_000_000_000)
    w.string(frame)


def image(stamp, frame, pixels, encoding='8UC1', endian='<', padded=False):
    w = Writer(endian)
    header(w, stamp, frame)
    w.number('I', 6)
    w.number('I', 8)
    w.string(encoding)
    w.number('B', 0)
    w.number('I', 10 if padded else 8)
    data = b''.join(pixels[y*8:y*8+8] + b'\xfe\xff' for y in range(6)) if padded else pixels
    w.number('I', len(data))
    w.payload.extend(data)
    return w.blob()


def imu(stamp, frame, vector, gyro, endian='<'):
    w = Writer(endian)
    header(w, stamp, frame)
    # Quaternion, orientation covariance, angular vector + covariance, accel + covariance.
    for values in ([0]*4, [-1]+[0]*8, vector if gyro else [0]*3,
                   [0]*9, [0]*3 if gyro else vector, [0]*9):
        for value in values:
            w.number('d', value)
    return w.blob()


def make(path, variant=None, endian='<', camera_stamps=None):
    con = sqlite3.connect(path)
    con.executescript('''
        CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT NOT NULL,type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,data BLOB NOT NULL);
        CREATE INDEX timestamp_idx ON messages(timestamp ASC);
    ''')
    names = [('ir1', 'Infrared_1', 'image'), ('ir2', 'Infrared_2', 'image'),
             ('gyro', 'Gyro_0', 'imu'), ('accel', 'Accel_0', 'imu')]
    expected = {name: [] for name, _, _ in names}
    con.execute('INSERT INTO topics VALUES(9,?, ?, ?, ?)', ('/file_version', 'std_msgs/msg/UInt32', 'cdr', ''))
    con.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(9,0,?)', (b'\0\1\0\0\4\0\0\0',))
    for i, (name, stream, family) in enumerate(names):
        prefix = f'/device_0/sensor_{0 if family == "image" else 1}/{stream}/{family}'
        kind = 'sensor_msgs/msg/Image' if family == 'image' else 'sensor_msgs/msg/Imu'
        con.execute('INSERT INTO topics VALUES(?,?,?,?,?)', (2*i+1, prefix+'/data', kind, 'cdr', ''))
        con.execute('INSERT INTO topics VALUES(?,?,?,?,?)', (2*i+2, prefix+'/metadata', 'std_msgs/msg/String', 'cdr', ''))
        count = 31 if family == 'image' else (201 if name == 'gyro' else 99)
        if family == 'image' and camera_stamps is not None:
            count = len(camera_stamps)
        period = 33_333_333 if family == 'image' else (5_000_000 if name == 'gyro' else 10_000_000)
        # Accel 10..990 ms, gyro 0..1000 ms: two excluded samples at each end.
        for j in range(count):
            if variant == 'missing_frame' and name == 'ir2' and j == 10:
                continue
            if variant == 'accel_gap' and name == 'accel' and j == 20:
                continue
            sdk = 1_234_567_891 + j*period + (10_000_000 if name == 'accel' else 0)
            if family == 'image' and camera_stamps is not None:
                sdk = camera_stamps[j]
            if variant == 'small_skew' and name == 'ir2' and j == 15:
                sdk += 500_000
            if variant == 'camera_backward' and name == 'ir1' and j == 10:
                sdk -= 2*period
            if variant == 'frame_mismatch' and name == 'ir2' and j == 15:
                frame_number = 1000
            else:
                frame_number = j+1
            if variant == 'backward' and name == 'gyro' and j == 50:
                sdk -= 8_000_000
            if variant == 'duplicate' and name == 'ir1' and j == 10:
                sdk -= period
            if variant == 'skew' and name == 'ir2' and j == 15:
                sdk += 2_000_000
            domain = 'Global Time' if variant == 'domain_change' and name == 'gyro' and j >= 50 else 'Hardware Clock'
            if variant == 'cross_domain' and name == 'accel':
                domain = 'System Time'
            if variant == 'unknown_domain' and name == 'ir1':
                domain = 'Other Clock'
            bag = 100_000_000_000 + j*period + (10_000_000 if name == 'accel' else 0)
            frame = {'ir1': 'Infrared1', 'ir2': 'Infrared2', 'gyro': 'Gyro', 'accel': 'Accel'}[name]
            vector = [float(j), float(2*j), float(-j)]
            pixels = bytes((k + j + i) % 256 for k in range(48))
            payload = image(bag, frame, pixels, 'rgb8' if variant == 'encoding' else ('mono8' if variant == 'mono8' else '8UC1'), endian, padded=True) if family == 'image' else imu(bag, frame, vector, name == 'gyro', endian)
            if variant == 'truncated' and name == 'ir1' and j == 0:
                payload = payload[:-3]
            if variant == 'encapsulation' and name == 'gyro' and j == 0:
                payload = b'\0\7\0\0' + payload[4:]
            con.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)', (2*i+1, bag, payload))
            text = (f'Frame number={frame_number};timestamp_domain={domain};'
                    f'system_time=1700000000000.000000;timestamp={Decimal(sdk)/1_000_000:.6f};'
                    'Sensor Timestamp=123;')
            if variant != 'missing_metadata' or name != 'ir1' or j != 0:
                con.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)', (2*i+2, bag, string(text, endian)))
            expected[name].append(dict(sdk_ns=sdk, pixels=pixels, vector=vector, bag_ns=bag))
    con.commit()
    con.close()
    return expected
