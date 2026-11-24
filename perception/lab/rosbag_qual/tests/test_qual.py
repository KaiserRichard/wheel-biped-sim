"""Offline qualification tests; no camera, serial, ROS graph, or network access."""
from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import struct
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from analyze_bag import main as analyze_main
from bag_core import analyze_database


REAL_DB3 = Path("./Desktop/Research/wbr-data/2026-10-14-d435i-ramtest15s/take/stereo_imu.db3")


def _align(data: bytearray, boundary: int) -> None:
    while (len(data) - 4) % boundary:
        data.append(0)


def _string(data: bytearray, value: str) -> None:
    raw = value.encode() + b"\0"
    _align(data, 4)
    data.extend(struct.pack("<I", len(raw)))
    data.extend(raw)


def _header(stamp_ns: int, frame: str = "camera_frame") -> bytearray:
    data = bytearray(b"\x00\x01\x00\x00")
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    data.extend(struct.pack("<iI", sec, nanosec))
    _string(data, frame)
    return data


def image(stamp_ns: int) -> bytes:
    data = _header(stamp_ns)
    _align(data, 4)
    data.extend(struct.pack("<II", 2, 2))
    _string(data, "mono8")
    data.extend(b"\0")
    _align(data, 4)
    data.extend(struct.pack("<II", 2, 4))
    data.extend(b"\x01\x02\x03\x04")
    return bytes(data)


def imu(stamp_ns: int) -> bytes:
    data = _header(stamp_ns)
    _align(data, 8)
    data.extend(struct.pack("<" + "d" * 4, 0.0, 0.0, 0.0, 1.0))
    data.extend(struct.pack("<" + "d" * 9, *([0.0] * 9)))
    data.extend(struct.pack("<ddd", 0.1, 0.2, 0.3))
    data.extend(struct.pack("<" + "d" * 9, *([0.0] * 9)))
    data.extend(struct.pack("<ddd", 0.0, 0.0, 9.81))
    data.extend(struct.pack("<" + "d" * 9, *([0.0] * 9)))
    return bytes(data)


def make_config() -> dict:
    return json.loads((HERE / "tests/synthetic_config.json").read_text())


def make_bag(path: Path, left_stamps: list[int], right_stamps: list[int] | None = None) -> None:
    right_stamps = right_stamps or left_stamps
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(path)
    conn.executescript("""
      CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                          serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
      CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL);
    """)
    topics = [("/left", "sensor_msgs/msg/Image"), ("/right", "sensor_msgs/msg/Image"),
              ("/gyro", "sensor_msgs/msg/Imu"), ("/accel", "sensor_msgs/msg/Imu")]
    for index, (name, msg_type) in enumerate(topics, 1):
        conn.execute("INSERT INTO topics VALUES (?, ?, ?, 'cdr', '')", (index, name, msg_type))
    message_id = 1
    for topic_id, stamps, maker in ((1, left_stamps, image), (2, right_stamps, image),
                                    (3, [1_000_000_000 + i * 250_000_000 for i in range(5)], imu),
                                    (4, [1_000_000_000 + i * 250_000_000 for i in range(5)], imu)):
        for stamp in stamps:
            conn.execute("INSERT INTO messages VALUES (?, ?, ?, ?)", (message_id, topic_id, stamp, maker(stamp)))
            message_id += 1
    conn.commit()
    conn.close()


class QualificationTests(unittest.TestCase):
    def test_pass_fixture_and_analyze_only(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); bag = root / "pass.db3"; config = make_config()
            make_bag(bag, [1_000_000_000, 1_500_000_000, 2_000_000_000])
            report, criteria = analyze_database(bag, config)
            self.assertEqual(report["pairing"]["matched"], 3)
            self.assertFalse([item for item in criteria if item["status"] == "FAIL"])
            config_path = root / "config.json"; config_path.write_text(json.dumps(config))
            output = root / "evidence"
            self.assertEqual(analyze_main(["--bag", str(bag), "--config", str(config_path), "--output-dir", str(output)]), 0)
            self.assertEqual(analyze_main(["--config", str(config_path), "--analyze-only", str(output)]), 0)
            self.assertEqual(json.loads((output / "result.json").read_text())["status"], "PASS")

    def test_drop_fixture_is_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Path(temp) / "drop.db3"; config = make_config()
            make_bag(bag, [1_000_000_000, 1_500_000_000, 3_000_000_000])
            report, criteria = analyze_database(bag, config)
            self.assertGreater(report["topics"]["/left"]["header_drop_count"], 0)
            self.assertTrue(any(item["status"] == "FAIL" for item in criteria))

    def test_stamp_reversal_is_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            bag = Path(temp) / "reverse.db3"; config = make_config()
            make_bag(bag, [1_000_000_000, 2_000_000_000, 1_500_000_000])
            report, criteria = analyze_database(bag, config)
            self.assertEqual(report["topics"]["/left"]["header_reversals"], 1)
            self.assertTrue(any(item["id"] == "topic\x3a/left:stamp_order" and item["status"] == "FAIL" for item in criteria))

    def test_missing_bag_is_blocked(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); config_path = root / "config.json"; config_path.write_text(json.dumps(make_config()))
            output = root / "blocked"
            code = analyze_main(["--bag", str(root / "missing.db3"), "--config", str(config_path), "--output-dir", str(output)])
            self.assertEqual(code, 2)
            self.assertEqual(json.loads((output / "result.json").read_text())["status"], "BLOCKED")

    @unittest.skipUnless(REAL_DB3.is_file(), "real saved DB3 evidence is not present")
    def test_real_saved_evidence_fixture(self):
        """Use the saved 2026-10-14 DB3 as an input fixture, never copy or modify it."""
        available = sqlite3.connect(f"file:{REAL_DB3}?mode=ro", uri=True)
        try:
            topics = {name: msg_type for name, msg_type in available.execute("SELECT name,type FROM topics")}
        finally:
            available.close()
        selected = [("/device_0/sensor_0/Infrared_1/image/data", "image_left"),
                    ("/device_0/sensor_0/Infrared_2/image/data", "image_right"),
                    ("/device_0/sensor_2/Gyro_0/imu/data", "gyro"),
                    ("/device_0/sensor_2/Accel_0/imu/data", "accel")]
        config = make_config()
        for spec, (name, role) in zip(config["topics"], selected):
            self.assertIn(name, topics)
            spec.update(name=name, role=role, type=topics[name], rate_min_hz=0.0, rate_max_hz=1000.0,
                        max_gap_s=1000.0, drop_gap_factor=1000.0)
        config["pairing"]["min_fraction"] = 0.0
        config["storage"]["max_write_rate_bps"] = 1_000_000_000
        report, _ = analyze_database(REAL_DB3, config)
        self.assertGreater(report["topics"][selected[0][0]]["count"], 0)
        self.assertGreater(report["topics"][selected[2][0]]["count"], 0)


if __name__ == "__main__":
    unittest.main()

