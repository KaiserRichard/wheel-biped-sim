import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from lab.mobile_demo.check_capture import RGBD_TOPICS, read_bag, summarize_records


def image(encoding, values, width, height=1):
    code = {"16UC1": "H", "32FC1": "f"}[encoding]
    data = b"".join(struct.pack("<" + code, value) for value in values)
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=width * struct.calcsize(code),
        data=data,
        is_bigendian=False,
    )


class CaptureCheckTest(unittest.TestCase):
    def test_counts_rates_and_depth_validity(self):
        records = []
        for index, topic in enumerate(RGBD_TOPICS):
            if topic.endswith("image_raw") or topic.endswith("image_rect_raw"):
                kind = RGBD_TOPICS[topic]
                msg = image("16UC1", [0, 1000, 2500, 0], 4) if "depth" in kind else object()
            else:
                msg = object()
            records.extend([(topic, 1_000_000_000, msg), (topic, 2_000_000_000, msg)])

        result = summarize_records(records)
        self.assertTrue(result["passed"])
        self.assertEqual(result["topics"]["/camera/camera/color/image_raw"]["count"], 2)
        for topic in ("/camera/camera/depth/image_rect_raw", "/camera/camera/aligned_depth_to_color/image_raw"):
            stats = result["topics"][topic]
            self.assertEqual(stats["rate_hz"], 1.0)
            self.assertEqual(stats["depth_valid_fraction"], 0.5)

    def test_missing_topic_and_invalid_depth_are_reported(self):
        result = summarize_records([
            ("/camera/camera/depth/image_rect_raw", 1, image("32FC1", [1.0, float("nan")], 2)),
        ])
        self.assertFalse(result["passed"])
        self.assertIn("/camera/camera/color/image_raw", result["missing_required_topics"])
        self.assertEqual(result["topics"]["/camera/camera/depth/image_rect_raw"]["depth_valid_fraction"], 0.5)

    def test_read_bag_uses_mocked_rosbags_reader(self):
        connections = [SimpleNamespace(topic=topic, msgtype="fixture/msg") for topic in RGBD_TOPICS]
        decoded = {}
        rows = []
        for index, connection in enumerate(connections):
            message = image("16UC1", [200, 400], 2) if RGBD_TOPICS[connection.topic] in {"raw_depth", "aligned_depth"} else object()
            decoded[str(index)] = message
            rows.extend([(connection, 1_000_000_000, str(index).encode()),
                         (connection, 2_000_000_000, str(index).encode())])

        class FakeReader:
            def __init__(self, _paths):
                self.connections = connections

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def messages(self, connections=()):
                return (row for row in rows if row[0] in connections)

            def deserialize(self, raw, _msgtype):
                return decoded[raw.decode()]

        with patch("rosbags.highlevel.AnyReader", FakeReader):
            result = read_bag(__import__("pathlib").Path("fixture.mcap"))
        self.assertTrue(result["passed"])
        self.assertEqual(result["topics"]["/camera/camera/aligned_depth_to_color/image_raw"]["count"], 2)


if __name__ == "__main__":
    unittest.main()
