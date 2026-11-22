"""Tiny MCAP integration tests; run in the offline runtime, never touches lab data."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np
from rosbags.rosbag2 import Writer, StoragePlugin
from rosbags.typesys import Stores, get_typestore

SPEC = importlib.util.spec_from_file_location('extract_stereo', Path(__file__).with_name('extract_stereo.py'))
extractor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(extractor)


class ExtractTests(unittest.TestCase):
    def bag(self, root, right_offset=0, encoding='mono8', duplicate=False):
        store = get_typestore(Stores.ROS2_JAZZY)
        time_type = store.types['builtin_interfaces/msg/Time']
        header_type = store.types['std_msgs/msg/Header']
        image_type = store.types['sensor_msgs/msg/Image']
        path = root / 'bag'
        # Deliberate stride padding verifies only pixel data reaches the runner.
        pixels = np.arange(480 * 644, dtype=np.uint8).reshape(480, 644)
        with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
            connections = [writer.add_connection(f'/camera/camera/infra{n}/image_rect_raw', image_type.__msgtype__, typestore=store) for n in (1, 2)]
            for index in range(3):
                for side, connection in enumerate(connections):
                    ns = 1700000000000000001 + (0 if duplicate else index) * 66666667 + side * right_offset
                    image = image_type(header_type(time_type(ns // 10**9, ns % 10**9), 'camera'), 480, 640, encoding, 0, 644, pixels.ravel())
                    writer.write(connection, 1700000000000000000 + index * 66666667 + side, store.serialize_cdr(image, image_type.__msgtype__))
        return path, pixels[:, :640]

    def test_exact_headers_lossless_pixels_and_exclusive_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag, pixels = self.bag(root)
            output = root / 'extracted'
            extractor.extract(bag, output, 3)
            self.assertTrue(np.array_equal(cv2.imread(str(output / 'left/1700000000000000001.png'), 0), pixels))
            self.assertIn('1700000000000000001,left/', (output / 'pairs.csv').read_text())
            with self.assertRaises(FileExistsError):
                extractor.extract(bag, output, 3)

    def test_header_mismatch_not_silently_paired(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag, _ = self.bag(root, right_offset=1)
            with self.assertRaisesRegex(ValueError, 'exact stereo coverage'):
                extractor.extract(bag, root / 'extracted', 3)

    def test_unexpected_count_and_encoding_fail(self):
        for encoding, count in (('mono8', 4), ('rgb8', 3)):
            with self.subTest(encoding=encoding), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                bag, _ = self.bag(root, encoding=encoding)
                with self.assertRaises(ValueError):
                    extractor.extract(bag, root / 'extracted', count)

    def test_duplicate_headers_fail_without_deduplication(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag, _ = self.bag(root, duplicate=True)
            with self.assertRaisesRegex(ValueError, 'non-increasing'):
                extractor.extract(bag, root / 'extracted', 3)


if __name__ == '__main__':
    unittest.main()
