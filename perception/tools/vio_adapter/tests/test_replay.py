import csv
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import zlib

HERE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('euroc_replay', HERE/'euroc_replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def chunk(tag, data):
    return struct.pack('>I', len(data))+tag+data+struct.pack('>I', zlib.crc32(tag+data)&0xffffffff)


def png(pixels=b'\x0a\x14', filt=0):
    # First row has zero upper neighbors; generate any of five filters.
    encoded = bytearray()
    for x, v in enumerate(pixels):
        a = pixels[x-1] if x else 0
        predict = a if filt in (1, 4) else a//2 if filt == 3 else 0
        encoded.append((v-predict)&255)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('>IIBBBBB', len(pixels), 1, 8, 0, 0, 0, 0))+chunk(b'IDAT', zlib.compress(bytes([filt])+encoded))+chunk(b'IEND', b'')


def fixture(root):
    (root/'mav0'/'imu0').mkdir(parents=True)
    (root/'manifest.json').write_text(json.dumps({'stamp_domains': ['device']}))
    (root/'times_ns.txt').write_text('5000000\n25000000\n')
    (root/'stamps.csv').write_text('stream,sdk_ns,domain\nir1,5000000,device\nir2,5000000,device\nir1,25000000,device\nir2,25000000,device\n')
    for cam in ['cam0', 'cam1']:
        p = root/'mav0'/cam
        (p/'data').mkdir(parents=True)
        (p/'data.csv').write_text('#timestamp [ns],filename\n5000000,5000000.png\n25000000,25000000.png\n')
        for ns in [5000000, 25000000]:
            (p/'data'/f'{ns}.png').write_bytes(png())
    for name, times in [('accel', [0, 10000000, 20000000, 30000000]),
                        ('gyro', [3000000, 7000000, 23000000, 27000000])]:
        (root/'mav0'/'imu0'/f'raw_{name}.csv').write_text('#sdk_timestamp [ns],domain,x,y,z\n'+''.join(f'{ns},device,1,2,3\n' for ns in times))


class ReplayTests(unittest.TestCase):
    def test_png_filters_and_crc(self):
        with tempfile.TemporaryDirectory(dir=HERE/'build') as tmp:
            p = Path(tmp)/'test.png'
            for filt in range(5):
                p.write_bytes(png(bytes([10, 20, 255, 0]), filt))
                self.assertEqual(replay.decode_png(p), (4, 1, bytes([10, 20, 255, 0])))
            bad = bytearray(p.read_bytes()); bad[20] ^= 1; p.write_bytes(bad)
            with self.assertRaises(ValueError): replay.decode_png(p)

    def test_full_replay_and_read_only(self):
        with tempfile.TemporaryDirectory(dir=HERE/'build') as tmp:
            root = Path(tmp)/'export'; root.mkdir(); fixture(root)
            before = {p: p.read_bytes() for p in root.rglob('*') if p.is_file()}
            result = replay.replay(root, Path(tmp)/'result', HERE/'build'/'replay_fake')
            self.assertEqual(result['adapter_counts']['pairs'], 2)
            self.assertEqual(result['adapter_counts']['combined'], 4)
            self.assertEqual(result['adapter_counts']['domain'], 0)
            logs = [json.loads(s) for s in (Path(tmp)/'result'/'snapshots.jsonl').read_text().splitlines()]
            self.assertEqual([s['sequence'] for s in logs], [1, 2])
            self.assertFalse(logs[0]['valid']); self.assertTrue(logs[1]['valid'])
            self.assertEqual(before, {p: p.read_bytes() for p in root.rglob('*') if p.is_file()})
            with self.assertRaises(ValueError): replay.replay(root, root/'output', HERE/'build'/'replay_fake')

    def test_does_not_hide_reversals_or_domains(self):
        with tempfile.TemporaryDirectory(dir=HERE/'build') as tmp:
            root = Path(tmp); fixture(root)
            path = root/'times_ns.txt'; path.write_text('25000000\n5000000\n')
            with self.assertRaises(ValueError): replay.load_events(root)
            path.write_text('5000000\n25000000\n')
            p = root/'mav0'/'imu0'/'raw_gyro.csv'
            p.write_text(p.read_text().replace('device', 'other'))
            with self.assertRaises(ValueError): replay.load_events(root)

    def test_binary_truncation_rejected(self):
        r = subprocess.run([str(HERE/'build'/'replay_fake'), 'device'], input=b'A\x00', capture_output=True)
        self.assertEqual(r.returncode, 2)
        self.assertIn(b'truncated', r.stderr)


if __name__ == '__main__':
    unittest.main()
