#!/usr/bin/env python3
"""Extract every exact-header stereo pair from MCAP, preserving pixels/timestamps."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def extract(bag, output, expected):
    import cv2
    import numpy as np
    from rosbags.highlevel import AnyReader

    output.mkdir()  # Evidence is exclusive; never overwrite or repair in place.
    for side in ('left', 'right'):
        (output / side).mkdir()
    topics = {'/camera/camera/infra1/image_rect_raw': 'left',
              '/camera/camera/infra2/image_rect_raw': 'right'}
    stamps = {side: [] for side in topics.values()}
    pixel_hashes = {side: hashlib.sha256() for side in topics.values()}
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        if {c.topic for c in connections} != set(topics):
            raise ValueError('both stereo image topics required')
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            side = topics[connection.topic]
            ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
            if stamps[side] and ns <= stamps[side][-1]:
                raise ValueError('non-increasing source headers; no sorting/deduplication')
            if msg.encoding not in ('mono8', '8UC1') or msg.height != 480 or msg.width != 640:
                raise ValueError('require recorded 640x480 mono8 IR pixels')
            if msg.step < msg.width or len(msg.data) != msg.height * msg.step:
                raise ValueError('invalid image stride/data size')
            image = np.asarray(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            path = output / side / f'{ns}.png'
            if not cv2.imwrite(str(path), image):
                raise RuntimeError('PNG write failed')
            if not np.array_equal(cv2.imread(str(path), cv2.IMREAD_UNCHANGED), image):
                raise RuntimeError('PNG pixel round-trip mismatch')
            pixel_hashes[side].update(image.tobytes())
            stamps[side].append(ns)
    if stamps['left'] != stamps['right'] or len(stamps['left']) != expected:
        raise ValueError(f'exact stereo coverage failed: {len(stamps["left"])}/{len(stamps["right"])}')
    with (output / 'pairs.csv').open('x') as stream:
        stream.write('source_ns,left,right\n')
        for ns in stamps['left']:
            stream.write(f'{ns},left/{ns}.png,right/{ns}.png\n')
    files = sorted(bag.glob('*.mcap'))
    result = {'bag': str(bag), 'input_sha256': {p.name: sha(p) for p in files},
              'pairs': len(stamps['left']), 'first_source_ns': stamps['left'][0], 'last_source_ns': stamps['left'][-1],
              'pixel_sha256_concatenated': {side: h.hexdigest() for side, h in pixel_hashes.items()},
              'pixels': 'lossless mono8 PNG; stride padding excluded; each decoded PNG compared to original pixels',
              'headers': 'exact integer nanoseconds; input reader order retained; no dropped/deduplicated/resampled pairs'}
    (output / 'extraction.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--expected-pairs', type=int, required=True)
    args = parser.parse_args()
    extract(args.bag, args.output, args.expected_pairs)


if __name__ == '__main__':
    main()
