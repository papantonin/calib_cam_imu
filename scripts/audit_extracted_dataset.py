#!/usr/bin/env python3
"""Independently compare an extracted native dataset against its source ROS1 bags."""
import argparse
import csv
import hashlib
import json
from contextlib import nullcontext
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time

import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify_pixels(msg, pixels):
    height, width = int(msg.height), int(msg.width)
    channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
    dtype = np.dtype(('>u2' if msg.is_bigendian else '<u2') if msg.encoding == 'mono16' else 'u1')
    shape = (height, width, channels) if channels == 3 else (height, width)
    strides = (msg.step, channels*dtype.itemsize, dtype.itemsize) if channels == 3 else (msg.step, dtype.itemsize)
    source = np.ndarray(shape, dtype=dtype, buffer=msg.data, strides=strides)
    if msg.encoding == 'bayer_rggb8':
        require(pixels.shape == (height, width, 3), 'Bayer output is not RGB')
        # Every measured sample must survive in its original color channel.
        for y, x, channel in ((0,0,0), (0,1,1), (1,0,1), (1,1,2)):
            require(np.array_equal(pixels[y::2,x::2,channel], source[y::2,x::2]), 'Measured Bayer sample changed')
        # Scalar geometric oracle for interpolated channels, including all borders.
        def color(y, x): return ((0,1),(1,2))[y%2][x%2]
        for y in sorted({0, 1, height//2, height-2, height-1}):
            for x in sorted({0, 1, width//2, width-2, width-1}):
                for channel in range(3):
                    if color(y,x) == channel:
                        continue
                    values = [int(source[j,i]) for j in range(max(0,y-1),min(height,y+2))
                              for i in range(max(0,x-1),min(width,x+2))
                              if color(j,i)==channel and (channel!=1 or j==y or i==x)]
                    require(int(pixels[y,x,channel]) == (sum(values)+len(values)//2)//len(values),
                            'Bayer interpolation mismatch')
    else:
        require(msg.encoding in ('mono8','mono16','rgb8','bgr8'), 'Unsupported audit encoding')
        if msg.encoding == 'bgr8': source = source[:,:,::-1]
        require(np.array_equal(pixels, source), 'PNG pixels differ from source')


def verify_frame(msg, metadata, path):
    if 'source_payload_sha256' in metadata:
        require(hashlib.sha256(msg.data).hexdigest() == metadata['source_payload_sha256'], 'Source payload mismatch')
    require(digest(path) == metadata['png_sha256'], 'PNG digest mismatch')
    with Image.open(path) as image:
        pixels = np.asarray(image)
        verify_pixels(msg, pixels)
        canonical = pixels.astype('<u2' if pixels.dtype.itemsize==2 else 'u1', copy=False)
        require(hashlib.sha256(canonical.tobytes()).hexdigest() == metadata['pixel_sha256'], 'Pixel digest mismatch')


def audit(dataset, workers=4):
    require(1 <= workers <= 32, 'workers must be in [1,32]')
    provenance = json.loads((dataset/'provenance.json').read_text())
    require(provenance['status'] == 'complete', 'Extraction is incomplete')
    require(provenance['timestamp_source'] == 'header.stamp (integer sec*1e9+nsec)',
            'This audit requires original sensor timestamps')
    sources = provenance.get('sources') or [{'path':provenance['source'], 'sha256':provenance['source_sha256']}]
    if len(sources) > 1:
        def recording_start(source):
            with AnyReader([Path(source['path'])]) as reader:
                return reader.start_time
        sources = sorted(sources, key=recording_start)
    records = {}
    with (dataset/'frames.jsonl').open() as frames:
        for line in frames:
            item = json.loads(line)
            require(item['path'] not in records, 'Repeated frame metadata')
            records[item['path']] = item
    topics = {topic: name for name, topic in provenance['topics'].items()}
    counts = {name: 0 for name in topics.values()}
    last = {}; zero_headers = 0
    imu_path = dataset/'imu0/imu0.csv'
    start = progress = time.monotonic()
    with (imu_path.open() if 'imu0' in counts else nullcontext()) as stream, ThreadPoolExecutor(max_workers=workers) as pool:
        csv_rows = iter(csv.DictReader(stream)) if stream else None
        pending = deque()
        for source in sources:
            path = Path(source['path'])
            require(digest(path) == source['sha256'], 'Source bag hash changed')
            with AnyReader([path]) as reader:
                selected = [c for c in reader.connections if c.topic in topics]
                if not selected: continue
                for connection, record_time, raw in reader.messages(connections=selected):
                    msg = reader.deserialize(raw, connection.msgtype)
                    name = topics[connection.topic]
                    stamp = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
                    require(stamp > last.get(name, -1), f'Nonincreasing sensor time: {name}')
                    last[name] = stamp; zero_headers += int(stamp == 0); counts[name] += 1
                    if name == 'imu0':
                        row = next(csv_rows)
                        expected = [getattr(v,a) for v in (msg.angular_velocity,msg.linear_acceleration) for a in ('x','y','z')]
                        require(int(row['timestamp_ns']) == stamp, 'IMU timestamp changed')
                        require([float(row[k]) for k in ('wx','wy','wz','ax','ay','az')] == expected, 'IMU measurement changed')
                    else:
                        relative = f'{name}/{stamp}.png'
                        metadata = records.pop(relative)
                        require(metadata['timestamp_ns'] == stamp and metadata['bag_timestamp_ns'] == record_time, 'Frame timing mismatch')
                        require(metadata.get('header_timestamp_ns',stamp) == stamp, 'Header time mismatch')
                        pending.append(pool.submit(verify_frame, msg, metadata, dataset/relative))
                        if len(pending) >= 2*workers:
                            pending.popleft().result()
                    if time.monotonic()-progress > 10:
                        print(dataset.name, counts, flush=True); progress = time.monotonic()
        while pending:
            pending.popleft().result()
        require(csv_rows is None or next(csv_rows, None) is None, 'Extra IMU rows')
    require(not records, 'Extra frame metadata')
    for name, count in counts.items():
        require(count == provenance['streams'][name]['messages'], 'Source count mismatch')
        if name != 'imu0': require(len(list((dataset/name).glob('*.png'))) == count, 'PNG count mismatch')
    for name, expected in provenance['output_sha256'].items():
        require(digest(dataset/name) == expected, 'Output metadata digest mismatch')
    return dict(status='passed', counts=counts, zero_sensor_timestamps=zero_headers, elapsed_s=time.monotonic()-start,
                checks=['source hashes', 'all sensor timestamps', 'all IMU values', 'all PNG hashes and measured pixels',
                        'sampled Bayer interpolated channels against scalar oracle', 'counts and metadata hashes'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    try:
        result = audit(args.dataset, args.workers)
    except Exception as error:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'status':'failed','error':str(error)}, indent=2)+'\n')
        raise
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
