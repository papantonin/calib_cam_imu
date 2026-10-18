#!/usr/bin/env python3
"""Offline ROS1 extraction into the native calibration layout; no ROS runtime."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import nullcontext
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader

IMAGE = 'sensor_msgs/msg/Image'
IMU = 'sensor_msgs/msg/Imu'


class SequentialBagReader:
    """Read disjoint ROS1 recording intervals with only one bag's chunks in memory."""
    def __init__(self, paths):
        self.paths = paths
        self.connections = []
        self.reader = None
        self.iterator = None

    def __enter__(self):
        intervals = []
        for path in self.paths:
            with path.open('rb') as stream:
                if stream.read(13) != b'#ROSBAG V2.0\n':
                    raise ValueError(f'Only ROS1 bag v2 is supported: {path}')
            with AnyReader([path]) as reader:
                self.connections.extend(reader.connections)
                intervals.append((reader.start_time, reader.end_time, path))
        intervals.sort(key=lambda item: (item[0], str(item[2])))
        for previous, current in zip(intervals, intervals[1:]):
            if current[0] < previous[1]:
                raise ValueError('Overlapping bag recording intervals are not supported')
        self.paths = [item[2] for item in intervals]
        return self

    def messages(self, connections):
        topics = {c.topic for c in connections}
        def iterate():
            for path in self.paths:
                with AnyReader([path]) as reader:
                    self.reader = reader
                    selected = [c for c in reader.connections if c.topic in topics]
                    if selected:
                        yield from reader.messages(connections=selected)
        self.iterator = iterate()
        return self.iterator

    def deserialize(self, raw, msgtype):
        return self.reader.deserialize(raw, msgtype)

    def __exit__(self, *args):
        if self.iterator is not None:
            self.iterator.close()
        self.reader = None


def timestamp_ns(stamp):
    sec, nano = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError('Invalid nonnegative header timestamp')
    value = sec * 1_000_000_000 + nano
    if value > 2**63-1:
        raise ValueError('Timestamp exceeds signed int64')
    return value


def debayer_rggb8(array):
    """Bilinear RGB, normalized available neighbors at borders; preserve measured sites."""
    height, width = array.shape
    if min(height, width) < 2:
        raise ValueError('Bayer interpolation requires at least 2x2 samples')
    result = np.empty((height, width, 3), dtype='u1')
    for channel, sites in enumerate((((0,0),), ((0,1),(1,0)), ((1,1),))):
        mask = np.zeros((height, width), dtype='uint16')
        for y, x in sites:
            mask[y::2, x::2] = 1
        samples = np.pad(array.astype('uint16')*mask, 1)
        available = np.pad(mask, 1)
        total = np.zeros_like(mask)
        weight = np.zeros_like(mask)
        for y in range(3):
            for x in range(3):
                coefficient = (4 if (y,x)==(1,1) else int(y==1 or x==1)) if channel==1 else (2 if y==1 else 1)*(2 if x==1 else 1)
                if coefficient:
                    total += coefficient*samples[y:y+height, x:x+width]
                    weight += coefficient*available[y:y+height, x:x+width]
        result[:,:,channel] = ((total+weight//2)//weight).astype('u1')
    return result


def image_array(message, bayer_conversion=None):
    formats = {'mono8': ('u1', 1), 'mono16': ('>u2' if message.is_bigendian else '<u2', 1),
               'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'bayer_rggb8': ('u1', 1)}
    if message.encoding not in formats:
        raise ValueError(f'Unsupported ROS image encoding: {message.encoding}')
    if message.encoding == 'bayer_rggb8' and bayer_conversion != 'bilinear':
        raise ValueError('Bayer input requires explicit --bayer-conversion bilinear')
    dtype, channels = formats[message.encoding]
    width, height, step = int(message.width), int(message.height), int(message.step)
    if not (0 < width <= 32768 and 0 < height <= 32768) or message.is_bigendian not in (0, 1):
        raise ValueError('Invalid image dimensions or endianness')
    rowbytes = width * channels * np.dtype(dtype).itemsize
    if step < rowbytes or len(message.data) != height * step:
        raise ValueError('Image step/payload size mismatch')
    rows = np.frombuffer(message.data, dtype='u1').reshape(height, step)[:, :rowbytes].copy()
    array = rows.view(dtype).reshape((height, width) if channels == 1 else (height, width, channels))
    if message.encoding == 'bgr8':
        array = array[:, :, ::-1]
    array = np.ascontiguousarray(array, dtype='uint16' if message.encoding == 'mono16' else 'uint8')
    return debayer_rggb8(array) if message.encoding == 'bayer_rggb8' else array


def pixel_digest(array):
    # Canonical numeric representation, independent of ROS and host endianness.
    return hashlib.sha256(array.astype('<u2' if array.dtype.itemsize == 2 else 'u1', copy=False).tobytes()).hexdigest()


def write_frame(folder, camera, timestamp, record_time, message, source_bag=None, bayer_conversion=None):
    array = image_array(message, bayer_conversion)
    relative = f'{camera}/{timestamp}.png'
    path = folder / relative
    with path.open('xb') as file:
        Image.fromarray(array).save(file, format='PNG', compress_level=1)
    # Decode the actual PNG and verify that no bit-depth/gamma conversion changed samples.
    with Image.open(path) as image:
        restored = np.asarray(image)
        if restored.shape != array.shape or not np.array_equal(restored, array):
            raise ValueError(f'PNG roundtrip changed samples: {relative}')
    return {'path': relative, 'timestamp_ns': timestamp, 'bag_timestamp_ns': record_time,
            'header_timestamp_ns': timestamp_ns(message.header.stamp), 'source_bag': source_bag,
            'topic_frame_id': message.header.frame_id, 'sequence': int(message.header.seq),
            'encoding': message.encoding, 'width': int(message.width), 'height': int(message.height),
            'conversion': 'rggb8_to_rgb8_bilinear_available_neighbors' if message.encoding == 'bayer_rggb8' else 'lossless_samples',
            'source_payload_sha256': hashlib.sha256(message.data).hexdigest(),
            'ros_step': int(message.step), 'ros_bigendian': int(message.is_bigendian),
            'pixel_sha256': pixel_digest(array), 'png_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'min_value': int(array.min()), 'max_value': int(array.max())}


def select_topics(connections, camera_specs, imu_topic, without_imu=False):
    available = {}
    for connection in connections:
        if connection.topic in available and available[connection.topic] != connection.msgtype:
            raise ValueError(f'Topic changes message type: {connection.topic}')
        available[connection.topic] = connection.msgtype
    cameras = {}
    if camera_specs:
        for spec in camera_specs:
            camera, topic = spec.split('=', 1)
            if not camera.startswith('cam') or not camera[3:].isdigit() or str(int(camera[3:])) != camera[3:]:
                raise ValueError(f'Invalid camera ID: {camera}')
            if camera in cameras or topic in cameras.values():
                raise ValueError('Repeated camera/topic mapping')
            cameras[camera] = topic
    else:
        import re
        for topic, msgtype in available.items():
            match = re.fullmatch(r'/(cam(?:0|[1-9][0-9]*))/image_raw', topic)
            if msgtype == IMAGE and match:
                cameras[match[1]] = topic
    if not cameras:
        raise ValueError('No canonical camera topics; provide --camera cam0=/topic explicitly')
    for camera, topic in cameras.items():
        if available.get(topic) != IMAGE:
            raise ValueError(f'{camera}: missing sensor_msgs/Image topic {topic}')
    if without_imu:
        if imu_topic:
            raise ValueError('--without-imu and --imu-topic are mutually exclusive')
        return dict(sorted(cameras.items(), key=lambda pair: int(pair[0][3:]))), None
    if not imu_topic:
        imus = [topic for topic, msgtype in available.items() if msgtype == IMU]
        if len(imus) != 1:
            raise ValueError('Expected one IMU topic; select --imu-topic explicitly')
        imu_topic = imus[0]
    if available.get(imu_topic) != IMU:
        raise ValueError(f'Missing sensor_msgs/Imu topic: {imu_topic}')
    return dict(sorted(cameras.items(), key=lambda pair: int(pair[0][3:]))), imu_topic


def extract(bag, output, camera_specs=(), imu_topic=None, workers=4, *, without_imu=False, timestamp_source='header', bayer_conversion=None):
    if workers < 1 or workers > 32:
        raise ValueError('workers must be in [1,32]')
    if timestamp_source not in ('header', 'bag'):
        raise ValueError('timestamp_source must be header or bag')
    if bayer_conversion not in (None, 'bilinear'):
        raise ValueError('Unknown Bayer conversion')
    if output.exists():
        raise FileExistsError(f'Output already exists: {output}')
    partial = output.with_name(output.name + '.incomplete')
    start = time.monotonic()
    bags = [bag] if isinstance(bag, Path) else list(bag)
    if not bags or len({p.resolve() for p in bags}) != len(bags):
        raise ValueError('Need distinct, nonempty source bags')
    sources = []
    for path in bags:
        stat = path.stat()
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        sources.append({'path': str(path.resolve()), 'size_bytes': stat.st_size,
                        'mtime_ns': stat.st_mtime_ns, 'sha256': digest})
    with SequentialBagReader(bags) as reader:
        cameras, imu_topic = select_topics(reader.connections, camera_specs, imu_topic, without_imu)
        mapping = {topic: camera for camera, topic in cameras.items()}
        if imu_topic is not None:
            mapping[imu_topic] = 'imu0'
        selected = [c for c in reader.connections if c.topic in mapping]
        expected = {name: sum(c.msgcount for c in selected if mapping[c.topic] == name) for name in mapping.values()}
        if any(count == 0 for count in expected.values()):
            raise ValueError('Selected stream is empty')
        output.parent.mkdir(parents=True, exist_ok=True)
        partial.mkdir()  # Deliberately refuse an earlier incomplete extraction.
        for name in mapping.values():
            (partial / name).mkdir()
        stats = {name: {'messages': 0} for name in mapping.values()}
        frame_ids = {name: set() for name in mapping.values()}
        provenance = {'schema_version': 2, 'sources': sources,
                      'timestamp_source': 'header.stamp (integer sec*1e9+nsec)' if timestamp_source == 'header' else 'bag record timestamp (not sensor exposure time)',
                      'imu_present': imu_topic is not None,
                      'image_conversion': 'lossless mono8/mono16; rgb8 unchanged, bgr8 channel reorder',
                      'bayer_conversion': 'rggb8_to_rgb8_bilinear_available_neighbors' if bayer_conversion else None,
                      'imu_units': {'angular_velocity': 'rad/s', 'linear_acceleration': 'm/s^2'},
                      'imu_conversion': 'ROS message values preserved; no rotation, bias correction or time shift',
                      'topics': {name: topic for topic, name in mapping.items()}, 'streams': stats,
                      'ignored_topics': sorted({c.topic for c in reader.connections if c.topic not in mapping}),
                      'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      'dependencies': {p: version(p) for p in ['rosbags', 'numpy', 'Pillow']}, 'workers': workers}
        if len(sources) == 1:  # Preserve the existing single-bag audit interface.
            provenance.update(source=sources[0]['path'], source_size_bytes=sources[0]['size_bytes'], source_sha256=sources[0]['sha256'])
        provenance['warnings'] = []
        if timestamp_source == 'bag':
            provenance['warnings'].append('Recording timestamps do not establish sensor timing or inter-camera synchronization.')
        if imu_topic is None:
            provenance['warnings'].append('No IMU exported; dataset cannot support camera-IMU calibration.')
        try:
            with ((partial/'imu0/imu0.csv').open('x') if imu_topic else nullcontext()) as imu, (partial/'frames.jsonl').open('x') as frames, \
                 ThreadPoolExecutor(max_workers=workers) as pool:
                if imu is not None:
                    imu.write('timestamp_ns,wx,wy,wz,ax,ay,az\n')
                pending = deque()
                shapes = {}
                last_progress = start
                for connection, record_time, raw in reader.messages(connections=selected):
                    message = reader.deserialize(raw, connection.msgtype)
                    name = mapping[connection.topic]
                    header_stamp = timestamp_ns(message.header.stamp)
                    stamp = header_stamp if timestamp_source == 'header' else int(record_time)
                    if not 0 <= stamp <= 2**63-1:
                        raise ValueError('Recording timestamp exceeds nonnegative int64')
                    stream = stats[name]
                    if stream['messages'] and stamp <= stream['last_timestamp_ns']:
                        raise ValueError(f'{connection.topic}: duplicate or non-increasing {timestamp_source} timestamp {stamp}')
                    if not stream['messages']:
                        stream['first_timestamp_ns'] = stamp
                        stream['min_bag_minus_header_ns'] = record_time-header_stamp
                        stream['max_bag_minus_header_ns'] = record_time-header_stamp
                    stream['last_timestamp_ns'] = stamp
                    stream['min_bag_minus_header_ns'] = min(stream['min_bag_minus_header_ns'], record_time-header_stamp)
                    stream['max_bag_minus_header_ns'] = max(stream['max_bag_minus_header_ns'], record_time-header_stamp)
                    stream['messages'] += 1
                    stream['zero_header_timestamps'] = stream.get('zero_header_timestamps', 0) + int(header_stamp == 0)
                    frame_ids[name].add(message.header.frame_id)
                    if connection.msgtype == IMU:
                        if message.angular_velocity_covariance[0] == -1 or message.linear_acceleration_covariance[0] == -1:
                            raise ValueError('IMU message marks angular velocity or acceleration unavailable')
                        values = [getattr(vector, axis) for vector in (message.angular_velocity, message.linear_acceleration) for axis in ('x','y','z')]
                        if not all(math.isfinite(x) for x in values):
                            raise ValueError('Non-finite IMU measurement')
                        imu.write(str(stamp) + ',' + ','.join(format(x, '.17g') for x in values) + '\n')
                    else:
                        shape = (message.width, message.height, message.encoding)
                        if name in shapes and shapes[name] != shape:
                            raise ValueError(f'{name}: image shape/encoding changes')
                        shapes[name] = shape
                        stream['resolution'] = [int(message.width), int(message.height)]
                        stream['encoding'] = message.encoding
                        pending.append(pool.submit(write_frame, partial, name, stamp, record_time, message, str(connection.owner.path.resolve()), bayer_conversion))
                        if len(pending) >= workers*2:
                            frames.write(json.dumps(pending.popleft().result())+'\n')
                    if time.monotonic()-last_progress > 5:
                        print(f'{output.name}: {sum(s["messages"] for s in stats.values())} messages extracted', file=sys.stderr, flush=True)
                        last_progress = time.monotonic()
                while pending:
                    frames.write(json.dumps(pending.popleft().result())+'\n')
            for name, stream in stats.items():
                if stream['messages'] != expected[name]:
                    raise ValueError(f'{name}: count differs from bag index')
                stream['frame_ids'] = sorted(frame_ids[name])
            for source in sources:
                current_stat = Path(source['path']).stat()
                if (current_stat.st_size, current_stat.st_mtime_ns) != (source['size_bytes'], source['mtime_ns']):
                    raise ValueError('Source bag changed during extraction')
            for name in (['imu0/imu0.csv'] if imu_topic else []) + ['frames.jsonl']:
                with (partial/name).open('rb') as stream:
                    provenance.setdefault('output_sha256', {})[name] = hashlib.file_digest(stream, 'sha256').hexdigest()
            provenance['elapsed_s'] = time.monotonic()-start
            provenance['status'] = 'complete'
            (partial/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
            # Output must still be absent. Concurrent runs share the exclusive partial directory.
            if output.exists():
                raise FileExistsError(f'Output appeared during extraction: {output}')
            partial.rename(output)
            return provenance
        except BaseException as error:
            (partial/'error.json').write_text(json.dumps({'status':'incomplete','error':str(error)}, indent=2)+'\n')
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True, nargs='+', help='ROS1 bags or directories of disjoint *.bag recordings')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--camera', action='append', default=[], metavar='camN=/topic')
    parser.add_argument('--imu-topic')
    parser.add_argument('--without-imu', action='store_true', help='Explicit camera-only extraction; no empty/fabricated IMU CSV')
    parser.add_argument('--timestamp-source', choices=('header', 'bag'), default='header', help='Use bag recording time only by explicit request; does not establish synchronization')
    parser.add_argument('--bayer-conversion', choices=('bilinear',), help='Explicit RGGB8 to RGB8 interpolation; source samples stay unchanged in the bag')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    bags = [p for item in args.bag for p in (sorted(item.glob('*.bag')) if item.is_dir() else [item])]
    if not bags:
        parser.error('No source bags found')
    if args.inspect:
        with SequentialBagReader(bags) as reader:
            print(json.dumps([{'topic':c.topic,'type':c.msgtype,'messages':c.msgcount} for c in reader.connections], indent=2))
    elif not args.output:
        parser.error('--output is required for extraction')
    else:
        print(json.dumps(extract(bags,args.output,args.camera,args.imu_topic,args.workers,
                                without_imu=args.without_imu,timestamp_source=args.timestamp_source,bayer_conversion=args.bayer_conversion),indent=2))

if __name__ == '__main__':
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        print(f'Extraction failed: {error}', file=sys.stderr)
        sys.exit(130 if isinstance(error, KeyboardInterrupt) else 2)
