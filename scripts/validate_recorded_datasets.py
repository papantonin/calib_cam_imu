#!/usr/bin/env python3
"""Local real-data regression: independent source audit, checks and detector stability."""
import argparse
import csv
import hashlib
import json
from itertools import zip_longest
from pathlib import Path
import subprocess
import time
import numpy as np
from PIL import Image
from rosbags.highlevel import AnyReader
from ruamel.yaml import YAML


def audit_correspondences(folder, target):
    geometry=json.loads((folder/'target_geometry.json').read_text())
    assert geometry['units']=='m' and geometry['frame']=='target'
    points={point['point_id']:point for point in geometry['points']}
    first=target.get('tagStartId',0)
    size=target['tagSize']; pitch=size*(1+target['tagSpacing'])
    assert len(points)==len(geometry['points'])==4*target['tagRows']*target['tagCols']
    for point_id, point in points.items():
        cell,corner=divmod(point_id,4)
        row,column=divmod(cell,target['tagCols'])
        # Consumer's grid definition, independent of the C++ geometry exporter.
        offsets=[(0,0),(size,0),(size,size),(0,size)]
        expected=[column*pitch+offsets[corner][0],row*pitch+offsets[corner][1],0]
        assert np.allclose(point['position_m'],expected,rtol=0,atol=1e-14)
        assert point['tag_id']==first+cell and point['corner_index']==corner
    count=frames=rejected=0
    with (folder/'detections.jsonl').open() as raw, (folder/'observations.jsonl').open() as observations:
        for raw_line, observation_line in zip_longest(raw,observations):
            assert raw_line is not None and observation_line is not None
            detection=json.loads(raw_line); observation=json.loads(observation_line)
            for key in ['camera','timestamp_ns','resolution']: assert observation[key]==detection[key]
            detected={tag['id']:tag for tag in detection['tags']}
            excluded={tag['id'] for tag in observation['rejected_tags']}
            assert len(observation['points'])==4*len(detected.keys()-excluded)
            assert len({p['point_id'] for p in observation['points']})==len(observation['points'])
            for point in observation['points']:
                physical=points[point['point_id']]
                assert point['pixel']==detected[physical['tag_id']]['corners_px'][physical['corner_index']]
            count+=len(observation['points']); frames+=1; rejected+=len(observation['rejected_tags'])
    summary=json.loads((folder/'summary.json').read_text())
    assert summary['observations']['accepted_corners']==count
    assert summary['observations']['rejected_tags']==rejected
    return {'status':'passed','frames':frames,'corners':count,'rejected_tags':rejected,'target_points':len(points)}


def audit(dataset):
    provenance=json.loads((dataset/'provenance.json').read_text())
    source=Path(provenance['source'])
    with source.open('rb') as stream:
        assert hashlib.file_digest(stream,'sha256').hexdigest()==provenance['source_sha256'],'Source bag digest changed'
    topics={topic:name for name,topic in provenance['topics'].items()}
    counts={name:0 for name in topics.values()}
    with (dataset/'imu0/imu0.csv').open() as csv_stream, AnyReader([source]) as reader:
        csv_rows=iter(csv.DictReader(csv_stream))
        selected=[c for c in reader.connections if c.topic in topics]
        for connection,_,raw in reader.messages(connections=selected):
            msg=reader.deserialize(raw,connection.msgtype)
            stamp=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            name=topics[connection.topic]
            counts[name]+=1
            if name=='imu0':
                row=next(csv_rows)
                assert int(row['timestamp_ns'])==stamp
                reference=[getattr(v,a) for v in [msg.angular_velocity,msg.linear_acceleration] for a in ['x','y','z']]
                assert [float(row[key]) for key in ['wx','wy','wz','ax','ay','az']]==reference,'IMU value changed'
            else:
                # Independent decoder from the extractor: a strided view, no padding copy/view sequence.
                assert msg.encoding=='mono16','This recorded-data audit currently expects mono16'
                reference=np.ndarray((msg.height,msg.width),dtype='>u2' if msg.is_bigendian else '<u2',buffer=msg.data,strides=(msg.step,2))
                with Image.open(dataset/name/f'{stamp}.png') as png:
                    assert np.array_equal(np.asarray(png),reference),f'Pixel mismatch {name}/{stamp}'
        assert next(csv_rows,None) is None,'Unexpected extra IMU rows'
    for name,count in counts.items():
        assert count==provenance['streams'][name]['messages']
        if name!='imu0': assert len(list((dataset/name).glob('*.png')))==count
    return {'status':'passed','source_sha256':provenance['source_sha256'],'verified_messages':counts,
            'checks':['all image pixels equal source mono16 values','all IMU timestamps/values equal source','source hash and stream counts']}


def run(binary,dataset,output,*options):
    result=subprocess.run([str(binary),'--dataset',str(dataset),'--output',str(output),*options],text=True,capture_output=True)
    if result.returncode:
        raise RuntimeError(f'{binary.name} failed ({result.returncode}): {result.stderr}')
    return json.loads(result.stdout)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root',type=Path,default=Path('dataset'))
    parser.add_argument('--build-dir',type=Path,default=Path('build/release'))
    parser.add_argument('--target',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--quad-decimate-512',type=float,default=1)
    parser.add_argument('--quad-decimate-1024',type=float,default=1)
    parser.add_argument('--baseline',type=Path,help='Prior run with the same detection parameters; require unchanged raw detections')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    binary=(args.build_dir/'calib_imu_camera').resolve()
    report={'status':'running','binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),
            'target_sha256':hashlib.sha256(args.target.read_bytes()).hexdigest(),'datasets':{}}
    started=time.monotonic()
    target=YAML(typ='safe').load(args.target.read_text())
    try:
        for name in ['512','1024']:
            dataset=args.dataset_root/name
            print(f'{name}: auditing every source message',flush=True)
            result={'audit':audit(dataset)}
            report['datasets'][name]=result
            result['check']=run(binary,dataset,args.output/f'{name}-check','--check-dataset','--threads','4')
            print(f'{name}: full detection, 4 and 1 workers',flush=True)
            for threads in [4,1]:
                result[f'detect_{threads}']=run(binary,dataset,args.output/f'{name}-detect-{threads}','--detect-only','--target',str(args.target),'--threads',str(threads),'--quad-decimate',str(getattr(args,'quad_decimate_'+name)))
            paths=[args.output/f'{name}-detect-{threads}'/'detections.jsonl' for threads in [4,1]]
            hashes=[]
            for path in paths:
                with path.open('rb') as stream: hashes.append(hashlib.file_digest(stream,'sha256').hexdigest())
            assert hashes[0]==hashes[1],'Observations differ across thread counts'
            result['observations_identical_1_vs_4_workers']=True
            result['observations_sha256']=hashes[0]
            if args.baseline:
                with (args.baseline/f'{name}-detect-4/detections.jsonl').open('rb') as stream:
                    assert hashes[0]==hashlib.file_digest(stream,'sha256').hexdigest(),'Raw detections changed from baseline'
                result['raw_detections_unchanged_from_baseline']=True
            folders=[args.output/f'{name}-detect-{threads}' for threads in [4,1]]
            for filename in ['target_geometry.json','observations.jsonl']:
                assert (folders[0]/filename).read_bytes()==(folders[1]/filename).read_bytes(),f'{filename} differs across thread counts'
            result['correspondence_audit']=audit_correspondences(folders[0],target)
        roots=[args.dataset_root/name for name in ['512','1024']]
        report['same_imu_csv']=((roots[0]/'imu0/imu0.csv').read_bytes()==(roots[1]/'imu0/imu0.csv').read_bytes())
        report['same_camera_timestamps']={camera:sorted(p.name for p in (roots[0]/camera).glob('*.png'))==sorted(p.name for p in (roots[1]/camera).glob('*.png')) for camera in ['cam0','cam1']}
        report['status']='passed'
    except BaseException as error:
        report['status']='failed';report['error']=str(error)
        raise
    finally:
        report['elapsed_s']=time.monotonic()-started
        (args.output/'regression.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__': main()
