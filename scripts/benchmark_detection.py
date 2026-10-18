#!/usr/bin/env python3
"""Reproducible synthetic warm-file benchmark; not a real acquisition benchmark."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import tempfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build-dir',type=Path,default=Path('build/release'))
parser.add_argument('--frames',type=int,default=32)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
if args.frames<4:
    parser.error('--frames must be at least 4')
build=args.build_dir.resolve()
report={'kind':'synthetic_warm_files_single_tag_not_real_dataset','platform':platform.platform(),'runs':[]}
report['cpu']=next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')),'unknown')
report['binary_sha256']=hashlib.sha256((build/'calib_cameras').read_bytes()).hexdigest()
with tempfile.TemporaryDirectory(prefix='calib-benchmark-') as folder:
    root=Path(folder)
    target=root/'target.yaml'
    target.write_text('target_type: aprilgrid\ntagRows: 1\ntagCols: 1\ntagSize: 0.04\ntagSpacing: 0.3\ntagFamily: tag36h11\ntagBorderBits: 1\n')
    for width,height,scale in [(1280,720,16),(1920,1080,24),(3840,2160,48)]:
        data=root/f'{height}'
        (data/'cam0').mkdir(parents=True)
        for rotation in range(4):
            subprocess.run([str(build/'render_test_tags'),str(data/f'{rotation}.png'),'tag36h11','0',str(rotation),str(width),str(height),str(scale)],check=True)
        for index in range(args.frames):
            (data/'cam0'/f'{1700000000000000000+index*10000000}.png').hardlink_to(data/f'{index%4}.png')
        reference_hashes={}
        for decimation in [1,2]:
            for threads in [1,4]:
                out=root/f'{height}-{decimation}-{threads}'
                completed=subprocess.run([str(build/'calib_cameras'),'--dataset',str(data),'--target',str(target),
                    '--detect-only','--output',str(out),'--threads',str(threads),'--quad-decimate',str(decimation),'--memory-budget-mb','4096'],
                    check=True,text=True,capture_output=True)
                summary=json.loads(completed.stdout)
                observations=(out/'detections.jsonl').read_bytes()
                digest=hashlib.sha256(observations).hexdigest()
                if threads==1: reference_hashes[decimation]=digest
                if digest!=reference_hashes[decimation]: raise RuntimeError('Non-deterministic observations between worker counts')
                if summary['detection']['tags']!=args.frames: raise RuntimeError('Missing synthetic tags')
                error=0
                for index,line in enumerate(observations.splitlines()):
                    corners=json.loads(line)['tags'][0]['corners_px']
                    half=4*scale
                    for observed,(x,y) in zip(corners,[(-half,half),(half,half),(half,-half),(-half,-half)]):
                        for _ in range(index%4): x,y=-y,x
                        error=max(error,abs(observed[0]-width/2-x),abs(observed[1]-height/2-y))
                report['runs'].append({'resolution':[width,height],'frames':args.frames,'threads':threads,
                    'quad_decimate':decimation,'max_axis_corner_error_px':error,'observations_sha256':digest,**summary['execution']})
                print(f'{width}x{height}, {threads} threads, decimate {decimation}: {summary["execution"]["elapsed_s"]:.3f}s',flush=True)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,indent=2)+'\n')
