#!/usr/bin/env python3
"""Private 512/1024 initialization regression; does not certify optical accuracy."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,default=Path('build/release/calib_cameras'))
    parser.add_argument('--dataset-root',type=Path,default=Path('dataset'))
    parser.add_argument('--target',type=Path,default=Path('dataset/dtvi_aprilgrid.resolved.yaml'))
    parser.add_argument('--baseline',type=Path,default=Path('dataset/validation/camera-models-v1'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    report=dict(status='running',binary_sha256=sha(args.binary),target_sha256=sha(args.target),datasets={})
    start=time.monotonic()
    try:
        for name,decimate in [('512',2),('1024',4)]:
            folders=[]
            for threads in (4,1):
                folder=args.output/f'{name}-init-{threads}'
                folders.append(folder)
                print(f'{name}: initialize with {threads} workers',flush=True)
                command=[str(args.binary.resolve()),'--dataset',str(args.dataset_root/name),
                         '--initialize-only','--target',str(args.target),'--models','pinhole-equi',
                         '--quad-decimate',str(decimate),'--threads',str(threads),'--output',str(folder)]
                result=subprocess.run(command,text=True,capture_output=True,timeout=300)
                (args.output/f'{name}-{threads}.stderr.log').write_text(result.stderr)
                require(result.returncode==0, result.stderr)
            for file in ('detections.jsonl','observations.jsonl','target_geometry.json','initialization.json','initial-camchain.yaml'):
                require(sha(folders[0]/file)==sha(folders[1]/file),f'{name}: nondeterministic {file}')
            for file in ('detections.jsonl','observations.jsonl','target_geometry.json'):
                require(sha(folders[0]/file)==sha(args.baseline/f'{name}-detect-4'/file),f'{name}: changed baseline {file}')
            data=json.loads((folders[0]/'initialization.json').read_text())
            report['datasets'][name]=dict(status='passed',identical_1_vs_4_workers=True,
                observations_unchanged=True,cameras=[{k:c[k] for k in
                  ('camera','intrinsics','distortion_coeffs','training_rms_px','held_out_rms_px','distortion_check')}
                  for c in data['cameras']])
        # Same acquisition in two resolutions: a consistency diagnostic, not ground truth.
        report['scaled_intrinsics_difference_1024_over_2_minus_512']={}
        for lo,hi in zip(report['datasets']['512']['cameras'],report['datasets']['1024']['cameras'],strict=True):
            require(lo['camera']==hi['camera'],'camera ordering changed')
            report['scaled_intrinsics_difference_1024_over_2_minus_512'][lo['camera']]=[
                b/2-a for a,b in zip(lo['intrinsics'],hi['intrinsics'],strict=True)]
        report['status']='passed'
    except Exception as error:
        report['status']='failed';report['error']=str(error)
        raise
    finally:
        report['elapsed_s']=time.monotonic()-start
        (args.output/'regression.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    main()
