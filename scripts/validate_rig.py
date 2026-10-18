#!/usr/bin/env python3
"""Private synchronized 512/1024 rig regression; no measured extrinsic ground truth."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

from validate_initialization import require, sha


def flatten(value):
    return [x for row in value for x in row]


def max_difference(a, b):
    require(len(a) == len(b), 'different parameter dimensions')
    require(all(math.isfinite(x) for x in a + b), 'nonfinite calibration')
    return max(abs(x-y) for x, y in zip(a, b, strict=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/release/calib_cameras'))
    parser.add_argument('--dataset-root', type=Path, default=Path('dataset'))
    parser.add_argument('--target', type=Path, default=Path('dataset/dtvi_aprilgrid.resolved.yaml'))
    parser.add_argument('--baseline', type=Path, default=Path('dataset/validation/camera-models-v1'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(status='running', binary_sha256=sha(args.binary), target_sha256=sha(args.target),
                  tolerances=dict(optical_parameters=1e-6, transform_entries=1e-8, rms_px=1e-6), datasets={})
    start = time.monotonic()
    try:
        for name, decimate in [('512', 2), ('1024', 4)]:
            runs, folders, executions = [], [], []
            for threads in (4, 1):
                folder = args.output/f'{name}-rig-{threads}'
                folders.append(folder)
                print(f'{name}: calibrate with {threads} threads', flush=True)
                command = [str(args.binary.resolve()), '--dataset', str(args.dataset_root/name),
                           '--target', str(args.target), '--models', 'pinhole-equi',
                           '--quad-decimate', str(decimate), '--threads', str(threads), '--output', str(folder)]
                result = subprocess.run(command, text=True, capture_output=True, timeout=300)
                (args.output/f'{name}-{threads}.stderr.log').write_text(result.stderr)
                require(result.returncode == 0, result.stderr)
                data = json.loads((folder/'calibration.json').read_text())
                require(data['status'] == 'converged', 'solver did not converge')
                require(data['diagnostics']['covariance'] is None, 'unknown covariance must remain null')
                reserved = {p['timestamp_ns'] for p in data['diagnostics']['poses'] if p['held_out']}
                reserved.update(p['timestamp_ns'] for p in data['diagnostics']['rejected_views'] if p['held_out'])
                initial = json.loads((folder/'initialization.json').read_text())
                for camera in initial['cameras']:
                    require(not reserved.intersection(p['timestamp_ns'] for p in camera['poses']),
                            'rig held-out timestamp leaked into monocular initialization')
                pdf = subprocess.run(['pdftotext', str(folder/'report.pdf'), '-'],
                                     check=True, capture_output=True, text=True, timeout=30)
                require('provisional' in pdf.stdout and 'Camera-camera calibration' in pdf.stdout,
                        'PDF lacks status or calibration report')
                runs.append(data)
                executions.append(json.loads((folder/'summary.json').read_text())['execution'])
            for file in ('detections.jsonl', 'observations.jsonl', 'target_geometry.json',
                         'initialization.json', 'initial-camchain.yaml'):
                require(sha(folders[0]/file) == sha(folders[1]/file), f'{name}: nondeterministic {file}')
            for file in ('detections.jsonl', 'observations.jsonl', 'target_geometry.json'):
                require(sha(folders[0]/file) == sha(args.baseline/f'{name}-detect-4'/file),
                        f'{name}: changed detection baseline {file}')
            differences = []
            for a, b in zip(runs[0]['cameras'], runs[1]['cameras'], strict=True):
                require(a['camera'] == b['camera'], 'camera order changed')
                differences.append(dict(camera=a['camera'],
                    optical_parameters=max_difference(a['intrinsics']+a['distortion_coeffs'], b['intrinsics']+b['distortion_coeffs']),
                    transform_entries=max_difference(flatten(a['T_cam_reference']), flatten(b['T_cam_reference'])),
                    rms_px=max_difference([a['training_rms_px'], a['held_out_rms_px']], [b['training_rms_px'], b['held_out_rms_px']])))
                for key, tolerance in report['tolerances'].items():
                    require(differences[-1][key] <= tolerance, f'{name}: thread stability failed for {key}')
            report['datasets'][name] = dict(cameras=runs[0]['cameras'], execution_4_then_1=executions,
                differences_4_vs_1=differences, observations_unchanged=True, held_out_excluded=True,
                training_views=runs[0]['diagnostics']['training_views'], held_out_views=runs[0]['diagnostics']['held_out_views'])
        lo, hi = [report['datasets'][name]['cameras'][1] for name in ('512', '1024')]
        center_difference = [b-a for a, b in zip(lo['camera_center_in_reference_m'], hi['camera_center_in_reference_m'], strict=True)]
        rotation_trace = sum(lo['T_cam_reference'][i][j]*hi['T_cam_reference'][i][j] for i in range(3) for j in range(3))
        report['cross_resolution_diagnostic_not_ground_truth'] = dict(
            camera_center_difference_m=center_difference, camera_center_distance_m=math.sqrt(sum(x*x for x in center_difference)),
            rotation_distance_rad=math.acos(max(-1., min(1., (rotation_trace-1)/2))))
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        report['elapsed_s'] = time.monotonic()-start
        (args.output/'regression.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
