#!/usr/bin/env python3
"""Native CLI contract and real PNG/AprilTag integration fixtures (stdlib only)."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import signal
import time
import tempfile
import zlib

CAM, VI, RENDER = map(str, map(Path, sys.argv[1:4]))
EPOCH = 1_700_000_000_000_000_000
checks = 0

def run(binary, *args, code=0):
    global checks
    result = subprocess.run([binary, *map(str, args)], text=True, capture_output=True, timeout=45)
    assert result.returncode == code, (args, result.returncode, code, result.stdout, result.stderr)
    checks += 1
    return result

def chunk(name, data):
    return struct.pack('>I', len(data)) + name + data + struct.pack('>I', zlib.crc32(name+data))

def png(path, width=640, height=480, depth=8, corrupt=False, rgb=False):
    header = struct.pack('>IIBBBBB', width, height, depth, 2 if rgb else 0, 0, 0, 0)
    payload = zlib.compress((b'\0' + b'\xff' * width*(3 if rgb else 1)*(depth//8))*height)
    if corrupt:
        payload = b'bad deflate payload'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',header)+chunk(b'IDAT',payload)+chunk(b'IEND',b''))

def target(path, family='tag36h11', first=0, rows=1, cols=1):
    path.write_text(f'target_type: aprilgrid\ntagRows: {rows}\ntagCols: {cols}\ntagSize: 0.04\ntagSpacing: 0.3\ntagFamily: {family}\ntagBorderBits: 1\ntagStartId: {first}\n')

def imu(root, rows=None, header='timestamp_ns,wx,wy,wz,ax,ay,az'):
    path = root/'imu0'/'imu0.csv'
    path.parent.mkdir(exist_ok=True)
    if rows is None:
        rows = [f'{EPOCH+i*1_000_000},0,0,0,0,0,9.81' for i in range(31)]
    path.write_text(header+'\n'+'\n'.join(rows)+'\n')

def inspect(root, binary=CAM, code=0, extra=()):
    result = run(binary, '--dataset', root, '--check-dataset', *extra, code=code)
    return json.loads(result.stdout) if result.stdout else None

with tempfile.TemporaryDirectory(prefix='calib-test-') as temporary:
    base = Path(temporary)
    data = base/'data'
    for camera in ['cam10','cam2']:
        for delta in [20_000_000, 0, 10_000_000]:
            png(data/camera/f'{EPOCH+delta}.png')
    summary = inspect(data)
    assert [c['id'] for c in summary['cameras']] == ['cam2','cam10']
    assert summary['cameras'][0]['mean_rate_hz'] == 100
    assert summary['decoded_frames'] == 6
    assert summary['calibration']['quality'] == 'not_assessed'
    assert inspect(data, extra=('--cameras','cam10','cam2'))['cameras'][0]['id']=='cam10'
    inspect(data, extra=('--cameras','cam0'), code=2)
    inspect(data, extra=('--cameras','cam2','cam2'), code=2)
    run(CAM, '--help')
    assert '3.4.5' in run(CAM, '--version').stdout
    run(CAM, '--dataset', data, code=2)
    run(CAM, '--dataset', data, '--check-dataset','--detect-only', code=2)
    run(CAM, '--dataset', data, '--detect-only', code=2)
    run(CAM, '--dataset', data, '--initialize-only', code=2)
    run(CAM, '--dataset', data, '--initialize-only', '--detect-only', code=2)
    run(CAM, '--dataset', data, '--check-dataset', '--models', 'pinhole-equi', code=2)
    run(CAM, '--dataset', data, '--check-dataset', '--init-max-views', 30, code=2)
    init_target=base/'init-target.yaml'
    target(init_target)
    init_args=('--dataset',data,'--initialize-only','--target',init_target,'--models','pinhole-equi')
    rig_args=('--dataset',data,'--target',init_target,'--models','pinhole-equi')
    too_short=run(CAM,*rig_args,'--output',base/'failed-rig',code=3)
    assert 'common camera timestamps' in too_short.stderr
    single=run(CAM,*rig_args,'--cameras','cam2','--output',base/'single-rig',code=3)
    assert 'two selected cameras' in single.stderr
    run(CAM,*init_args,'--output',base/'failed-init',code=3)
    assert (base/'failed-init'/'error.json').exists()
    assert not (base/'failed-init'/'initial-camchain.yaml').exists()
    run(CAM,*init_args,'--cam','unavailable.yaml','--output',base/'cam-init',code=8)
    run(CAM,*init_args,'--init-max-views',9,'--output',base/'invalid-budget',code=2)
    run(CAM,'--dataset',data,'--initialize-only','--target',init_target,'--models','unknown',
        '--output',base/'unknown-model',code=2)
    assert not (base/'unknown-model').exists()
    inspect(data, extra=('--quad-decimate',2), code=2)
    inspect(data, VI, code=5)
    imu(data)
    noise=base/'imu-noise.yaml'
    noise.write_text('gyroscope_noise_density: 0.002\naccelerometer_noise_density: 0.02\n')
    run(VI,'--dataset',data,code=2)
    run(VI,'--dataset',data,'--check-dataset','--imu',noise,code=2)
    run(CAM,*rig_args,'--imu',noise,'--output',base/'invalid-vi-command',code=2)
    run(VI,*rig_args,'--imu',noise,'--output',base/'multiple-vi-cameras',code=2)
    bad_noise=base/'bad-imu-noise.yaml'
    bad_noise.write_text('gyroscope_noise_density: 0\naccelerometer_noise_density: 0.02\n')
    run(VI,*rig_args,'--cameras','cam2','--imu',bad_noise,'--output',base/'invalid-noise',code=2)
    for invalid_noise in ['[]\n','gyroscope_noise_density: 0.002\ngyroscope_noise_density: 0.003\naccelerometer_noise_density: 0.02\n']:
        bad_noise.write_text(invalid_noise)
        run(VI,*rig_args,'--cameras','cam2','--imu',bad_noise,'--output',base/'invalid-noise',code=2)
    bad_noise.write_text('gyroscope_noise_density: 0.002\naccelerometer_noise_density: 0.02\ngyroscope_random_walk: 0.001\n')
    run(VI,*rig_args,'--cameras','cam2','--imu',bad_noise,'--output',base/'invalid-noise',code=2)
    vi = inspect(data, VI)
    assert vi['imu']['samples']==31
    assert abs(vi['imu']['mean'][5]-9.81)<1e-12
    assert abs(vi['imu']['mean_rate_hz']-1000)<1e-9
    imu(data, [f'9.81,{EPOCH},0,0,0,0,0',f'9.81,{EPOCH+30_000_000},0,0,0,0,0'], 'az,timestamp_ns,wx,wy,wz,ax,ay')
    assert inspect(data, VI)['imu']['mean'][5]==9.81
    for rows in [ [f'{EPOCH},0,0,0,0,0,9.81']*2,
                  [f'{EPOCH},nan,0,0,0,0,9.81',f'{EPOCH+1},0,0,0,0,0,9.81'],
                  [f'{EPOCH+1},0,0,0,0,0,9.81',f'{EPOCH},0,0,0,0,0,9.81'] ]:
        imu(data,rows)
        inspect(data, VI, code=2)
    imu(data,['1,0,0,0,0,0,9.81','2,0,0,0,0,0,9.81'])
    inspect(data, VI, code=3)
    imu(data)
    original = data/'cam2'/f'{EPOCH}.png'
    duplicate = data/'cam2'/f'0{EPOCH}.png'
    shutil.copyfile(original,duplicate)
    inspect(data,code=2)
    duplicate.unlink()
    invalid = data/'cam2'/'9223372036854775808.png'
    shutil.copyfile(original,invalid)
    inspect(data,code=2)
    invalid.unlink()
    saved = original.read_bytes()
    png(original,corrupt=True)
    inspect(data,code=2)
    original.write_bytes(saved)
    png(original,width=641)
    inspect(data,code=2)
    original.write_bytes(saved)
    chain = base/'chain.yaml'
    camera_yaml = '  camera_model: pinhole\n  distortion_model: radtan\n  intrinsics: [400,400,320,240]\n  distortion_coeffs: [0,0,0,0]\n  resolution: [640,480]\n'
    for delay in ['line_delay: 0.00002','lineDelay: -0.00002','line_delay: 0']:
        chain.write_text('cam2:\n'+camera_yaml+'  '+delay+'\ncam10:\n'+camera_yaml)
        result = inspect(data,extra=('--cam',chain))
        assert abs(result['camchain'][0]['readout_time_s']-479*abs(result['camchain'][0]['line_delay_s']))<1e-14
    vi_args=('--dataset',data,'--target',init_target,'--cameras','cam2','--cam',chain,'--imu',noise)
    run(VI,*vi_args,'--models','pinhole-equi','--output',base/'vi-optics-conflict',code=2)
    run(VI,*vi_args,'--init-max-views',30,'--output',base/'vi-unused-views',code=2)
    run(VI,*vi_args,'--vi-bootstrap-replicates',8,'--output',base/'vi-bootstrap-missing-block',code=2)
    run(VI,*vi_args,'--vi-bootstrap-block-seconds',1,'--output',base/'vi-bootstrap-missing-count',code=2)
    run(VI,*vi_args,'--vi-bootstrap-seed',7,'--output',base/'vi-bootstrap-unused-seed',code=2)
    run(CAM,*rig_args,'--vi-bootstrap-replicates',8,'--vi-bootstrap-block-seconds',1,
        '--output',base/'rig-bootstrap-option',code=2)
    chain.write_text('cam2:\n'+camera_yaml+'  line_delay: 1e-5\n')
    run(VI,*vi_args,'--output',base/'vi-rolling-shutter',code=8)
    chain.write_text('cam2:\n'+camera_yaml+'  line_delay: 1e-5\n  lineDelay: 2e-5\n')
    inspect(data,extra=('--cam',chain),code=2)
    chain.write_text('cam2:\n'+camera_yaml+'  line_delay: 1e-5\n  line_delay: 1e-5\n')
    inspect(data,extra=('--cam',chain),code=2)
    big=base/'big'
    png(big/'cam0'/f'{EPOCH}.png',3840,2160)
    assert inspect(big)['cameras'][0]['resolution']==[3840,2160]
    inspect(big,extra=('--memory-budget-mb',32),code=2)
    png(big/'cam0'/f'{EPOCH}.png',depth=16)
    assert inspect(big)['cameras'][0]['bit_depth']==16
    png(big/'cam0'/f'{EPOCH}.png',rgb=True)
    assert inspect(big)['cameras'][0]['channels']==3
    grid=base/'target.yaml'
    output=base/'empty-detection'
    target(grid)
    result=run(CAM,'--dataset',data,'--detect-only','--target',grid,'--output',output,code=3)
    assert json.loads(result.stdout)['status']=='insufficient_data'
    assert (output/'detections.jsonl').exists()
    run(CAM,'--dataset',data,'--detect-only','--target',grid,'--output',output,code=5)
    for family in ['tag36h11','tagStandard41h12']:
        tags=base/family
        (tags/'cam0').mkdir(parents=True)
        target(grid,family,first=7)
        for rotation in range(4):
            run(RENDER,tags/'cam0'/f'{EPOCH+rotation*10_000_000}.png',family,7,rotation,640,480,16)
        serial=base/(family+'-serial')
        parallel=base/(family+'-parallel')
        for out, threads in [(serial,1),(parallel,4)]:
            result=run(CAM,'--dataset',tags,'--detect-only','--target',grid,'--output',out,'--threads',threads)
            assert json.loads(result.stdout)['detection']['tags']==4
        assert (serial/'detections.jsonl').read_bytes()==(parallel/'detections.jsonl').read_bytes()
        records=[json.loads(line) for line in (serial/'detections.jsonl').read_text().splitlines()]
        assert (serial/'observations.jsonl').read_bytes()==(parallel/'observations.jsonl').read_bytes()
        geometry=json.loads((serial/'target_geometry.json').read_text())
        assert geometry['units']=='m'
        assert [point['position_m'] for point in geometry['points']]==[[0,0,0],[.04,0,0],[.04,.04,0],[0,.04,0]]
        assert [point['tag_id'] for point in geometry['points']]==[7]*4
        corresponding=[json.loads(line) for line in (serial/'observations.jsonl').read_text().splitlines()]
        assert len(corresponding)==len(records)
        for raw, observation in zip(records,corresponding):
            assert observation['timestamp_ns']==raw['timestamp_ns'] and isinstance(observation['timestamp_ns'],int)
            assert [point['point_id'] for point in observation['points']]==[0,1,2,3]
            assert [point['pixel'] for point in observation['points']]==raw['tags'][0]['corners_px']
        # Independent border geometry: black detection border is 8 cells (36h11), 5 cells (41h12).
        half=(8 if family=='tag36h11' else 5)*16/2
        canonical=[[-half,half],[half,half],[half,-half],[-half,-half]]
        for rotation,record in enumerate(records):
            assert record['tags'][0]['id']==7 and record['tags'][0]['hamming']==0
            for observed,xy in zip(record['tags'][0]['corners_px'],canonical):
                x,y=xy
                for _ in range(rotation): x,y=-y,x
                assert abs(observed[0]-(320+x))<1.0 and abs(observed[1]-(240+y))<1.0, (family,rotation,observed,x,y)
        decimated=base/(family+'-decimated')
        run(CAM,'--dataset',tags,'--detect-only','--target',grid,'--output',decimated,'--quad-decimate',2)
        decimated_records=[json.loads(line) for line in (decimated/'detections.jsonl').read_text().splitlines()]
        for a,b in zip(records,decimated_records):
            for p,q in zip(a['tags'][0]['corners_px'],b['tags'][0]['corners_px']):
                assert max(abs(x-y) for x,y in zip(p,q))<1
        target(grid,family,first=0)
        rejected=base/(family+'-rejected')
        result=run(CAM,'--dataset',tags,'--detect-only','--target',grid,'--output',rejected,code=3)
        assert json.loads(result.stdout)['detection']['rejected_outside_grid']==4
        rejected_record=json.loads((rejected/'detections.jsonl').read_text().splitlines()[0])
        assert rejected_record['rejected_tags'][0]['id']==7
        assert rejected_record['rejected_tags'][0]['reason']=='outside_grid_id_range'
        rejected_observation=json.loads((rejected/'observations.jsonl').read_text().splitlines()[0])
        assert rejected_observation['points']==[]
        assert rejected_observation['rejected_tags']==[{'id':7,'reason':'outside_grid_id_range'}]
    legacy_data=base/'legacy-data'
    (legacy_data/'cam0').mkdir(parents=True)
    target(grid,first=7)
    grid.write_text(grid.read_text().replace('tagBorderBits: 1','tagBorderBits: 2'))
    for rotation in range(4):
        run(RENDER,legacy_data/'cam0'/f'{EPOCH+rotation}.png','tag36h11',7,rotation,640,480,16,'legacy')
    for threads in [1,4]:
        result=run(CAM,'--dataset',legacy_data,'--detect-only','--target',grid,'--output',base/f'legacy-{threads}','--threads',threads)
        assert json.loads(result.stdout)['detection']['tags']==4
    assert (base/'legacy-1/detections.jsonl').read_bytes()==(base/'legacy-4/detections.jsonl').read_bytes()
    for rotation,line in enumerate((base/'legacy-1/detections.jsonl').read_text().splitlines()):
        record=json.loads(line)
        for point,(x,y) in zip(record['tags'][0]['corners_px'],[(-80,80),(80,80),(80,-80),(-80,-80)]):
            for _ in range(rotation): x,y=-y,x
            assert abs(point[0]-320-x)<1 and abs(point[1]-240-y)<1
    for decimate in [2,4]:
        run(CAM,'--dataset',legacy_data,'--detect-only','--target',grid,'--output',base/f'legacy-decimate-{decimate}','--quad-decimate',decimate)
        for rotation,line in enumerate((base/f'legacy-decimate-{decimate}/detections.jsonl').read_text().splitlines()):
            record=json.loads(line)
            assert record['tags'][0]['id']==7
            for point,(x,y) in zip(record['tags'][0]['corners_px'],[(-80,80),(80,80),(80,-80),(-80,-80)]):
                for _ in range(rotation): x,y=-y,x
                assert abs(point[0]-320-x)<1 and abs(point[1]-240-y)<1
    target(grid,first=7)
    run(CAM,'--dataset',legacy_data,'--detect-only','--target',grid,'--output',base/'wrong-border',code=3)
    degraded=base/'degraded'
    (degraded/'cam0').mkdir(parents=True)
    target(grid)
    run(RENDER,degraded/'cam0'/f'{EPOCH}.png','tag36h11',0,1,640,480,16,'degraded')
    diagnostic=run(CAM,'--dataset',degraded,'--detect-only','--tag-family','tag36h11','--output',base/'family-scan')
    assert json.loads(diagnostic.stdout)['target']['geometry'] is None
    assert not (base/'family-scan/target_geometry.json').exists()
    assert not (base/'family-scan/observations.jsonl').exists()
    assert json.loads(diagnostic.stdout)['detection']['tags']==1
    run(CAM,'--dataset',degraded,'--detect-only','--tag-family','tag36h11','--target',grid,'--output',base/'ambiguous',code=2)
    result=run(CAM,'--dataset',degraded,'--detect-only','--target',grid,'--output',base/'degraded-out')
    assert json.loads(result.stdout)['detection']['tags']==1
    png(degraded/'cam1'/f'{EPOCH}.png')
    result=run(CAM,'--dataset',degraded,'--detect-only','--target',grid,'--output',base/'missing-camera-tags',code=3)
    assert json.loads(result.stdout)['detection']['cameras']['cam1']['tags']==0
    target(grid)
    run(RENDER,big/'cam0'/f'{EPOCH}.png','tag36h11',0,0,3840,2160,32)
    out=base/'4k-detection'
    result=run(CAM,'--dataset',big,'--detect-only','--target',grid,'--output',out,'--threads',4)
    assert json.loads(result.stdout)['detection']['tags']==1
    bad=base/'bad'
    png(bad/'cam0'/f'{EPOCH}.png',corrupt=True)
    failed=base/'failed'
    run(CAM,'--dataset',bad,'--detect-only','--target',grid,'--output',failed,code=2)
    assert (failed/'error.json').exists() and not (failed/'detections.jsonl').exists()
    assert not (failed/'observations.jsonl').exists()
    assert not (failed/'summary.json').exists()
    # Stop after the output has been opened: interruption must not publish a complete file.
    long_data=base/'interrupt-data'
    (long_data/'cam0').mkdir(parents=True)
    for i in range(64):
        (long_data/'cam0'/f'{EPOCH+i*1_000_000}.png').hardlink_to(big/'cam0'/f'{EPOCH}.png')
    interrupted_output=base/'interrupted'
    process=subprocess.Popen([CAM,'--dataset',str(long_data),'--detect-only','--target',str(grid),
                              '--output',str(interrupted_output),'--threads','1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        deadline=time.monotonic()+15
        while not (interrupted_output/'detections.jsonl.incomplete').exists() and process.poll() is None and time.monotonic()<deadline:
            time.sleep(.005)
        assert process.poll() is None, 'process must still be active at interruption'
        process.send_signal(signal.SIGINT)
        stdout,stderr=process.communicate(timeout=15)
        assert process.returncode==130, (process.returncode,stdout,stderr)
        assert not (interrupted_output/'detections.jsonl').exists()
        assert not (interrupted_output/'observations.jsonl').exists()
        assert json.loads((interrupted_output/'error.json').read_text())['exit_code']==130
        checks+=1
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    grid.write_text(grid.read_text().replace('tagBorderBits: 1','tagBorderBits: 3'))
    run(CAM,'--dataset',data,'--detect-only','--target',grid,'--output',base/'legacy',code=8)
print(f'{checks} subprocess checks passed; PNG decoding, IMU validation, row timing import, families, rotations, determinism and 4K')
