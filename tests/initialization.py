"""Independent synthetic optical/pose truth: Python math only, no OpenCV renderer."""
import copy
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def synthesize(model, noise=0., outliers=False, static=False):
    rng = random.Random(327)
    width, height = (640, 480) if model == 'radtan' else (512, 512)
    params = ([410., 397., 318., 239., -.07, .015, .002, -.001]
              if model == 'radtan' else [205., 210., 253., 249., .025, -.006, .001, -.0001])
    frames, truth = [], {}
    for index in range(30):
        n = 0 if static else index
        ax, ay, az = .5*math.sin(n*1.3), .5*math.cos(n*.7), .25*math.sin(n*.4)
        sx, cx, sy, cy, sz, cz = math.sin(ax), math.cos(ax), math.sin(ay), math.cos(ay), math.sin(az), math.cos(az)
        R = multiply(multiply([[cz,-sz,0],[sz,cz,0],[0,0,1]], [[cy,0,sy],[0,1,0],[-sy,0,cy]]),
                     [[1,0,0],[0,cx,-sx],[0,sx,cx]])
        center = [.13*math.sin(n*.9), .09*math.cos(n*1.1), .46+.20*(1+math.sin(n*.6))/2]
        translation = [center[i]-.15*(R[i][0]+R[i][1]) for i in range(3)]
        stamp = 1_700_000_000_000_000_001+index*50_000_000
        truth[stamp] = [R[i]+[translation[i]] for i in range(3)]+[[0,0,0,1]]
        points = []
        for tag in range(36):
            for corner, (dx,dy) in enumerate(((0,0),(1,0),(1,1),(0,1))):
                X, Y = (tag%6)*.052+dx*.04, (tag//6)*.052+dy*.04
                camera = [R[i][0]*X+R[i][1]*Y+translation[i] for i in range(3)]
                x,y = camera[0]/camera[2],camera[1]/camera[2]
                r2=x*x+y*y
                k1,k2,d3,d4=params[4:]
                if model=='radtan':
                    scale=1+k1*r2+k2*r2*r2
                    xd=x*scale+2*d3*x*y+d4*(r2+2*x*x)
                    yd=y*scale+d3*(r2+2*y*y)+2*d4*x*y
                else:
                    r=math.sqrt(r2); theta=math.atan(r)
                    td=theta+k1*theta**3+k2*theta**5+d3*theta**7+d4*theta**9
                    xd,yd=(x*td/r,y*td/r) if r else (0,0)
                u,v=params[0]*xd+params[2]+rng.gauss(0,noise),params[1]*yd+params[3]+rng.gauss(0,noise)
                if outliers and index%5!=4 and tag==10 and corner==0:
                    u+=20; v-=15
                if 0<=u<width and 0<=v<height:
                    points.append(dict(point_id=4*tag+corner,pixel=[u,v]))
        frames.append(dict(timestamp_ns=stamp,points=points))
    return dict(model=model,resolution=[width,height],target=dict(rows=6,cols=6,size=.04,spacing=.3),frames=frames),params,truth


def run(data, output=None, code=0):
    command=[sys.argv[1]]+([str(output)] if output else [])
    result=subprocess.run(command,input=json.dumps(data),text=True,capture_output=True,timeout=120)
    assert result.returncode==code,(result.returncode,result.stderr)
    return json.loads(result.stdout) if code==0 else result.stderr


def main():
    with tempfile.TemporaryDirectory() as temporary:
        for model in ('radtan','equidistant'):
            data,params,poses=synthesize(model)
            output=Path(temporary)/(model+'.yaml')
            result=run(data,output)
            assert result['status']=='initial_estimate'
            assert result['covariance'] is None
            for actual,expected in zip(result['intrinsics'],params[:4]):
                assert abs(actual-expected)<.002,(model,actual,expected)
            for actual,expected in zip(result['distortion_coeffs'],params[4:]):
                assert abs(actual-expected)<.0003,(model,actual,expected)
            assert result['training_rms_px']<1e-4,result
            assert result['held_out_rms_px']<1e-4,result
            assert result['distortion_check']['max_inverse_roundtrip_ray_error']<1e-7
            assert sum(p['held_out'] for p in result['poses'])==6
            for pose in result['poses']:
                expected=poses[pose['timestamp_ns']]
                assert max(abs(a-b) for row,truth in zip(pose['T_cam_target'],expected) for a,b in zip(row,truth))<1e-5
            # Parse scalar arrays independently of yaml-cpp and check no invented transforms/readout.
            fields={line.split(':',1)[0].strip():line.split(':',1)[1].strip()
                    for line in output.read_text().splitlines() if ':' in line and not line.startswith('#')}
            assert json.loads(fields['intrinsics'])==result['intrinsics']
            assert json.loads(fields['distortion_coeffs'])==result['distortion_coeffs']
            assert 'T_cam_imu' not in fields and 'T_cn_cnm1' not in fields and 'line_delay' not in fields
            noisy,_,_=synthesize(model,.12,True)
            robust=run(noisy)
            assert len(robust['rejected_points'])>=20,robust['rejected_points']
            assert robust['held_out_rms_px']<.3,robust['held_out_rms_px']
            for actual,expected in zip(robust['intrinsics'],params[:4]):
                assert abs(actual-expected)<1.5,(model,actual,expected)
            degenerate,_,_=synthesize(model,static=True)
            run(degenerate,code=3)
            small=copy.deepcopy(data);small['frames']=small['frames'][:9];run(small,code=3)
            invalid=copy.deepcopy(data);invalid['frames'][0]['points'][0]['point_id']=1000;run(invalid,code=2)
            invalid=copy.deepcopy(data);invalid['frames'][1]['timestamp_ns']=invalid['frames'][0]['timestamp_ns'];run(invalid,code=2)
            cancelled=copy.deepcopy(data);cancelled['cancelled']=True;run(cancelled,code=130)
            print(model, 'truth, noisy/outlier recovery, held-out poses, export and degeneracy passed',flush=True)


if __name__=='__main__':
    main()
