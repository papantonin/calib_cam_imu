"""Independent Python geometry generates truth; no OpenCV/Ceres synthesis."""
import copy
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from initialization import synthesize, multiply


def project(p, parameters, model):
    x,y=p[0]/p[2],p[1]/p[2];r2=x*x+y*y
    k1,k2,d3,d4=parameters[4:]
    if model=='radtan':
        scale=1+k1*r2+k2*r2*r2
        xd=x*scale+2*d3*x*y+d4*(r2+2*x*x)
        yd=y*scale+d3*(r2+2*y*y)+2*d4*x*y
    else:
        r=math.sqrt(r2);t=math.atan(r)
        scale=(t+k1*t**3+k2*t**5+d3*t**7+d4*t**9)/r if r else 1
        xd,yd=x*scale,y*scale
    return [parameters[0]*xd+parameters[2],parameters[1]*yd+parameters[3]]


def inverse(T):
    R=[list(r) for r in zip(*[row[:3] for row in T[:3]])]
    return [R[i]+[-sum(R[i][k]*T[k][3] for k in range(3))] for i in range(3)]+[[0,0,0,1]]


def fixture(model,noise=0.,outliers=False,chain=False,static=False):
    source,p,poses=synthesize(model,static=static)
    rng=random.Random(831);cameras=[];transforms=[];truth=[]
    for i in range(3 if chain else 2):
        angle=-.05*i;R=[[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]]
        center=[.12*i,.004*i,.002*i]
        T=[R[r]+[-sum(R[r][k]*center[k] for k in range(3))] for r in range(3)]+[[0,0,0,1]]
        transforms.append(T)
        params=p.copy();params[0]*=1+.02*i;params[1]*=1-.01*i;params[2]-=3*i
        truth.append(params)
        guess=params.copy();guess[0]*=1.01;guess[1]*=.99;guess[2]+=.5
        cameras.append(dict(id=f'cam{i}',model=model,intrinsics=guess[:4],distortion=guess[4:],resolution=source['resolution']))
    views=[]
    for index,frame in enumerate(source['frames']):
        view=dict(timestamp_ns=frame['timestamp_ns'],held_out=index%5==4,cameras=[])
        active=([0,1] if index<15 else [1,2]) if chain else [0,1]
        for i in active:
            T=multiply(transforms[i],poses[frame['timestamp_ns']]);points=[]
            for tag in range(36):
                for corner,(dx,dy) in enumerate(((0,0),(1,0),(1,1),(0,1))):
                    X,Y=(tag%6)*.052+dx*.04,(tag//6)*.052+dy*.04
                    point=[T[r][0]*X+T[r][1]*Y+T[r][3] for r in range(3)]
                    uv=project(point,truth[i],model)
                    uv=[v+rng.gauss(0,noise) for v in uv]
                    if outliers and not view['held_out'] and tag==10 and corner==0:
                        uv[0]+=15;uv[1]-=12
                    if 0<=uv[0]<source['resolution'][0] and 0<=uv[1]<source['resolution'][1]:
                        points.append(dict(point_id=tag*4+corner,pixel=uv))
            view['cameras'].append(dict(camera=f'cam{i}',resolution=source['resolution'],points=points))
        views.append(view)
    return dict(cameras=cameras,views=views),transforms,truth


def run(data,output=None,code=0):
    command=[sys.argv[1]]+([str(output)] if output else [])
    process=subprocess.run(command,input=json.dumps(data),text=True,capture_output=True,timeout=120)
    assert process.returncode==code,(process.returncode,process.stderr)
    return json.loads(process.stdout) if code==0 else process.stderr


def max_difference(a,b):
    return max(abs(x-y) for row,other in zip(a,b) for x,y in zip(row,other))


def main():
    with tempfile.TemporaryDirectory() as temporary:
        for model in ('radtan','equidistant'):
            data,T,truth=fixture(model)
            folder=Path(temporary)/model
            result=run(data,folder)
            for i,c in enumerate(result['cameras']):
                assert max_difference(c['T_cam_reference'],T[i])<2e-5,c
                assert max(abs(a-b) for a,b in zip(c['intrinsics'],truth[i][:4]))<.01,c
                assert c['held_out_rms_px']<1e-4,c
            assert result['cameras'][1]['T_cam_reference'][0][3]<0
            assert result['cameras'][1]['camera_center_in_reference_m'][0]>0
            assert result['diagnostics']['covariance'] is None
            # Independently parse the row arrays in the exported Kalibr transformation.
            rows=[];inside=False
            for line in (folder/'camchain.yaml').read_text().splitlines():
                if 'T_cn_cnm1:' in line:inside=True
                elif inside and line.strip().startswith('- ['):rows.append(json.loads(line.strip()[2:]))
            assert max_difference(rows,T[1])<2e-5,rows
            info=subprocess.run([sys.argv[2],str(folder/'report.pdf')],text=True,capture_output=True,check=True)
            assert 'Pages:' in info.stdout and '3' in info.stdout,info.stdout
            text=subprocess.run([sys.argv[3],str(folder/'report.pdf'),'-'],text=True,capture_output=True,check=True).stdout
            assert 'Training residual histogram' in text and 'Camera optical center' in text,text
            swapped=copy.deepcopy(data);swapped['cameras'].reverse()
            reverse=run(swapped)
            assert reverse['reference_camera']=='cam1'
            assert max_difference(reverse['cameras'][1]['T_cam_reference'],inverse(T[1]))<2e-5
            bad=copy.deepcopy(data);bad['cameras'].append(dict(data['cameras'][0],id='cam2'))
            assert 'Disconnected' in run(bad,code=3)
            # Explicit timestamp mismatch, one nanosecond only.
            bad=copy.deepcopy(data);bad['views'][0]['cameras'][1]['timestamp_ns']=bad['views'][0]['timestamp_ns']+1
            run(bad,code=2)
            bad=copy.deepcopy(data);bad['cancelled']=True;run(bad,code=130)
            static,_,_=fixture(model,static=True);run(static,code=3)
            noisy,_,_=fixture(model,.1,True)
            robust=run(noisy)
            assert max_difference(robust['cameras'][1]['T_cam_reference'],T[1])<.003,robust['cameras'][1]
            assert robust['cameras'][1]['held_out_rms_px']<.5
            assert len(robust['diagnostics']['downweighted_points'])>=30
            print(model,'truth, transform direction, noise/outliers, degeneracy, YAML/PDF passed',flush=True)
        chain,T,_=fixture('equidistant',chain=True)
        result=run(chain)
        assert max_difference(result['cameras'][2]['T_cam_reference'],T[2])<2e-5
        print('Three-camera connected chain without cam0/cam2 co-observations passed',flush=True)


if __name__=='__main__':main()
