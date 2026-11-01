import ast,math,os,shutil,subprocess,sys,time,traceback
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from context import ROOT,SOURCE,STUDY,read,write,sha
sys.path.insert(0,str(ROOT/'.local-tools/plotting-packages'))
import numpy as np
from pinhole import camera_rotation,project

def assess(e,out):
    scene=read(out/'sources/scene.json');capture=read(out/'capture.json');o=scene['cube'];center=np.array(o['center']);R=np.array(o['rotation']);corners=np.array([center+R@(np.array([x,y,z])*np.array(o['size'])/2) for x in [-1,1] for y in [-1,1] for z in [-1,1]]);views=[]
    for c in scene['cameras']:
        p=out/(c['id']+'-mask.bgra');mask=np.frombuffer(p.read_bytes(),dtype=np.uint8).reshape(480,640,4);rgb=np.frombuffer((out/(c['id']+'-rgb.bgra')).read_bytes(),dtype=np.uint8).reshape(480,640,4);target=(mask[:,:,:3]>250).all(axis=2);y,x=np.where(target);pix,inside,z=project(corners,np.array(c['position']),camera_rotation(c['direction']));bbox=[pix.min(axis=0).tolist(),pix.max(axis=0).tolist()];b=[int(x.min()),int(y.min()),int(x.max()),int(y.max())] if len(x) else None;meta=next(v for v in capture['views'] if v['camera']==c['id']);check=dict(dimensions=mask.shape==rgb.shape==(480,640,4),optics=meta['width']==640 and meta['height']==480 and abs(meta['fov']-math.radians(70))<1e-10 and abs(meta['near']-.001)<1e-12,pngs_saved=all((out/(c['id']+'-'+k+'.png')).stat().st_size>100 for k in ['rgb','mask']),mask_inside_projected_cube=(b is None or b[0]>=bbox[0][0]-2 and b[1]>=bbox[0][1]-2 and b[2]<=bbox[1][0]+2 and b[3]<=bbox[1][1]+2))
        if e['id']=='control_visible':check['manufactured_visible']=len(x)>5000 and meta['ground_truth_objects']==1
        elif e['id']=='control_occluded':check['manufactured_occluded']=len(x)==0 and meta['ground_truth_objects']==0
        elif c.get('group')=='shaft_ends_aimed':check['aimed_cube_visible']=len(x)>0;check['all_cube_corners_in_frame']=bool(inside.all())
        views.append(dict(camera=c['id'],target_pixels=len(x),mask_bbox_px=b,projected_cube_bbox_px=bbox,checks=check,passed=all(check.values())))
    launch=read(out/'launch-inputs.json');checks=dict(static_sandbox=capture['static_render_sandbox'] and capture['robot_dynamic_trials']==0,all_views=len(views)==len(scene['cameras']) and all(v['passed'] for v in views),launch_inputs_bound=all(sha(out/'sources'/n)==h for n,h in launch['sha256'].items()) and launch['plan_sha256']==sha(STUDY/'test-plan.json'),no_stderr=(out/'webots-stderr.log').stat().st_size==0)
    return dict(entry=e,directory=str(out.relative_to(STUDY)),checks=checks,views=views,passed=all(checks.values()),input_sha256={str(f.relative_to(out)):sha(f) for f in sorted(out.iterdir()) if f.is_file() and f.name!='audit.json'})

def one(e):
    p=ROOT/e['project'];out=p/'results';out.mkdir();hashes={}
    for f in p.rglob('*'):
        rel=f.relative_to(p)
        if f.is_file() and 'results' not in rel.parts and '__pycache__' not in rel.parts:
            d=out/'sources'/rel;d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,d);hashes[str(rel)]=sha(d)
    write(out/'launch-inputs.json',dict(entry=e,plan_sha256=sha(STUDY/'test-plan.json'),sha256=hashes));install=ROOT/'.local-tools/webots';binary=install/'msys64/mingw64/bin';system=Path(os.environ['SystemRoot']);env=dict(os.environ,WEBOTS_HOME=str(install),CAMERA_AUDIT_DIR=str(out));env['PATH']=';'.join(map(str,[binary,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow=subprocess.SW_HIDE;start=time.perf_counter()
    try:
        with (out/'webots-stdout.log').open('w') as a,(out/'webots-stderr.log').open('w') as b:r=subprocess.run([str(binary/'webots.exe'),'--batch','--mode=fast','--stdout','--stderr',str(p/'worlds/scene.wbt')],cwd=p,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=a,stderr=b,timeout=90)
        write(out/'process.json',dict(returncode=r.returncode,wall_seconds=time.perf_counter()-start));assert r.returncode==0 and (out/'capture.json').exists();result=assess(e,out);write(out/'audit.json',result)
    except Exception:result=dict(entry=e,directory=str(out.relative_to(STUDY)),checks=dict(infrastructure_or_capture=False),passed=False,error=traceback.format_exc());write(out/'infrastructure-error.json',result)
    print(dict(id=e['id'],passed=result['passed'],failed=[k for k,v in result['checks'].items() if not v]),flush=True);return result

def main():
    assert not (STUDY/'checkpoint-index.json').exists() and not list(STUDY.glob('*_status.json'));subprocess.run([sys.executable,str(SOURCE/'preflight.py')],check=True);entries=read(STUDY/'manifest.json')['entries'];assert len(entries)==10;results=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for i in range(0,len(entries),2):
            pair=list(pool.map(one,entries[i:i+2]));results+=pair;okay=all(v['passed'] for v in pair);write(STUDY/f'{i//2:02d}_status.json',dict(complete=True,results=pair));write(STUDY/'summary.json',dict(simulator_runs=len(results),dynamic_robot_trials=0,results=results,static_render_screen_admitted=len(results)==10 and all(r['passed'] for r in results),camera_perception_admitted=False,full_m2_admitted=False))
            if not okay:write(STUDY/'matrix-stop.json',dict(pair=i//2,reason='Capture/infrastructure/rendered-pixel gate failure;later scenes unrun.'));break
if __name__=='__main__':main()
