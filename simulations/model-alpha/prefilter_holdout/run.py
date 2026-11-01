from concurrent.futures import ThreadPoolExecutor
import math,cv2,numpy as np
from context import ROOT,SOURCE,STUDY,read,write,sha
import native_base as base
from pose_estimator import estimate,angle

def assess(e,out):
    cal=read(out/'sources/calibration.json');variants={v:[] for v in ['clean','dim','blur','noise']}
    for i,c in enumerate(cal['cameras']):
        raw=np.frombuffer((out/(c['id']+'-rgb.bgra')).read_bytes(),np.uint8).reshape(480,640,4);rgb=raw[:,:,[2,1,0]].copy();rng=np.random.default_rng(20261027+e['sequence']*11+i);images=dict(clean=rgb,dim=np.clip(rgb.astype(float)*.7,0,255).astype(np.uint8),blur=cv2.GaussianBlur(rgb,(3,3),.6),noise=np.clip(rgb.astype(float)+rng.normal(0,2,rgb.shape),0,255).astype(np.uint8))
        for name,image in images.items():variants[name].append(dict(camera_id=c['id'],width=640,height=480,rgb=image.tobytes()))
    estimates={name:estimate(frames,cal,'apriltag') for name,frames in variants.items()};write(out/'image-estimates.json',estimates)
    scene=read(out/'sources/scene.json');evaluated=[]
    for name,r in estimates.items():
        chosen=r['chosen'];pos=float(np.linalg.norm(np.array(chosen['position_m'])-scene['cube']['center'])) if chosen else None;rot=angle(chosen['rotation'],scene['cube']['rotation']) if chosen else None;evaluated.append(dict(variant=name,status=r['status'],returned_pose=chosen is not None,center_error_m=pos,rotation_error_rad=rot,returned_pose_within_diagnostic_limits=pos<=.001 and rot<=math.radians(1) if chosen else None,controller_usable=r['controller_usable']))
    capture=read(out/'capture.json');launch=read(out/'launch-inputs.json');checks=dict(capture_complete=len(capture['views'])==len(cal['cameras']) and all(v['width']==640 and v['height']==480 for v in capture['views']),static_only=capture['robot_dynamic_trials']==0,source_bound=all(sha(out/'sources'/n)==h for n,h in launch['sha256'].items()),stderr_empty=(out/'webots-stderr.log').stat().st_size==0,frozen_algorithm=all(sha(SOURCE/n)==h for n,h in read(STUDY/'frozen-estimator.json')['sha256'].items()))
    if e['role']=='positive':checks['positive_clean_pose']=bool(evaluated[0]['returned_pose_within_diagnostic_limits'])
    if e['role']=='negative':checks['negative_no_pose']=all(not v['returned_pose'] for v in evaluated)
    return dict(entry=e,directory=str(out.relative_to(STUDY)),passed=all(checks.values()),checks=checks,pose_outcomes=evaluated,input_sha256={str(f.relative_to(out)):sha(f) for f in out.iterdir() if f.is_file() and f.name!='audit.json'})
def main():
    assert not (STUDY/'checkpoint-index.json').exists() and not list(STUDY.glob('*_status.json'));entries=read(STUDY/'manifest.json')['entries'];assert len(entries)==14;assert all(not (ROOT/e['project']/'results').exists() for e in entries);base.assess=assess;results=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for i in range(0,14,2):
            pair=list(pool.map(base.one,entries[i:i+2]));results+=pair;write(STUDY/f'{i//2:02}_status.json',dict(complete=True,results=pair));write(STUDY/'summary.json',dict(simulator_runs=len(results),dynamic_robot_trials=0,results=results,selected_pose_admitted=False,camera_perception_admitted=False,full_m2_admitted=False))
            if not all(r['passed'] for r in pair):write(STUDY/'matrix-stop.json',dict(pair=i//2,reason='Capture/control/frozen-input gate rejection;later cases unrun.'));break
    s=read(STUDY/'summary.json');metrics=[]
    for name in ['clean','dim','blur','noise']:
        rows=[v for r in results if r['entry']['role']=='task' for v in r['pose_outcomes'] if v['variant']==name];returned=[v for v in rows if v['returned_pose']];metrics.append(dict(variant=name,task_cases=len(rows),returned=len(returned),missing_or_rejected=len(rows)-len(returned),returned_within_limits=sum(bool(v['returned_pose_within_diagnostic_limits']) for v in returned),max_center_error_m=max(v['center_error_m'] for v in returned) if returned else None,max_rotation_error_rad=max(v['rotation_error_rad'] for v in returned) if returned else None))
    s.update(metrics=metrics,static_benchmark_complete=len(results)==14 and all(r['passed'] for r in results));write(STUDY/'summary.json',s);print(metrics,flush=True)
if __name__=='__main__':main()
