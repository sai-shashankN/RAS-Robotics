import time,json,hashlib
import numpy as np
from context import ROOT,SOURCE,STUDY,read,write,sha
from pose_estimator import estimate
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,allow_nan=False).encode()).hexdigest()
def main():
    assert not (STUDY/'checkpoint-index.json').exists() and not (STUDY/'timing-profile.json').exists();write(STUDY/'timing-plan.json',dict(scope='Descriptive host timing only,zero native launches.60warm image-estimator invocations:5repetitions each of12frozen task images. PreloadRGB/calibration. Time RGBprefilter,decode and joint solve with monotonic wall and process CPU clocks. Exclude native capture,disk load,transport,synchronization and control scheduling. No hardware or deadline qualification and no threshold adjustment.',frozen_estimator_sha256=sha(STUDY/'frozen-estimator.json')));cases=[]
    for r in read(STUDY/'summary.json')['results']:
        if r['entry']['role']!='task':continue
        out=STUDY/r['directory'];cal=read(out/'sources/calibration.json');frames=[]
        for c in cal['cameras']:
            a=np.frombuffer((out/(c['id']+'-rgb.bgra')).read_bytes(),np.uint8).reshape(480,640,4);frames.append(dict(camera_id=c['id'],width=640,height=480,rgb=a[:,:,[2,1,0]].copy().tobytes()))
        cases.append((r['entry']['id'],frames,cal,digest(read(out/'image-estimates.json')['clean'])))
    estimate(cases[0][1],cases[0][2],'apriltag');samples=[]
    for repetition in range(5):
        for name,frames,cal,expected in cases:
            wall=time.perf_counter_ns();cpu=time.process_time_ns();r=estimate(frames,cal,'apriltag');cpu=time.process_time_ns()-cpu;wall=time.perf_counter_ns()-wall;samples.append(dict(case=name,repetition=repetition,wall_ms=wall/1e6,cpu_ms=cpu/1e6,exact_saved_output=digest(r)==expected))
    walls=[s['wall_ms'] for s in samples];result=dict(simulator_runs=0,estimator_invocations=len(samples),samples=samples,all_outputs_exact=all(s['exact_saved_output'] for s in samples),wall_median_ms=float(np.median(walls)),wall_p95_ms=float(np.percentile(walls,95)),wall_max_ms=max(walls),runtime_deadline_admitted=False);write(STUDY/'timing-profile.json',result);print({k:v for k,v in result.items() if k!='samples'},flush=True)
if __name__=='__main__':main()
