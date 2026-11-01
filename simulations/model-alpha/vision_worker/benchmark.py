import os,time,json,hashlib,numpy as np
from context import ROOT,SOURCE,STUDY,read,write,sha
from broker import Broker
from test_transport import wait_for,ready
def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,allow_nan=False).encode()).hexdigest()
def main():
    assert not (STUDY/'checkpoint-index.json').exists() and not (STUDY/'summary.json').exists();prior=STUDY.parent/'prefilter_holdout_v1';source=read(prior/'summary.json')['results'];cal=read(prior/source[0]['directory']/'sources/calibration.json');cal_id=sha(prior/source[0]['directory']/'sources/calibration.json');records=[];publish_times=[];poll_times=[]
    with Broker(cal,cal_id,minimum_latency_s=.08,max_age_s=.1) as b:
        ready(b)
        for sequence,entry in enumerate(source,1):
            out=prior/entry['directory'];assert read(out/'sources/calibration.json')==cal;frames=[]
            for c in cal['cameras']:
                a=np.frombuffer((out/(c['id']+'-rgb.bgra')).read_bytes(),np.uint8).reshape(480,640,4);frames.append(a[:,:,[2,1,0]].copy().tobytes())
            capture=sequence*.2;start=time.perf_counter_ns();status=b.publish(sequence,(capture,capture),tuple(frames),capture);publish_times.append((time.perf_counter_ns()-start)/1e6);assert status=='published';wait_for(lambda:b.stats[2]==sequence);before=b.poll(capture+.04);assert before['status']=='scheduled_wait';start=time.perf_counter_ns();result=b.poll(capture+.088);poll_times.append((time.perf_counter_ns()-start)/1e6);assert result['status']=='delivered';r=result['packet'];baseline=read(out/'image-estimates.json')['clean'];record=dict(id=entry['entry']['id'],sequence=sequence,worker_pid=r['worker_pid'],producer_pid=os.getpid(),exact_pose=digest(r['chosen'])==digest(baseline['chosen']),exact_status=r['status']==baseline['status'],rgb_sha256_matches=r['input_rgb_sha256']==hashlib.sha256(b''.join(frames)).hexdigest(),capture_times_s=r['capture_times_s'],delivered_at_sim_s=r['delivered_at_sim_s'],age_sim_s=r['age_sim_s'],processing_host_ms=r['processing_host_ms'],controller_usable=r['controller_usable'],source_directory=str(out.relative_to(ROOT)),source_estimates_sha256=sha(out/'image-estimates.json'));records.append(record);assert record['exact_pose'] and record['exact_status'] and record['rgb_sha256_matches'];print(dict(id=record['id'],exact=True,host_ms=record['processing_host_ms']),flush=True)
        stats=dict(b.metrics,worker_processed=int(b.stats[3]))
    write(STUDY/'summary.json',dict(simulator_runs=0,results=records,actual_saved_image_worker_jobs=len(records),explicit_static_native_record_reuses=14,producer_publish_wall_ms=publish_times,producer_poll_wall_ms=poll_times,broker_stats=stats,functional_worker_benchmark_admitted=all(r['exact_pose'] and r['exact_status'] for r in records),runtime_deadline_admitted=False,controller_integrated=False,camera_perception_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
