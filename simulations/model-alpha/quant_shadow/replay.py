"""Passive observer sensitivity to three predeclared quantization-only profiles."""
import csv,gzip,io,json,math
from itertools import zip_longest
from context import ROOT,STUDY,read,write,sha
from history200 import ApiHistory200
from consumer200 import HistoryConsumer200
from projection200 import project
from attitude_shadow import matrix,transpose_apply
from scale import profile

METRICS=['tilt_deg','relative_yaw_deg','roll_deg','pitch_deg','pitch_rate_rad_s']
HEADER=['original_producer_read_s','state_read_s','original_host_read_s','qw','qx','qy','qz','roll_rad','pitch_rad','projected_pitch_rate_rad_s']+METRICS

def profiles():
    return [dict(id='2g_125dps',acc_code=0,gyro_code=4),dict(id='4g_250dps',acc_code=1,gyro_code=3),dict(id='16g_2000dps',acc_code=3,gyro_code=0)]

def stream(path):
    with path.open(encoding='utf-8') as f:
        for line in f:yield json.loads(line)

def flattened(path):
    for host in stream(path):
        if host['fault'] is not None or host['usable_for_feedback'] is not False:raise ValueError('original host state')
        yield from host['outputs']

def wrap(x):return math.atan2(math.sin(x),math.cos(x))

def score(state,pitch,ref,initial):
    R=matrix(state['quaternion_WB']);rr=ref['orientation'];a=transpose_apply(R,(0.,0.,1.));b=transpose_apply(rr,(0.,0.,1.));tilt=math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(a,b))))));y,z=math.atan2(R[3],R[0]),math.atan2(rr[3],rr[0])
    if initial is None:initial=(y,z)
    tr=math.atan2(rr[7],rr[8]);tp=math.atan2(-rr[6],math.hypot(rr[7],rr[8]));rates=transpose_apply(rr,ref['velocity'][3:]);rate=math.cos(tr)*rates[1]-math.sin(tr)*rates[2]
    return dict(tilt_deg=tilt,relative_yaw_deg=math.degrees(abs(wrap(wrap(y-initial[0])-wrap(z-initial[1])))),roll_deg=math.degrees(abs(wrap(pitch['roll_rad']-tr))),pitch_deg=math.degrees(abs(wrap(pitch['pitch_rad']-tp))),pitch_rate_rad_s=abs(pitch['pitch_rate_rad_s']-rate)),initial

def execute(entry):
    native=ROOT/entry['native_directory'];counts_path=ROOT/entry['counts_path'];dest=STUDY/'records'/entry['id'];dest.mkdir(parents=True);models={};files=[];writers={}
    import contextlib
    with contextlib.ExitStack() as stack:
        for p in [dict(id='ideal_reference',acc_code=None,gyro_code=None)]+profiles():
            name=p['id'];models[name]=dict(spec=p,history=ApiHistory200(),observer=HistoryConsumer200(),initial=None,max_errors={k:0. for k in METRICS},accepted=0,first3=[],status_valid=True)
            if name!='ideal_reference':
                q=profile(1,p['acc_code'],p['gyro_code'],0x88);models[name]['range_profile']=q.description()
                path=dest/(name+'.csv.gz');f=stack.enter_context(path.open('wb'));z=stack.enter_context(gzip.GzipFile(filename='',fileobj=f,mode='wb',mtime=0,compresslevel=1));txt=stack.enter_context(io.TextIOWrapper(z,encoding='utf-8',newline=''));writer=csv.writer(txt,lineterminator='\n');writer.writerow(HEADER);writers[name]=writer;files.append(path)
        f=stack.enter_context(gzip.open(counts_path,'rt',newline='',encoding='utf-8'));codes=csv.DictReader(f);refs=stream(native/'raw200-reference.jsonl');assert next(refs)['time_s']==0.;actual=flattened(native/'raw200-host-observer.jsonl');n=0;baseline_exact=True;last_host=None;last_t=None
        for row,ref,code,old in zip_longest(stream(native/'raw200-producer.jsonl'),refs,codes,actual):
            if any(v is None for v in [row,ref,code,old]):raise ValueError('full input join cardinality')
            n+=1;t=row['producer_read_s'];original=row['samples']['identity']['snapshot'];host=old['host_read_s'];last_host=host;last_t=t
            if original['read_s']!=t or ref['time_s']!=t or old['producer_read_s']!=t or float(code['original_simulation_producer_read_s'])!=t or abs(t-.005*n)>1e-9:raise ValueError('original clock join')
            if host<t or original['acquisition_s'] is not None or original['hardware_measurement'] is not False or original['usable_for_feedback'] is not False:raise ValueError('source grants')
            for name,m in models.items():
                sample=dict(original);p=m['spec']
                if name!='ideal_reference':
                    q=profile(1,p['acc_code'],p['gyro_code'],0x88)
                    sample['specific_force']=[int(code[f"accel_{p['acc_code']}_{a}_count"])*q.acc_scale for a in 'xyz'];sample['angular_rate']=[int(code[f"gyro_{p['gyro_code']}_{a}_count"])*q.gyr_scale for a in 'xyz']
                history=m['history'].update(sample,dict(read_s=t,accel_period_ms=5,gyro_period_ms=5));state=m['observer'].update(sample,history);pitch=project(state,sample)
                if name=='ideal_reference':baseline_exact &= history==old['history'] and state==old['attitude'] and pitch==old['pitch']
                if n<=3:m['first3'].append(dict(read_s=t,host_read_s=host,history_phase=history['phase'],state=state,pitch=pitch))
                m['status_valid'] &= history['fault'] is None and state['fault'] is None and state['acquisition_s'] is None and state['usable_for_feedback'] is False and state['hardware_measurement'] is False and state['yaw_absolute_observed'] is False and pitch['usable_for_feedback'] is False
                if state['quaternion_WB'] is None:
                    m['status_valid'] &= n==1 and state['status']=='waiting_for_history';continue
                m['status_valid'] &= state['state_read_s']==t and pitch['state_read_s']==t and pitch['status']=='shadow_projection';m['accepted']+=1;values,m['initial']=score(state,pitch,ref,m['initial'])
                for k,v in values.items():m['max_errors'][k]=max(m['max_errors'][k],v)
                if name!='ideal_reference':writers[name].writerow([repr(t),repr(state['state_read_s']),repr(host)]+state['quaternion_WB']+[pitch['roll_rad'],pitch['pitch_rad'],pitch['pitch_rate_rad_s']]+[values[k] for k in METRICS])
        if n!=entry['expected_rows'] or not baseline_exact:raise ValueError('baseline replay differs from complete actual226observer outputs')
    limits=read(STUDY/'test-plan.json')['original_observer_limits'];results=[]
    for name,m in models.items():
        checks=dict(full_expected_states=m['accepted']==n-1,all_states_and_no_grants_valid=m['status_valid'],first_snapshot_quarantined=m['first3'][0]['state']['state_read_s'] is None,second_snapshot_at_original10ms=m['first3'][1]['state']['state_read_s']==.01 and m['first3'][1]['host_read_s']==.016)
        gates={k:m['max_errors'][k]<=limits[k] for k in METRICS};r=dict(profile=name,range_profile=m.get('range_profile'),accepted_states=m['accepted'],max_errors=m['max_errors'],original_error_gates=gates,original_error_gates_passed=all(gates.values()),checks=checks,source_integrity_passed=all(checks.values()),first3=m['first3'])
        if name!='ideal_reference':path=dest/(name+'.csv.gz');r.update(saved_trace=str(path.relative_to(STUDY)),saved_trace_sha256=sha(path))
        results.append(r)
    result=dict(entry=entry,producer_rows=n,last_original_producer_s=last_t,last_original_host_s=last_host,ideal_observer_every_state_exact_to_actual226=baseline_exact,profiles=results,evaluation_complete=all(r['source_integrity_passed'] for r in results),new_native_trials=0,hardware_executions=0,raw_imu_feedback_admitted=False);write(dest/'audit.json',result);return result

def main():
    assert not(STUDY/'summary.json').exists();results=[]
    for e in read(STUDY/'manifest.json')['entries']:
        r=execute(e);results.append(r);print(json.dumps(dict(id=e['id'],evaluation_complete=r['evaluation_complete'],max_errors={p['profile']:p['max_errors'] for p in r['profiles']})),flush=True)
    write(STUDY/'summary.json',dict(simulator_runs=0,results=results,observer_quantization_evaluation_complete=all(r['evaluation_complete'] for r in results),all_quantization_profiles_meet_old_numerical_limits=all(p['original_error_gates_passed'] for r in results for p in r['profiles']),explicit_existing_native_histories=4,quantized_profile_replays=12,new_native_robot_trials=0,new_hardware_driver_executions=0,hardware_configuration_qualified=False,calibrated_SI_accuracy_qualified=False,conversion_freshness_admitted=False,acquisition_timestamp_mapping_admitted=False,raw_imu_feedback_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
