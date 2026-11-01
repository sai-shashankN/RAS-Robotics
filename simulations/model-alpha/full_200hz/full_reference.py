import math
from context import STUDY,read,write,sha
from mission_reference import assess as original
from raw_metrics import sidecar_metrics,jsonrows
from timed_shadow import TimedShadowAttitude
from attitude_shadow import matrix,transpose_apply
from original_compare import compare as original_compare
from full_wrench import audit as wrench_audit
def assess(e,out):
    r=original(e,'nominal',out);r.pop('input_sha256',None);original_pass=r['passed'];metrics=sidecar_metrics(e,out);r['checks'].update(metrics['checks']);raw=jsonrows(out/'raw-imu.jsonl');refs=jsonrows(out/'raw-imu-reference.jsonl');actual=jsonrows(out/'imu-shadow.jsonl');o=TimedShadowAttitude();expected=[o.update(s) for s in raw];initial=None;errors=[]
    for s,w in zip(actual,refs[1:]):
        R=matrix(s['quaternion_WB']);a=transpose_apply(R,(0,0,1));b=transpose_apply(w['orientation'],(0,0,1));tilt=math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(a,b))))));y,z=math.atan2(R[3],R[0]),math.atan2(w['orientation'][3],w['orientation'][0])
        if initial is None:initial=(y,z)
        drift=math.degrees(abs(math.atan2(math.sin((y-initial[0])-(z-initial[1])),math.cos((y-initial[0])-(z-initial[1])))));errors.append(dict(read_s=s['read_s'],tilt_deg=tilt,relative_yaw_deg=drift))
    mt=max(v['tilt_deg'] for v in errors);my=max(v['relative_yaw_deg'] for v in errors);lim=read(STUDY/'test-plan.json')['limits'];wrench=wrench_audit(out,e['physics_ms'],e['expected_control_ticks']);r['checks'].update(wrench['checks']);same={n:sha(out/n)==h for n,h in read(STUDY/'references.json')[e['id']].items()};r['checks'].update(all_original_full_physical_bytes=all(same.values()),actual_endpoint_capture=len(raw)==len(actual)==e['expected_control_ticks'] and len(refs)==e['expected_control_ticks']+1 and raw[-1]['read_s']==refs[-1]['time_s']==e['expected_endpoint_s'],complete_native_shadow_replay=actual==expected,all_shadow_unarmed_unknown_acquisition=all(v['fault'] is None and v['state_read_s']==v['read_s'] and v['acquisition_s'] is None and v['usable_for_feedback'] is False and v['hardware_measurement'] is False and v['yaw_absolute_observed'] is False for v in actual),all_time_tilt_screen=mt<=lim['tilt_deg'],all_time_relative_yaw_screen=my<=lim['relative_yaw_deg']);r.update(passed=all(r['checks'].values()),full_physical_byte_checks=same,raw_imu=metrics,shadow_max_tilt_deg=mt,shadow_max_relative_yaw_deg=my,deferred_wrench=wrench,new_complete_nominal_mission=bool(original_pass));write(out/'shadow-errors.json',errors);write(out/'full-mission-raw-audit.json',r);return r
def compare(pair):
    r=original_compare(pair);a,b=sorted(pair,key=lambda v:v['entry']['physics_ms'],reverse=True);x,y=[jsonrows(STUDY/v['directory']/'raw-imu-reference.jsonl') for v in [a,b]];r['checks']['full_declared_actual_time_coverage']=len(x)==len(y)==a['entry']['expected_control_ticks']+1 and all(p['time_s']==q['time_s'] for p,q in zip(x,y));position=max(math.dist(p['position'],q['position']) for p,q in zip(x,y));rotation=max(math.acos(max(-1.,min(1.,(sum(u*v for u,v in zip(p['orientation'],q['orientation']))-1)/2))) for p,q in zip(x,y));r['checks'].update(full_independent_root_position=position<=.0005,full_independent_root_rotation=rotation<=math.radians(.5));r['metrics']['independent_root_witness']=dict(position_m=position,rotation_rad=rotation);r['passed']=all(r['checks'].values());return r
