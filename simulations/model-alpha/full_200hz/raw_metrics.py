import csv,json,math
from context import ROOT,STUDY,read,write
from window import sampled_reference,point_velocity,forward
from measurement import transpose_apply

def jsonrows(path):return [json.loads(l) for l in path.read_text().splitlines()]
def sidecar_metrics(e,out):
    raw=jsonrows(out/'raw-imu.jsonl');reference=jsonrows(out/'raw-imu-reference.jsonl');limits=read(STUDY/'test-plan.json')['limits'];mount=dict(r_B=[-v for v in e['root_com_local_m']],R_BS=[1,0,0,0,1,0,0,0,1]);accel=0.;gyro=0.;coverage=len(raw)==e['expected_control_ticks'] and len(reference)==e['expected_control_ticks']+1;schema=True
    with(out/'telemetry.csv').open() as f:telemetry=list(csv.DictReader(f))
    if not coverage:return dict(checks=dict(complete_sidecar_coverage=False),raw_rows=len(raw),reference_rows=len(reference))
    for i,(a,b,s,t) in enumerate(zip(reference,reference[1:],raw,telemetry)):
        expected_time=(i+1)*.008;coverage &= abs(a['time_s']-i*.008)<1e-9 and abs(b['time_s']-expected_time)<1e-9 and abs(s['read_s']-expected_time)<1e-9 and s['read_s']==float(t['time_s']);schema &= s['valid'] and s['sampling_period_ms']==8 and s['acquisition_s'] is None and s['usable_for_feedback'] is False and s['hardware_measurement'] is False and a['audit_only'] and b['audit_only'];expected=sampled_reference(a,b,mount);accel=max(accel,max(abs(x-y) for x,y in zip(s['specific_force'],expected['specific_force'])));gyro=max(gyro,max(abs(s['angular_rate'][k]-float(t['gyro_'+axis+'_rad_s'])) for k,axis in enumerate('xyz')));schema &= all(math.isfinite(x) for x in s['specific_force']+s['angular_rate'])
    with(out/'native-motion.csv').open() as f:native=list(csv.DictReader(f))
    nerr=0.;ncoverage=len(native)==e['expected_control_ticks']
    for i,(n,r) in enumerate(zip(native,reference)):
        ncoverage &= abs(float(n['time_s'])-r['time_s'])<1e-9 and float(n['guard_valid'])==1.;R=r['orientation'];com_offset=forward(R,e['root_com_local_m']);com=[x+y for x,y in zip(r['position'],com_offset)];pv=point_velocity(r,mount);values=[(r['position'][k],float(n[axis+'_m'])) for k,axis in enumerate('xyz')]+[(r['velocity'][k],float(n['v'+axis+'_m_s'])) for k,axis in enumerate('xyz')]+[(r['velocity'][3+k],float(n['w'+axis+'_rad_s'])) for k,axis in enumerate('xyz')]+[(com[k],float(n['com_'+axis+'_m'])) for k,axis in enumerate('xyz')]+[(pv[k],float(n['origin_v'+axis+'_m_s'])) for k,axis in enumerate('xyz')]+[(R[k],float(n['r'+str(k)])) for k in range(9)];nerr=max(nerr,max(abs(a-b) for a,b in values))
    return dict(checks=dict(complete_raw_and_reference_times=coverage,finite_valid_readonly_no_feedback_or_clock_grant=schema,sampled_acceleration_all_declared_intervals=accel<=limits['sampled_accel_error_m_s2'],raw_gyro_matches_original_telemetry=gyro<=limits['gyro_telemetry_error_rad_s'],supervisor_reference_reconciles_native_point_COM_frame=ncoverage and nerr<=limits['native_reference_error']),raw_rows=len(raw),reference_rows=len(reference),native_reference_rows=len(native),sampled_accel_max_error_m_s2=accel,gyro_telemetry_max_error_rad_s=gyro,native_reference_max_error=nerr)
