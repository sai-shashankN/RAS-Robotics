"""Full-task5msreference/cache/queue audit;native and endpoint coverage explicit."""
import csv,math,json
from context import read
from measurement import transpose_apply
from window import point_velocity,forward
from buffer import SnapshotBuffer
def stream(p):
    with p.open(encoding='utf-8') as f:
        for line in f:yield json.loads(line)
def audit(e,out,lim):
    end=e['expected_endpoint_s'];ms=round(end*1000);n=ms//5;control=e['expected_control_ticks'];native_n=(ms-1)//5+1
    raw=list(stream(out/'raw200-producer.jsonl'));refs=list(stream(out/'raw200-reference.jsonl'));by_ms={round(r['time_s']*1000):r for r in refs};mount=dict(r_B=[-x for x in e['root_com_local_m']],R_BS=[1,0,0,0,1,0,0,0,1])
    checks=dict(complete_original8msendpoint=abs(end-control*.008)<1e-9 and ms%8==0,complete_actual5msproducer=len(raw)==n and all(abs(r['producer_read_s']-(i+1)*.005)<1e-9 and set(r['samples'])=={'identity'} for i,r in enumerate(raw)),complete_independent5msreferences=len(refs)==len(by_ms)==n+1 and all(abs(r['time_s']-i*.005)<1e-9 and r['audit_only'] is True for i,r in enumerate(refs)),exact_initial_enable=read(out/'raw200-init.json')==dict(read_s=0.,before=[0,0],after=[5,5]),no_device_reconfiguration=(out/'raw200-actions.jsonl').read_bytes()==b'',exact_completion=read(out/'raw200-completion.json')==dict(time_s=end,pending=0,queue_fault=None,usable_for_feedback=False,hardware_driver_executed=False,acquisition_s=None))
    acc=gyro=0.;schema=True
    for previous,r,row in zip(refs,refs[1:],raw):
        value=row['samples']['identity'];s=value['snapshot'];t=row['producer_read_s']
        schema &= s['valid'] is True and s['read_s']==r['time_s']==t and s['sampling_period_ms']==value['accel_period_ms']==value['gyro_period_ms']==5 and s['frame']=='sensor_axes' and s['acquisition_s'] is None and s['hardware_measurement'] is False and s['usable_for_feedback'] is False and all(math.isfinite(x) for x in s['specific_force']+s['angular_rate'])
        p,v=point_velocity(previous,mount),point_velocity(r,mount);expected=transpose_apply(r['orientation'],[(v[k]-p[k])/.005+(9.81 if k==2 else 0.) for k in range(3)]);w=transpose_apply(r['orientation'],r['velocity'][3:]);acc=max(acc,max(abs(a-b) for a,b in zip(expected,s['specific_force'])));gyro=max(gyro,max(abs(a-b) for values in [s['angular_rate'],value['gyro_api']] for a,b in zip(w,values)))
    checks.update(all_current_raw_schema_and_periods=schema,every_consecutive5mscache_derivative=acc<=lim['sampled_accel_error_m_s2'],every_gyro_frame=gyro<=lim['gyro200_reference_error_rad_s'])
    q=SnapshotBuffer();index=hosts=delivered=0;queue_ok=pattern_ok=clock_ok=payload_ok=True;last_host=None
    for receipt in stream(out/'raw200-host.jsonl'):
        hosts+=1;t=receipt['host_read_s']
        while index<len(raw) and raw[index]['producer_read_s']<=t+1e-12:q.push(raw[index]);index+=1
        expected=q.read(t);queue_ok &= expected==receipt
        pattern_ok &= len(receipt['batch'])==(hosts*8)//5-((hosts-1)*8)//5
        for row in receipt['batch']:
            payload_ok &= delivered<len(raw) and row==raw[delivered];delivered+=1
        clock_ok &= abs(t-hosts*.008)<1e-9 and receipt['fault'] is None and receipt['acquisition_s'] is None and all(receipt[k] is False for k in ['conversion_freshness_proven','hardware_driver_executed','usable_for_feedback']) and all(v['producer_read_s']<=t+1e-12 for v in receipt['batch']);last_host=receipt
    checks.update(complete_8mshost_and_exact_causal_replay=hosts==control and queue_ok,all_producer_payloads_delivered_once=delivered==index==n and payload_ok and not q.pending,actual_final_partial_batch_pattern=pattern_ok,no_host_clock_or_freshness_grant=clock_ok,original_final_host_and_actual_latest_read_age=last_host is not None and last_host['host_read_s']==end and round(last_host['latest_producer_read_s']*1000)==n*5 and abs(last_host['api_read_age_s']-(ms-n*5)*.001)<1e-9)
    nerr=0.;nc=0;coverage=True;last_native=None
    with(out/'raw200-native.csv').open() as f:
        for i,source in enumerate(csv.DictReader(f)):
            nc+=1;nrow={k:float(v) for k,v in source.items()};r=refs[i] if i<len(refs) else None
            if r is None:coverage=False;continue
            coverage &= abs(nrow['time_s']-i*.005)<1e-9 and abs(nrow['time_s']-r['time_s'])<1e-9 and nrow['time_s']<end and nrow['guard_valid']==1
            R=r['orientation'];offset=forward(R,e['root_com_local_m']);com=[x+y for x,y in zip(r['position'],offset)];pv=point_velocity(r,mount)
            pairs=[(r['position'][k],nrow[a+'_m']) for k,a in enumerate('xyz')]+[(com[k],nrow['com_'+a+'_m']) for k,a in enumerate('xyz')]+[(r['velocity'][k],nrow['v'+a+'_m_s']) for k,a in enumerate('xyz')]+[(pv[k],nrow['origin_v'+a+'_m_s']) for k,a in enumerate('xyz')]+[(r['velocity'][3+k],nrow['w'+a+'_rad_s']) for k,a in enumerate('xyz')]+[(R[k],nrow['r'+str(k)]) for k in range(9)]
            nerr=max(nerr,max(abs(a-b) for a,b in pairs));last_native=nrow
    common=True;legacy_end=None
    for r in stream(out/'raw-imu-reference.jsonl'):
        if round(r['time_s']*1000)%5==0:common &= r==by_ms[round(r['time_s']*1000)]
        legacy_end=r
    endpoint=read(out/'raw200-endpoint.json');checks.update(complete_real_native_interior_root_COM_point_frame=nc==native_n and coverage and nerr<=lim['native_reference_error'],common40msindependent_references_exact=common,actual_distinct_endpoint_not_invented=endpoint==legacy_end and endpoint['time_s']==end and endpoint['audit_only'] is True and refs[-1]['time_s']<=end and last_native is not None and last_native['time_s']<end,actual_nonaligned_task_end=ms%5!=0 and nc==n+1 and round(last_native['time_s']*1000)==n*5 and round(refs[-1]['time_s']*1000)==n*5)
    return dict(checks=checks,producer_rows=len(raw),host_rows=hosts,reference_rows=len(refs),native_interior_rows=nc,actual_task_endpoint_s=end,last_producer_s=refs[-1]['time_s'],last_api_read_age_s=last_host['api_read_age_s'],native_task_endpoint_present=False,max_accel_error_m_s2=acc,max_gyro_error_rad_s=gyro,max_native_reference_error=nerr)
