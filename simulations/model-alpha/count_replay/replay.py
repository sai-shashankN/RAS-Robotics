"""Quantization envelope on four explicit existing full native histories.

No native launch, time synthesis, hardware model, noise, or attitude feedback.
"""
import csv,gzip,hashlib,io,json,math
from context import ROOT,STUDY,read,write,sha
from scale import profile,quantize_axis

def scales():
    out=[]
    for sensor,n in [('accel',4),('gyro',5)]:
        for code in range(n):
            p=profile(1,code if sensor=='accel' else 0,code if sensor=='gyro' else 3,0x88)
            out.append(dict(id=f'{sensor}_{code}',sensor=sensor,range_code=code,scale=p.acc_scale if sensor=='accel' else p.gyr_scale,range_value=p.description()['range_g' if sensor=='accel' else 'range_dps'],units='m/s^2' if sensor=='accel' else 'rad/s'))
    return out

def header():return ['original_simulation_producer_read_s']+[s['id']+'_'+a+'_count' for s in scales() for a in 'xyz']

def validate(row,index):
    if type(row) is not dict or set(row.get('samples',{}))!={'identity'}:raise ValueError('identity pair required')
    v=row['samples']['identity'];s=v['snapshot'];t=row['producer_read_s']
    if type(t) not in (int,float) or not math.isfinite(t) or abs(t-.005*index)>1e-9 or s['read_s']!=t:raise ValueError('original consecutive producer clock')
    if s['schema']!=1 or s['frame']!='sensor_axes' or s['accel_units']!='m/s^2_specific_force' or s['gyro_units']!='rad/s_body_rates':raise ValueError('source units/frame/schema')
    if s['valid'] is not True or s['hardware_measurement'] is not False or s['usable_for_feedback'] is not False or s['acquisition_s'] is not None:raise ValueError('source scope')
    if s['sampling_period_ms']!=5 or v['accel_period_ms']!=5 or v['gyro_period_ms']!=5:raise ValueError('source periods')
    for key in ['specific_force','angular_rate']:
        if type(s[key]) is not list or len(s[key])!=3 or any(type(x) not in (float,int) or not math.isfinite(x) for x in s[key]):raise ValueError('source vector')
    if v['gyro_api']!=s['angular_rate']:raise ValueError('gyro API disagreement')
    return s

def summarize(entry, writer=None):
    spec=scales();stats={s['id']:dict(**s,max_absolute_input_SI=[0.,0.,0.],max_absolute_error_SI=[0.,0.,0.],rms_error_SI=[0.,0.,0.],rail_occupancy=[0,0,0],inputs_outside_representable_interval=[0,0,0],max_error_read_s=[None]*3,bound_violations=0) for s in spec}
    count=0;clock_hash=hashlib.sha256();count_hash=hashlib.sha256();first=None;last=None;inputs=ROOT/entry['input']
    for index,line in enumerate(inputs.open(encoding='utf-8'),1):
        row=json.loads(line);sample=validate(row,index);t=row['producer_read_s'];first=t if first is None else first;last=t;count+=1;counts=[]
        clock_hash.update((json.dumps(t)+'\n').encode())
        for s in spec:
            st=stats[s['id']];values=sample['specific_force' if s['sensor']=='accel' else 'angular_rate']
            for j,x in enumerate(values):
                q=quantize_axis(x,s['scale']);counts.append(q['count']);st['max_absolute_input_SI'][j]=max(st['max_absolute_input_SI'][j],abs(x));st['rms_error_SI'][j]+=q['absolute_error_SI']**2;st['rail_occupancy'][j]+=q['at_raw_rail'];st['inputs_outside_representable_interval'][j]+=q['input_outside_representable_interval'];st['bound_violations']+=q['absolute_error_SI']>q['bound_SI']
                if q['absolute_error_SI']>st['max_absolute_error_SI'][j]:st['max_absolute_error_SI'][j]=q['absolute_error_SI'];st['max_error_read_s'][j]=t
        count_hash.update((','.join(map(str,counts))+'\n').encode())
        if writer is not None:writer.writerow([repr(t)]+counts)
    if count!=entry['expected_rows']:raise ValueError('complete history row count')
    for st in stats.values():st['rms_error_SI']=[math.sqrt(v/count) for v in st['rms_error_SI']]
    return dict(entry=entry,producer_rows=count,first_original_read_s=first,last_original_read_s=last,clock_stream_sha256=clock_hash.hexdigest(),count_stream_sha256=count_hash.hexdigest(),scale_results=list(stats.values()),passed=all(s['bound_violations']==0 for s in stats.values()),acquisition_timestamp_mapping_admitted=False,hardware_configuration_qualified=False,calibrated_SI_accuracy_qualified=False,raw_imu_feedback_admitted=False,new_native_robot_trials=0,new_hardware_executions=0)

def verify_saved(entry,path):
    """Independent direct nearest/clip oracle;does not call the quantizer.

    Re-join every saved timestamp and count to the original source rows.
    """
    rows=0;clock_hash=hashlib.sha256();count_hash=hashlib.sha256();spec=scales()
    with gzip.open(path,'rt',newline='',encoding='utf-8') as f,(ROOT/entry['input']).open(encoding='utf-8') as raw:
        reader=csv.reader(f)
        if next(reader)!=header():raise ValueError('saved columns')
        from itertools import zip_longest
        for i,(r,line) in enumerate(zip_longest(reader,raw),1):
            if r is None or line is None:raise ValueError('saved row loss')
            original=json.loads(line);s=validate(original,i);t=original['producer_read_s'];expected=[]
            for scale in spec:
                for x in s['specific_force' if scale['sensor']=='accel' else 'angular_rate']:
                    expected.append(max(-32768,min(32767,round(x/scale['scale']))))
            if len(r)!=28 or float(r[0])!=t or [int(x) for x in r[1:]]!=expected:raise ValueError('count or original clock mismatch')
            clock_hash.update((json.dumps(t)+'\n').encode());count_hash.update((','.join(map(str,expected))+'\n').encode());rows+=1
    return dict(producer_rows=rows,clock_stream_sha256=clock_hash.hexdigest(),count_stream_sha256=count_hash.hexdigest())

def main():
    assert not(STUDY/'summary.json').exists();results=[]
    for e in read(STUDY/'manifest.json')['entries']:
        dest=STUDY/'records'/e['id'];dest.mkdir(parents=True);p=dest/'quantized-counts.csv.gz'
        with p.open('wb') as raw,gzip.GzipFile(filename='',fileobj=raw,mode='wb',mtime=0) as z,io.TextIOWrapper(z,encoding='utf-8',newline='') as text:
            writer=csv.writer(text,lineterminator='\n');writer.writerow(header());r=summarize(e,writer)
        independent=verify_saved(e,p)
        assert all(r[k]==v for k,v in independent.items());r.update(saved_counts=str(p.relative_to(STUDY)),saved_counts_sha256=sha(p),independent_oracle=independent);write(dest/'audit.json',r);results.append(r)
        print(json.dumps(dict(id=e['id'],rows=r['producer_rows'],passed=r['passed'],outside_inputs={v['id']:sum(v['inputs_outside_representable_interval']) for v in r['scale_results']})),flush=True)
    write(STUDY/'summary.json',dict(simulator_runs=0,results=results,counts_quantization_replay_complete=all(r['passed'] for r in results),explicit_existing_native_histories=4,producer_rows=sum(r['producer_rows'] for r in results),scalar_quantizations=27*sum(r['producer_rows'] for r in results),new_native_robot_trials=0,new_hardware_driver_executions=0,hardware_configuration_qualified=False,calibrated_SI_accuracy_qualified=False,conversion_freshness_admitted=False,acquisition_timestamp_mapping_admitted=False,raw_imu_feedback_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
