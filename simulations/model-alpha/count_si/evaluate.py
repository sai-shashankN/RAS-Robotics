"""Offline exhaustive signed-code checks; no robot/hardware execution."""
import hashlib,math,random,struct
from context import STUDY,write
from scale import profile,decode_axis,quantize_axis,convert_pair

def exhaustive_records():
    records=[]
    for sensor,n in [('accelerometer',4),('gyro',5)]:
        for code in range(n):
            p=profile(1,code if sensor=='accelerometer' else 0,code if sensor=='gyro' else 3,0x88)
            scale=p.acc_scale if sensor=='accelerometer' else p.gyr_scale
            digest=hashlib.sha256();failures=0;rail_codes=0;max_error=0.
            for count in range(-32768,32768):
                value=decode_axis(count,scale);r=quantize_axis(value,scale)
                failures+=r['count']!=count or r['absolute_error_SI']>r['bound_SI'];rail_codes+=r['at_raw_rail'];max_error=max(max_error,r['absolute_error_SI'])
                digest.update(struct.pack('<hd',r['count'],value))
            records.append(dict(sensor=sensor,range_code=code,profile=p.description(),codes_checked=65536,roundtrip_failures=failures,rail_codes=rail_codes,max_reconstruction_error_SI=max_error,decoded_stream_sha256=digest.hexdigest()))
    return records

def random_bounds():
    rng=random.Random(23016);records=[]
    for sensor,n in [('accelerometer',4),('gyro',5)]:
        for code in range(n):
            p=profile(1,code if sensor=='accelerometer' else 0,code if sensor=='gyro' else 3,0x88);s=p.acc_scale if sensor=='accelerometer' else p.gyr_scale
            for i in range(1000):
                x=rng.uniform(-50000,50000)*s;r=quantize_axis(x,s)
                assert r['absolute_error_SI']<=r['bound_SI'] and not r['physical_saturation_proven']
                records.append(dict(sensor=sensor,range_code=code,input_SI=x,**r))
    return records

def conversion_examples():
    p=profile(1,0,3,0x88)
    return [convert_pair(p,a,g,epoch=1,sequence=i+1,read_clock_s=.005*(i+1)) for i,(a,g) in enumerate([([0,0,16384],[0,0,0]),([-32768,32767,0],[32767,-32768,0]),([3,-7,100],[1,2,-3])])]

def main():
    assert not(STUDY/'summary.json').exists();exhaustive=exhaustive_records();bounds=random_bounds();examples=conversion_examples()
    write(STUDY/'exhaustive-counts.json',exhaustive);write(STUDY/'random-quantization-bounds.json',bounds);write(STUDY/'manufactured-conversions.json',examples)
    passed=all(r['roundtrip_failures']==0 and r['rail_codes']==2 for r in exhaustive)
    write(STUDY/'summary.json',dict(simulator_runs=0,results=[dict(id='range_semantics_and_ideal_quantization',passed=passed)],counts_to_SI_source_contract_prepared=passed,exhaustive_signed_code_cases=9*65536,random_bound_cases=len(bounds),manufactured_pair_examples=len(examples),new_native_robot_trials=0,new_hardware_driver_executions=0,calibrated_SI_accuracy_qualified=False,hardware_configuration_qualified=False,conversion_freshness_admitted=False,host_device_clock_mapping_qualified=False,acquisition_timestamp_mapping_admitted=False,raw_imu_feedback_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
