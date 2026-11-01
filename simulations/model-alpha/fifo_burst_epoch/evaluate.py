"""Existing byte corpus classification plus explicit manufactured owner traces."""
import json
from context import ROOT,SOURCE,STUDY,read,write,sha
from frames import parse,FrameError
from model import BurstEpochModel
from test_controls import data,time,config,FINGERPRINT,ready
def canonical(v):return json.loads(json.dumps(v,allow_nan=False))
def snapshot(m):
    def one(record):
        if record is None:return None
        r=dict(record)
        for key in ['expected_register_bytes','register_bytes']:
            if key in r:r[key]=sorted([list(pair) for pair in r[key].items()])
        return r
    return canonical(dict(fault=m.fault,owner_fault=m.owner.fault,sequence=m.sequence,current=one(m.owner.current),pending=one(m.owner.pending),last_declared_host_s=m.owner.last_clock))
def execute_trace(actions):
    m=BurstEpochModel();outputs=[]
    for a in actions:
        action=a['action']
        if action=='declare':m.declare(a['epoch'],dict(a['register_bytes']),a['expected_flags'],a['command_host_s'],a.get('provenance','manufactured'));out=None
        elif action=='ack':m.acknowledge(dict(a['register_bytes']),a['read_start_s'],a['read_end_s'],a['observed_host_s'],a.get('provenance','manufactured'));out=None
        else:out=m.update(bytes.fromhex(a['hex']),a['read_start_s'],a['read_end_s'],a['sequence'],provenance=a.get('provenance','manufactured'),requested_bytes=a.get('requested_bytes'))
        outputs.append(dict(action=action,result=out,snapshot=snapshot(m)))
    return canonical(outputs)
def fixtures():
    declare=dict(action='declare',epoch=1,register_bytes=sorted([list(p) for p in FINGERPRINT.items()]),expected_flags=0,command_host_s=0.)
    boot=dict(action='read',hex=(config()+data()+time()).hex(),read_start_s=.001,read_end_s=.002,sequence=1)
    ack=dict(action='ack',register_bytes=declare['register_bytes'],read_start_s=.003,read_end_s=.004,observed_host_s=.004)
    def case(name,tail):return dict(id=name,actions=[declare,boot,ack]+tail)
    return [case('partial_and_empty_are_not_dead_stream',[dict(action='read',hex=data().hex(),read_start_s=.005,read_end_s=.006,sequence=2),dict(action='read',hex='',read_start_s=.007,read_end_s=.008,sequence=3)]),case('planned_epoch2_waits_for_later_readback',[dict(action='declare',epoch=2,register_bytes=[[0x40,0xaa]],expected_flags=1,command_host_s=.005),dict(action='read',hex=(data()+config(1,1234)+data()+time()).hex(),read_start_s=.006,read_end_s=.007,sequence=2),dict(action='ack',register_bytes=[[0x40,0xaa]],read_start_s=.008,read_end_s=.009,observed_host_s=.009),dict(action='read',hex=(data()+time()+b'\x80\x00').hex(),read_start_s=.010,read_end_s=.011,sequence=3)]),case('unexpected_config_stays_latched',[dict(action='read',hex=(config()+data()).hex(),read_start_s=.005,read_end_s=.006,sequence=2),dict(action='read',hex=data().hex(),read_start_s=.007,read_end_s=.008,sequence=3)]),case('timestamp_interior_rejected',[dict(action='read',hex=(data()+time()+data()).hex(),read_start_s=.005,read_end_s=.006,sequence=2)]),dict(id='planned_marker_with_malformed_tail_never_commits',actions=[declare,dict(action='read',hex=(config()+data()+b'\x7f').hex(),read_start_s=.001,read_end_s=.002,sequence=1)]),dict(id='readback_mismatch_no_implicit_reset',actions=[declare,boot,dict(action='ack',register_bytes=[[0x40,0]],read_start_s=.003,read_end_s=.004,observed_host_s=.004),dict(action='read',hex=data().hex(),read_start_s=.005,read_end_s=.006,sequence=2)])]
def corpus():
    prior=STUDY.parent/'bmi_fifo_contract_v1';result=[]
    with(prior/'manufactured-differential-cases.jsonl').open() as f:
        for i,line in enumerate(f):
            old=json.loads(line);blob=bytes.fromhex(old['hex']);structural=True;reason=None;fields_match=False
            try:
                frames=parse(blob);acc=[r['accel_counts'] for r in frames if r['accel_counts'] is not None];gyr=[r['gyro_counts'] for r in frames if r['gyro_counts'] is not None];times=[r['sensor_time_ticks'] for r in frames if r['sensor_time_ticks'] is not None];skips=[r['skipped_frames'] for r in frames if r['skipped_frames'] is not None];fields_match=acc==old['vendor']['accel'] and gyr==old['vendor']['gyro'] and (times[-1] if times else 0)==old['vendor']['last_sensor_time_ticks'] and (skips[-1] if skips else 0)==old['vendor']['last_skipped_frame_count']
            except FrameError as error:structural=False;reason=str(error)
            output=ready().update(blob,.005,.006,2)
            # 227 supports a bare single-byte80end marker as byte-helper behavior;228requires80/00.
            deliberate_old_bare_overread=blob in [b'\x80',b'\x84\x01\x00\x02\x00\x03\x00\x80']
            assert structural and fields_match or not structural and deliberate_old_bare_overread,(i,reason)
            result.append(dict(source_case=i,hex=old['hex'],frame_structure_compatible=structural,frame_rejection=reason,decoded_fields_match_saved_vendor=fields_match if structural else None,prior_bare_overread_is_intentionally_incomplete=deliberate_old_bare_overread,grammar_model_output=output))
    assert len(result)==268;return result
def main():
    assert not(STUDY/'summary.json').exists();records=corpus();traces=[dict(**f,outputs=execute_trace(f['actions'])) for f in fixtures()]
    with(STUDY/'existing-corpus-classification.jsonl').open('w',encoding='utf-8') as f:
        for r in records:f.write(json.dumps(r,allow_nan=False)+'\n')
    write(STUDY/'manufactured-owner-traces.json',traces)
    accepted=sum(r['grammar_model_output']['grammar_compatible'] for r in records);fields=sum(r['decoded_fields_match_saved_vendor'] is True for r in records);write(STUDY/'summary.json',dict(simulator_runs=0,results=[dict(id='existing_byte_corpus_and_manufactured_owner_model',passed=fields==266)],fifo_burst_epoch_source_contract_prepared=fields==266,explicit_existing_manufactured_byte_cases=268,selected_frame_fields_reconciled_with_saved_vendor=fields,old_bare_overread_intentionally_rejected=268-fields,grammar_compatible_corpus_cases=accepted,grammar_rejected_corpus_cases=268-accepted,new_manufactured_owner_traces=len(traces),new_native_robot_trials=0,new_hardware_driver_executions=0,conversion_freshness_admitted=False,hardware_configuration_qualified=False,hardware_driver_executed=False,host_device_clock_mapping_qualified=False,acquisition_timestamp_mapping_admitted=False,raw_imu_feedback_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
