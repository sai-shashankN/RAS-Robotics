import json,random
from context import ROOT,SOURCE,STUDY,read,write
from phase import CounterPhaseModel
from adapter import ManufacturedBurstCounter
from test_controls import ready_burst,data,stamp
def canonical(v):return json.loads(json.dumps(v,allow_nan=False))
def configurations():
    normal=dict(period_ticks=128,modulus_ticks=1<<24,max_gap_ticks=4096,origin_raw_tick=0,epoch=1)
    def case(name,observations,cfg=normal):return dict(id=name,configuration=cfg,observations=[dict(raw_read_tick=t,total_output_frames=n,observation_sequence=seq,epoch=e) for t,n,seq,e in observations])
    return [case('ambiguous_original8frames',[(1030,8,1,1)]),case('narrowing_does_not_grant_acquisition',[(1030,8,1,1),(1055,8,2,1),(1056,9,3,1)]),case('bounded_wrap',[(4,3,1,1),(24,5,2,1),(44,8,3,1)],dict(period_ticks=8,modulus_ticks=256,max_gap_ticks=64,origin_raw_tick=240,epoch=1)),case('same_read_counter_is_not_new_sample',[(0,0,1,1),(0,0,2,1),(5,0,3,1),(5,0,4,1)]),case('inconsistent_counts_latch',[(1030,8,1,1),(1031,100,2,1),(1055,8,2,1)]),case('epoch_change_latches',[(1030,8,1,1),(1055,8,2,2),(1055,8,2,1)])]
def replay(case):
    m=CounterPhaseModel(**case['configuration']);return canonical([m.observe(**row) for row in case['observations']])
def oracle_records():
    rng=random.Random(229128);records=[]
    for episode in range(100):
        period=rng.randrange(1,32);modulus=128;origin=rng.randrange(modulus);true=rng.randrange(1,period+1);cfg=dict(period_ticks=period,modulus_ticks=modulus,max_gap_ticks=31,origin_raw_tick=origin,epoch=1);m=CounterPhaseModel(**cfg);elapsed=0;constraints=[]
        for sequence in range(1,26):
            elapsed+=rng.randrange(0,32);n=len(list(range(true,elapsed+1,period)));constraints.append((elapsed,n));raw=(origin+elapsed)%modulus;expected=[p for p in range(1,period+1) if all(len(list(range(p,t+1,period)))==count for t,count in constraints)];actual=m.observe(raw,n,sequence,1);assert actual['candidate_phases_ticks']==expected and true in expected and actual['fault'] is None
            records.append(dict(episode=episode,configuration=cfg,true_manufactured_phase=true,manufactured_elapsed_ticks=elapsed,raw_read_tick=raw,cumulative_output_frames=n,sequence=sequence,explicit_event_list_oracle_phases=expected,result=actual))
    return records
def composed_trace():
    b=ready_burst();a=ManufacturedBurstCounter(baseline_transaction_sequence=1);rows=[b.update(data(5),.005,.006,2),b.update(data(3)+stamp(1030),.007,.008,3),b.update(data()+stamp(1056),.009,.010,4)]
    return canonical(dict(assumptions='After a manufactured boot drain,declare a new empty counter origin0and baseline transaction1. Exact128tick output period,no loss and sub-wrap gaps are external assumptions,not inferred from supplied owner bytes.',configuration=dict(epoch=1,baseline_transaction_sequence=1),rows=rows,outputs=[a.update(r) for r in rows]))
def main():
    assert not(STUDY/'summary.json').exists();cases=[dict(**c,outputs=replay(c)) for c in configurations()];oracle=oracle_records();composed=composed_trace();write(STUDY/'manufactured-counter-traces.json',cases);write(STUDY/'manufactured-burst-composition.json',composed)
    with(STUDY/'event-list-oracle.jsonl').open('w',encoding='utf-8') as f:
        for record in oracle:f.write(json.dumps(record,allow_nan=False)+'\n')
    all_match=all(r['result']['candidate_phases_ticks']==r['explicit_event_list_oracle_phases'] for r in oracle)
    write(STUDY/'summary.json',dict(simulator_runs=0,results=[dict(id='conditional_counter_phase_and_manufactured_composition',passed=all_match)],counter_phase_source_contract_prepared=all_match,manufactured_schedule_traces=len(cases),independent_event_list_episodes=100,independent_event_list_observations=len(oracle),manufactured_burst_compositions=1,new_native_robot_trials=0,new_hardware_driver_executions=0,conversion_freshness_admitted=False,host_device_clock_mapping_qualified=False,acquisition_timestamp_mapping_admitted=False,output_event_phase_physically_qualified=False,raw_imu_feedback_admitted=False,full_m2_admitted=False))
if __name__=='__main__':main()
