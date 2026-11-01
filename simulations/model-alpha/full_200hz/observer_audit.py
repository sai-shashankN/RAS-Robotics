"""Complete causal host replay and independent producer-time attitude scoring."""
import math,json
from context import read
from metrics200 import stream
from host_consumer import HostConsumer
from attitude_shadow import matrix,transpose_apply
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def audit(e,out,lim):
    refs={r['time_s']:r for r in stream(out/'raw200-reference.jsonl')};c=HostConsumer();actual=iter(stream(out/'raw200-host-observer.jsonl'));count=accepted=host_count=0;exact=True;valid=True;initial=None;maximum={k:0. for k in ['tilt_deg','relative_yaw_deg','roll_deg','pitch_deg','pitch_rate_rad_s']};first=[];last=None
    with(out/'observer200-errors.jsonl').open('w',encoding='utf-8') as errors:
        for receipt in stream(out/'raw200-host.jsonl'):
            expected=c.update(receipt);got=next(actual,None);exact &= got==expected;host_count+=1
            for row,raw in zip(expected['outputs'],receipt['batch']):
                count+=1;t=row['producer_read_s'];state=row['attitude'];pitch=row['pitch'];ref=refs[t];last=row
                if len(first)<3:first.append(row)
                valid &= expected['fault'] is None and expected['acquisition_s'] is None and all(expected[k] is False for k in ['conversion_freshness_proven','hardware_driver_executed','usable_for_feedback']) and row['host_read_s']==receipt['host_read_s'] and t==raw['producer_read_s'] and row['history']['fault'] is None and row['history']['acquisition_s'] is None and row['history']['conversion_freshness_proven'] is False and state['fault'] is None and state['acquisition_s'] is None and state['usable_for_feedback'] is False and state['yaw_absolute_observed'] is False and pitch['usable_for_feedback'] is False
                if state['quaternion_WB'] is None:continue
                valid &= state['state_read_s']==t and pitch['state_read_s']==t and pitch['status']=='shadow_projection';accepted+=1
                R=matrix(state['quaternion_WB']);rr=ref['orientation'];a=transpose_apply(R,(0.,0.,1.));b=transpose_apply(rr,(0.,0.,1.));tilt=math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(a,b))))));y,z=math.atan2(R[3],R[0]),math.atan2(rr[3],rr[0])
                if initial is None:initial=y,z
                tr=math.atan2(rr[7],rr[8]);tp=math.atan2(-rr[6],math.hypot(rr[7],rr[8]));rates=transpose_apply(rr,ref['velocity'][3:]);rate=math.cos(tr)*rates[1]-math.sin(tr)*rates[2]
                values=dict(tilt_deg=tilt,relative_yaw_deg=math.degrees(abs(wrap(wrap(y-initial[0])-wrap(z-initial[1])))),roll_deg=math.degrees(abs(wrap(pitch['roll_rad']-tr))),pitch_deg=math.degrees(abs(wrap(pitch['pitch_rad']-tp))),pitch_rate_rad_s=abs(pitch['pitch_rate_rad_s']-rate))
                for k,v in values.items():maximum[k]=max(maximum[k],v)
                errors.write(json.dumps(dict(producer_read_s=t,host_read_s=row['host_read_s'],state_read_s=state['state_read_s'],api_delivery_age_s=row['api_delivery_age_s'],**values),allow_nan=False)+'\n')
    extra=next(actual,None);n=round(e['expected_endpoint_s']*1000)//5
    checks=dict(complete_native_host_observer_exact_replay=exact and extra is None and host_count==e['expected_control_ticks'] and count==n,every_accepted_state_at_actual_producer_time=valid and accepted==n-1,first5msquarantine_then10msstate_at16mshost=len(first)==3 and first[0]['state_read_s'] is None and first[0]['attitude']['status']=='waiting_for_history' and round(first[1]['state_read_s']*1000)==10 and round(first[1]['host_read_s']*1000)==16 and round(first[2]['state_read_s']*1000)==15 and round(first[2]['host_read_s']*1000)==16,last_state_not_retimed_to_task_endpoint=last is not None and round(last['state_read_s']*1000)==n*5 and last['host_read_s']==e['expected_endpoint_s'] and abs(last['state_api_age_s']-(round(e['expected_endpoint_s']*1000)-n*5)*.001)<1e-9)
    for k,v in maximum.items():checks['full200_'+k+'_original_limit']=v<=lim['observer200_'+k]
    done=read(out/'raw200-observer-completion.json');checks['actual_consumer_completion']=done==dict(time_s=e['expected_endpoint_s'],pending_owner_events=0,consumer_transport_fault=None,usable_for_feedback=False,hardware_driver_executed=False,acquisition_s=None)
    return dict(checks=checks,host_rows=host_count,producer_outputs=count,accepted_states=accepted,max_errors=maximum,last_state_read_s=last['state_read_s'] if last else None,last_host_read_s=last['host_read_s'] if last else None,last_state_API_age_s=last['state_api_age_s'] if last else None)
