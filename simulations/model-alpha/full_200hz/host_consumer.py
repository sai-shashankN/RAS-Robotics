"""Process every queued five-ms snapshot at its original state read clock."""
import copy,math
from buffer import SnapshotBuffer
from history200 import ApiHistory200
from consumer200 import HistoryConsumer200
from projection200 import project

def finite(t):return isinstance(t,(int,float)) and not isinstance(t,bool) and math.isfinite(t)

class HostConsumer:
    def __init__(self):self.transport=SnapshotBuffer();self.history=ApiHistory200();self.observer=HistoryConsumer200();self.fault=None
    def update(self,receipt,owner_actions=()):
        outputs=[];host=receipt.get('host_read_s') if isinstance(receipt,dict) else None
        if self.fault is None:
            try:
                if not isinstance(receipt,dict) or not isinstance(owner_actions,(tuple,list)):raise ValueError('receipt/action dictionaries required')
                for row in receipt['batch']:self.transport.push(row)
                if self.transport.read(host)!=receipt or receipt['fault'] is not None:raise ValueError('invalid causal receipt')
                times=[r['producer_read_s'] for r in receipt['batch']];events={t:[] for t in times}
                for event in owner_actions:
                    if not isinstance(event,dict) or set(event)!={'after_producer_read_s','action','before_accel_period_ms','after_accel_period_ms','gyro_period_ms'}:raise ValueError('owner event schema')
                    t=event['after_producer_read_s']
                    if not finite(t) or t not in events or event['gyro_period_ms']!=5:raise ValueError('owner event outside delivered batch')
                    events[t].append(dict(after_sample_s=t,action=event['action'],before_accel_period_ms=event['before_accel_period_ms'],after_accel_period_ms=event['after_accel_period_ms']))
                for row in receipt['batch']:
                    t=row['producer_read_s']
                    if set(row['samples'])!={'identity'}:raise ValueError('one declared identity pair')
                    v=row['samples']['identity'];sample=v['snapshot']
                    if sample['read_s']!=t:raise ValueError('producer/sample clock mismatch')
                    period=dict(read_s=t,accel_period_ms=v['accel_period_ms'],gyro_period_ms=v['gyro_period_ms'])
                    metadata=self.history.update(sample,period,events[t]);state=self.observer.update(sample,metadata);pitch=project(state,sample);state_t=state['state_read_s']
                    outputs.append(dict(producer_read_s=t,host_read_s=host,api_delivery_age_s=host-t,state_read_s=state_t,state_api_age_s=None if state_t is None else host-state_t,history=metadata,attitude=state,pitch=pitch))
            except (ValueError,TypeError,KeyError,AttributeError) as error:self.fault=type(error).__name__+':'+str(error);outputs=[]
        return dict(host_read_s=host if finite(host) else None,status='latched_transport_unavailable' if self.fault else 'processed_shadow_batch',fault=self.fault,outputs=copy.deepcopy(outputs),acquisition_s=None,conversion_freshness_proven=False,hardware_driver_executed=False,usable_for_feedback=False)
