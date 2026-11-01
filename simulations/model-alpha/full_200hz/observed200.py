"""Thin passive live subscriber; all original raw and host journals are retained."""
import json
from sidecar200 import Alpha200
from host_consumer import HostConsumer

class ObservedAlpha200(Alpha200):
    def __init__(self,*args,**kwargs):
        self.pending_owner=[];self.current_receipt=None;self.consumer=HostConsumer()
        super().__init__(*args,**kwargs)
        self.observer_stream=(self.out/'raw200-host-observer.jsonl').open('w',encoding='utf-8')
    def write(self,name,row):
        super().write(name,row)
        if name=='raw200-actions.jsonl':self.pending_owner.append(row)
        elif name=='raw200-host.jsonl':self.current_receipt=row
    def tick(self,t):
        self.current_receipt=None;super().tick(t)
        if self.current_receipt is not None:
            clocks={r['producer_read_s'] for r in self.current_receipt['batch']};actions=[e for e in self.pending_owner if e['after_producer_read_s'] in clocks]
            self.pending_owner=[e for e in self.pending_owner if e['after_producer_read_s'] not in clocks]
            output=self.consumer.update(self.current_receipt,actions)
            self.observer_stream.write(json.dumps(output,allow_nan=False)+'\n')
    def close(self,t):
        super().close(t);self.observer_stream.close()
        (self.out/'raw200-observer-completion.json').write_text(json.dumps(dict(time_s=t,pending_owner_events=len(self.pending_owner),consumer_transport_fault=self.consumer.fault,usable_for_feedback=False,hardware_driver_executed=False,acquisition_s=None),allow_nan=False),encoding='utf-8')
