"""Burst grammar and supplied owner-evidence epochs;all physical grants stay false."""
import copy
from frames import parse,grammar,FrameError
from owner import OwnerEpoch,finite
class BurstEpochModel:
    def __init__(self):self.owner=OwnerEpoch();self.sequence=0;self.fault=None
    def declare(self,*args,**kwargs):
        if not self.fault:self.owner.declare(*args,**kwargs)
        self.fault=self.fault or self.owner.fault
    def acknowledge(self,*args,**kwargs):
        if not self.fault:self.owner.acknowledge(*args,**kwargs)
        self.fault=self.fault or self.owner.fault
    def update(self,data,read_start_s,read_end_s,transaction_sequence,provenance='manufactured',requested_bytes=None):
        events=[];meta=dict(observed_terminal_read_time_ticks=None,drain_observation=None,regular_frames=0);raw=data.hex() if isinstance(data,bytes) else None;compatible=False
        if self.fault is None:
            try:
                if provenance!='manufactured':raise ValueError('only manufactured transaction evidence')
                if type(transaction_sequence) is not int or transaction_sequence!=self.sequence+1:raise ValueError('transaction sequence gap/duplicate')
                if not all(finite(t) for t in [read_start_s,read_end_s]) or read_start_s<self.owner.last_clock-1e-12 or read_end_s<read_start_s:raise ValueError('read interval violates declared host chronology')
                if requested_bytes is not None and (type(requested_bytes) is not int or not isinstance(data,bytes) or requested_bytes!=len(data)):raise ValueError('short/incomplete read unqualified')
                staged=parse(data);meta=grammar(staged);label=self.owner.current['epoch'] if self.owner.current else None
                # Work on a copy so a later transaction failure cannot commit an earlier marker.
                staged_owner=copy.deepcopy(self.owner)
                for event in staged:
                    if event['kind']=='input_config':label=staged_owner.marker(event,read_end_s)
                    event=dict(event,model_epoch_id=label,model_owner_bytes_match=staged_owner.pending is None and staged_owner.current is not None,acquisition_s=None,conversion_freshness_proven=False,sample_sequence_proven=False,hardware_driver_executed=False,hardware_configuration_qualified=False,usable_for_feedback=False)
                    events.append(event)
                staged_owner.clock(read_end_s);self.owner=staged_owner;self.sequence=transaction_sequence;compatible=True
            except (FrameError,ValueError,TypeError,KeyError) as error:self.fault=type(error).__name__+':'+str(error);events=[]
        return dict(transaction_sequence=transaction_sequence if type(transaction_sequence) is int else None,read_started_host_s=read_start_s if finite(read_start_s) else None,read_finished_host_s=read_end_s if finite(read_end_s) else None,raw_burst_hex=raw,status='latched_unavailable' if self.fault else 'pending_owner_evidence' if self.owner.pending or self.owner.current is None else 'model_owner_bytes_consistent',fault=self.fault,grammar_compatible=compatible,current_model_epoch=self.owner.current['epoch'] if self.owner.current else None,pending_model_epoch=self.owner.pending['epoch'] if self.owner.pending else None,model_owner_bytes_match=self.fault is None and self.owner.current is not None and self.owner.pending is None,events=events,**meta,acquisition_s=None,conversion_freshness_proven=False,sample_sequence_proven=False,host_device_clock_mapping_qualified=False,hardware_driver_executed=False,hardware_configuration_qualified=False,consumer_reset_authorized=False,usable_for_feedback=False)
