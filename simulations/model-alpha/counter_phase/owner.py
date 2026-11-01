"""A manufactured owner-evidence model,not register semantics or board readiness."""
import copy,math
def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
def fingerprint(value):
    if not isinstance(value,dict) or not value:raise ValueError('nonempty register-byte fingerprint')
    if any(type(k) is not int or type(v) is not int or not(0<=k<=255 and 0<=v<=255) for k,v in value.items()):raise ValueError('byte address/value fingerprint')
    return copy.deepcopy(value)
class OwnerEpoch:
    def __init__(self):self.current=None;self.pending=None;self.last_declared=0;self.last_clock=0.;self.fault=None
    def clock(self,t):
        if not finite(t) or t<self.last_clock-1e-12:raise ValueError('nonmonotonic declared host clock')
        self.last_clock=t
    def declare(self,epoch,register_bytes,expected_flags,command_host_s,provenance):
        if self.fault:return
        try:
            self.clock(command_host_s)
            if provenance!='manufactured':raise ValueError('only manufactured evidence model')
            if type(epoch) is not int or epoch!=self.last_declared+1 or self.pending is not None:raise ValueError('one pending consecutive epoch')
            if type(expected_flags) is not int or not 0<=expected_flags<=15:raise ValueError('selected accel/gyro flag byte')
            self.pending=dict(epoch=epoch,expected_register_bytes=fingerprint(register_bytes),expected_flags=expected_flags,command_host_s=command_host_s,marker=None);self.last_declared=epoch
        except (ValueError,TypeError) as error:self.fault=type(error).__name__+':'+str(error)
    def marker(self,event,read_end_s):
        if self.pending is None or self.pending['marker'] is not None:raise ValueError('unexpected configuration control')
        if event['config_flags']!=self.pending['expected_flags']:raise ValueError('configuration flags mismatch')
        self.pending['marker']=dict(read_observed_host_s=read_end_s,raw_next_frame_ticks=event['config_next_frame_ticks']);return self.pending['epoch']
    def acknowledge(self,register_bytes,read_start_s,read_end_s,observed_host_s,provenance):
        if self.fault:return
        try:
            if self.pending is None or self.pending['marker'] is None:raise ValueError('configuration marker still pending')
            if provenance!='manufactured':raise ValueError('only manufactured evidence model')
            if not all(finite(t) for t in [read_start_s,read_end_s,observed_host_s]) or not(self.pending['marker']['read_observed_host_s']<=read_start_s<=read_end_s<=observed_host_s):raise ValueError('readback must follow observed marker and precede declared observation')
            self.clock(observed_host_s)
            if fingerprint(register_bytes)!=self.pending['expected_register_bytes']:raise ValueError('readback byte fingerprint mismatch')
            self.current=dict(epoch=self.pending['epoch'],register_bytes=copy.deepcopy(register_bytes),owner_evidence_provenance='manufactured',readback_observed_host_s=observed_host_s);self.pending=None
        except (ValueError,TypeError) as error:self.fault=type(error).__name__+':'+str(error)
