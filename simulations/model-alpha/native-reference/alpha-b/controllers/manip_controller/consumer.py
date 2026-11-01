"""Wait only at initial startup. Once started, never hide faults or reset the core."""
from timed_shadow import TimedShadowAttitude
class HistoryConsumer:
    def __init__(self):self.started=False;self.core=TimedShadowAttitude();self.waiting_fault=None
    def update(self,sample,history):
        if not self.started:
            if history['fault'] is not None or sample.get('valid') is not True:self.waiting_fault=self.waiting_fault or 'startup_input_unavailable'
            if self.waiting_fault is not None or not history['conditional_same_epoch_api_history']:
                return dict(read_s=sample['read_s'],status='startup_latched_unavailable' if self.waiting_fault else 'waiting_for_history',fault=self.waiting_fault,quaternion_WB=None,state_read_s=None,acquisition_s=None,usable_for_feedback=False,hardware_measurement=False,yaw_absolute_observed=False,gravity_assumption_applied=False)
            self.started=True
        effective=dict(sample)
        if history['fault'] is not None or not history['conditional_same_epoch_api_history']:effective['valid']=False
        return self.core.update(effective)
