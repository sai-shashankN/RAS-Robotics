"""Conditional API history only. Never proves a conversion occurred or grants feedback."""
import math
class ApiHistory:
    def __init__(self,enabled_read_s=0.):
        if not isinstance(enabled_read_s,(int,float)) or isinstance(enabled_read_s,bool) or not math.isfinite(enabled_read_s):raise ValueError('initial owner read clock')
        self.enabled=True;self.epoch=1;self.enabled_at=enabled_read_s;self.last_read=None;self.last_enabled=None;self.fault=None
    def update(self,sample,period,actions=()):
        t=sample.get('read_s') if isinstance(sample,dict) else None;previous=self.last_enabled;phase='latched_unavailable';conditional=False;epoch=self.epoch;enabled_at=self.enabled_at
        if self.fault is None:
            try:
                if not isinstance(sample,dict) or not isinstance(period,dict):raise ValueError('snapshot dictionaries required')
                if not isinstance(t,(int,float)) or isinstance(t,bool) or not math.isfinite(t):raise ValueError('read clock')
                if self.last_read is not None and abs(t-self.last_read-.008)>1e-9:raise ValueError('consecutive read clock required')
                if period.get('read_s')!=t or period.get('gyro_period_ms')!=8 or period.get('accel_period_ms')!=(8 if self.enabled else 0):raise ValueError('unowned period change')
                if not self.enabled:
                    if sample.get('valid') is not False or sample.get('specific_force') is not None or sample.get('angular_rate') is not None:raise ValueError('disabled snapshot contract')
                    self.last_enabled=None;phase='unavailable'
                else:
                    if sample.get('valid') is not True or sample.get('hardware_measurement',False) is not False:raise ValueError('enabled snapshot contract')
                    if not all(len(sample[k])==3 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in sample[k]) for k in ['specific_force','angular_rate']):raise ValueError('finite vectors')
                    conditional=self.last_enabled is not None and abs(t-self.last_enabled-.008)<1e-9;phase='consecutive_api_history' if conditional else 'first_api_snapshot_quarantined';self.last_enabled=t
                self.last_read=t
                for event in actions:
                    if event['after_sample_s']!=t or event['before_accel_period_ms']!=(8 if self.enabled else 0):raise ValueError('owner action clock/state')
                    if event['action']=='disable' and self.enabled and event['after_accel_period_ms']==0:self.enabled=False;self.last_enabled=None
                    elif event['action']=='enable' and not self.enabled and event['after_accel_period_ms']==8:self.enabled=True;self.epoch+=1;self.enabled_at=t;self.last_enabled=None
                    else:raise ValueError('owner action transition')
            except (ValueError,TypeError,KeyError,AttributeError) as error:self.fault=type(error).__name__+':'+str(error);phase='latched_unavailable';conditional=False;self.last_enabled=None
        safe_t=t if isinstance(t,(int,float)) and not isinstance(t,bool) and math.isfinite(t) else None
        return dict(read_s=safe_t,configuration_epoch=epoch,epoch_enabled_read_s=enabled_at,previous_enabled_api_read_s=previous,phase=phase,conditional_same_epoch_api_history=conditional,fault=self.fault,acquisition_s=None,conversion_freshness_proven=False,usable_for_feedback=False,consumer_reset_authorized=False,native_driver_integrated=False,hardware_measurement=False)
