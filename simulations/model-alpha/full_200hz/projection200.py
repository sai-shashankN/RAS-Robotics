"""Passive signed body roll/pitch and Euler pitch-rate projection; no feedback grant."""
import math
from attitude_shadow import matrix,forward
from measurement import I,vector,rotation
def finite_scalar(x):return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)
def project(shadow,sample,R_BS=I):
    t=sample.get('read_s') if isinstance(sample,dict) else None
    row=dict(read_s=t if finite_scalar(t) else None,status='projection_unavailable',reason=None,roll_rad=None,pitch_rad=None,pitch_rate_rad_s=None,state_read_s=None,acquisition_s=None,frame='body_axes',usable_for_feedback=False,hardware_measurement=False,yaw_absolute_observed=False)
    try:
        if not isinstance(shadow,dict) or not isinstance(sample,dict):raise ValueError('input dictionaries')
        if shadow.get('fault') is not None:row.update(status='observer_unavailable',reason='observer_fault');return row
        if shadow.get('quaternion_WB') is None:row.update(status='waiting_for_attitude',reason='no_observer_state');return row
        if not finite_scalar(t) or shadow.get('read_s')!=t or shadow.get('state_read_s')!=t:raise ValueError('current state read clock')
        if shadow.get('status') not in ['initialized_from_gravity','shadow_attitude']:raise ValueError('observer status')
        if sample.get('valid') is not True or sample.get('sampling_period_ms')!=5 or sample.get('frame')!='sensor_axes' or sample.get('hardware_measurement',False) is not False:raise ValueError('native raw snapshot contract')
        q=shadow['quaternion_WB']
        if len(q)!=4 or not all(finite_scalar(v) for v in q) or abs(sum(v*v for v in q)-1)>1e-6:raise ValueError('unit observer quaternion')
        R=matrix(q);roll=math.atan2(R[7],R[8]);pitch=math.atan2(-R[6],math.hypot(R[7],R[8]))
        if abs(math.cos(pitch))<.5:raise ValueError('Euler singularity guard')
        rates=forward(rotation(R_BS),vector(sample['angular_rate']));rate=math.cos(roll)*rates[1]-math.sin(roll)*rates[2]
        row.update(status='shadow_projection',reason=None,roll_rad=roll,pitch_rad=pitch,pitch_rate_rad_s=rate,state_read_s=t);return row
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:row['reason']=type(error).__name__+':'+str(error);return row
