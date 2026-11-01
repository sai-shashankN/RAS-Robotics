import math
from context import STUDY
from m1_audit import rows
from robot_native_audit import rotation
def compare(pair):
    a,b=sorted(pair,key=lambda v:v['entry']['physics_ms'],reverse=True);da,db=rows(STUDY/a['directory']/'telemetry.csv'),rows(STUDY/b['directory']/'telemetry.csv');checks=dict(both_actual_trials_pass=a['passed'] and b['passed']);metrics={}
    if not checks['both_actual_trials_pass']:return dict(passed=False,checks=checks,metrics=metrics)
    pairs=list(zip(da,db));checks['common_control_times']=all(abs(x['time_s']-y['time_s'])<1e-9 for x,y in pairs)
    for label,prefix,cube in [('root','body_',False),('cube','object_',True)]:
        pos=max(math.dist([x[prefix+k+'_m'] for k in 'xyz'],[y[prefix+k+'_m'] for k in 'xyz']) for x,y in pairs);angle=max(math.acos(max(-1.,min(1.,(sum(v*w for v,w in zip([x['object_r'+str(i)] for i in range(9)] if cube else rotation(x),[y['object_r'+str(i)] for i in range(9)] if cube else rotation(y)))-1)/2))) for x,y in pairs);checks[label+'_full_pose']=pos<=.0005 and angle<=math.radians(.5);metrics[label]=dict(max_position_difference_m=pos,max_rotation_difference_rad=angle)
    for k,limit in [('final_center_error_m',.0003),('min_hold_lift_m',.0003),('peak_pitch_rad',max(.005,.1*b['task']['metrics']['peak_pitch_rad']))]:
        value=abs(a['task']['metrics'][k]-b['task']['metrics'][k]);checks[k]=value<=limit;metrics[k]=dict(difference=value,limit=limit)
    return dict(axis=a['entry']['axis'],passed=all(checks.values()),checks=checks,metrics=metrics)
