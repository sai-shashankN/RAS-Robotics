import csv,math
from robot_native_audit import rotation
def assess(out,data):
    indexed={round(v['time_s']/.008):v for v in data};errors=[];mode=[]
    with (out/'native-drive.csv').open() as f:
        for v in csv.DictReader(f):
            slot=int(v['link_id']);tick=round(float(v['time_s'])/.008)
            if slot not in [8,9] or tick not in indexed:continue
            row=indexed[tick];R=rotation(row);axis=[R[1],R[4],R[7]];projection=sum(axis[i]*float(v['t'+k+'_Nm']) for i,k in enumerate('xyz'));errors.append(abs(projection-row[('left' if slot==8 else 'right')+'_torque_nm']))
    with (out/'native-actuator-mode.csv').open() as f:
        for v in csv.DictReader(f):
            if v['slot'] in ['0','1']:mode.append(float(v['velocity_motor_fmax_Nm'])==0)
    checks=dict(wheel_torque_matches=len(errors)>=2*(len(data)-2) and max(errors,default=1)<1e-9,no_velocity_brake=len(mode)==2*len(data) and all(mode));return dict(passed=all(checks.values()),checks=checks,max_torque_error_Nm=max(errors,default=1),compared_wheel_observations=len(errors))
