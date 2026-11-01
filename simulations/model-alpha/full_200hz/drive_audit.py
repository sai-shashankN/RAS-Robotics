import csv,math
from robot_native_audit import rotation

def drive(out,d):
    groups={};modes={};unique=True;aligned=True
    for name,index,dest in [('native-drive.csv','link_id',groups),('native-actuator-mode.csv','slot',modes)]:
        for v in csv.DictReader((out/name).read_text().splitlines()):
            t=float(v['time_s']);tick=round(t/.008);slot=int(v[index]);aligned&=abs(t-tick*.008)<1e-8;unique&=slot not in dest.setdefault(tick,{})
            dest[tick][slot]=v
    # The final controller observation quits before another native prePhysics.
    expected=set(range(len(d)));coverage=bool(d) and set(groups)==set(modes)==expected and all(set(g)==set(range(10)) for g in groups.values()) and all(set(g)==set(range(9)) for g in modes.values()) and unique and aligned
    errors=[];valid_modes=[]
    for v in d:
        tick=round(v['time_s']/.008)
        if tick not in groups or set(groups[tick])!=set(range(10)) or set(modes.get(tick,{}))!=set(range(9)):continue
        R=rotation(v);axis=[R[1],R[4],R[7]];parts=groups[tick];measured=[sum(sum(axis[j]*float(parts[i]['t'+k+'_Nm']) for j,k in enumerate('xyz')) for i in group) for group in [[1,2,3],[2,3],[3]]];errors.extend(abs(x-v[n+'_torque_command_nm']) for n,x in zip(['shoulder','elbow','wrist'],measured))
        for slot in [2,3,4]:valid_modes.append(float(modes[tick][slot]['velocity_motor_fmax_Nm'])==0 if v['torque_mode'] else float(modes[tick][slot]['velocity_motor_fmax_Nm'])>0)
    velocities=[abs((b[n]-a[n])/.008) for a,b in zip(d,d[1:]) for n in ['shoulder','elbow','wrist']];checks=dict(native_drive=bool(errors) and max(errors)<1e-9,native_modes=bool(valid_modes) and all(valid_modes),main_torque_bounds=all(abs(v[n+'_torque_command_nm'])<=limit+1e-12 for v in d for n,limit in zip(['shoulder','elbow','wrist'],[.9,.585,.27])),main_velocity_bounds=max(velocities,default=0)<.85,coupled_mode_exercised=any(v['torque_mode'] for v in d),fault_leaves_torque=all(not v['torque_mode'] for v in d if v['failure_latched']))
    checks['complete_native_ledger']=coverage
    return dict(passed=all(checks.values()),checks=checks,max_native_torque_error_Nm=max(errors,default=math.inf),max_main_velocity_rad_s=max(velocities,default=0),native_observations=len(groups),compared_controller_rows=len(errors)//3)
