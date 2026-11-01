import csv,json,math
from context import read
MOTORS={'left_wheel':('ROBOT_MODEL','left_wheel'),'right_wheel':('ROBOT_MODEL','right_wheel'),'shoulder':('ROBOT_MODEL','ARM_BASE'),'elbow':('ARM_BASE','FOREARM'),'wrist':('FOREARM','PALM'),'finger_0':('PALM','FINGER_0'),'distal_0':('FINGER_0','TIP_0'),'finger_1':('PALM','FINGER_1'),'distal_1':('FINGER_1','TIP_1')}
def rows(p):return list(csv.DictReader(p.read_text().splitlines()))

def rotation(r):
    p,t,y=[float(r[k]) for k in ['roll_rad','pitch_rad','yaw_rad']];cr,sr,cp,sp,cy,sy=math.cos(p),math.sin(p),math.cos(t),math.sin(t),math.cos(y),math.sin(y)
    return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr]

def native(out,d):
    config=read(out/'sources/normalization.json');initial=rows(out/'native-init.csv');motion=rows(out/'native-motion.csv');params=len(initial)==9 and {r['motor'] for r in initial}==set(MOTORS)
    for r in initial:
        params &= (r['parent'],r['child'])==MOTORS[r['motor']] and r['geom_count']=='1' and r['unknown']=='0' and r['dreal_bytes']=='8'
        for key,value in [('cfm_before',config['world_cfm']),('cfm_after',.00001),('world_erp',config['world_erp']),('world_cfm',config['world_cfm']),('stop_erp',config['world_erp']),('stop_cfm',config['world_cfm']),('susp_erp',config['world_erp']),('susp_cfm',config['world_cfm'])]:params &= abs(float(r[key])-value)<1e-13
    indexed={round(r['time_s']/.008):r for r in d};pos=[];mat=[];speed=[];gyro=[];separation=[]
    for n in motion:
        r=indexed.get(round(float(n['time_s'])/.008))
        if r is None:continue
        R=[float(n['r'+str(i)]) for i in range(9)];w=[float(n['w'+k+'_rad_s']) for k in 'xyz'];local=[sum(R[3*j+i]*w[j] for j in range(3)) for i in range(3)]
        pos.append(math.dist([float(n[k+'_m']) for k in 'xyz'],[r[k+'_m'] for k in 'xyz']));mat.append(max(abs(a-b) for a,b in zip(R,rotation(r))));speed.append(abs(math.hypot(float(n['vx_m_s']),float(n['vy_m_s']))-r['body_speed_m_s']));gyro.append(max(abs(local[i]-r['gyro_'+k+'_rad_s']) for i,k in enumerate('xyz')) if 'gyro_x_rad_s' in r else max(abs(local[1]-r['pitch_rate_rad_s']),abs(local[2]-r['yaw_rate_rad_s'])));separation.append(math.dist([float(n[k+'_m']) for k in 'xyz'],[float(n['com_'+k+'_m']) for k in 'xyz']))
    cadence=bool(motion) and all(abs(float(b['time_s'])-float(a['time_s'])-.008)<1e-7 for a,b in zip(motion,motion[1:]));finite=all(math.isfinite(float(v)) for r in motion for v in r.values());valid=params and finite and cadence and len(motion)==len({r['time_s'] for r in motion}) and all(r['guard_valid']=='1' for r in motion) and len(pos)>=len(d)-2 and max(pos)<1e-9 and max(mat)<1e-7 and max(speed)<1e-9 and max(gyro)<1e-7
    return dict(valid=bool(valid),motor_parameters_valid=bool(params),finite=finite,cadence=cadence,aligned_samples=len(pos),max_origin_position_error_m=max(pos),max_rotation_matrix_error=max(mat),max_com_speed_error_m_s=max(speed),max_body_gyro_error_rad_s=max(gyro),max_origin_com_separation_m=max(separation))
