import csv,json,math
from cube_geometry import corners,bottom,point_on_top,resting_on_top

def rows(path):
    with path.open(encoding='utf-8') as f:return [{k:v if k=='stage' else (v=='True' if v in ['True','False'] else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]

def longest(data,predicate):
    start=None;best=0.
    for r in data:
        if predicate(r):
            if start is None:start=r['time_s']
            best=max(best,r['time_s']-start)
        else:start=None
    return best

def observe(data,contact_path,setup):
    match=True
    with contact_path.open(encoding='utf-8') as stream:
        for r,line in zip(data,stream):
            raw=json.loads(line);R=[r['object_r'+str(i)] for i in range(9)];xyz=[r['object_'+k+'_m'] for k in ['x','y','z']];points=corners(xyz,R,setup['object_size_m']);low=bottom(points);speed=math.sqrt(sum(r['object_v'+k+'_m_s']**2 for k in ['x','y','z']));center=setup['source_object_xyz_m'][:2];half=[v/2 for v in setup['platform_size_m']];top=setup['platform_top_m'];mask=0;bad=0
            for c in raw['arm']:
                paired=c['node'].startswith(('FINGER','TIP')) and any(math.dist(c['point_m'],p)<.0001 for p in raw['object_points_m'])
                if paired:mask|=1<<int(c['node'][-1])
                else:bad+=1
            support=any(point_on_top(p,center,half,top) for p in raw['object_points_m']) or resting_on_top(points,center,half,top,speed)
            match&=abs(raw['time_s']-r['time_s'])<1e-8 and mask==int(r['finger_mask']) and int(support)==int(r['source_support']) and bad<=r['bad_contact_count']
            r.update(obs_bottom_m=low,obs_speed_m_s=speed,obs_tilt_rad=math.acos(max(-1.,min(1.,R[8]))),obs_error_m=math.hypot(xyz[0]-center[0],xyz[1]-center[1]),obs_gap_m=low-top,obs_fingers=mask.bit_count(),obs_support=bool(support),obs_bad_arm=bad)
    return bool(match)

def mobile_audit(data,setup,p):
    stable=lambda r:r['obs_support'] and abs(r['obs_gap_m'])<=.0003 and r['obs_error_m']<=.003 and r['obs_tilt_rad']<=math.radians(3) and r['obs_speed_m_s']<.005
    close=[r for r in data if r['stage']=='close'];hold=[r for r in data if r['stage']=='hold'];verify=[r for r in data if r['stage']=='verify'];prelift=close[-1]['object_z_m'] if close else None;lift=min((r['object_z_m']-prelift for r in hold),default=-1.) if prelift is not None else -1.
    valid_hold=lambda r:r['stage']=='hold' and r['obs_fingers']==2 and not r['obs_support'] and r['object_z_m']-prelift>=.008 if prelift is not None else False
    hold_time=longest(data,valid_hold);first_open=next((i for i,r in enumerate(data) if r['stage']=='open'),None);pre_s=0.
    if first_open and stable(data[first_open-1]):
        end=data[first_open-1]['time_s']
        for r in reversed(data[:first_open]):
            if not stable(r):break
            pre_s=end-r['time_s']
    opened=lambda r:all(abs(r[f'finger_{i}']+.3)<=.02 and abs(r[f'distal_{i}']-.3)<=.02 for i in range(2));clear_s=longest(data,lambda r:r['stage']=='verify' and stable(r) and r['obs_fingers']==0 and opened(r));final=data[-1];tail=[r for r in data if r['time_s']>=final['time_s']-2]
    slip=0.
    if hold:
        origin=[hold[0]['object_'+k+'_m']-hold[0]['palm_'+k+'_m'] for k in ['x','y','z']]
        slip=max(math.dist([r['object_'+k+'_m']-r['palm_'+k+'_m'] for k in ['x','y','z']],origin) for r in hold)
    wheel=max(abs(r[n+'_speed_rad_s']) for r in data for n in ['left','right']);main_rate=finger_rate=0.;names=['shoulder','elbow','wrist']+[f'{part}_{i}' for part in ['finger','distal'] for i in range(2)]
    for a,b in zip(data,data[1:]):
        dt=b['time_s']-a['time_s'];wheel=max(wheel,*[abs(b[n+'_encoder_rad']-a[n+'_encoder_rad'])/dt for n in ['left','right']]);main_rate=max(main_rate,*[abs(b[n+'_command_rad']-a[n+'_command_rad'])/dt for n in ['shoulder','elbow','wrist']]);finger_rate=max(finger_rate,*[abs(b[n+'_command_rad']-a[n+'_command_rad'])/dt for n in names[3:]])
    torque=max(abs(r[n+'_torque_nm']) for r in data for n in ['left','right']);palm_error=max(math.acos(max(-1.,min(1.,-r['palm_axis_z']))) for r in data);hand_error=max(math.hypot(r['palm_x_m']-r['target_x_m'],r['palm_z_m']-r['target_z_m']) for r in data)
    final_pitch=max(abs(r['pitch_rad']) for r in tail);final_speed=max(r['body_speed_m_s'] for r in tail)
    checks=dict(unsupported_hold_6s=hold_time>=6-1e-8,lift_8mm=lift>=.008,hold_slip=slip<=.003,stable_before_open_600ms=pre_s>=.6-1e-8,stable_clear_3s=clear_s>=3-1e-8,full_opening=bool(verify) and all(opened(r) for r in verify),actual_withdrawal=bool(verify) and min(r['palm_z_m'] for r in verify)>=setup['source_palm_xz_m'][1]+.012,no_forbidden_contact=all(r['bad_contact_count']==r['obs_bad_arm']==0 for r in data),no_penetration=not any(r['obs_gap_m']<-.001 and r['obs_error_m']<.009 for r in data),finite=all(math.isfinite(v) for r in data for k,v in r.items() if isinstance(v,(int,float)) and not k.endswith('_effort_nm')),ranges=max(r['range_excess_rad'] for r in data)<=.03,main_rate=main_rate<=.08+1e-7,finger_rate=finger_rate<=.3+1e-7,wheel_speed=wheel<=p['wheel_speed_limit']+.5,wheel_reserve=torque<=p['wheel_torque_limit']*.9,pitch=max(abs(r['pitch_rad']) for r in data)<math.radians(20),mass_guard=max(abs(r['mass_lean_rad']) for r in data)<=.3,final_pitch=final_pitch<math.radians(1),final_speed=final_speed<.01,yaw=max(abs(r['yaw_rad']) for r in data)<.05,palm_level=palm_error<math.radians(2),hand_tracking=hand_error<.003,robot_mass_model=max(r['robot_com_model_error_m'] for r in data)<.002,load_during_hold=bool(hold) and all(r['payload_estimate_kg']==.002 for r in hold),load_removed_for_verify=bool(verify) and all(r['payload_estimate_kg']==0 for r in verify))
    return dict(passed=all(checks.values()),checks=checks,unsupported_hold_s=hold_time,min_hold_lift_m=lift,hold_slip_m=slip,stable_before_open_s=pre_s,stable_clear_s=clear_s,final_center_error_m=final['obs_error_m'],final_tilt_rad=final['obs_tilt_rad'],max_hand_error_m=hand_error,max_palm_error_rad=palm_error,peak_pitch_rad=max(abs(r['pitch_rad']) for r in data),peak_wheel_command_nm=torque,peak_wheel_speed_rad_s=wheel,final_pitch_error_rad=final_pitch,final_body_speed_m_s=final_speed,duration_s=final['time_s'])
