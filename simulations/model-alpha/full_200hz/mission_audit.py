"""Independent actual-trajectory mission evaluation; no controller imports."""
import math
from cube_geometry import corners,bottom,point_on_top
from m1_audit import longest
MAIN=['shoulder','elbow','wrist']
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def rotate(a,v):
    c,s=math.cos(a),math.sin(a);return [c*v[0]-s*v[1],s*v[0]+c*v[1],v[2]]
def seat(v):
    delta=[v['object_'+k+'_m']-v['palm_'+k+'_m'] for k in 'xyz'];return [sum(v['palm_r'+str(3*j+i)]*delta[j] for j in range(3)) for i in range(3)]
def reference(t,approach):
    clock=0.
    for i,duration in enumerate([50.,4.,60.,4.,150.,4.],1):
        if t<clock+duration or i==6:
            u=max(0.,min(1.,(t-clock)/duration));f=u*u*(3-2*u);path=-.1;world=-.1;yaw=0.
            if i==1:path=world=-.1*f
            elif i==3:yaw=math.pi*f
            elif i>=4:yaw=math.pi
            if i==5:path=-.1+approach*f;world=-.1-approach*f
            if i==6:path=-.1+approach;world=-.1-approach
            return i,path,world,yaw
        clock+=duration
def support_features(v,raw,face,setup,profile):
    f=profile['fixtures'][4];center=f['center_m'];half=f['half_m'];top=center[2]+half[2];R=[v['object_r'+str(i)] for i in range(9)];xyz=[v['object_'+k+'_m'] for k in 'xyz'];gap=bottom(corners(xyz,R,setup['object_size_m']))-top;error=math.dist(xyz[:2],center[:2]);tilt=math.acos(max(-1.,min(1.,R[8])));speed=math.sqrt(sum(v['object_v'+k+'_m_s']**2 for k in 'xyz'));native=int(face['fixture_4_top_points'])>0 and all(int(face[f'fixture_{i}_other_points'])==0 and (i==4 or int(face[f'fixture_{i}_top_points'])==0) for i in range(8));raw_support=any(point_on_top(p,center[:2],half,top) for p in raw['object_points_m']);stable=native and raw_support and abs(gap)<=.0003 and error<=.003 and tilt<=math.radians(3) and speed<.005
    return dict(gap=gap,error=error,tilt=tilt,speed=speed,support=native and raw_support,stable=stable)
def transit(data,report,profile):
    event=next((e for e in report['events'] if e.get('event')=='transit_reference_snapshot'),None);d=[v for v in data if v['stage']=='transit'];checks=dict(actual_transit=bool(event and d));metrics={}
    if not all(checks.values()):return dict(passed=False,checks=checks,metrics=metrics)
    origin=event['root_estimate_m'];h0=event['heading_rad'];cube_local=rotate(-h0,[event['object_origin_m'][i]-origin[i] for i in range(3)]);palm_local=rotate(-h0,[event['palm_estimate_m'][i]-origin[i] for i in range(3)]);back=[a+b for a,b in zip(origin,rotate(h0,[-.1,0,0]))];turned=[a+b for a,b in zip(back,rotate(h0+math.pi,cube_local))];delta=[profile['fixtures'][4]['center_m'][i]-turned[i] for i in range(2)]+[0.];approach=rotate(-(h0+math.pi),delta)[0];initial=next(v for v in data if abs(v['time_s']-event['time_s'])<1e-9);initial_seat=seat(initial);errors=dict(object=0.,palm=0.,root=0.,seat=0.,heading=0.,protocol=0.);phases=set()
    for v in d:
        elapsed=v['time_s']-event['time_s'];i,path,dx,angle=reference(elapsed,approach);phases.add(i);root=[a+b for a,b in zip(origin,rotate(h0,[dx,0.,0.]))];obj=[a+b for a,b in zip(root,rotate(h0+angle,cube_local))];palm=[a+b for a,b in zip(root,rotate(h0+angle,palm_local))]
        for key,expected,prefix in [('object',obj,'object_'),('palm',palm,'palm_'),('root',root,'body_')]:errors[key]=max(errors[key],math.dist(expected,[v[prefix+k+'_m'] for k in 'xyz']))
        errors['seat']=max(errors['seat'],math.dist(seat(v),initial_seat));errors['heading']=max(errors['heading'],abs(wrap(h0+angle-v['yaw_rad'])));errors['protocol']=max(errors['protocol'],abs(v['planned_yaw_rad']-(h0+angle)),abs(v['transit_phase']-i),abs(v['carry_displacement_reference_m']-path),*[abs(v['planned_object_'+k+'_m']-obj[j]) for j,k in enumerate('xyz')],*[abs(v['planned_palm_'+k+'_m']-palm[j]) for j,k in enumerate('xyz')])
    checks.update(complete272s=event['duration_s']==272 and phases==set(range(1,7)) and d[-1]['time_s']-event['time_s']>=272-1e-8,independent_protocol=errors['protocol']<1e-9,object3mm=errors['object']<=.003,palm3mm=errors['palm']<=.003,root3mm=errors['root']<=.003,local_seat3mm=errors['seat']<=.003,heading05rad=errors['heading']<=.05,final_heading02rad=abs(wrap(h0+math.pi-d[-1]['yaw_rad']))<=.02,retained_admitted2g=all(v['payload_estimate_kg']==v['model_payload_reference_kg']==.002 for v in d),fixed_main_reference=all(abs(v[n+'_command_rad']-event['main_reference_rad'][n])<1e-12 for v in d for n in MAIN),actual_bilateral=all(v['obs_fingers']==2 for v in d))
    return dict(passed=all(checks.values()),checks=checks,metrics=dict(errors=errors,approach_m=approach),start_s=event['time_s'],end_s=d[-1]['time_s'])
def nominal(data,raw,faces,report,setup,profile):
    d=data;checks={};close=[r for r in d if r['stage']=='close'];hold=[r for r in d if r['stage']=='hold'];prelift=close[-1]['object_z_m'] if close else float('inf');lift=min((r['object_z_m']-prelift for r in hold),default=-1.)
    checks.update(lift8mm=lift>=.008,unsupported_hold6s=longest(d,lambda r:r['stage']=='hold' and r['obs_fingers']==2 and not r['obs_support'] and r['object_z_m']-prelift>=.008)>=6-1e-8)
    held_seat=seat(hold[0]) if hold else [0,0,0];checks['hold_slip3mm']=bool(hold) and max(math.dist(seat(v),held_seat) for v in hold)<=.003
    aligned=[]
    for v in reversed(close):
        if v['obs_fingers']!=2 or any(abs(v[f'finger_{i}']+v[f'distal_{i}'])>.001 for i in range(2)):break
        aligned.append(v)
    checks['aligned_bilateral_close1s']=bool(aligned) and aligned[0]['time_s']-aligned[-1]['time_s']>=1-1e-8
    checks['close_displacement250um']=bool(close) and max(math.dist([v['object_'+k+'_m'] for k in 'xy'],setup['source_object_xyz_m'][:2]) for v in close)<=.00025
    travel=transit(d,report,profile);checks.update({'transit_'+k:v for k,v in travel['checks'].items()})
    feats=[support_features(v,r,f,setup,profile) for v,r,f in zip(d,raw,faces)];opened=lambda v:all(abs(v[f'finger_{i}']+.3)<=.02 and abs(v[f'distal_{i}']-.3)<=.02 for i in range(2))
    for v,f in zip(d,feats):v['destination_stable']=f['stable']
    first_open=next((i for i,v in enumerate(d) if v['stage']=='dest_open'),None);stable_before=0.
    if first_open:
        end=d[first_open-1]['time_s']
        for v in reversed(d[:first_open]):
            if not v['destination_stable']:break
            stable_before=end-v['time_s']
    verify=[v for v in d if v['stage']=='dest_verify'];end=d[-1];goal=profile['fixtures'][4];entry=next((v for v in d if travel.get('start_s')==v['time_s']),None);rest_palm_z=setup['source_palm_xz_m'][1]
    if entry:
        local=seat(entry);world_seat=[sum(entry['palm_r'+str(3*i+j)]*local[j] for j in range(3)) for i in range(3)];rest_palm_z=goal['center_m'][2]+goal['half_m'][2]+setup['object_size_m'][2]/2-world_seat[2]
    clear=longest(d,lambda v:v['stage']=='dest_verify' and v['destination_stable'] and v['obs_fingers']==0 and opened(v) and v['palm_z_m']>=rest_palm_z+.012)
    checks.update(controller_success=report['controller_success'] and report['failure'] is None,complete_destination_stages={v['stage'] for v in d}>={'dest_lower','dest_settle','dest_open','dest_withdraw','dest_verify'},stable600ms_before_open=stable_before>=.6-1e-8,stable_clear3s=clear>=3-1e-8,actual_open_and_withdraw=bool(verify) and all(opened(v) and v['obs_fingers']==0 and v['palm_z_m']>=rest_palm_z+.012 and v['payload_estimate_kg']==v['model_payload_reference_kg']==0 for v in verify),fixed_destination3mm=feats[-1]['error']<=.003 and feats[-1]['stable'],heading_retained_at_destination=all(abs(wrap(v['yaw_rad']-v['planned_yaw_rad']))<=.05 for v in d if v['stage'].startswith('dest_')))
    return dict(passed=all(checks.values()),checks=checks,transit=travel,metrics=dict(min_hold_lift_m=lift,stable_before_open_s=stable_before,clear_s=clear,final_center_error_m=feats[-1]['error'],peak_pitch_rad=max(abs(v['pitch_rad']) for v in d)))
