import csv,json,math
from context import ROOT,SOURCE,STUDY,read,write,sha
from m1_audit import rows,longest
from support_audit import observe
from mission_audit import nominal,wrap
from robot_native_audit import native
from drive_audit import drive
from wheel_drive_audit import assess as wheel_drive
from spatial_audit import assess as spatial
from solid_math import measure
from fixture_world import parse
from packet_audit import packet_checks

def csvrows(p):
    with p.open() as f:return list(csv.DictReader(f))
def native_checks(out,d,raw,profile,h,case):
    native_rows=csvrows(out/'native-contacts.csv');faces=csvrows(out/'native-fixture-contacts.csv');events=csvrows(out/'native-forbidden-contacts.csv');init=csvrows(out/'native-contact-init.csv');n=round(8/h);step=h*.001
    checks=dict(complete_native_windows=len(native_rows)==len(faces)==len(d) and all(abs(float(a['window_end_s'])-v['time_s'])<1e-8 and a['window_end_s']==f['window_end_s'] and int(a['native_steps'])==int(f['native_steps'])==n and abs(float(a['first_native_s'])-i*.008)<1e-8 and abs(float(a['last_native_s'])-((i+1)*.008-step))<1e-8 for i,(a,f,v) in enumerate(zip(native_rows,faces,d))),mapped21=len(init)==1 and int(init[0]['mapped_geoms'])==21 and int(init[0]['valid'])==1,finite_native=all(math.isfinite(float(x)) for r in native_rows+faces+events for x in r.values()),valid_native=all(int(a['valid'])==int(f['valid'])==1 and int(a['unknown_points'])==int(a['overflow'])==0 and int(f['active_fixtures'])==8 for a,f in zip(native_rows,faces)),no_native_forbidden=all(int(a['forbidden_points'])==0 for a in native_rows),event_coverage=sum(int(x['point_count']) for x in events)==sum(int(x['forbidden_points'])+int(x['unknown_points']) for x in native_rows),cube_depth1mm=max(float(a['max_cube_static_depth_m']) for a in native_rows)<=.001,wheel_floor_positive=any(int(a['wheel_floor_points']) for a in native_rows),fixture_partitions=all(sum(int(f[f'fixture_{i}_points']) for i in range(8))==int(a['cube_fixture_points']) and all(int(f[f'fixture_{i}_points'])==int(f[f'fixture_{i}_top_points'])+int(f[f'fixture_{i}_other_points']) for i in range(8)) for a,f in zip(native_rows,faces)))
    if case=='no_grasp':checks['case_grip_absent']=all(int(a['grip_cube_points'])==int(a['bilateral_native_steps'])==int(a['min_grip_fingers'])==0 for a in native_rows)
    else:checks['case_grip_positive']=any(int(a['grip_cube_points'])>0 for a in native_rows)
    if case=='nominal':
        checks['only_source_destination_top']=all(all(int(f[f'fixture_{i}_other_points'])==0 and (i in [0,4] or int(f[f'fixture_{i}_points'])==0) for i in range(8)) for f in faces)
        carried=[(v,a) for v,a in zip(d,native_rows) if v['stage'] in ['hold','transit']];checks['all_native_carry_steps_bilateral_unsupported']=bool(carried) and all(int(a['cube_fixture_points'])==int(a['cube_floor_points'])==0 and int(a['min_grip_fingers'])==2 and int(a['bilateral_native_steps'])==n for v,a in carried)
        destination=[f for v,f in zip(d,faces) if v['stage'].startswith('dest_')];checks['destination_never_source']=bool(destination) and all(int(f['fixture_0_points'])==0 for f in destination)
    return checks,faces

def safety_checks(d,p):
    tail=[v for v in d if v['time_s']>=d[-1]['time_s']-2];main_rate=max((abs(b[n+'_command_rad']-a[n+'_command_rad'])/.008 for a,b in zip(d,d[1:]) for n in ['shoulder','elbow','wrist']),default=0.);finger_rate=max((abs(b[n+'_command_rad']-a[n+'_command_rad'])/.008 for a,b in zip(d,d[1:]) for n in ['finger_0','finger_1','distal_0','distal_1']),default=0.)
    return dict(finite=all(math.isfinite(v) for r in d for v in r.values() if isinstance(v,(int,float))),actual_contact_clear=all(r['bad_contact_count']==r['obs_bad_arm']==0 for r in d),body_tilt20deg=all(max(abs(r['pitch_rad']),abs(r['roll_rad']))<math.radians(20) for r in d),final_pitch1deg=all(max(abs(r['pitch_rad']),abs(r['roll_rad']))<math.radians(1) for r in tail),final_speed01=all(r['body_speed_m_s']<.01 for r in tail),joint_ranges=max(r['range_excess_rad'] for r in d)<=.03,main_reference_rate=main_rate<=.08+1e-7,finger_reference_rate=finger_rate<=.3+1e-7,wheel_speed=max(abs(r[k+'_speed_rad_s']) for r in d for k in ['left','right'])<=p['wheel_speed_limit']+.5,wheel_torque_reserve=max(abs(r[k+'_torque_nm']) for r in d for k in ['left','right'])<=p['wheel_torque_limit']*.9,yaw_torque_reserve=max(abs(r['yaw_turn_torque_nm']) for r in d)<=.08+1e-12,yaw_request_bound=max(abs(r['yaw_request_rad_s']) for r in d)<=.15+1e-12,mass_guard=max(abs(r['mass_lean_rad']) for r in d)<=.3,mass_model=max(r['robot_com_model_error_m'] for r in d)<.002,palm_level2deg=all(math.acos(max(-1.,min(1.,-r['palm_r8'])))<math.radians(2) for r in d),no_controller_penetration=all(not r['penetration'] for r in d))

def source_prefix(out,d,e):
    gold=ROOT/e['golden_directory'];event=next((v for v in read(out/'report.json')['events'] if v.get('event')=='transit_reference_snapshot'),None)
    if event is None:return dict(passed=False)
    end=event['time_s'];a=[v for v in csvrows(out/'telemetry.csv') if float(v['time_s'])<=end+1e-9];b=csvrows(gold/'telemetry.csv')[:len(a)];keys=set(b[0]);same=bool(a) and all({k:v for k,v in x.items() if k in keys}==y for x,y in zip(a,b));raw=(out/'contacts.jsonl').read_text().splitlines()[:len(a)]==(gold/'contacts.jsonl').read_text().splitlines()[:len(a)]
    return dict(passed=same and raw,telemetry=same,raw=raw,through_s=end)

def fault_checks(out,d,report,case,e):
    failed=next((r for r in d if r['failure_latched']),None);event=next((v for v in report['events'] if v.get('event')=='position_recovery'),None);checks=dict(task_rejected=not report['controller_success'] and report['failure'] is not None,recovery5s=failed is not None and d[-1]['time_s']-failed['time_s']>=5-1e-8,immediate_reference_clear=failed is not None and all(v['payload_estimate_kg']==v['model_payload_reference_kg']==0 and not v['torque_mode'] for v in d if v['time_s']>=failed['time_s']),measured_pose_handoff=False,measured_pose_held=False)
    if event:
        at=next(v for v in d if abs(v['time_s']-event['time_s'])<1e-8);checks['measured_pose_handoff']=all(at[n]==a==at[n+'_command_rad'] for n,a in event['pose_rad'].items());checks['measured_pose_held']=all(all(v[n+'_command_rad']==a for n,a in event['pose_rad'].items()) for v in d if v['time_s']>=event['time_s'])
    trigger=report['fault_trigger_time_s']
    if case=='no_grasp':
        trigger=next((v['time_s']-.008 for v in d if v['stage']=='close'),None);checks.update(expected_rejection=report['failure']=='grasp_not_established',never_admitted=all(v['payload_estimate_kg']==v['model_payload_reference_kg']==0 and v['finger_mask']==0 for v in d),open_commands=all(abs(v[f'finger_{i}_command_rad']+.3)<1e-12 and abs(v[f'distal_{i}_command_rad']-.3)<1e-12 for v in d for i in range(2)))
    else:
        checks['actual_injection']=trigger is not None and any(v.get('event')=='fault_triggered' and v.get('case')==case for v in report['events']);checks['previously_admitted']=trigger is not None and any(v['payload_estimate_kg']==.002 and v['time_s']<trigger for v in d)
        checks['expected_rejection']=report['failure']=={'drop_transit':'grasp_lost','transit_stop':'injected_transit_stop','blocked_open':'opening_blocked'}[case]
        if case=='drop_transit':checks['actual_grip_loss']=any(v['obs_fingers']<2 and v['time_s']>=trigger for v in d)
        if case=='blocked_open':checks['actual_jam']=any(v.get('event')=='fault_triggered' and 'actual_grasp_angles_rad' in v for v in report['events'])
    gold=ROOT/e['project']/'results'/('nominal_'+str(e['physics_ms']).replace('.','p'));prefix=False
    if trigger is not None and (gold/'report.json').exists():
        a=[v for v in csvrows(out/'telemetry.csv') if float(v['time_s'])<trigger-1e-9];b=csvrows(gold/'telemetry.csv')[:len(a)];raw=(out/'contacts.jsonl').read_text().splitlines()[:len(a)]==(gold/'contacts.jsonl').read_text().splitlines()[:len(a)];na=[v for v in csvrows(out/'native-contacts.csv') if float(v['window_end_s'])<=trigger+1e-9];nb=csvrows(gold/'native-contacts.csv')[:len(na)];prefix=bool(a) and a==b and raw and na==nb
    checks['exact_nominal_prefix']=prefix
    return dict(passed=all(checks.values()),checks=checks)

def assess(e,case,out):
    d=rows(out/'telemetry.csv');setup=read(out/'sources/setup.json');p=read(out/'sources/parameters.json');profile=read(out/'sources/collision-fixtures.json');report=read(out/'report.json');raw=[json.loads(v) for v in (out/'contacts.jsonl').read_text().splitlines()];consistent=observe(d,out/'contacts.jsonl',setup);checks=dict(raw_contact_consistency=consistent and len(raw)==len(d),complete_control_cadence=bool(d) and abs(d[0]['time_s']-.008)<1e-8 and all(abs(b['time_s']-a['time_s']-.008)<1e-8 for a,b in zip(d,d[1:])),free_cube_and_root=report['cube_free'] and report['physical_root'],end_matches_report=abs(report['duration_s']-d[-1]['time_s'])<1e-8,stderr_empty=(out/'webots-stderr.log').stat().st_size==0)
    checks.update(safety_checks(d,p));native_gate,faces=native_checks(out,d,raw,profile,e['physics_ms'],case);checks.update(native_gate)
    checks['hand_tracking3mm']=all(math.dist([v['palm_'+k+'_m'] for k in 'xyz'],[v['planned_palm_'+k+'_m'] for k in 'xyz'])<.003 if v['destination_mode'] else math.hypot(v['palm_x_m']-v['target_x_m'],v['palm_z_m']-v['target_z_m'])<.003 for v in d if v['stage']!='recovery')
    checks['declared_membership_and_reference_range']=all(v['payload_estimate_kg'] in [0.,.002] and 0<=v['model_payload_reference_kg']<=.002 for v in d)
    if case=='nominal':
        first=next((v['time_s'] for v in d if v['stage']=='dest_open'),None)
        checks['support_gated_smooth_reference']=first is not None and all(abs(v['model_payload_reference_kg']-.002*(1-(lambda u:u*u*(3-2*u))(max(0.,min(1.,(v['time_s']-first)/1.)))))<1e-12 for v in d if v['time_s']>=first)
        releases=[(v,f) for prev,v,f in zip(d,d[1:],faces[1:]) if prev['payload_estimate_kg'] and not v['payload_estimate_kg']]
        checks['actual_supported_release']=len(releases)==1 and all(v['stage']=='dest_open' and v['obs_fingers']==0 and int(f['fixture_4_top_points'])>0 for v,f in releases)
    world=out/'sources/worlds'/f'integrated_{e["physics_ms"]}.wbt';actual=parse(world.read_text(),8);checks['fixed_physical_fixtures']=profile['fixtures']==[{k:v for k,v in f.items() if k!='id'} for f in actual] and profile['world_sha256']==sha(world) and abs(actual[0]['center_m'][0]-actual[4]['center_m'][0]-.6)<1e-12 and profile['tolerance_m']==.001
    penetration=False;geometry_error=0.
    for v in d:
        state=measure([v['object_'+k+'_m'] for k in 'xyz'],[v['object_r'+str(i)] for i in range(9)],setup['object_size_m'],profile);penetration |= state['violated'];geometry_error=max(geometry_error,abs(state['minimum_fixture_gap_m']-v['solid_gap_m']),abs(state['floor_bottom_m']-v['floor_bottom_m']))
    checks.update(independent_cube_geometry=geometry_error<1e-10,no_actual_penetration=not penetration)
    frame=native(out,[dict(v,x_m=v['body_x_m'],y_m=v['body_y_m'],z_m=v['body_z_m']) for v in d]);act=drive(out,d);wheel=wheel_drive(out,d);space=spatial(d);packet=packet_checks(out,e['physics_ms']);checks.update(native_frame=frame['valid'],native_main_drive=act['passed'],native_wheel_drive=wheel['passed'],full3D_estimates=space['passed'],live_contact_receipt=packet['passed'])
    launch=read(out/'launch-inputs.json');checks['all_launched_sources_bound']=launch['entry']==e and launch['case']==case and launch['plan_sha256']==sha(STUDY/'test-plan.json') and launch['preflight_sha256']==sha(out/'preflight-before-launch.json') and all(sha(out/'sources'/n)==h for n,h in launch['sha256'].items())
    task=nominal(d,raw,faces,report,setup,profile) if case=='nominal' else fault_checks(out,d,report,case,e);checks['full_task_or_fault']=task['passed'];prefix=source_prefix(out,d,e) if case=='nominal' else None
    if prefix is not None:checks['qualified_source_pickup_prefix']=prefix['passed']
    result=dict(entry=e,case=case,directory=str(out.relative_to(STUDY)),passed=all(checks.values()),checks=checks,task=task,source_prefix=prefix,frame=frame,actuation=act,wheel_actuation=wheel,spatial=space,packet=packet,report_failure=report['failure'],geometry_error_m=geometry_error,input_sha256={str(f.relative_to(out)):sha(f) for f in out.iterdir() if f.is_file() and f.name not in ['audit.json']});write(out/'audit.json',result);return result
