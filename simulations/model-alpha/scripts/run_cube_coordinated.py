"""Private arm-height/wheel-horizontal coordination and stricter release study."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import importlib.util
import itertools
import json
import math
import shutil
import subprocess
import sys
from run_contact_width_study import sha,replace_once
from run_profile_physics_checks import run_one
from coordinated_pick_place import nominal_plan
from cartesian_pick_place import at_height

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]
PRIOR=MODEL/'experiments/cube_cartesian'
STUDY=MODEL/'experiments/cube_coordinated'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
write=lambda p,v:p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete coordination preparation')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Original path evidence changed')
    common=STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    shutil.copy2(MODEL/'scripts/coordinated_pick_place.py',common/'coordinated_pick_place.py')
    path=common/'cartesian_pick_place.py';source=path.read_text(encoding='utf-8')
    source=replace_once(source,'def update(self,now,payload,on_stand,speed,schedule):','def update(self,now,payload,on_stand,speed,schedule,placement_is_ready=True):')
    source=replace_once(source,'elif speed<.005:','elif speed<.005 and placement_is_ready:')
    source=replace_once(source,'Ideal simulated pedestal-support witness and speed below 5 mm/s for 0.6 s;','Ideal simulator pedestal support, speed below 5 mm/s, cube tilt below 2 deg and centering within 2 mm for 0.6 s;')
    path.write_text(source,encoding='utf-8')
    path=common/'grasp_controller.py';source=path.read_text(encoding='utf-8')
    source=replace_once(source,'from cartesian_pick_place import VerticalCycle','from cartesian_pick_place import VerticalCycle\nfrom coordinated_pick_place import HeightBaseCoordinator, placement_ready')
    source=replace_once(source,"'palm_leveling.py', 'cartesian_pick_place.py']]","'palm_leveling.py', 'cartesian_pick_place.py', 'coordinated_pick_place.py']]")
    source=replace_once(source,'motion = VerticalCycle(PLAN,NEGATIVE,dt)',"motion = VerticalCycle(PLAN,NEGATIVE,dt)\ncoordinator = HeightBaseCoordinator(P,PLAN['anchor_world_xz_m'],dt)\nabort_reason = None\nabort_time = None\nabort_wheel_position = None")
    old='''    if not (load_rejected or alignment_rejected or alignment_failed):
        target,cartesian_stage = motion.update(now,payload_fraction,on_stand,ball_speed,S)
        if cartesian_stage:stage = cartesian_stage'''
    new='''    cube_tilt = math.acos(max(-1,min(1,ball.getOrientation()[8])))
    cube_center_error = math.hypot(ball_pos[0]-center_xy[0],ball_pos[1]-center_xy[1])
    ready_to_release = placement_ready(on_stand,ball_speed,cube_tilt,cube_center_error)
    if bad_contacts and abort_reason is None:
        abort_reason,abort_time = 'unintended_arm_contact',now
        S['finish'] = now+3
    if not (load_rejected or alignment_rejected or alignment_failed or abort_reason):
        target,cartesian_stage = motion.update(now,payload_fraction,on_stand,ball_speed,S,ready_to_release)
        if cartesian_stage:stage = cartesian_stage
    if abort_reason:
        target = dict(commanded);stage = 'aborted_'+abort_reason'''
    source=replace_once(source,old,new)
    source=replace_once(source,"'planned_palm_height_m':motion.height,","'planned_palm_height_m':motion.height,'placement_observer_ready':ready_to_release,'abort_reason':abort_reason,")
    old="    alignment_error = wheel_position-position_reference\n    if capture_plan['enabled']"
    new="""    if abort_reason is None:
        coordinated_target,position_reference,velocity_reference = coordinator.update(motion.height,pitch,angles,target)
        if coordinated_target is None:
            abort_reason,abort_time = 'height_ik_unreachable',now
            S['finish'] = now+3
        else:target = coordinated_target
    if abort_reason:
        if abort_wheel_position is None:abort_wheel_position = wheel_position
        target = dict(commanded);stage = 'aborted_'+abort_reason
        position_reference,velocity_reference = abort_wheel_position,0.
    alignment_error = wheel_position-position_reference
    if capture_plan['enabled']"""
    source=replace_once(source,old,new)
    source=replace_once(source,"    target = dict(target,wrist=max(low,min(high,goal)))","    if abort_reason is None:target = dict(target,wrist=max(low,min(high,goal)))")
    source=replace_once(source,"        checks['support_confirmed_before_release']", "        checks['coordination_completed_without_abort'] = abort_reason is None\n        checks['height_ik_remained_in_range'] = coordinator.invalid_samples==0\n        checks['support_confirmed_before_release']")
    source=replace_once(source,"        report = {'cartesian_cycle'","        report = {'coordinator':coordinator.report(),'abort_reason':abort_reason,'abort_time_s':abort_time,'cartesian_cycle'")
    source=replace_once(source,'        if tracefile:\n            tracefile.close()',"        report['validation_passed'] = bool(report['validation_passed'] and abort_reason is None)\n        if tracefile:\n            tracefile.close()")
    source=replace_once(source,'ideal simulated pedestal support and object speed gate opening.','ideal simulator support/speed/tilt/centering gate opening; unintended arm contact aborts arm motion.')
    path.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    from control_math import palm_position
    spec=importlib.util.spec_from_file_location('coordinated_audit',common/'audit_clearance_profile.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    import numpy as np
    entries=[]
    for v in ['a','b']:
        prior=PRIOR/'projects'/v;project=STUDY/'projects'/v
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        shutil.copy2(prior/'parameters.json',project/'parameters.json');p=read(project/'parameters.json')
        e=read(prior/'experiments/physics/cases'/f'{v}_vertical'/'experiment.json')
        anchor=read(prior/'cartesian-plan.json')['anchor_world_xz_m'];plan=nominal_plan(p,e,anchor)
        e['lift_pose']=at_height(plan,'closed_full',.008);write(project/'cartesian-plan.json',plan)
        minimum=float('inf');closest=None;fkerror=0
        for family,waypoints in plan['families'].items():
            for row in waypoints:
                palm=np.array(palm_position(p,row['pose']));world=audit.ry(row['pitch_rad'])@palm+np.array([row['base_x_m'],0,p['wheel_radius']])
                fkerror=max(fkerror,math.dist([world[0],world[2]],[anchor[0],anchor[1]+row['height_m']]))
                for a,b in itertools.combinations(audit.boxes(p,row['pose']),2):
                    if a[1]==b[1] or frozenset((a[1],b[1])) in audit.EXCLUDED:continue
                    gap=audit.gap(a,b)
                    if gap<minimum:minimum=gap;closest=[family,row['height_m'],a[0],b[0]]
        closing=audit.audit_path(p,at_height(plan,'open',0),at_height(plan,'closed_empty',0))
        screen=dict(minimum_sampled_self_gap_m=minimum,closest=closest,fk_max_error_m=fkerror,closing_path=closing,
            scope='Nominal equilibrium family samples and linear closing envelope. Direct joint neighbors excluded; real contact-constrained feedback/continuous paths not certified.')
        write(project/'robot-clearance.json',screen)
        if minimum<=0 or fkerror>1e-8 or not closing['conservative_nonadjacent_clearance_passed']:raise ValueError('Nominal geometry admission failed')
        generator=(prior/'scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
        generator=replace_once(generator,"'palm_leveling.py', 'cartesian_pick_place.py']:","'palm_leveling.py', 'cartesian_pick_place.py', 'coordinated_pick_place.py']:")
        (project/'scripts/generate_grasp_worlds.py').write_text(generator,encoding='utf-8')
        for negative in [False,True]:
            case=v+'_coordinated'+('_open' if negative else '');folder=project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config=folder/'experiment.json'
            write(config,dict(e,open_claw_control=negative))
            result=subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            setup=read(folder/'setup.json');top,bottom=setup['stand_top_m'],setup['tray_top_m'];xy=setup['initial_object_position_m'][:2]
            stand=('stand','fixture',np.array([*xy,(top+bottom)/2]),np.eye(3),np.array([.012,.012,top-bottom])/2)
            cube=('cube','object',np.array(setup['initial_object_position_m']),np.eye(3),np.array([.01]*3))
            gaps=[];initial=[]
            for row in plan['families']['open']:
                if row['height_m']<0:continue
                rot=audit.ry(row['pitch_rad'])
                for shape in audit.boxes(p,row['pose']):
                    ws=(shape[0],shape[1],np.array([row['base_x_m'],0,p['wheel_radius']])+rot@shape[2],rot@shape[3],shape[4])
                    for obstacle in [stand,cube]:
                        entry=(audit.gap(ws,obstacle),row['height_m'],shape[0],obstacle[0]);gaps.append(entry)
                        if row['height_m']==0:initial.append(entry)
            sweep=min(gaps);write(folder/'fixture-screen.json',dict(minimum_sampled_open_sweep_gap_m=sweep[0],closest=sweep[1:],minimum_initial_gap_m=min(initial)[0],scope='Nominal moving-base equilibrium open sweep. Actual joint/IMU/odometry tracking requires dynamic empty controls.'))
            if sweep[0]<=0:raise ValueError('Nominal moving-base open sweep failed')
            entries.append(dict(case=case,variant=v,open_control=negative,project=str(project.relative_to(WORKSPACE)),world=str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))))
        print(json.dumps(dict(prepared=v,minimum_self_gap_mm=minimum*1000,minimum_empty_fixture_gap_mm=sweep[0]*1000)),flush=True)
    manifest=dict(date='2026-10-24',cases=entries,prior_checkpoint_sha256=sha(PRIOR/'checkpoint-index.json'),
        question='Does fixed-shoulder height control plus wheel hand-X tracking improve both cube axes without stronger balance gains?',
        changes='Same physical model, original gains, motor caps, 8 mm lift and timing. Elbow height target uses IMU pitch; wheel reference uses encoder-derived palm reach. Reference shaped to 6 mm/s and 10 mm/s². Release requires ideal support/speed/tilt/centering; unwanted arm contact or out-of-range height IK aborts arm commands and holds current base position for three seconds.',
        limitations='Prepared planar diagnostic with fixed root height/no-slip odometry assumptions, provisional rigid masses/friction/actuators. Object state only qualifies opening, never determines arm/wheel targets. No pose/force writes or attachments. Sampled screens exclude joint neighbors. No hardware or main-model adoption.')
    write(STUDY/'manifest.json',manifest);return manifest


def main():
    m=prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Coordinated evidence protected')
    results,errors=[],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks={pool.submit(run_one,e):e for e in m['cases']}
        for task in as_completed(tasks):
            try:r=task.result();results.append(r)
            except Exception as err:r=dict(case=tasks[task]['case'],error=str(err));errors.append(r)
            print(json.dumps(r),flush=True)
    write(STUDY/'run-status.json',dict(results=results,infrastructure_errors=errors))
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
