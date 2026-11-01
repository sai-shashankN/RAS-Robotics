"""Isolated balanced vertical-path cube diagnostic, including failed outcomes."""
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
from palm_leveling import nominal_pose
from cartesian_pick_place import make_plan,balanced_pose,at_height

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]
PRIOR=MODEL/'experiments/cube_leveling_v2'
STUDY=MODEL/'experiments/cube_cartesian'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
write=lambda p,v:p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete Cartesian preparation')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Previous leveling evidence changed: '+name)
    common=STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    shutil.copy2(MODEL/'scripts/cartesian_pick_place.py',common/'cartesian_pick_place.py')
    f=common/'grasp_controller.py';source=f.read_text(encoding='utf-8')
    source=replace_once(source,'from return_planning import ReturnSequence','from return_planning import ReturnSequence\nfrom cartesian_pick_place import VerticalCycle')
    source=replace_once(source,"SETUP = json.loads(SETUP_PATH.read_text())","SETUP = json.loads(SETUP_PATH.read_text())\nPLAN_PATH = ROOT/'cartesian-plan.json'\nPLAN = json.loads(PLAN_PATH.read_text())")
    source=replace_once(source,"'cube_geometry.py', 'palm_leveling.py']]","'cube_geometry.py', 'palm_leveling.py', 'cartesian_pick_place.py']]")
    source=replace_once(source,'E_PATH, SETUP_PATH]','E_PATH, SETUP_PATH, PLAN_PATH]')
    source=replace_once(source,'dt = step/1000','dt = step/1000\nmotion = VerticalCycle(PLAN,NEGATIVE,dt)')
    source=replace_once(source,'load_plan = plan_level_load(P, E)',"load_plan = PLAN['load_plan']")
    source=replace_once(source,"    palm = nodes['PALM']","    if not (load_rejected or alignment_rejected or alignment_failed):\n        target,cartesian_stage = motion.update(now,payload_fraction,on_stand,ball_speed,S)\n        if cartesian_stage:stage = cartesian_stage\n    palm = nodes['PALM']")
    source=replace_once(source,"'object_palm_m':relative,","'object_palm_m':relative,'palm_world_position_m':list(palm_pos),'planned_palm_height_m':motion.height,")
    source=replace_once(source,"        checks.update({'palm_attitude_within_2_deg'","        checks['support_confirmed_before_release'] = motion.released_at is not None and not motion.blocked\n        checks.update({'palm_attitude_within_2_deg'")
    source=replace_once(source,"        report = {'palm_reference_frame'","        report = {'cartesian_cycle':motion.report(),'palm_reference_frame'")
    source=replace_once(source,"'contact_trace_scope'] = 'Observational 8 ms simulator state/contact trace; no extra object feedback or pose/force writes.'","'contact_trace_scope'] = 'Observational 8 ms trace; ideal simulated pedestal support and object speed gate opening. Object pose never targets joints; no object pose/force writes.'")
    f.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    from control_math import palm_position
    spec=importlib.util.spec_from_file_location('cartesian_audit',common/'audit_clearance_profile.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    import numpy as np
    rows=[];design=[]
    for v in ['a','b']:
        prior=PRIOR/'projects'/f'{v}_world';project=STUDY/'projects'/v
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        p=read(prior/'parameters.json');p['wrist_standoff']=.032;p['finger_targets']['claw_open']['finger']=-.3
        e=read(prior/'experiments/physics/cases'/f'{v}_world'/'experiment.json')
        opened=nominal_pose(p,e,dict(p['dynamic_arm_targets']['arm_reach'],**p['finger_targets']['claw_open']),'world',0)
        p['dynamic_arm_targets']['arm_reach']['wrist']=opened['wrist']
        plan=make_plan(p,e,.008);e['lift_pose']=at_height(plan,'closed_full',.008)
        write(project/'parameters.json',p);write(project/'cartesian-plan.json',plan)
        reach=[]
        for height in [.008,.010,.015,.020]:
            try:
                r=balanced_pose(p,e,[plan['anchor_world_xz_m'][0],plan['anchor_world_xz_m'][1]+height],0,.25,1,0)
                reach.append(dict(height_m=height,admitted=True,solution=r))
            except ValueError:reach.append(dict(height_m=height,admitted=False))
        # Verify every family waypoint by independent forward kinematics.
        error=0.;minimum=float('inf');closest=None
        for family,waypoints in plan['families'].items():
            for row in waypoints:
                palm=palm_position(p,row['pose']);point=audit.ry(row['pitch_rad'])@np.array(palm)+np.array([0,0,p['wheel_radius']])
                error=max(error,math.dist([point[0],point[2]],[plan['anchor_world_xz_m'][0],plan['anchor_world_xz_m'][1]+row['height_m']]))
                for a,b in itertools.combinations(audit.boxes(p,row['pose']),2):
                    if a[1]==b[1] or frozenset((a[1],b[1])) in audit.EXCLUDED:continue
                    gap=audit.gap(a,b)
                    if gap<minimum:minimum=gap;closest=[family,row['height_m'],a[0],b[0]]
        closing=audit.audit_path(p,at_height(plan,'open',0),at_height(plan,'closed_empty',0))
        screen=dict(sampled_self_clearance_m=minimum,closest=closest,fk_max_position_error_m=error,closing_path=closing,
            lift_reach_probes=reach,scope='Nominal sampled balanced waypoint families plus bounded linear empty closing path. Contact-constrained feedback paths and joint mounting pockets require dynamic/CAD checks.')
        write(project/'robot-clearance.json',screen)
        if minimum<=0 or error>1e-8 or not closing['conservative_nonadjacent_clearance_passed']:raise ValueError('Nominal self/FK screen failed')
        generator=(prior/'scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
        generator=replace_once(generator,"'cube_geometry.py', 'palm_leveling.py']:","'cube_geometry.py', 'palm_leveling.py', 'cartesian_pick_place.py']:")
        (project/'scripts/generate_grasp_worlds.py').write_text(generator,encoding='utf-8')
        for negative in [False,True]:
            case=v+'_vertical'+('_open' if negative else '');folder=project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config=folder/'experiment.json'
            write(config,dict(e,open_claw_control=negative))
            result=subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            setup=read(folder/'setup.json');rotation=audit.ry(setup['start_body_pitch_rad']);fixture=[];objects=[]
            top,bottom=setup['stand_top_m'],setup['tray_top_m'];xy=setup['initial_object_position_m'][:2]
            stand=('stand','fixture',np.array([*xy,(top+bottom)/2]),np.eye(3),np.array([.012,.012,top-bottom])/2)
            cube=('cube','object',np.array(setup['initial_object_position_m']),np.eye(3),np.array([.01]*3))
            for shape in audit.boxes(p,setup['start_pose']):
                worldshape=(shape[0],shape[1],np.array([0,0,p['wheel_radius']])+rotation@shape[2],rotation@shape[3],shape[4])
                fixture.append(audit.gap(worldshape,stand));objects.append(audit.gap(worldshape,cube))
            initial=dict(pedestal_gap_m=min(fixture),cube_gap_m=min(objects),passed=min(fixture)>0 and min(objects)>0)
            write(folder/'initial-clearance.json',initial)
            if not initial['passed']:raise ValueError('Initial fixture screen failed')
            # Open-claw nominal sweep relative to the stationary fixture/cube.
            gaps=[]
            for row in plan['families']['open']:
                if row['height_m']<0:continue
                rotation=audit.ry(row['pitch_rad'])
                for shape in audit.boxes(p,row['pose']):
                    ws=(shape[0],shape[1],np.array([0,0,p['wheel_radius']])+rotation@shape[2],rotation@shape[3],shape[4])
                    gaps.append((audit.gap(ws,stand),'pedestal',row['height_m'],shape[0]))
                    gaps.append((audit.gap(ws,cube),'cube',row['height_m'],shape[0]))
            sweep=min(gaps);write(folder/'empty-sweep-screen.json',dict(minimum_gap_m=sweep[0],closest=sweep[1:],scope='Sampled nominal open-only upward/downward path, fixed root x=0; actual balance drift/contact must pass its separate dynamic control.'))
            if sweep[0]<=0:raise ValueError('Nominal empty fixture sweep rejected')
            rows.append(dict(case=case,variant=v,frame='world',open_control=negative,project=str(project.relative_to(WORKSPACE)),world=str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))))
        design.append(dict(variant=v,**screen));print(json.dumps(dict(prepared=v,minimum_self_gap_mm=minimum*1000,fk_error_m=error,reach=reach)),flush=True)
    manifest=dict(date='2026-10-24',cases=rows,prior_checkpoint_sha256=sha(PRIOR/'checkpoint-index.json'),
        question='Can a level palm, balanced nominal vertical arm path, wider opening and contact-gated release avoid A diagonal pickup/release drag?',
        shared_geometry='32 mm wrist standoff; proximal opening -0.3 rad; 8 mm vertical nominal lift; up to 2 mm slow support seek. Same flat fingers, 20 mm/2 g cube, 12x12 mm pedestal, 2 ms physics/8 ms control, gains, joint ranges and motor caps.',
        limitations='Small prepared-pose diagnostic, not full reach or hardware validation. The 15/20 mm balanced fixed-base lift proposals fail this geometry/lean screen and are retained. Waypoints assume no base translation, with actual path error measured separately. Open/closed and payload families interpolate; live IMU levels the wrist. Ideal simulated support and speed gate release; no object pose targets, attachments, force or pose writes. Provisional fixed neck/palm mass retained. Main A/B models unchanged.')
    write(STUDY/'manifest.json',manifest);return manifest


def main():
    manifest=prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed Cartesian results protected')
    results,errors=[],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks={pool.submit(run_one,e):e for e in manifest['cases']}
        for task in as_completed(tasks):
            try:row=task.result();results.append(row)
            except Exception as error:row=dict(case=tasks[task]['case'],error=str(error));errors.append(row)
            print(json.dumps(row),flush=True)
    write(STUDY/'run-status.json',dict(results=results,infrastructure_errors=errors))
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
