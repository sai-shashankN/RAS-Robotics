"""Keep curling joints; coordinate distal faces for the cube benchmark."""
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
from palm_leveling import nominal_pose

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]
PRIOR=MODEL/'experiments/cube_coordinated'
STUDY=MODEL/'experiments/cube_parallel_pads'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
write=lambda p,v:p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete parallel-face candidate')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Coordination evidence changed')
    common=STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    shutil.copy2(MODEL/'scripts/coordinated_pick_place.py',common/'coordinated_pick_place.py')
    path=common/'generate_worlds.py';source=path.read_text(encoding='utf-8')
    source=replace_once(source,"'distal':[0,1.1]","'distal':P.get('distal_joint_range_rad',[0,1.1])")
    source=replace_once(source,"1.5, [0, 1.1])}","1.5, P.get('distal_joint_range_rad',[0,1.1]))}")
    path.write_text(source,encoding='utf-8')
    path=common/'grasp_controller.py';source=path.read_text(encoding='utf-8')
    source=replace_once(source,"commanded = dict(SETUP['start_pose'])","commanded = dict(SETUP['start_pose'])\ndistal_commands = {i:commanded['distal'] for i in range(FINGER_COUNT)}")
    old="""    for name in names:
        motors[name].setPosition(commanded[name.split('_')[0]])"""
    new="""    for name in names:
        if name.startswith('distal_'):
            i = int(name.rsplit('_',1)[1])
            if abort_reason is None:
                # Encoder-only compensation: keep distal face parallel to palm.
                lo,hi = P['distal_joint_range_rad']
                desired = max(lo,min(hi,-sensors[f'finger_{i}'].getValue()))
                limit = P['finger_target_rate_rad_s']*dt
                distal_commands[i] += max(-limit,min(limit,desired-distal_commands[i]))
            motors[name].setPosition(distal_commands[i])
        else:
            motors[name].setPosition(commanded[name.split('_')[0]])"""
    source=replace_once(source,old,new)
    source=replace_once(source,"        report = {'coordinator'","        report = {'distal_face_coordination':'Each distal command tracks negative measured proximal angle at the same finger target rate; extended modeled distal minimum -0.35 rad. No object pose target.','coordinator'")
    path.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    from control_math import palm_position
    spec=importlib.util.spec_from_file_location('parallel_audit',common/'audit_clearance_profile.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    import numpy as np
    entries=[]
    for v in ['a','b']:
        prior=PRIOR/'projects'/v;project=STUDY/'projects'/v
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        p=read(prior/'parameters.json');p['distal_joint_range_rad']=[-.35,1.1]
        p['finger_targets']['claw_open']=dict(finger=-.3,distal=.3)
        p['finger_targets']['claw_closed']=dict(finger=.12,distal=-.12)
        e=read(prior/'experiments/physics/cases'/f'{v}_coordinated'/'experiment.json')
        opened=nominal_pose(p,e,dict(p['dynamic_arm_targets']['arm_reach'],**p['finger_targets']['claw_open']),'world',0)
        p['dynamic_arm_targets']['arm_reach']['wrist']=opened['wrist']
        from grasp_planning import loaded_reference
        pitch=loaded_reference(p,e,opened,0);point=audit.ry(pitch)@np.array(palm_position(p,opened))+np.array([0,0,p['wheel_radius']])
        plan=nominal_plan(p,e,[float(point[0]),float(point[2])]);e['lift_pose']=at_height(plan,'closed_full',.008)
        write(project/'parameters.json',p);write(project/'cartesian-plan.json',plan)
        minimum=float('inf');closest=None
        for family,waypoints in plan['families'].items():
            for row in waypoints:
                for a,b in itertools.combinations(audit.boxes(p,row['pose']),2):
                    if a[1]==b[1] or frozenset((a[1],b[1])) in audit.EXCLUDED:continue
                    gap=audit.gap(a,b)
                    if gap<minimum:minimum=gap;closest=[family,row['height_m'],a[0],b[0]]
        closing=audit.audit_path(p,at_height(plan,'open',0),at_height(plan,'closed_empty',0))
        write(project/'robot-clearance.json',dict(minimum_sampled_self_gap_m=minimum,closest=closest,closing_path=closing,
            scope='Nominal equal-angle open/closed family screens, excluding direct joint neighbors. Independent distal encoder compensation/contact-constrained paths require actual dynamics.'))
        if minimum<=0 or not closing['conservative_nonadjacent_clearance_passed']:raise ValueError('Parallel-face nominal self-clearance failed')
        shutil.copy2(prior/'scripts/generate_grasp_worlds.py',project/'scripts/generate_grasp_worlds.py')
        for negative in [False,True]:
            case=v+'_parallel'+('_open' if negative else '');folder=project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config=folder/'experiment.json'
            write(config,dict(e,open_claw_control=negative))
            result=subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            setup=read(folder/'setup.json');top,bottom=setup['stand_top_m'],setup['tray_top_m'];xy=setup['initial_object_position_m'][:2]
            stand=('stand','fixture',np.array([*xy,(top+bottom)/2]),np.eye(3),np.array([.012,.012,top-bottom])/2)
            cube=('cube','object',np.array(setup['initial_object_position_m']),np.eye(3),np.array([.01]*3))
            gaps=[]
            for row in plan['families']['open']:
                if row['height_m']<0:continue
                rot=audit.ry(row['pitch_rad'])
                for shape in audit.boxes(p,row['pose']):
                    ws=(shape[0],shape[1],np.array([row['base_x_m'],0,p['wheel_radius']])+rot@shape[2],rot@shape[3],shape[4])
                    for obstacle in [stand,cube]:gaps.append((audit.gap(ws,obstacle),row['height_m'],shape[0],obstacle[0]))
            sweep=min(gaps);write(folder/'fixture-screen.json',dict(minimum_sampled_open_sweep_gap_m=sweep[0],closest=sweep[1:],scope='Nominal equal-angle moving-base open sweep; dynamic independent-digit motion separately observed.'))
            if sweep[0]<=0:raise ValueError('Parallel-face nominal fixture clearance failed')
            entries.append(dict(case=case,variant=v,open_control=negative,project=str(project.relative_to(WORKSPACE)),world=str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))))
        print(json.dumps(dict(prepared=v,minimum_self_gap_mm=minimum*1000,minimum_open_fixture_gap_mm=sweep[0]*1000)),flush=True)
    manifest=dict(date='2026-10-24',cases=entries,prior_study='cube_coordinated',prior_checkpoint_sha256=sha(PRIOR/'checkpoint-index.json'),
        question='Can encoder-coordinated distal faces prevent the retained cube tilt while preserving the two curling fingers and coordinated base/arm motion?',
        changes='Private distal physical/motor range extends from [0,1.1] to [-0.35,1.1] rad. Open proximal/distal targets -0.3/+0.3; closed +0.12/-0.12. Each distal command follows negative measured proximal angle at the existing 0.3 rad/s cap. Recomputed initial world-level wrist and nominal equilibria; fixture follows prepared pose, so world target shifts slightly. Same physical dimensions/masses, base/height coordinator, gains, 8 mm lift, timing, speed/torque caps, ideal strict release gate, contact abort, 2 ms physics/8 ms control.',
        limitations='Actuated curling mechanism is retained; face alignment is task-specific control, not a new parallel-jaw mechanism. Negative distal travel requires actual mechanical/joint design verification. Contact model, masses and actuators provisional; encoder-only face compensation and planar fixed-height/no-slip assumptions. Ideal object state only qualifies opening, never supplies arm/wheel targets. No attachments, object pose/force writes, hardware or main-model adoption.')
    write(STUDY/'manifest.json',manifest);return manifest


def main():
    m=prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Parallel-face evidence protected')
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
