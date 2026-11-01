"""Matched body/world palm attitude comparison after the user's diagonal-grip diagnosis."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import importlib.util
import json
import math
import shutil
import subprocess
import sys
from run_contact_width_study import sha,replace_once
from run_profile_physics_checks import run_one
from palm_leveling import nominal_pose

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/cube_baseline'
STUDY = MODEL/'experiments/cube_leveling'
read = lambda p:json.loads(p.read_text(encoding='utf-8'))


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete level-palm preparation')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Cube baseline changed')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    shutil.copy2(MODEL/'scripts/palm_leveling.py',common/'palm_leveling.py')
    f = common/'grasp_controller.py';source = f.read_text(encoding='utf-8')
    source = replace_once(source,'from grasp_planning import loaded_reference, plan_load','from grasp_planning import loaded_reference, plan_load\nfrom palm_leveling import wrist_goal, angle_error, plan_level_load')
    source = replace_once(source,"'generate_worlds.py', 'cube_geometry.py']]","'generate_worlds.py', 'cube_geometry.py', 'palm_leveling.py']]")
    source = replace_once(source,'load_plan = plan_load(P, E)','load_plan = plan_level_load(P, E)')
    source = replace_once(source,'unexpected_contacts = 0','max_palm_attitude_error = 0.0\nwrist_goal_clips = 0\nunexpected_contacts = 0')
    source = replace_once(source,"'object_box_corners_m':box_points", "'palm_world_pitch_rad':pitch+sum(sensors[n].getValue() for n in ['shoulder','elbow','wrist']),'object_box_corners_m':box_points")
    anchor = "    duration = max(abs(target[j]-commanded[j])"
    replacement = '''    goal = wrist_goal(sensors['shoulder'].getValue(),sensors['elbow'].getValue(),pitch,E['palm_reference_frame'])
    low,high = P['joint_ranges']['wrist']
    wrist_goal_clips += int(not low<=goal<=high)
    target = dict(target,wrist=max(low,min(high,goal)))
    observed = sum(sensors[n].getValue() for n in ['shoulder','elbow','wrist'])+(pitch if E['palm_reference_frame']=='world' else 0)
    if S['close']-.5<=now<=S['release']+2:
        max_palm_attitude_error = max(max_palm_attitude_error,abs(angle_error(observed)))
    duration = max(abs(target[j]-commanded[j])'''
    source = replace_once(source,anchor,replacement)
    source = replace_once(source,"        report = {'finger_count':FINGER_COUNT", "        checks.update({'palm_attitude_within_2_deg':max_palm_attitude_error<math.radians(2),'wrist_attitude_goal_within_range':wrist_goal_clips==0})\n        report = {'palm_reference_frame':E['palm_reference_frame'],'max_palm_attitude_error_deg':math.degrees(max_palm_attitude_error),'wrist_goal_clip_samples':wrist_goal_clips,'finger_count':FINGER_COUNT")
    f.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    from grasp_planning import loaded_reference
    spec = importlib.util.spec_from_file_location('level_geometry',common/'audit_clearance_profile.py')
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    import numpy as np
    review = STUDY/'design-review';review.mkdir()
    for name in ['level-pose-survey.json','level-clearance-survey.json']:
        if (WORKSPACE/'.local-tools'/name).exists():shutil.copy2(WORKSPACE/'.local-tools'/name,review/name)
    original_p = read(PRIOR/'projects/a/parameters.json');original_e = read(PRIOR/'projects/a/experiments/physics/cases/a_cube/experiment.json')
    oldposes = [nominal_pose(original_p,original_e,dict(original_p['dynamic_arm_targets']['arm_reach'],**original_p['finger_targets'][name]),'world',load) for name,load in [('claw_open',0),('claw_closed',1)]]
    oldposes += [nominal_pose(original_p,original_e,original_e['lift_pose'],'world',1)]
    (review/'original-15mm-level-rejection.json').write_text(json.dumps({'close':audit.audit_path(original_p,oldposes[0],oldposes[1]),'lift':audit.audit_path(original_p,oldposes[1],oldposes[2]),'note':'Envelope overlaps, not proof of exact mesh collisions; not admitted as the level-grip test geometry.'},indent=2)+'\n',encoding='utf-8')
    generator = (PRIOR/'projects/a/scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
    generator = replace_once(generator,"'mass_properties.py', 'cube_geometry.py']:","'mass_properties.py', 'cube_geometry.py', 'palm_leveling.py']:")
    rows = []
    for v in ['a','b']:
        for frame in ['body','world']:
            project = STUDY/'projects'/f'{v}_{frame}'
            for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
            p = read(PRIOR/'projects'/v/'parameters.json');p['wrist_standoff'] = .035;p['finger_targets']['claw_open']['finger'] = -.2
            e = dict(read(PRIOR/'projects'/v/'experiments/physics/cases'/f'{v}_cube'/'experiment.json'),palm_reference_frame=frame)
            opened = nominal_pose(p,e,dict(p['dynamic_arm_targets']['arm_reach'],**p['finger_targets']['claw_open']),frame,0)
            closed = nominal_pose(p,e,dict(p['dynamic_arm_targets']['arm_reach'],**p['finger_targets']['claw_closed']),frame,1)
            lifted = nominal_pose(p,e,e['lift_pose'],frame,1)
            p['dynamic_arm_targets']['arm_reach']['wrist'] = opened['wrist'];e['lift_pose'] = lifted
            (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
            (project/'scripts/generate_grasp_worlds.py').write_text(generator,encoding='utf-8')
            paths = {}
            for name,start,end in [('close',opened,closed),('lift_lower',closed,lifted),('open_control_lift',opened,dict(lifted,**p['finger_targets']['claw_open']))]:
                for delta in [-.015,0,.015]:
                    paths[f'{name}_wrist_{delta:+.3f}'] = audit.audit_path(p,dict(start,wrist=start['wrist']+delta),dict(end,wrist=end['wrist']+delta))
            screen = {'paths':paths,'all_passed':all(r['conservative_nonadjacent_clearance_passed'] for r in paths.values()),
                'scope':'Specified linear nominal pose paths and +/-0.015 rad wrist-offset paths; nonlinear feedback/contact-constrained motion is checked dynamically. Direct joint neighbors excluded. Not an arbitrary-workspace certificate.',
                'nominal_poses':{'opened':opened,'closed':closed,'lifted':lifted},
                'source_sha256':{str(path.relative_to(WORKSPACE)):sha(path) for path in [Path(__file__),common/'palm_leveling.py',common/'generate_worlds.py',common/'audit_clearance_profile.py',project/'parameters.json']}}
            (project/'robot-clearance.json').write_text(json.dumps(screen,indent=2)+'\n',encoding='utf-8')
            if not screen['all_passed']:raise ValueError('Nominal attitude corridor failed: '+project.name)
            for negative in [False,True]:
                case = f'{v}_{frame}'+('_open' if negative else '');folder = project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config = folder/'experiment.json'
                config.write_text(json.dumps(dict(e,open_claw_control=negative),indent=2)+'\n',encoding='utf-8')
                result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
                if result.returncode:raise RuntimeError(result.stdout+result.stderr)
                setup = read(folder/'setup.json');rotation = audit.ry(setup['start_body_pitch_rad']);fixture=[];objects=[]
                top = setup['stand_top_m'];bottom = setup['tray_top_m'];xy = setup['initial_object_position_m'][:2]
                stand = ('stand','fixture',np.array([*xy,(top+bottom)/2]),np.eye(3),np.array([.012,.012,top-bottom])/2)
                cube = ('cube','object',np.array(setup['initial_object_position_m']),np.eye(3),np.array([.01,.01,.01]))
                for shape in audit.boxes(p,setup['start_pose']):
                    worldshape = (shape[0],shape[1],np.array([0,0,p['wheel_radius']])+rotation@shape[2],rotation@shape[3],shape[4])
                    fixture.append(audit.gap(worldshape,stand));objects.append(audit.gap(worldshape,cube))
                initial = {'pedestal_gap_m':min(fixture),'cube_gap_m':min(objects),'passed':min(fixture)>0 and min(objects)>0}
                (folder/'initial-clearance.json').write_text(json.dumps(initial,indent=2)+'\n',encoding='utf-8')
                if not initial['passed']:raise ValueError('Initial cube/pedestal envelope overlap: '+case)
                rows.append({'case':case,'variant':v,'frame':frame,'open_control':negative,'project':str(project.relative_to(WORKSPACE)),
                    'world':str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))})
            print(json.dumps({'prepared':project.name,'minimum_robot_bound_mm':1000*min(r['conservative_clearance_bound_m'] for r in paths.values()),'nominal_max_lean_rad':max(abs(loaded_reference(p,e,x,1)) for x in [opened,closed,lifted])}),flush=True)
    manifest = {'date':'2026-10-24','cases':rows,'prior_checkpoint_sha256':sha(PRIOR/'checkpoint-index.json'),
        'question':'User hypothesis: diagonal palm approach/placement contributes to A release drag. Test level world-frame attitude against tilted body-frame attitude in both fixed pinch axes.',
        'shared_geometry':'35 mm wrist-to-palm axial standoff (15 mm prior); open proximal target -0.2 rad (-0.45 prior); same 20 mm/2 g cube, 12x12 pedestal, flat pads, base geometry, gains, torque caps, speed limits, schedule, 2 ms physics and 8 ms control.',
        'comparison':'Both frames use encoder-based wrist coordination to hold palm pitch pi in the selected frame. World mode additionally compensates measured IMU body pitch. Same geometry within each pair. No object pose feedback.',
        'limitations':'Prepared-frame fixtures follow the generated initial hand pose; initial world target heights/positions are not identical across modes. This is an attitude/contact diagnostic, not a same-world-target reach comparison. Fixed provisional palm/neck mass/COM assumption retained despite longer neck; no fabricated part specification. Nominal clearance screens do not certify arbitrary feedback/blocked-finger paths. Supported-placement acceptance analyzed separately from inherited drop cycle. Main A/B models remain unchanged.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def main():
    manifest = prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed leveling study protected')
    results,errors = [],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {pool.submit(run_one,e):e for e in manifest['cases']}
        for task in as_completed(tasks):
            try:row = task.result();results.append(row)
            except Exception as error:row = {'case':tasks[task]['case'],'error':str(error)};errors.append(row)
            print(json.dumps(row),flush=True)
    (STUDY/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors},indent=2)+'\n',encoding='utf-8')
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
