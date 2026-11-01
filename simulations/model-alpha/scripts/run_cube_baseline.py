"""First cube grip/drop diagnostic with stable pedestal and corrected time/stop assumptions."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import importlib.util
import json
import shutil
import subprocess
import sys
from run_contact_width_study import sha,replace_once
from run_profile_physics_checks import run_one
from cube_geometry import self_check

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/fixed_cadence_checks'
STUDY = MODEL/'experiments/cube_baseline'
read = lambda p:json.loads(p.read_text(encoding='utf-8'))


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete cube preparation')
    self_check()
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Earlier cadence checkpoint changed')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    shutil.copy2(MODEL/'scripts/cube_geometry.py',common/'cube_geometry.py')
    f = common/'grasp_controller.py';source = f.read_text(encoding='utf-8')
    source = replace_once(source,'from control_math import finger_angles','from control_math import finger_angles\nfrom cube_geometry import corners, bottom, point_on_top, resting_on_top')
    source = replace_once(source,"'mass_properties.py', 'generate_worlds.py']]","'mass_properties.py', 'generate_worlds.py', 'cube_geometry.py']]")
    start = source.index('    # Static fixtures do not provide a symmetric contact list here.')
    end = source.index('    contacting = set()',start)
    source = source[:start]+'''    box_points = corners(ball_pos,ball.getOrientation(),E['object_size_m'])
    object_bottom = bottom(box_points)
    ball_speed = math.sqrt(sum(v*v for v in ball.getVelocity()[:3]))
    center_xy = SETUP['initial_object_position_m'][:2]
    stand_half = [v/2 for v in E['support_top_size_m']]
    on_stand = any(point_on_top(point,center_xy,stand_half,SETUP['stand_top_m']) for point in ball_points) or resting_on_top(box_points,center_xy,stand_half,SETUP['stand_top_m'],ball_speed)
    on_tray = any(point_on_top(point,center_xy,[.10,.08],SETUP['tray_top_m']) for point in ball_points) or resting_on_top(box_points,center_xy,[.10,.08],SETUP['tray_top_m'],ball_speed)
    supported = on_stand or on_tray
''' +source[end:]
    source = replace_once(source,"    gap = ball_pos[2]-E['object_radius_m']-SETUP['stand_top_m']","    gap = object_bottom-SETUP['stand_top_m']")
    start = source.index('        on_stand = stand_rest or any(')
    end = source.index('        tracefile.write(',start)
    source = source[:start]+source[end:]
    source = replace_once(source,"'object_palm_m':relative,'stand_gap_m':gap", "'object_palm_m':relative,'object_box_corners_m':box_points,'object_bottom_m':object_bottom,'stand_gap_m':gap")
    source = replace_once(source,"'MODEL ALPHA | CONTACT GRASP TEST'", "f'MODEL {P[\"model\"]} | CUBE GRIP TEST'")
    source = source.replace('| sphere:','| cube:').replace('Prepared pose. Free sphere;','Prepared pose. Free cube;')
    f.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    spec = importlib.util.spec_from_file_location('cube_clearance',common/'audit_clearance_profile.py')
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    import numpy as np
    generator = (PRIOR/'projects/a_reference_stops8_ctrl8_phys2/scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
    generator = replace_once(generator,"    radius = experiment['object_radius_m']", "    radius = experiment['object_size_m'][2]/2\n    object_geometry = 'Box { size '+' '.join(map(str,experiment['object_size_m']))+' }'\n    support_size = ' '.join(map(str,experiment['support_top_size_m']))")
    generator = replace_once(generator,"{g.shape(f'Cylinder {{ radius {experiment[\"stand_radius_m\"]} height {height} }}', '0.45 0.48 0.52')}","{g.box(f'{support_size} {height}', '0.45 0.48 0.52')}")
    generator = replace_once(generator,"boundingObject Cylinder {{ radius {experiment['stand_radius_m']} height {height} }}","boundingObject Box {{ size {support_size} {height} }}")
    generator = replace_once(generator,"{g.shape(f'Sphere {{ radius {radius} subdivision 3 }}', '1 0.65 0.02')}","{g.shape(object_geometry, '1 0.65 0.02')}")
    generator = replace_once(generator,'boundingObject Sphere {{ radius {radius} }}','boundingObject {object_geometry}')
    generator = generator.replace('name "test sphere"','name "test cube"')
    generator = replace_once(generator,"'control_math.py', 'mass_properties.py']:","'control_math.py', 'mass_properties.py', 'cube_geometry.py']:")
    rows = []
    for v in ['a','b']:
        previous = PRIOR/'projects'/f'{v}_reference_stops8_ctrl8_phys2';project = STUDY/'projects'/v
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        p = dict(read(previous/'parameters.json'),distal_profile_angle_deg=0)
        (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
        (project/'scripts/generate_grasp_worlds.py').write_text(generator,encoding='utf-8')
        base = read(previous/'experiments/physics/cases'/previous.name/'experiment.json')
        base = dict(base,object_shape='box',object_size_m=[.020,.020,.020],support_top_size_m=[.012,.012])
        for negative in [False,True]:
            case = f'{v}_cube'+('_open' if negative else '');folder = project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config = folder/'experiment.json'
            config.write_text(json.dumps(dict(base,open_claw_control=negative),indent=2)+'\n',encoding='utf-8')
            result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            setup = read(folder/'setup.json');pose = setup['start_pose'];theta = setup['start_body_pitch_rad'];rotation = audit.ry(theta)
            stand = ('pedestal','fixture',np.array([setup['initial_object_position_m'][0],setup['initial_object_position_m'][1],(setup['stand_top_m']+setup['tray_top_m'])/2]),np.eye(3),np.array([.012,.012,setup['stand_top_m']-setup['tray_top_m']])/2)
            cube = ('cube','object',np.array(setup['initial_object_position_m']),np.eye(3),np.array(base['object_size_m'])/2)
            fixture_gaps,object_gaps = [],[]
            for shape in audit.boxes(p,pose):
                world_shape = (shape[0],shape[1],np.array([0,0,p['wheel_radius']])+rotation@shape[2],rotation@shape[3],shape[4])
                fixture_gaps.append((audit.gap(world_shape,stand),shape[0]));object_gaps.append((audit.gap(world_shape,cube),shape[0]))
            screen = {'scope':'Initial opened pose only; conservative robot envelopes versus pedestal and initial box. Not a closure/contact or entire-workspace certificate.',
                'minimum_pedestal_gap_m':min(fixture_gaps)[0],'pedestal_closest':min(fixture_gaps)[1],
                'minimum_initial_object_gap_m':min(object_gaps)[0],'object_closest':min(object_gaps)[1]}
            screen['passed'] = min(fixture_gaps)[0]>0 and min(object_gaps)[0]>0
            (folder/'initial-clearance.json').write_text(json.dumps(screen,indent=2)+'\n',encoding='utf-8')
            if not screen['passed']:raise ValueError('Initial geometry needs review before dynamics: '+case+' '+json.dumps(screen))
            rows.append({'case':case,'variant':v,'open_control':negative,'project':str(project.relative_to(WORKSPACE)),
                'world':str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))})
        print(json.dumps({'prepared':v,'initial_geometry_passed':True}),flush=True)
    manifest = {'date':'2026-10-24','cases':rows,'prior_checkpoint_sha256':sha(PRIOR/'checkpoint-index.json'),
        'task':'Initial prepared cube grip/lift/hold/drop diagnostic, not supported-placement validation.',
        'design':'20 mm cube, nominal 2 g; actual Box collision/inertia. Stable 12x12 mm square pedestal under a 20x20 mm flat bottom. Flat distal contact faces, A/B fixed axes. Explicit arm/finger stops, 2 ms physics and 8 ms control/sensors.',
        'limitations':'New benchmark, not a one-factor sphere comparison. No final hardware material or payload specification. Initial support is stable; supported placement after closure-induced movement still needs a separate motion/criterion. Curling slopes and joint geometry remain. Main models unchanged.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def main():
    manifest = prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed cube evidence protected')
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
