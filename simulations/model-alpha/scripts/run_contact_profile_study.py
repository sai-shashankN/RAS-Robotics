"""Isolated shallow V contact-profile comparison; preserves original A/B."""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from run_contact_width_study import sha, replace_once

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/contact_profile_study'

PROFILE_HELPER = '''"""Two convex half-prisms form a concave V; no convex hull across the cup."""
import math

def section(angle_deg):
    slope = math.tan(math.radians(angle_deg))
    return [[(-.0065-abs(y)*slope,y),(.0065,y),(.0065,z),(-.0065-abs(z)*slope,z)]
            for y,z in [(-.005,0),(0,.005)]]

def meshes(angle_deg,length):
    result = []
    for polygon in section(angle_deg):
        points = [(x,y,z) for z in [0,length] for x,y in polygon]
        faces = [(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        coords = ', '.join(' '.join(map(str,p)) for p in points)
        indices = ' '.join(' '.join(map(str,f))+' -1' for f in faces)
        result.append(f'IndexedFaceSet {{ coord Coordinate {{ point [ {coords} ] }} coordIndex [ {indices} ] }}')
    return result

def envelope(angle_deg):
    # Also encloses the baseline visual pad for a conservative common screen.
    lo = min(-.00725,-.0065-.005*math.tan(math.radians(angle_deg)))
    hi = .00725
    return (lo+hi)/2,hi-lo
'''


def prepare():
    if (STUDY/'manifest.json').exists():
        return json.loads((STUDY/'manifest.json').read_text(encoding='utf-8'))
    if STUDY.exists():
        raise FileExistsError('Preserve/review incomplete study before preparing again')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    hashes = {}
    for path in (MODEL.parent/'common').glob('*.py'):
        shutil.copy2(path,common/path.name)
        hashes[str(path.relative_to(WORKSPACE))] = sha(path)
    (common/'contact_profile.py').write_text(PROFILE_HELPER,encoding='utf-8')
    f = common/'grasp_controller.py'
    f.write_text(replace_once(f.read_text(encoding='utf-8'),'WORKSPACE = ROOT.parents[1]',
        "WORKSPACE = next(parent for parent in ROOT.parents if (parent/'.local-tools').is_dir() and (parent/'simulations').is_dir())"),encoding='utf-8')
    f = common/'generate_worlds.py';source = f.read_text(encoding='utf-8')
    source = replace_once(source,'from control_math import finger_angles','from control_math import finger_angles\nfrom contact_profile import meshes')
    source = replace_once(source,'    phi = finger_angles(P)[index]',
        '''    angle = P.get('distal_profile_angle_deg', 0)
    baseline_collision = f'Pose {{ translation 0 0 {distal_length/2} children [ Box {{ size 0.013 0.010 {distal_length} }} ] }}'
    baseline_pad = box(f'0.0025 0.009 {distal_length*0.9}', BLUE, f'-0.006 0 {distal_length/2}')
    pieces = meshes(angle,distal_length) if angle else []
    distal_collision = 'Group { children [ '+' '.join(pieces)+' ] }' if angle else baseline_collision
    distal_pad = ''.join(shape(piece,BLUE) for piece in pieces) if angle else baseline_pad
    phi = finger_angles(P)[index]''')
    source = replace_once(source,"{box(f'0.0025 0.009 {distal_length*0.9}', BLUE, f'-0.006 0 {distal_length/2}')}",'{distal_pad}')
    source = replace_once(source,'boundingObject Pose {{ translation 0 0 {distal_length/2} children [ Box {{ size 0.013 0.010 {distal_length} }} ] }}','boundingObject {distal_collision}')
    f.write_text(source,encoding='utf-8')
    source = (MODEL/'scripts/audit_clearance.py').read_text(encoding='utf-8')
    source = replace_once(source,'MODEL = Path(__file__).resolve().parents[1]\nWORKSPACE = MODEL.parents[1]',
        "WORKSPACE = next(parent for parent in Path(__file__).resolve().parents if (parent/'.local-tools').is_dir() and (parent/'simulations').is_dir())\nMODEL = WORKSPACE/'simulations/model-alpha'")
    source = replace_once(source,"sys.path.insert(0,str(WORKSPACE/'simulations/common'))",'sys.path.insert(0,str(Path(__file__).resolve().parent))')
    source = replace_once(source,'from control_math import finger_angles','from control_math import finger_angles\nfrom contact_profile import envelope')
    source = replace_once(source,"        add(f'tip {i}', f'tip{i}', tip_base+distal_r @ np.array([0, 0, p['finger_distal_length']/2]), distal_r, [.0145, .010, p['finger_distal_length']])",
        "        offset, radial_size = envelope(p.get('distal_profile_angle_deg',0))\n        add(f'tip {i}', f'tip{i}', tip_base+distal_r @ np.array([offset, 0, p['finger_distal_length']/2]), distal_r, [radial_size, .010, p['finger_distal_length']])")
    audit_path = common/'audit_clearance_profile.py';audit_path.write_text(source,encoding='utf-8')
    sys.path.insert(0,str(common))
    spec = importlib.util.spec_from_file_location('profile_geometry',audit_path)
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    from contact_profile import section,envelope
    # Mesh vertices must stay within the audited envelope; each half is convex.
    for angle in [0,10,15]:
        offset,size = envelope(angle)
        for polygon in section(angle):
            assert all(offset-size/2-1e-12<=x<=offset+size/2+1e-12 and abs(y)<=.005+1e-12 for x,y in polygon)
            cross = [(polygon[(i+1)%4][0]-polygon[i][0])*(polygon[(i+2)%4][1]-polygon[(i+1)%4][1])-(polygon[(i+1)%4][1]-polygon[i][1])*(polygon[(i+2)%4][0]-polygon[(i+1)%4][0]) for i in range(4)]
            assert all(c>0 for c in cross)
    experiment = json.loads((MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json').read_text())
    generator = (MODEL/'scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
    generator = replace_once(generator,"text = 'Webots Project File version R2025a\\nrenderingDevicePerspectives: Model Alpha:front_camera;0;1;0;0\\n'",
        "text = f'Webots Project File version R2025a\\nrenderingDevicePerspectives: Model {p[\"model\"]}:front_camera;0;1;0;0\\n'")
    rows = []
    for variant,baseline in [('a',MODEL),('b',MODEL.parent/'model-alpha-b')]:
        original = json.loads((baseline/'parameters.json').read_text())
        hashes[str((baseline/'parameters.json').relative_to(WORKSPACE))] = sha(baseline/'parameters.json')
        for angle in [0,10,15]:
            project = STUDY/'projects'/f'{variant}_{angle}deg'
            for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
            p = dict(original,distal_profile_angle_deg=angle)
            (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
            (project/'scripts/generate_grasp_worlds.py').write_text(generator,encoding='utf-8')
            raised = p['dynamic_arm_targets']['arm_reach']
            opened,closed = [dict(raised,**p['finger_targets'][name]) for name in ['claw_open','claw_closed']]
            paths = {'close':audit.audit_path(p,opened,closed),'lift_lower':audit.audit_path(p,closed,experiment['lift_pose']),
                'open_control_lift':audit.audit_path(p,opened,dict(experiment['lift_pose'],**p['finger_targets']['claw_open']))}
            screen = {'paths':paths,'all_passed':all(r['conservative_nonadjacent_clearance_passed'] for r in paths.values()),
                'method':'Conservative envelope includes both convex V half-prisms and original visual pad envelope; between-sample motion allowance.',
                'limitations':'Specified robot-only paths; direct joint neighbors excluded. Fixture/object contacts checked dynamically.',
                'source_sha256':{str(path.relative_to(WORKSPACE)):sha(path) for path in [Path(__file__),audit_path,common/'contact_profile.py',common/'control_math.py',common/'generate_worlds.py',project/'parameters.json']}}
            (project/'robot-clearance.json').write_text(json.dumps(screen,indent=2)+'\n',encoding='utf-8')
            for negative in ([False,True] if angle==15 else [False]):
                case = f'{variant}_{angle}deg'+('_open' if negative else '')
                folder = project/'experiments/profile/cases'/case;folder.mkdir(parents=True)
                config = folder/'experiment.json'
                config.write_text(json.dumps(dict(experiment,open_claw_control=negative),indent=2)+'\n',encoding='utf-8')
                result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','profile'],capture_output=True,text=True)
                if result.returncode:raise RuntimeError(result.stdout+result.stderr)
                rows.append({'case':case,'variant':variant,'profile_angle_deg':angle,'open_control':negative,
                    'project':str(project.relative_to(WORKSPACE)),
                    'world':str((project/'worlds'/f'alpha_profile_{case}_validation.wbt').relative_to(WORKSPACE)),
                    'admitted_by_robot_clearance':screen['all_passed']})
            print(json.dumps({'prepared':project.name,'clear':screen['all_passed'],'minimum_bound_mm':1000*min(r['conservative_clearance_bound_m'] for r in paths.values())}),flush=True)
    manifest = {'date':'2026-10-24','cases':rows,'baseline_source_sha256':hashes,
        'factor':'Distal inner face V angle across tangential width: flat/10/15 degrees per side, unchanged 10 mm width. Center face x=-6.5 mm; inward edge protrusion 0/0.882/1.340 mm.',
        'held_fixed':'All model and experiment inputs except distal profile angle: proximal geometry, distal length/width, palm, mass/COM, limits, gains, closing motion, timing, friction and sphere.',
        'caveat':'Two convex half-prisms retain the concave V in collision geometry. Automatic inertia changes at fixed 8 g segment mass/COM; not a manufactured part. Contact forces are outcomes. Prepared scene monitors arm contacts, not whole robot.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def run_one(entry):
    project = WORKSPACE/entry['project'];out = project/'results/profile'/entry['case'];out.mkdir(parents=True,exist_ok=True)
    if (out/'report.json').exists():raise FileExistsError('Case already has evidence: '+entry['case'])
    install = WORKSPACE/'.local-tools/webots';binary = install/'msys64/mingw64/bin/webots.exe';system = Path(os.environ['SystemRoot'])
    env = dict(os.environ,WEBOTS_HOME=str(install));env['PATH'] = ';'.join(map(str,[binary.parent,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO();startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow = subprocess.SW_HIDE
    with (out/'webots-stdout.log').open('w') as stdout,(out/'webots-stderr.log').open('w') as stderr:
        process = subprocess.run([str(binary),'--batch','--mode=fast','--stdout','--stderr',str(WORKSPACE/entry['world'])],cwd=project,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=stdout,stderr=stderr,timeout=240)
    r = json.loads((out/'report.json').read_text())
    if r['case']!=entry['case'] or process.returncode!=(0 if r['validation_passed'] else 1):raise ValueError('Report/exit mismatch')
    for name,value in r['source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Stale input: '+name)
    if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(out/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace hash mismatch')
    return {'case':entry['case'],'passed':r['passed'],'validation_passed':r['validation_passed'],'exit_code':process.returncode,
        'failed_checks':[k for k,v in r['checks'].items() if not v],'report':str((out/'report.json').relative_to(WORKSPACE))}


def main():
    parser = argparse.ArgumentParser();parser.add_argument('--prepare-only',action='store_true');args = parser.parse_args()
    manifest = prepare()
    if args.prepare_only:return
    if (STUDY/'run-status.json').exists():raise FileExistsError('Study already complete; preserve before a new run')
    results,errors,skips = [],[],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {}
        for entry in manifest['cases']:
            if not entry['admitted_by_robot_clearance']:skips.append({'case':entry['case'],'reason':'Robot-envelope clearance failed'});continue
            tasks[pool.submit(run_one,entry)] = entry
        for task in as_completed(tasks):
            try:row = task.result();results.append(row)
            except Exception as error:row = {'case':tasks[task]['case'],'error':str(error)};errors.append(row)
            print(json.dumps(row),flush=True)
    for name,value in manifest['baseline_source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Authoritative baseline changed')
    (STUDY/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors,'clearance_skips':skips},indent=2)+'\n',encoding='utf-8')
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
