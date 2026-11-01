"""Isolated distal-face width experiment; preserves both authoritative models.

Only distal tangential width changes physically. Mass and motor/friction inputs
stay fixed; automatic collision-geometry inertia changes with width. This is a
geometry/contact diagnostic, not a fabricated constant-mass part specification.
"""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/contact_width_study'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(source,old,new):
    if source.count(old)!=1:
        raise ValueError('Fork anchor changed: '+old[:80])
    return source.replace(old,new,1)


def prepare():
    if (STUDY/'manifest.json').exists():
        return json.loads((STUDY/'manifest.json').read_text())
    if STUDY.exists():
        raise FileExistsError('Incomplete study exists; preserve/review before preparing again')
    common = STUDY/'projects/common'
    common.mkdir(parents=True)
    baseline_hashes = {}
    for path in (MODEL.parent/'common').glob('*.py'):
        shutil.copy2(path,common/path.name)
        baseline_hashes[str(path.relative_to(WORKSPACE))] = sha(path)
    # A private logging path adaptation; controller behavior is otherwise intact.
    f = common/'grasp_controller.py'
    source = replace_once(f.read_text(encoding='utf-8'),'WORKSPACE = ROOT.parents[1]',
        "WORKSPACE = next(parent for parent in ROOT.parents if (parent/'.local-tools').is_dir() and (parent/'simulations').is_dir())")
    f.write_text(source,encoding='utf-8')
    # Match the baseline at 10 mm. Keep radial thickness, lengths and proximal
    # geometry; widen distal visual housing/pad and its collision face together.
    f = common/'generate_worlds.py';source = f.read_text(encoding='utf-8')
    source = replace_once(source,'    phi = finger_angles(P)[index]',
        "    face_width = P.get('distal_contact_width_m', .010)\n    visible_width = face_width-.001\n    phi = finger_angles(P)[index]")
    source = replace_once(source,"{box(f'0.010 0.009 {distal_length}', GREY, f'0 0 {distal_length/2}')}",
        "{box(f'0.010 {visible_width} {distal_length}', GREY, f'0 0 {distal_length/2}')}")
    source = replace_once(source,"{box(f'0.0025 0.009 {distal_length*0.9}', BLUE, f'-0.006 0 {distal_length/2}')}",
        "{box(f'0.0025 {visible_width} {distal_length*0.9}', BLUE, f'-0.006 0 {distal_length/2}')}")
    source = replace_once(source,'Box {{ size 0.013 0.010 {distal_length} }}',
        'Box {{ size 0.013 {face_width} {distal_length} }}')
    f.write_text(source,encoding='utf-8')
    audit_source = (MODEL/'scripts/audit_clearance.py').read_text(encoding='utf-8')
    audit_source = replace_once(audit_source,'MODEL = Path(__file__).resolve().parents[1]\nWORKSPACE = MODEL.parents[1]',
        "WORKSPACE = next(parent for parent in Path(__file__).resolve().parents if (parent/'.local-tools').is_dir() and (parent/'simulations').is_dir())\nMODEL = WORKSPACE/'simulations/model-alpha'")
    audit_source = replace_once(audit_source,"sys.path.insert(0,str(WORKSPACE/'simulations/common'))",
        'sys.path.insert(0,str(Path(__file__).resolve().parent))')
    audit_source = replace_once(audit_source,"distal_r, [.0145, .010, p['finger_distal_length']])",
        "distal_r, [.0145, p.get('distal_contact_width_m', .010), p['finger_distal_length']])")
    audit_path = common/'audit_clearance_width.py';audit_path.write_text(audit_source,encoding='utf-8')
    spec = importlib.util.spec_from_file_location('width_geometry',audit_path)
    audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
    base_experiment = json.loads((MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json').read_text())
    generator_source = (MODEL/'scripts/generate_grasp_worlds.py').read_text(encoding='utf-8')
    generator_source = replace_once(generator_source,"text = 'Webots Project File version R2025a\\nrenderingDevicePerspectives: Model Alpha:front_camera;0;1;0;0\\n'",
        "text = f'Webots Project File version R2025a\\nrenderingDevicePerspectives: Model {p[\"model\"]}:front_camera;0;1;0;0\\n'")
    rows = []
    for variant,baseline in [('a',MODEL),('b',MODEL.parent/'model-alpha-b')]:
        original = json.loads((baseline/'parameters.json').read_text())
        baseline_hashes[str((baseline/'parameters.json').relative_to(WORKSPACE))] = sha(baseline/'parameters.json')
        for mm in [10,16,24]:
            project = STUDY/'projects'/f'{variant}_{mm}mm'
            (project/'scripts').mkdir(parents=True)
            for name in ['worlds','controllers','results']:
                (project/name).mkdir()
            p = dict(original,distal_contact_width_m=mm/1000)
            (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
            (project/'scripts/generate_grasp_worlds.py').write_text(generator_source,encoding='utf-8')
            # Same contact trajectories as the original prepared experiment.
            raised = p['dynamic_arm_targets']['arm_reach']
            opened = dict(raised,**p['finger_targets']['claw_open'])
            closed = dict(raised,**p['finger_targets']['claw_closed'])
            lifted = base_experiment['lift_pose']
            paths = {'close':audit.audit_path(p,opened,closed),
                     'lift_lower':audit.audit_path(p,closed,lifted),
                     'open_control_lift':audit.audit_path(p,opened,dict(lifted,**p['finger_targets']['claw_open']))}
            audit_report = {'paths':paths,'all_passed':all(row['conservative_nonadjacent_clearance_passed'] for row in paths.values()),
                'method':'Conservative robot envelopes with between-sample allowance; distal width included.',
                'limitations':'Specified robot-only contact paths. Direct joint neighbors excluded; fixture contacts checked in dynamics.',
                'source_sha256':{str(path.relative_to(WORKSPACE)):sha(path) for path in [Path(__file__),audit_path,project/'parameters.json',common/'control_math.py',common/'generate_worlds.py']}}
            (project/'robot-clearance.json').write_text(json.dumps(audit_report,indent=2)+'\n',encoding='utf-8')
            for negative in ([False,True] if mm==24 else [False]):
                case = f'{variant}_{mm}mm'+('_open' if negative else '')
                folder = project/'experiments/width/cases'/case;folder.mkdir(parents=True)
                experiment = dict(base_experiment,open_claw_control=negative)
                config = folder/'experiment.json'
                config.write_text(json.dumps(experiment,indent=2)+'\n',encoding='utf-8')
                generated = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),
                    '--case',case,'--study','width'],capture_output=True,text=True)
                if generated.returncode:
                    raise RuntimeError(generated.stdout+generated.stderr)
                rows.append({'case':case,'variant':variant,'width_mm':mm,'open_control':negative,
                    'project':str(project.relative_to(WORKSPACE)),
                    'world':str((project/'worlds'/f'alpha_width_{case}_validation.wbt').relative_to(WORKSPACE)),
                    'admitted_by_robot_clearance':audit_report['all_passed']})
            print(json.dumps({'prepared':f'{variant}_{mm}mm','self_clear':audit_report['all_passed'],
                'minimum_bound_mm':1000*min(row['conservative_clearance_bound_m'] for row in paths.values())}),flush=True)
    manifest = {'date':'2026-10-24','cases':rows,'baseline_source_sha256':baseline_hashes,
        'factor':'Distal contact face tangential width: 10/16/24 mm; corresponding visual housing/pad width is face width minus 1 mm.',
        'held_fixed':'Proximal geometry, radial thickness, link lengths, palm, segment masses/COM, motor torque/speed limits, gains, timing, friction, sphere and prepared frame.',
        'caveat':'Constant segment mass is an experiment assumption; derived inertia changes with width. Actual contact forces are outcomes, not fixed force commands. No hardware part or material design.',
        'fork':'Private copied common sources with width input and logging-root adaptation; authoritative A/B files and evidence unchanged.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def run_one(entry):
    project = WORKSPACE/entry['project']
    out = project/'results/width'/entry['case'];out.mkdir(parents=True,exist_ok=True)
    if (out/'report.json').exists():
        raise FileExistsError('Case already has evidence: '+entry['case'])
    binary = WORKSPACE/'.local-tools/webots/msys64/mingw64/bin/webots.exe'
    install = WORKSPACE/'.local-tools/webots'
    system = Path(os.environ['SystemRoot'])
    env = dict(os.environ,WEBOTS_HOME=str(install))
    env['PATH'] = ';'.join(map(str,[binary.parent,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO();startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow = subprocess.SW_HIDE
    with (out/'webots-stdout.log').open('w') as stdout,(out/'webots-stderr.log').open('w') as stderr:
        process = subprocess.run([str(binary),'--batch','--mode=fast','--stdout','--stderr',str(WORKSPACE/entry['world'])],
            cwd=project,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=stdout,stderr=stderr,timeout=240)
    report = json.loads((out/'report.json').read_text())
    if report['case']!=entry['case'] or process.returncode!=(0 if report['validation_passed'] else 1):
        raise ValueError('Report/exit mismatch: '+entry['case'])
    for name,value in report['source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Stale source: '+name)
    if sha(WORKSPACE/entry['world'])!=report['world_sha256'] or sha(out/'contact-trace.jsonl')!=report['contact_trace_sha256']:
        raise ValueError('World/trace hash mismatch')
    return {'case':entry['case'],'passed':report['passed'],'validation_passed':report['validation_passed'],
        'exit_code':process.returncode,'failed_checks':[k for k,v in report['checks'].items() if not v],
        'report':str((out/'report.json').relative_to(WORKSPACE))}


def main():
    parser = argparse.ArgumentParser();parser.add_argument('--prepare-only',action='store_true');args = parser.parse_args()
    manifest = prepare()
    if args.prepare_only:return
    if (STUDY/'run-status.json').exists():raise FileExistsError('Study already complete; preserve before a new run')
    results,errors,skips = [],[],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {}
        for entry in manifest['cases']:
            if not entry['admitted_by_robot_clearance']:
                skips.append({'case':entry['case'],'reason':'Robot-envelope clearance failed'});continue
            tasks[pool.submit(run_one,entry)] = entry
        for task in as_completed(tasks):
            try:
                row = task.result();results.append(row);print(json.dumps(row),flush=True)
            except Exception as error:
                row = {'case':tasks[task]['case'],'error':str(error)};errors.append(row);print(json.dumps(row),flush=True)
    for name,value in manifest['baseline_source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Authoritative baseline changed: '+name)
    (STUDY/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors,'clearance_skips':skips},indent=2)+'\n',encoding='utf-8')
    if errors:raise SystemExit(2)


if __name__=='__main__':
    main()
