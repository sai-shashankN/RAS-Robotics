"""Matched one-factor neighborhood of the isolated 10-degree V candidate."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import sys
from run_contact_width_study import sha

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PREVIOUS = MODEL/'experiments/contact_profile_study'
STUDY = MODEL/'experiments/profile_robustness'


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def conditions():
    rows = [('reference',{}),('reference_repeat',{})]
    for axis,index in [('x',0),('y',1)]:
        for mm in [-4,-2,2,4]:
            offset = [0,0,0];offset[index] = mm/1000
            rows.append((f'{axis}_{"neg" if mm<0 else "pos"}{abs(mm)}mm',{'placement_offset_world_m':offset}))
    rows += [('diameter_16mm',{'object_radius_m':.008}),('diameter_24mm',{'object_radius_m':.012}),
             ('friction_04',{'finger_object_friction':.4}),('friction_02',{'finger_object_friction':.2}),
             ('open_control',{'open_claw_control':True})]
    return rows


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve/review incomplete study before preparing again')
    # Verify the complete earlier checkpoint before reusing any candidate input.
    index = read(PREVIOUS/'checkpoint-index.json')['files']
    for name,digest in index.items():
        if sha(PREVIOUS/name)!=digest:raise ValueError('Previous checkpoint changed: '+name)
    common = STUDY/'projects/common';common.mkdir(parents=True)
    inherited = {}
    for path in (PREVIOUS/'projects/common').glob('*.py'):
        shutil.copy2(path,common/path.name)
        inherited[str(path.relative_to(WORKSPACE))] = sha(path)
    baseline = read(MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json')
    rows = []
    for variant in ['a','b']:
        prior = PREVIOUS/'projects'/f'{variant}_10deg'
        project = STUDY/'projects'/variant
        for folder in ['scripts','worlds','controllers','results']:(project/folder).mkdir(parents=True)
        for name in ['parameters.json','scripts/generate_grasp_worlds.py']:
            shutil.copy2(prior/name,project/name)
            inherited[str((prior/name).relative_to(WORKSPACE))] = sha(prior/name)
        screen = read(prior/'robot-clearance.json')
        for name,digest in screen['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Prior geometry screen stale')
        if not screen['all_passed']:raise ValueError('Candidate not clear on robot paths')
        inheritance = {'prior_screen':str((prior/'robot-clearance.json').relative_to(WORKSPACE)),
            'prior_screen_sha256':sha(prior/'robot-clearance.json'),'all_passed':True,
            'reason':'Identical robot geometry, parameters, private common helpers and commanded paths. Scene offsets/size/friction do not alter the robot-only screen.',
            'limitations':'Direct joint neighbors excluded; fixture/object contacts require each dynamic trial. Prepared controller monitors arm contacts, not root body/wheels.'}
        (project/'clearance-inheritance.json').write_text(json.dumps(inheritance,indent=2)+'\n',encoding='utf-8')
        for condition,changes in conditions():
            case = f'{variant}_{condition}';folder = project/'experiments/robustness/cases'/case;folder.mkdir(parents=True)
            config = folder/'experiment.json'
            config.write_text(json.dumps(dict(baseline,**changes),indent=2)+'\n',encoding='utf-8')
            result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','robustness'],capture_output=True,text=True)
            if result.returncode:raise RuntimeError(result.stdout+result.stderr)
            rows.append({'case':case,'condition':condition,'variant':variant,'changes':changes,
                'open_control':changes.get('open_claw_control',False),'project':str(project.relative_to(WORKSPACE)),
                'world':str((project/'worlds'/f'alpha_robustness_{case}_validation.wbt').relative_to(WORKSPACE))})
        print(json.dumps({'prepared':variant,'cases':len(conditions()),'candidate_inputs_identical':True}),flush=True)
    original = read(PREVIOUS/'manifest.json')['baseline_source_sha256']
    for name,digest in original.items():
        if sha(WORKSPACE/name)!=digest:raise ValueError('Original A/B source changed')
    manifest = {'date':'2026-10-24','cases':rows,'inherited_source_sha256':inherited,'baseline_source_sha256':original,
        'candidate':'Original 10 mm distal width with 10-degree tangential V; private experiment, not adopted.',
        'design':'Paired A/B one-factor trials: world X/Y offsets +/-2,+/-4 mm; sphere diameter 16/20/24 mm; friction 0.6/0.4/0.2; reference repeat and open control per axis.',
        'held_fixed':'Robot geometry, explicit masses/COM, control gains, motor limits, trajectories, schedule, 2 g object mass and supplied nominal target/load. Only listed experiment keys differ.',
        'limitations':'No factor combinations, statistical success rates, perception, adaptive recovery or precise-placement claim. Object mass stays 2 g across diameters, so density changes. Position errors are in initial world X/Y; the pole/tray follow the object to preserve initial support. Size tests keep the sphere center fixed and adjust pole height. Nominal load feedback remains unchanged, not live object-pose correction. Robot-only clearance is inherited; dynamic arm contacts are monitored per run.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def run_one(entry):
    project = WORKSPACE/entry['project'];out = project/'results/robustness'/entry['case'];out.mkdir(parents=True,exist_ok=True)
    if (out/'report.json').exists():raise FileExistsError('Case already has evidence')
    install = WORKSPACE/'.local-tools/webots';binary = install/'msys64/mingw64/bin/webots.exe';system = Path(os.environ['SystemRoot'])
    env = dict(os.environ,WEBOTS_HOME=str(install));env['PATH'] = ';'.join(map(str,[binary.parent,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO();startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow = subprocess.SW_HIDE
    with (out/'webots-stdout.log').open('w') as stdout,(out/'webots-stderr.log').open('w') as stderr:
        result = subprocess.run([str(binary),'--batch','--mode=fast','--stdout','--stderr',str(WORKSPACE/entry['world'])],cwd=project,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=stdout,stderr=stderr,timeout=240)
    r = read(out/'report.json')
    if r['case']!=entry['case'] or result.returncode!=(0 if r['validation_passed'] else 1):raise ValueError('Report/exit mismatch')
    for name,digest in r['source_sha256'].items():
        if sha(WORKSPACE/name)!=digest:raise ValueError('Stale source: '+name)
    if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(out/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
    return {'case':entry['case'],'passed':r['passed'],'validation_passed':r['validation_passed'],'exit_code':result.returncode,
        'failed_checks':[name for name,value in r['checks'].items() if not value],
        'report':str((out/'report.json').relative_to(WORKSPACE))}


def main():
    parser = argparse.ArgumentParser();parser.add_argument('--prepare-only',action='store_true');args = parser.parse_args()
    manifest = prepare()
    if args.prepare_only:return
    if (STUDY/'run-status.json').exists():raise FileExistsError('Study already complete; preserve it')
    results,errors = [],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        # Alternate orientations; every run has an independent world/experiment/output folder.
        order = sorted(manifest['cases'],key=lambda e:(list(name for name,_ in conditions()).index(e['condition']),e['variant']))
        tasks = {pool.submit(run_one,e):e for e in order}
        for task in as_completed(tasks):
            try:row = task.result();results.append(row)
            except Exception as error:row = {'case':tasks[task]['case'],'error':str(error)};errors.append(row)
            print(json.dumps(row),flush=True)
    for group in ['baseline_source_sha256','inherited_source_sha256']:
        for name,digest in manifest[group].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Protected source changed: '+name)
    (STUDY/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors},indent=2)+'\n',encoding='utf-8')
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
