"""Separate physical-stop and time-resolution diagnostics after robustness failures."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys
from run_contact_width_study import sha,replace_once

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/profile_robustness'
STUDY = MODEL/'experiments/profile_physics_checks'
read = lambda p:json.loads(p.read_text(encoding='utf-8'))


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete physics study')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Prior checkpoint changed')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    for p in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(p,common/p.name)
    f = common/'generate_worlds.py';source = f.read_text(encoding='utf-8')
    anchor = 'def finger(index, initial):'
    helper = '''def physical_stops(name):
    if not P.get('physical_joint_stops',False):return ''
    limits = {'finger':[-.65,.35],'distal':[0,1.1]}.get(name,P['joint_ranges'].get(name))
    return f'minStop {limits[0]} maxStop {limits[1]}'


'''
    source = replace_once(source,anchor,helper+anchor)
    source = replace_once(source,'axis 0 -1 0 anchor 0.018 0 0.009 position {q} dampingConstant',"axis 0 -1 0 anchor 0.018 0 0.009 position {q} {physical_stops('finger')} dampingConstant")
    source = replace_once(source,'axis 0 -1 0 anchor 0 0 {length} position {d} dampingConstant',"axis 0 -1 0 anchor 0 0 {length} position {d} {physical_stops('distal')} dampingConstant")
    source = replace_once(source,'axis 0 1 0 anchor {anchor} position {q} dampingConstant',"axis 0 1 0 anchor {anchor} position {q} {physical_stops(BASE)} dampingConstant")
    source = replace_once(source,'axis 0 1 0 anchor 0 0 {r+upper} position {e} dampingConstant',"axis 0 1 0 anchor 0 0 {r+upper} position {e} {physical_stops('elbow')} dampingConstant")
    source = replace_once(source,'axis 0 1 0 anchor 0 0 {forearm} position {w} dampingConstant',"axis 0 1 0 anchor 0 0 {forearm} position {w} {physical_stops('wrist')} dampingConstant")
    source = replace_once(source,'basicTimeStep 8 coordinateSystem',"basicTimeStep {P.get('world_basic_time_step_ms',8)} coordinateSystem")
    f.write_text(source,encoding='utf-8')
    specs = [('a_reference_stops8','a','reference',True,8),('b_reference_stops8','b','reference',True,8),
             ('a_y_pos4_stops8','a','y_pos4mm',True,8),('b_16mm_step2','b','diameter_16mm',False,2),
             ('a_open_stops8','a','open_control',True,8),('b_open_stops8','b','open_control',True,8)]
    rows = []
    for case,v,condition,stops,step in specs:
        project = STUDY/'projects'/case
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        p = dict(read(PRIOR/'projects'/v/'parameters.json'),physical_joint_stops=stops,world_basic_time_step_ms=step)
        (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
        shutil.copy2(PRIOR/'projects'/v/'scripts/generate_grasp_worlds.py',project/'scripts/generate_grasp_worlds.py')
        old_case = f'{v}_{condition}';old_experiment = PRIOR/'projects'/v/'experiments/robustness/cases'/old_case/'experiment.json'
        folder = project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config = folder/'experiment.json';shutil.copy2(old_experiment,config)
        result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        rows.append({'case':case,'variant':v,'condition':condition,'physical_stops':stops,'basic_time_step_ms':step,
            'open_control':condition=='open_control','project':str(project.relative_to(WORKSPACE)),
            'world':str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE)),
            'comparison_report':str((PRIOR/'projects'/v/'results/robustness'/old_case/'report.json').relative_to(WORKSPACE))})
    manifest = {'date':'2026-10-24','cases':rows,'prior_checkpoint_sha256':sha(PRIOR/'checkpoint-index.json'),
        'changes':'Stops-only at original 8 ms for A/B reference/open controls and A +Y4 failure. Separately, time-step-only 8 to 2 ms for B 16 mm; its stops remain disabled. No simultaneous geometry/gain/friction/torque/timing changes.',
        'scope':'Private diagnostics, not adoption, complete timestep convergence or new hardware limits. Mechanical stops match existing motor ranges on seven actuated arm/finger hinges; wheels remain continuous.',
        'documentation':'https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/jointparameters.md'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def run_one(entry):
    project = WORKSPACE/entry['project'];out = project/'results/physics'/entry['case'];out.mkdir(parents=True,exist_ok=True)
    if (out/'report.json').exists():raise FileExistsError('Existing diagnostic evidence')
    install = WORKSPACE/'.local-tools/webots';binary = install/'msys64/mingw64/bin/webots.exe';system = Path(os.environ['SystemRoot'])
    env = dict(os.environ,WEBOTS_HOME=str(install));env['PATH'] = ';'.join(map(str,[binary.parent,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO();startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow = subprocess.SW_HIDE
    with (out/'webots-stdout.log').open('w') as stdout,(out/'webots-stderr.log').open('w') as stderr:
        result = subprocess.run([str(binary),'--batch','--mode=fast','--stdout','--stderr',str(WORKSPACE/entry['world'])],cwd=project,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=stdout,stderr=stderr,timeout=240)
    r = read(out/'report.json')
    if r['case']!=entry['case'] or result.returncode!=(0 if r['validation_passed'] else 1):raise ValueError('Report/exit mismatch')
    for name,digest in r['source_sha256'].items():
        if sha(WORKSPACE/name)!=digest:raise ValueError('Stale source')
    if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(out/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
    return {'case':entry['case'],'passed':r['passed'],'validation_passed':r['validation_passed'],'exit_code':result.returncode,
        'failed_checks':[k for k,v in r['checks'].items() if not v],'report':str((out/'report.json').relative_to(WORKSPACE))}


def main():
    manifest = prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed physics diagnostic protected')
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
