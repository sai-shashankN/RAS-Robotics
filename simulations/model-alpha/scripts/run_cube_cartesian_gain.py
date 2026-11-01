"""One-factor wheel-position feedback comparison; preserve first path failures."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import json
import shutil
import subprocess
import sys
from run_contact_width_study import sha
from run_profile_physics_checks import run_one

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]
PRIOR=MODEL/'experiments/cube_cartesian'
STUDY=MODEL/'experiments/cube_cartesian_gain'
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
write=lambda p,v:p.write_text(json.dumps(v,indent=2)+'\n',encoding='utf-8')


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete gain study')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('First path evidence changed')
    common=STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    rows=[]
    for entry in read(PRIOR/'manifest.json')['cases']:
        prior=WORKSPACE/entry['project'];project=STUDY/'projects'/prior.name
        if not project.exists():
            for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
            for name in ['parameters.json','cartesian-plan.json','robot-clearance.json']:shutil.copy2(prior/name,project/name)
            shutil.copy2(prior/'scripts/generate_grasp_worlds.py',project/'scripts/generate_grasp_worlds.py')
        case=entry['case'];folder=project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config=folder/'experiment.json'
        e=read(prior/'experiments/physics/cases'/case/'experiment.json');e['grasp_balance_gains']['position']=.64;write(config,e)
        for name in ['initial-clearance.json','empty-sweep-screen.json']:shutil.copy2(prior/'experiments/physics/cases'/case/name,folder/name)
        result=subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        rows.append(dict(entry,project=str(project.relative_to(WORKSPACE)),world=str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))))
    m=dict(read(PRIOR/'manifest.json'),cases=rows,prior_checkpoint_sha256=sha(PRIOR/'checkpoint-index.json'),prior_study='cube_cartesian',
        revision='Only wheel position-error feedback increases from 0.16 to 0.64 Nm/m in the same balance-controller equation; velocity/pitch gains, geometry, nominal paths, gates, motor speed/torque limits, 2 ms physics and 8 ms control remain unchanged. Prior path failures retained.',
        question='Does stronger odometry position feedback reduce base/path drift and improve the unchanged cube setdown?')
    write(STUDY/'manifest.json',m);return m


def main():
    manifest=prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed gain trials protected')
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
