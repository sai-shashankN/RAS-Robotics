"""Hold control at 8 ms while using 2 ms physics; separate the cadence factors."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import json
import shutil
import subprocess
import sys
from run_contact_width_study import sha,replace_once
from run_profile_physics_checks import run_one

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/profile_physics_checks'
STUDY = MODEL/'experiments/fixed_cadence_checks'
read = lambda p:json.loads(p.read_text(encoding='utf-8'))


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete cadence check')
    for name,digest in read(PRIOR/'checkpoint-index.json')['files'].items():
        if sha(PRIOR/name)!=digest:raise ValueError('Prior diagnostic changed')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    path = common/'grasp_controller.py'
    path.write_text(replace_once(path.read_text(encoding='utf-8'),'step = int(robot.getBasicTimeStep())','step = 8  # fixed control/sensor period; physics may substep at 2 ms'),encoding='utf-8')
    rows = []
    for prior_case in ['a_reference_stops8','b_reference_stops8','a_y_pos4_stops8','b_16mm_step2']:
        previous = PRIOR/'projects'/prior_case;case = prior_case+'_ctrl8_phys2';project = STUDY/'projects'/case
        for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
        p = dict(read(previous/'parameters.json'),world_basic_time_step_ms=2)
        (project/'parameters.json').write_text(json.dumps(p,indent=2)+'\n',encoding='utf-8')
        shutil.copy2(previous/'scripts/generate_grasp_worlds.py',project/'scripts/generate_grasp_worlds.py')
        folder = project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config = folder/'experiment.json'
        shutil.copy2(previous/'experiments/physics/cases'/prior_case/'experiment.json',config)
        result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        rows.append({'case':case,'prior_case':prior_case,'project':str(project.relative_to(WORKSPACE)),
            'world':str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE)),
            'prior_report':str((previous/'results/physics'/prior_case/'report.json').relative_to(WORKSPACE))})
    manifest = {'date':'2026-10-24','cases':rows,'prior_checkpoint_sha256':sha(PRIOR/'checkpoint-index.json'),
        'change':'All runs have fixed 8 ms motor/control/sensor/observation cadence and 2 ms physics. For stopped references and A +Y4, this changes physics only from the previous stopped 8 ms cases. For B 16 mm, it changes control cadence only from the previous unstopped 2 ms case.',
        'limitations':'Four diagnostics; no open controls or complete neighborhood/empty regressions at this setting, no timestep convergence certificate, no adoption. Trace/sensor observations remain 8 ms, not every physics substep.'}
    (STUDY/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return manifest


def main():
    manifest = prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed cadence checks protected')
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
