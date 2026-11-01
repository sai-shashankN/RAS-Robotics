"""Matched prepared-sphere contact diagnostics, not precise-placement validation."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
import json
import shutil
import subprocess
import sys

import run_grasp_sweep as runner

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = 'axis_contact_probe'


def main():
    b_model = MODEL.parent/'model-alpha-b'
    status = json.loads((b_model/'results/empty-validation-status.json').read_text())
    if status['errors'] or len(status['cases'])!=14 or not all(row['passed'] for row in status['cases']):
        raise ValueError('Finish Alpha-B empty validation before contact probing')
    if (MODEL/'results'/STUDY/'run-status.json').exists():
        raise FileExistsError('Preserve previous contact study before rerunning')
    setup = json.loads((b_model/'comparison-setup.json').read_text())
    for key,path in [('baseline_sha256',MODEL/'parameters.json'),('variant_sha256',b_model/'parameters.json')]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=setup[key]:
            raise ValueError('A/B baseline changed')
    base = json.loads((MODEL/'grasp_parameters.json').read_text())
    entries = []
    for variant,model in [('a',MODEL),('b',b_model)]:
        if model!=MODEL:
            shutil.copyfile(MODEL/'scripts/generate_grasp_worlds.py',model/'scripts/generate_grasp_worlds.py')
        for name,negative in [('reference',False),('open_control',True)]:
            case = variant+'_'+name
            folder = model/'experiments'/STUDY/'cases'/case
            folder.mkdir(parents=True,exist_ok=True)
            config = folder/'experiment.json'
            experiment = dict(base,status='Initial matched two-finger sphere diagnostic; no precise-placement claim',
                              trace_contacts=True,open_claw_control=negative)
            config.write_text(json.dumps(experiment,indent=2)+'\n',encoding='utf-8')
            subprocess.run([sys.executable,str(model/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),
                            '--case',case,'--study',STUDY],check=True,capture_output=True,text=True)
            entries.append({'case':case,'variant':variant,'model_path':str(model.relative_to(WORKSPACE)),
                            'experiment_path':str(config.relative_to(WORKSPACE)),
                            'world_path':str((model/'worlds'/f'alpha_{STUDY}_{case}_validation.wbt').relative_to(WORKSPACE))})
    # The runner's output root can aggregate A/B without changing controller ROOT.
    # Each controller writes to its own model folder; use per-call runner globals
    # only through sequential variant batches so there is no cross-thread mutation.
    results,errors = [],[]
    for variant,model in [('a',MODEL),('b',b_model)]:
        runner.MODEL = model
        runner.RESULTS = model/'results'/STUDY
        with ThreadPoolExecutor(max_workers=2) as pool:
            tasks = {pool.submit(runner.run_one,entry):entry for entry in entries if entry['variant']==variant}
            for task in as_completed(tasks):
                try:
                    row = task.result();results.append(row);print(json.dumps(row),flush=True)
                except Exception as error:
                    errors.append({'case':tasks[task]['case'],'error':str(error)})
                    print(str(error),flush=True)
    output = MODEL/'results'/STUDY;output.mkdir(parents=True,exist_ok=True)
    (output/'manifest.json').write_text(json.dumps({'study':STUDY,'cases':entries,
        'comparison':'Same 2 g/20 mm sphere, prepared frame, friction, timing, known-load compensation and targets. Only finger orientation and labels differ; no object pose writes.',
        'limitation':'A sphere is an initial seating diagnostic; directional objects and precise supported placement remain untested.'},indent=2)+'\n',encoding='utf-8')
    (output/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors},indent=2)+'\n',encoding='utf-8')
    if errors:
        raise SystemExit(2)


if __name__=='__main__':
    main()
