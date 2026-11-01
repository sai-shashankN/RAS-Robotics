"""Run geometry and 13 empty Alpha dynamics cases after mechanism changes."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def run_one(case):
    mode,scenario = case
    suffix = '_'+scenario if scenario!='balance' else ''
    parameters = json.loads((MODEL/'parameters.json').read_text())
    world = MODEL/'worlds'/f'{parameters["model"].lower()}_{mode}{suffix}_validation.wbt'
    folder = MODEL/'results'/mode
    if scenario!='balance':
        folder /= scenario
    folder.mkdir(parents=True,exist_ok=True)
    report_path = folder/'report.json'
    report_path.unlink(missing_ok=True)
    install = WORKSPACE/'.local-tools/webots'
    binary = install/'msys64/mingw64/bin/webots.exe'
    system = Path(os.environ['SystemRoot'])
    env = dict(os.environ,WEBOTS_HOME=str(install))
    env['PATH'] = ';'.join(map(str,[binary.parent,install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    with (folder/'webots-stdout.log').open('w') as out,(folder/'webots-stderr.log').open('w') as err:
        result = subprocess.run([str(binary),'--batch','--mode=fast','--stdout','--stderr',str(world)],cwd=MODEL,env=env,
                                startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=out,stderr=err,timeout=240)
    report = json.loads(report_path.read_text(encoding='utf-8'))
    key = 'geometry_joint_and_fk_check_passed' if mode=='geometry' else 'dynamics_validation_passed'
    if (report['mode']!=mode or report['scenario']!=scenario or report['finger_count']!=2
            or report['model']!=parameters['model'] or report['finger_angles_rad']!=parameters['finger_angles_rad']):
        raise ValueError('Wrong report: '+str(case))
    for source,sha in report['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/source).read_bytes()).hexdigest()!=sha:
            raise ValueError('Stale source: '+source)
    if result.returncode!=(0 if report[key] else 1):
        raise ValueError('Report/exit mismatch: '+str(case))
    return {'mode':mode,'scenario':scenario,'passed':report[key],
            'failed_checks':[k for k,v in (report.get('dynamics_checks') or {}).items() if not v],
            'report_sha256':hashlib.sha256(report_path.read_bytes()).hexdigest(),
            'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest()}


def main():
    global MODEL
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',type=Path,default=MODEL)
    args = parser.parse_args()
    MODEL = args.model.resolve()
    if MODEL.parent!=WORKSPACE/'simulations':
        raise ValueError('Expected a model directly under this workspace simulations directory')
    p = json.loads((MODEL/'parameters.json').read_text())
    cases = [('geometry','balance')]+[('dynamics',s) for s in ['balance','tilt_positive','tilt_negative','push','steering','drive',*p['dynamic_arm_targets'],'pose_guard','claw_cycle_folded','claw_cycle_raised']]
    rows,errors = [],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {pool.submit(run_one,case):case for case in cases}
        for task in as_completed(tasks):
            try:
                row = task.result();rows.append(row);print(json.dumps(row),flush=True)
            except Exception as error:
                errors.append({'case':tasks[task],'error':str(error)});print(str(error),flush=True)
    (MODEL/'results/empty-validation-status.json').write_text(json.dumps({'cases':rows,'errors':errors},indent=2)+'\n')
    if errors or not all(row['passed'] for row in rows):
        raise SystemExit(1)


if __name__=='__main__':
    main()
