"""Correct wrist slew coordination without changing geometry, limits or acceptance."""
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
PRIOR = MODEL/'experiments/cube_leveling'
STUDY = MODEL/'experiments/cube_leveling_v2'
read = lambda p:json.loads(p.read_text(encoding='utf-8'))


def prepare():
    if (STUDY/'manifest.json').exists():return read(STUDY/'manifest.json')
    if STUDY.exists():raise FileExistsError('Preserve incomplete wrist-slew revision')
    manifest,status = read(PRIOR/'manifest.json'),read(PRIOR/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=8:raise ValueError('Prior attitude trials incomplete')
    baseline = {}
    for entry in manifest['cases']:
        folder = WORKSPACE/entry['project']/'results/physics'/entry['case'];r = read(folder/'report.json')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Prior source changed')
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('Prior trace/world changed')
        baseline[str((folder/'report.json').relative_to(WORKSPACE))] = sha(folder/'report.json')
    common = STUDY/'projects/common';common.mkdir(parents=True)
    for path in (PRIOR/'projects/common').glob('*.py'):shutil.copy2(path,common/path.name)
    path = common/'grasp_controller.py';source = path.read_text(encoding='utf-8')
    source = replace_once(source,"for j in commanded)\n    fraction = min(1, dt/duration)","for j in commanded if j!='wrist')\n    fraction = min(1, dt/duration)")
    source = replace_once(source,"    for j in commanded:\n        commanded[j] += (target[j]-commanded[j])*fraction",
        "    for j in commanded:\n        if j=='wrist':\n            limit = P['dynamic_arm_rate_rad_s']*dt\n            commanded[j] += max(-limit,min(limit,target[j]-commanded[j]))\n        else:\n            commanded[j] += (target[j]-commanded[j])*fraction")
    path.write_text(source,encoding='utf-8')
    rows = []
    for entry in manifest['cases']:
        prior = WORKSPACE/entry['project'];project = STUDY/'projects'/prior.name
        if not project.exists():
            for name in ['scripts','worlds','controllers','results']:(project/name).mkdir(parents=True)
            shutil.copy2(prior/'parameters.json',project/'parameters.json')
            shutil.copy2(prior/'scripts/generate_grasp_worlds.py',project/'scripts/generate_grasp_worlds.py')
            shutil.copy2(prior/'robot-clearance.json',project/'inherited-robot-clearance.json')
        case = entry['case'];folder = project/'experiments/physics/cases'/case;folder.mkdir(parents=True);config = folder/'experiment.json'
        shutil.copy2(prior/'experiments/physics/cases'/case/'experiment.json',config)
        result = subprocess.run([sys.executable,str(project/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),'--case',case,'--study','physics'],capture_output=True,text=True)
        if result.returncode:raise RuntimeError(result.stdout+result.stderr)
        rows.append(dict(entry,project=str(project.relative_to(WORKSPACE)),world=str((project/'worlds'/f'alpha_physics_{case}_validation.wbt').relative_to(WORKSPACE))))
    revised = dict(manifest,cases=rows,previous_reports_sha256=baseline,
        revision='Independent wrist command slew at the same 0.08 rad/s limit; non-wrist joints retain common interpolation. Geometry, poses, gains, friction, motor limits, timing, initial setup strategy and two-degree attitude criterion unchanged. First controller remains preserved in cube_leveling.')
    (STUDY/'manifest.json').write_text(json.dumps(revised,indent=2)+'\n',encoding='utf-8')
    return revised


def main():
    manifest = prepare()
    if (STUDY/'run-status.json').exists():raise FileExistsError('Completed revised controller trials protected')
    results,errors = [],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        order = sorted(manifest['cases'],key=lambda e:(e['open_control'],e['frame']!='world',e['variant']))
        tasks = {pool.submit(run_one,e):e for e in order}
        for task in as_completed(tasks):
            try:row = task.result();results.append(row)
            except Exception as error:row = {'case':tasks[task]['case'],'error':str(error)};errors.append(row)
            print(json.dumps(row),flush=True)
    (STUDY/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors},indent=2)+'\n',encoding='utf-8')
    if errors:raise SystemExit(2)


if __name__=='__main__':main()
