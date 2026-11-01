"""Folded pickup, verified release, retreat and folding; retain failure controls."""

from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import argparse
import json
import subprocess
import sys

import run_grasp_sweep as runner

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase',choices=['comparison','validated'],default='validated')
    parser.add_argument('--prepare-only',action='store_true')
    args = parser.parse_args()
    study = 'return_cycle_'+args.phase
    cases = [('reference_50mm',.050,{})]
    if args.phase=='comparison':
        cases += [('stationary_robot_control',.050,{'test_hold_after_release':True})]
    if args.phase=='validated':
        cases = [('approach_40mm',.040,{}),('approach_60mm',.060,{}),('mass_5g_50mm',.050,{'object_mass_kg':.005}),
                  ('reference_repeat',.050,{}),('release_gate_control',.050,{'test_block_release_confirmation':True}),
                  ('retreat_timeout_control',.050,{'return_retreat_timeout_s':6.0}),
                  ('open_claw_control',.050,{'open_claw_control':True}),
                  ('load_rejection_control',.050,{'object_mass_kg':.020}),
                  ('hold_40mm',.040,{'test_hold_after_release':True}),
                  ('hold_60mm',.060,{'test_hold_after_release':True}),
                  ('hold_5g_50mm',.050,{'test_hold_after_release':True,'object_mass_kg':.005})]
    base = json.loads((MODEL/'grasp_parameters.json').read_text(encoding='utf-8'))
    entries = []
    for name,distance,overrides in cases:
        folder = MODEL/'experiments'/study/'cases'/name
        folder.mkdir(parents=True,exist_ok=True)
        experiment = dict(base,start_folded=True,return_after_release=True,placement_offset_world_m=[distance,0,0],
                          alignment_target_offset_world_m=[distance,0,0],**overrides)
        config = folder/'experiment.json'
        config.write_text(json.dumps(experiment,indent=2)+'\n',encoding='utf-8')
        subprocess.run([sys.executable,str(MODEL/'scripts/generate_grasp_worlds.py'),'--experiment',str(config),
                        '--case',name,'--study',study],check=True,capture_output=True,text=True)
        entries.append({'case':name,'experiment_path':str(config.relative_to(WORKSPACE)),
                        'world_path':str((MODEL/'worlds'/f'alpha_{study}_{name}_validation.wbt').relative_to(WORKSPACE))})
    (MODEL/'experiments'/study/'manifest.json').write_text(json.dumps({'study':study,'cases':entries},indent=2)+'\n',encoding='utf-8')
    if args.prepare_only:
        return
    runner.RESULTS = MODEL/'results'/study
    results,errors = [],[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {pool.submit(runner.run_one,entry):entry['case'] for entry in entries}
        for task in as_completed(tasks):
            try:
                r = task.result(); results.append(r); print(json.dumps(r),flush=True)
            except Exception as error:
                errors.append({'case':tasks[task],'error':str(error)}); print(str(error),flush=True)
    (runner.RESULTS/'run-status.json').write_text(json.dumps({'results':results,'infrastructure_errors':errors},indent=2)+'\n',encoding='utf-8')
    if errors:
        raise SystemExit(2)


if __name__=='__main__':
    main()
