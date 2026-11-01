"""Generate and run staged folded-arm pickup cases with supplied straight targets."""

from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse
import json
import subprocess
import sys

import run_grasp_sweep as runner

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def definitions(phase):
    if phase=='trial':
        return [('reference', .050, .050, {})]
    return [('reference_50mm', .050, .050, {}), ('approach_40mm', .040, .040, {}),
            ('approach_60mm', .060, .060, {}), ('mass_5g_50mm', .050, .050, {'object_mass_kg': .005}),
            ('reference_repeat', .050, .050, {}), ('open_claw_control', .050, .050, {'open_claw_control': True}),
            ('wrong_target_control', .060, .050, {}), ('load_rejection_control', .050, .050, {'object_mass_kg': .020}),
            ('alignment_timeout_control', .050, .050, {'alignment_deadline_after_staging_s': 6.0})]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=['validated'], default='validated')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    study = 'folded_approach_'+args.phase
    base = json.loads((MODEL/'grasp_parameters.json').read_text(encoding='utf-8'))
    entries = []
    for name, actual, target, overrides in definitions(args.phase):
        folder = MODEL/'experiments'/study/'cases'/name
        folder.mkdir(parents=True, exist_ok=True)
        experiment = dict(base, start_folded=True, placement_offset_world_m=[actual, 0, 0],
                          alignment_target_offset_world_m=[target, 0, 0], **overrides)
        config = folder/'experiment.json'
        config.write_text(json.dumps(experiment, indent=2)+'\n', encoding='utf-8')
        subprocess.run([sys.executable, str(MODEL/'scripts/generate_grasp_worlds.py'), '--experiment', str(config),
                        '--case', name, '--study', study], check=True, capture_output=True, text=True)
        entries.append({'case': name, 'experiment_path': str(config.relative_to(WORKSPACE)),
                        'world_path': str((MODEL/'worlds'/f'alpha_{study}_{name}_validation.wbt').relative_to(WORKSPACE))})
    (MODEL/'experiments'/study/'manifest.json').write_text(json.dumps({'study': study, 'cases': entries}, indent=2)+'\n', encoding='utf-8')
    if args.prepare_only:
        return
    runner.RESULTS = MODEL/'results'/study
    results, errors = [], []
    with ThreadPoolExecutor(max_workers=2) as pool:
        tasks = {pool.submit(runner.run_one, entry): entry['case'] for entry in entries}
        for task in as_completed(tasks):
            try:
                result = task.result()
                results.append(result)
                print(json.dumps(result), flush=True)
            except Exception as error:
                errors.append({'case': tasks[task], 'error': str(error)})
                print(str(error), flush=True)
    (runner.RESULTS/'run-status.json').write_text(json.dumps({'results': results, 'infrastructure_errors': errors}, indent=2)+'\n', encoding='utf-8')
    if errors:
        raise SystemExit(2)


if __name__=='__main__':
    main()
