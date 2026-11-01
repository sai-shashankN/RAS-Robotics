"""Controlled comparisons for load-transfer/stand clearance; retain failed trials."""

from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import argparse
import copy
import json
import subprocess
import sys

import run_grasp_sweep as runner

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def definitions(phase):
    if phase == 'ablation':
        rows = [('baseline', {})]
        rows += [('damping_2x', {'grasp_balance_gains': {'position': .16, 'velocity': .24}}),
                 ('position_4x', {'grasp_balance_gains': {'position': .32, 'velocity': .24}}),
                 ('ramp_fast', {'payload_ramp_s': .1}),
                 ('ramp_slow', {'payload_ramp_s': 3.0})]
        return [(name, dict(object_mass_kg=.005, **overrides)) for name, overrides in rows]
    if phase == 'screening':
        return [(f'{strategy}_{grams}g', dict(object_mass_kg=grams/1000, **overrides))
                for strategy, overrides in [('damping_2x', {'grasp_balance_gains': {'position': .16, 'velocity': .24}}),
                                             ('ramp_fast', {'payload_ramp_s': .1})]
                for grams in [2, 10, 20, 50]]
    if phase == 'refinement':
        return [(name, dict(object_mass_kg=.010, **overrides)) for name, overrides in [
            ('position_4x', {'grasp_balance_gains': {'position': .32, 'velocity': .24}}),
            ('damping_4x', {'grasp_balance_gains': {'position': .32, 'velocity': .48}}),
            ('combined', {'grasp_balance_gains': {'position': .16, 'velocity': .24}, 'payload_ramp_s': .1})]]
    if phase == 'validation':
        return [(f'mass_{str(g).replace(".", "p")}g', {'object_mass_kg': g/1000}) for g in [.5, 2, 5, 10, 20, 50]]+[
            ('mass_10g_repeat', {'object_mass_kg': .010})]
    raise ValueError(phase)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', default='validation', choices=['validation'])
    parser.add_argument('--workers', type=int, default=2, choices=[1, 2])
    args = parser.parse_args()
    study = 'loaded_clearance_'+args.phase
    inputs = MODEL/'experiments'/study
    runner.RESULTS = MODEL/'results'/study
    entries = []
    base = json.loads((MODEL/'grasp_parameters.json').read_text(encoding='utf-8'))
    if args.phase != 'validation':
        raise ValueError('Exploratory trials are frozen. Replay their saved sources/inputs; current validation uses --phase validation.')
    for name, overrides in definitions(args.phase):
        folder = inputs/'cases'/name
        folder.mkdir(parents=True, exist_ok=True)
        config = folder/'experiment.json'
        config.write_text(json.dumps(dict(copy.deepcopy(base), **overrides), indent=2)+'\n', encoding='utf-8')
        subprocess.run([sys.executable, str(MODEL/'scripts/generate_grasp_worlds.py'),
                        '--experiment', str(config), '--case', name, '--study', study],
                       check=True, capture_output=True, text=True)
        entries.append({'case': name, 'overrides': overrides,
                        'experiment_path': str(config.relative_to(WORKSPACE)),
                        'world_path': str((MODEL/'worlds'/f'alpha_{study}_{name}_validation.wbt').relative_to(WORKSPACE))})
    (inputs/'manifest.json').write_text(json.dumps({'study': study, 'cases': entries}, indent=2)+'\n', encoding='utf-8')
    results, errors = [], []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        tasks = {pool.submit(runner.run_one, row): row['case'] for row in entries}
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


if __name__ == '__main__':
    main()
