"""Prepared-target capture tests. No perception or autonomous base approach."""

from concurrent.futures import ThreadPoolExecutor, as_completed
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
    parser.add_argument('--phase', choices=['validated', 'controls'], default='validated')
    args = parser.parse_args()
    study = 'capture_alignment_'+args.phase
    entries = []
    base = json.loads((MODEL/'grasp_parameters.json').read_text(encoding='utf-8'))
    rows = [('reference', [0, 0, 0])]
    rows += [(f'offset_{axis}_{"minus" if mm<0 else "plus"}{abs(mm)}mm',
              [mm/1000 if axis=='x' else 0, mm/1000 if axis=='y' else 0, 0])
             for axis, values in [('x', [-10, -5, 10]), ('y', [-10, 10])] for mm in values]
    if args.phase == 'controls':
        rows = [('wrong_target_x_plus10mm', [.01, 0, 0]), ('height_plus10mm', [0, 0, .01]),
                ('distance_plus20mm', [.02, 0, 0]), ('aligned_x_plus10mm_repeat', [.01, 0, 0])]
    for name, placement in rows:
        folder = MODEL/'experiments'/study/'cases'/name
        folder.mkdir(parents=True, exist_ok=True)
        experiment = dict(base, placement_offset_world_m=placement)
        experiment['alignment_target_offset_world_m'] = [0, 0, 0] if name.startswith('wrong_target') else placement
        config = folder/'experiment.json'
        config.write_text(json.dumps(experiment, indent=2)+'\n', encoding='utf-8')
        subprocess.run([sys.executable, str(MODEL/'scripts/generate_grasp_worlds.py'), '--experiment', str(config),
                        '--case', name, '--study', study], check=True, capture_output=True, text=True)
        entries.append({'case': name, 'placement_offset_world_m': placement,
                        'experiment_path': str(config.relative_to(WORKSPACE)),
                        'world_path': str((MODEL/'worlds'/f'alpha_{study}_{name}_validation.wbt').relative_to(WORKSPACE))})
    (MODEL/'experiments'/study/'manifest.json').write_text(json.dumps({'study': study, 'cases': entries}, indent=2)+'\n', encoding='utf-8')
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


if __name__ == '__main__':
    main()
