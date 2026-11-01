"""Generate isolated one-factor grasp cases and run Webots with bounded workers.

Criterion failures are experimental outcomes, not infrastructure errors. Reference
inputs are never rewritten. Each case retains its configuration, world and evidence.
"""

from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse
import copy
import hashlib
import json
import os
import subprocess
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = 'grasp_sweep_v2'
CASES = MODEL/'experiments'/STUDY/'cases'
RESULTS = MODEL/'results'/STUDY


def definitions():
    rows = [('reference', 'reference', 0, {})]
    rows += [(f'mass_{str(g).replace(".", "p")}g', 'mass_g', g, {'object_mass_kg': g/1000}) for g in [.5, 5, 10, 20, 50]]
    rows += [(f'friction_{str(mu).replace(".", "p")}', 'friction', mu, {'finger_object_friction': mu}) for mu in [0.0, .1, .3, .9]]
    rows += [(f'diameter_{mm}mm', 'diameter_mm', mm, {'object_radius_m': mm/2000}) for mm in [8, 12, 16, 24, 28, 40]]
    for axis, index in [('x', 0), ('y', 1)]:
        for mm in [-10, -5, -2, 2, 5, 10]:
            offset = [0, 0, 0]
            offset[index] = mm/1000
            rows.append((f'offset_{axis}_{"minus" if mm<0 else "plus"}{abs(mm)}mm', f'offset_{axis}_mm', mm, {'placement_offset_world_m': offset}))
    rows.append(('reference_repeat', 'repeat', 0, {}))
    return rows


def prepare():
    base = json.loads((MODEL/'grasp_parameters.json').read_text(encoding='utf-8'))
    entries = []
    for name, factor, value, overrides in definitions():
        folder = CASES/name
        folder.mkdir(parents=True, exist_ok=True)
        experiment = copy.deepcopy(base)
        experiment.update(overrides)
        config = folder/'experiment.json'
        config.write_text(json.dumps(experiment, indent=2)+'\n', encoding='utf-8')
        result = subprocess.run([sys.executable, str(MODEL/'scripts/generate_grasp_worlds.py'), '--experiment', str(config), '--case', name, '--study', STUDY], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        entries.append({'case': name, 'factor': factor, 'value': value, 'overrides': overrides,
                        'experiment_path': str(config.relative_to(WORKSPACE)),
                        'world_path': str((MODEL/'worlds'/f'alpha_{STUDY}_{name}_validation.wbt').relative_to(WORKSPACE))})
    manifest = {'study': STUDY, 'design': 'One factor at a time; all unlisted inputs fixed to reference. Known mass supplied to payload compensation. One deterministic trial per point plus a reference repeat.',
                'criteria': 'Unchanged 48 s grasp-cycle criteria: see reference report checks; failures are retained.',
                'fixture_rule': 'Stand/tray follow object x/y placement; stand height follows radius to support the sphere. Claw starts fixed. Payload location estimate remains nominal.',
                'reference_sha256': hashlib.sha256((MODEL/'grasp_parameters.json').read_bytes()).hexdigest(),
                'cases': entries}
    (CASES.parent/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    print(f'Prepared {len(entries)} isolated cases.', flush=True)
    return manifest


def run_one(entry):
    name = entry['case']
    folder = RESULTS/name
    folder.mkdir(parents=True, exist_ok=True)
    report_path = folder/'report.json'
    report_path.unlink(missing_ok=True)
    install = WORKSPACE/'.local-tools/webots'
    binary = install/'msys64/mingw64/bin/webots.exe'
    env = dict(os.environ)
    system = Path(os.environ['SystemRoot'])
    env['WEBOTS_HOME'] = str(install)
    env['PATH'] = ';'.join(map(str, [binary.parent, install/'msys64/usr/bin', system/'System32', system, Path('C:/Python313')]))
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    with (folder/'webots-stdout.log').open('w') as out, (folder/'webots-stderr.log').open('w') as err:
        process = subprocess.run([str(binary), '--batch', '--mode=fast', '--stdout', '--stderr', str(WORKSPACE/entry['world_path'])],
                                 cwd=MODEL, env=env, stdout=out, stderr=err, startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW, timeout=240)
    if not report_path.exists():
        raise RuntimeError(f'{name}: no report, simulator exit {process.returncode}')
    report = json.loads(report_path.read_text())
    if report['case'] != name or process.returncode != (0 if report['validation_passed'] else 1):
        raise RuntimeError(f'{name}: inconsistent report/exit status {process.returncode}')
    for source, sha in report['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/source).read_bytes()).hexdigest() != sha:
            raise RuntimeError(f'{name}: stale evidence: {source}')
    if hashlib.sha256((WORKSPACE/entry['world_path']).read_bytes()).hexdigest() != report['world_sha256']:
        raise RuntimeError(f'{name}: world hash mismatch')
    failures = [key for key, passed in report['checks'].items() if not passed]
    return {'case': name, 'passed': report['passed'], 'failed_checks': failures,
            'load_rejected': report.get('load_rejected', False), 'validation_passed': report['validation_passed'],
            'time_s': report['time_s'], 'exit_code': process.returncode}


def main():
    global STUDY, CASES, RESULTS
    parser = argparse.ArgumentParser()
    parser.add_argument('--study', default=STUDY)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--workers', type=int, default=2, choices=[1, 2])
    args = parser.parse_args()
    STUDY = args.study
    if not STUDY or not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in STUDY):
        raise ValueError('Study name must use lowercase letters, digits, or underscores')
    if STUDY == 'grasp_sweep':
        raise ValueError('grasp_sweep is frozen historical evidence; use a new study name')
    CASES, RESULTS = MODEL/'experiments'/STUDY/'cases', MODEL/'results'/STUDY
    manifest = prepare()
    if args.prepare_only:
        return
    results, errors = [], []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        tasks = {pool.submit(run_one, row): row['case'] for row in manifest['cases']}
        for task in as_completed(tasks):
            try:
                result = task.result()
                results.append(result)
                reason = ', '.join(result['failed_checks']) or 'all checks'
                print(f'{len(results)+len(errors)}/{len(tasks)} {result["case"]}: {"PASS" if result["passed"] else "FAIL"} ({reason})', flush=True)
            except Exception as error:
                errors.append({'case': tasks[task], 'error': str(error)})
                print(f'INFRASTRUCTURE ERROR {tasks[task]}: {error}', flush=True)
    (RESULTS/'run-status.json').write_text(json.dumps({'results': results, 'infrastructure_errors': errors}, indent=2)+'\n')
    if errors:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
