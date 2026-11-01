"""Check provenance and report accepted grasps separately from rejected loads."""

from pathlib import Path
import csv
import hashlib
import json
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'
ARCHIVE = MODEL/'archive/grasp-sweep-before-clearance'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archived_path(name):
    path = Path(name)
    if path.parts[:2] == ('simulations', 'common'):
        return ARCHIVE/'common'/Path(*path.parts[2:])
    if path.parts[:2] == ('simulations', 'model-alpha'):
        return ARCHIVE/Path(*path.parts[2:])
    raise ValueError(name)


def verify(report, resolve):
    for name, digest in report['source_sha256'].items():
        if sha(resolve(name)) != digest:
            raise ValueError(f'Stale source: {name}')
    world = Path(report['world_path']).relative_to(WORKSPACE)
    if sha(resolve(str(world))) != report['world_sha256']:
        raise ValueError(f'Stale world: {world}')


def telemetry(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def main():
    before = {}
    baseline_manifest = json.loads((ARCHIVE/'experiments/grasp_sweep/manifest.json').read_text(encoding='utf-8'))
    for entry in baseline_manifest['cases']:
        name = entry['case']
        r = json.loads((ARCHIVE/'results/grasp_sweep'/name/'report.json').read_text(encoding='utf-8'))
        verify(r, archived_path)
        before[name] = r
    studies = {}
    for phase in ['ablation', 'screening', 'refinement', 'validation']:
        study = 'loaded_clearance_'+phase
        status = json.loads((RESULTS/study/'run-status.json').read_text(encoding='utf-8'))
        manifest = json.loads((MODEL/'experiments'/study/'manifest.json').read_text(encoding='utf-8'))
        if status['infrastructure_errors'] or len(status['results']) != len(manifest['cases']):
            raise ValueError(f'Incomplete study {study}')
        rows = []
        for entry in manifest['cases']:
            r = json.loads((RESULTS/study/entry['case']/'report.json').read_text(encoding='utf-8'))
            snapshot = MODEL/'experiments'/study/'source_snapshot'
            # Exploratory sources were frozen before the final guard/helper change.
            resolve = (lambda name: snapshot/name) if phase != 'validation' else (lambda name: WORKSPACE/name)
            for name, digest in r['source_sha256'].items():
                if sha(resolve(name)) != digest:
                    raise ValueError(f'Stale {study}/{entry["case"]}: {name}')
            if sha(WORKSPACE/entry['world_path']) != r['world_sha256']:
                raise ValueError(f'Stale world {study}/{entry["case"]}')
            if r['experiment'] != json.loads(resolve(entry['experiment_path']).read_text(encoding='utf-8')):
                raise ValueError(f'Wrong input: {entry["case"]}')
            rows.append(r)
        studies[phase] = rows
    after = studies['validation']
    accepted = [r for r in after if not r['load_rejected']]
    rejected = [r for r in after if r['load_rejected']]
    if len(accepted) != 5 or not all(r['passed'] for r in accepted):
        raise ValueError('Accepted-load regression failed')
    if len(rejected) != 2 or not all(r['load_guard_passed'] and not r['passed'] for r in rejected):
        raise ValueError('Expected guarded refusals failed')
    compact = lambda r: {k: r.get(k) for k in ['case', 'passed', 'load_rejected', 'load_guard_passed', 'checks', 'load_plan',
                        'max_body_drift_m', 'max_pitch_error_deg', 'unexpected_contact_samples', 'arm_added_lift_m', 'hold_time_s']}
    summary = {'date': '2026-10-24', 'original_checkpoint_date': '2026-10-23', 'accepted_grasp_trials': len(accepted), 'accepted_grasp_passes': sum(r['passed'] for r in accepted),
               'historical_sweep_verified_trials': len(before),
               'guard_trials': len(rejected), 'safe_rejections': sum(r['load_guard_passed'] for r in rejected),
               'baseline_verified_against_archive': str(ARCHIVE.relative_to(WORKSPACE)),
               'exploratory_trials': {phase: [compact(r) for r in rows] for phase, rows in studies.items() if phase != 'validation'},
               'validation': [compact(r) for r in after]}
    (RESULTS/'loaded-clearance-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    lines = ['# Alpha loaded-motion correction', '', 'Original checkpoint: 23 October 2026; revalidated 24 October 2026, Webots R2025a.', '',
             '**Five accepted-load trials passed the complete 48 s grasp cycle:** 0.5, 2, 5, and 10 g, plus a 10 g repeat. Two additional 20/50 g trials were rejected before closing; they are successful guard tests, not successful grasps.', '',
             '## Change and diagnosis', '',
             'The previous 5/10 g failures retained the sphere but contacted the stand during a forward chassis transient. A fresh 5 g baseline reproduced the contact. Doubling grasp-specific position/velocity feedback (0.08/0.12 to 0.16/0.24) and shortening the contact-driven known-load ramp from 1.0 to 0.1 s resolved the tested 5/10 g cycles in combination. Robot geometry, wheel/joint torque caps, arm path, and grasp thresholds stayed fixed. These are tuned simulation gains; a faster estimator is not evidence of available hardware sensing.', '',
             'The nominal full-load plan requires about 19/25 degrees lean at 20/50 g, exceeding the existing 0.30 rad (17.19 degree) command limit. A sampled quasi-static preflight now checks closure and lift with the full specified mass at the nominal palm location. If it exceeds the limit, the claw stays open and no pickup is attempted. This uses known mass and commanded geometry; it is conservative about support transfer and does not certify actual dynamics or clearance.', '',
             '## Final validation', '',
             '| Sphere mass | Result | Planned peak lean | Unintended contacts | Peak chassis drift | Peak reference error | Added arm lift |',
             '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for r in after:
        outcome = 'REJECTED; guard passed' if r['load_rejected'] else 'GRASP PASS'
        mass = f"{r['experiment']['object_mass_kg']*1000:g} g"+(' repeat' if 'repeat' in r['case'] else '')
        lift = 'No pickup attempted' if r['load_rejected'] else f"{r['arm_added_lift_m']*1000:.2f} mm"
        lines.append(f"| {mass} | {outcome} | {r['load_plan']['peak_nominal_loaded_lean_rad']*180/3.141592653589793:.2f} deg | {r['unexpected_contact_samples']} | {r['max_body_drift_m']*1000:.2f} mm | {r['max_pitch_error_deg']:.2f} deg | {lift} |")
    lines += ['', 'Each accepted trial must meet all prior lift/retention/balance/release criteria, plus admission by the lean preflight. Rejections must complete 48 s balancing, with no finger/object contact, no unintended arm contact, less than 2 mm object displacement, and the sphere settled on its original support. They deliberately fail positive grasp/lift criteria.', '',
              '## Alternatives retained', '',
              '| Study | Case | Complete grasp cycle | Unintended contact samples |', '| --- | --- | --- | ---: |']
    for phase in ['ablation', 'screening', 'refinement']:
        for r in studies[phase]:
            lines.append(f"| {phase} | {r['case']} | {'PASS' if r['passed'] else 'FAIL'} | {r['unexpected_contact_samples']} |")
    lines += ['', 'The strongest damping alternative fell at 31.608 s. It remains a failed exploratory trial; increasing gains is not uniformly stabilizing. The faster-ramp-only 10 g test avoided stand contact but had a later arm/sphere contact during release. The combined change was therefore tested through the complete cycle, not just the hold.', '',
              '## Provenance and limits', '',
              'Original sweep evidence is frozen under archive/grasp-sweep-before-clearance and checked against its own source/world copies. All 16 exploratory tuning trials retain source snapshots under experiments/loaded_clearance_*/source_snapshot, input/setup/world files, reports, telemetry, logs, and renders. Seven final mass/guard trials are verified against current sources and worlds. A separate updated 2 g reference and open-claw control are recorded in GRASP_RESULTS.md; the preceding 13 base/empty-claw cases retain unchanged source/configuration hashes.', '',
              'The prior 29-case sensitivity sweep describes the previous controller. Its friction, diameter, and placement results have not been re-established for the new gains. This is a handful of prepared-position point tests with known mass, ideal sensing, provisional motors, and rigid contacts. No continuous payload rating, unknown-load handling, arbitrary obstacle clearance, autonomous approach, or hardware performance is established. No parts purchased.', '',
              'Next: diagnose and improve prepared capture alignment, then integrate approach from the resting pose. Test combinations and sensing/actuator uncertainty in later stages.', '']
    (RESULTS/'LOADED_CLEARANCE_RESULTS.md').write_text('\n'.join(lines), encoding='utf-8')
    sys.path.insert(0, str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, name, mass in zip(axes, ['mass_5g', 'mass_10g'], [5, 10]):
        for folder, label, color in [(ARCHIVE/'results/grasp_sweep'/name, 'Previous controller', '#a64332'),
                                     (RESULTS/'loaded_clearance_validation'/name, 'Revised controller', '#15776b')]:
            rows = telemetry(folder/'telemetry.csv')
            ax.plot([float(r['time_s']) for r in rows], [float(r['body_x_m'])*1000 for r in rows], label=label, color=color)
        ax.set_ylabel(f'{mass} g: chassis X (mm)')
        ax.grid(alpha=.2)
        ax.axhline(0, color='#666', linewidth=.6)
        ax.legend(loc='upper right')
    axes[-1].set_xlabel('Simulated time (s)')
    fig.suptitle('Alpha: loaded-motion comparison, same robot and torque limits')
    fig.text(.5, .015, 'Prepared fixture | zero unintended contacts in revised 5/10 g runs | known payload | simulation only', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .95))
    fig.savefig(RESULTS/'loaded-clearance-comparison.png', dpi=160)
    plt.close(fig)
    print(json.dumps({k: summary[k] for k in ['accepted_grasp_trials', 'accepted_grasp_passes', 'guard_trials', 'safe_rejections']}, indent=2))


if __name__ == '__main__':
    main()
