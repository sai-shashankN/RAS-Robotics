"""Verify all sweep evidence, distinguish retention from complete-cycle success."""

from pathlib import Path
from collections import Counter
import argparse
import hashlib
import json
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'
RETENTION = ['held_for_at_least_5_s', 'object_lifted_above_stand', 'arm_motion_added_at_least_6_mm_lift',
             'object_unsupported_through_hold', 'at_least_two_fingers_through_hold', 'object_slip_under_3_mm']
BALANCE = ['completed_duration', 'did_not_fall', 'peak_pitch_under_20_deg', 'pitch_error_under_5_deg',
           'final_pitch_error_under_1_deg', 'final_body_drift_under_5_cm', 'wheel_speed_within_limit']


def label(row):
    factor, value = row['factor'], row['value']
    return {'reference': 'Reference: 2 g, 20 mm', 'repeat': 'Reference repeat'}.get(factor, {
        'mass_g': f'Mass {value:g} g', 'friction': f'Friction {value:g}', 'diameter_mm': f'Diameter {value:g} mm',
        'offset_x_mm': f'World X offset {value:+g} mm', 'offset_y_mm': f'World Y offset {value:+g} mm'}.get(factor, factor))


def fixture_hint(report):
    setup, e = report['setup'], report['experiment']
    for sample in report['unexpected_contact_examples']:
        x, y, z = sample['point_m']
        radius = ((x-setup['initial_object_position_m'][0])**2+(y-setup['initial_object_position_m'][1])**2)**.5
        if setup['tray_top_m']<=z<=setup['stand_top_m']+.0002 and abs(radius-e['stand_radius_m'])<.0002:
            return sample['node']+' touching stand surface'
    return 'See contact examples/telemetry' if report['unexpected_contact_samples'] else ''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--study', default='grasp_sweep_v2')
    args = parser.parse_args()
    study = args.study
    if not study or not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in study):
        raise ValueError('Invalid study name')
    if study == 'grasp_sweep':
        raise ValueError('Historical sweep is frozen; see archive/grasp-sweep-before-clearance and LOADED_CLEARANCE_RESULTS.md')
    manifest_path = MODEL/'experiments'/study/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if hashlib.sha256((MODEL/'grasp_parameters.json').read_bytes()).hexdigest()!=manifest['reference_sha256']:
        raise ValueError('Reference input changed after sweep preparation')
    status = json.loads((RESULTS/study/'run-status.json').read_text())
    if status['infrastructure_errors'] or len(status['results'])!=len(manifest['cases']):
        raise ValueError('Sweep incomplete or infrastructure errors present')
    rows = []
    for entry in manifest['cases']:
        path = RESULTS/study/entry['case']/'report.json'
        r = json.loads(path.read_text())
        for source, sha in r['source_sha256'].items():
            if hashlib.sha256((WORKSPACE/source).read_bytes()).hexdigest()!=sha:
                raise ValueError(f'Stale evidence: {entry["case"]}, {source}')
        if hashlib.sha256((WORKSPACE/entry['world_path']).read_bytes()).hexdigest()!=r['world_sha256']:
            raise ValueError(f'World differs: {entry["case"]}')
        experiment = json.loads((WORKSPACE/entry['experiment_path']).read_text())
        if r['experiment']!=experiment or r['case']!=entry['case']:
            raise ValueError(f'Wrong case inputs: {entry["case"]}')
        c = r['checks']
        rows.append({**entry, 'description': label(entry), 'passed': r['passed'],
                     'retention_passed': all(c[key] for key in RETENTION), 'balance_passed': all(c[key] for key in BALANCE),
                     'setup_supported': c['object_initially_supported'], 'contact_free': c['no_unintended_arm_contacts'],
                     'release_passed': c['released_and_settled'], 'failed_checks': [key for key, passed in c.items() if not passed],
                     'contact_hint': fixture_hint(r), 'peak_pitch_deg': r['max_pitch_deg'],
                     'peak_reference_error_deg': r['max_pitch_error_deg'], 'final_drift_mm': r['final_body_drift_m']*1000,
                     'arm_added_lift_mm': r['arm_added_lift_m']*1000,
                     'min_contacting_fingers': r['min_hold_contacting_fingers'],
                     'min_stand_gap_mm': r['min_hold_stand_gap_m']*1000 if r['min_hold_stand_gap_m'] is not None else None,
                     'unexpected_contact_samples': r['unexpected_contact_samples']})
    reference = next(row for row in rows if row['case']=='reference')
    repeat = next(row for row in rows if row['case']=='reference_repeat')
    if not reference['passed'] or not repeat['passed']:
        raise ValueError('Reference did not reproduce')
    failures = Counter(check for row in rows for check in row['failed_checks'])
    groups = {}
    reference_values = {'mass_g': 2, 'friction': .6, 'diameter_mm': 20, 'offset_x_mm': 0, 'offset_y_mm': 0}
    for factor in reference_values:
        group = [row for row in rows if row['factor']==factor]
        groups[factor] = {'reference_value': reference_values[factor],
                          'tested_full_cycle_pass_values': sorted([reference_values[factor]]+[r['value'] for r in group if r['passed']]),
                          'tested_full_cycle_fail_values': sorted(r['value'] for r in group if not r['passed']),
                          'tested_retention_pass_values': sorted([reference_values[factor]]+[r['value'] for r in group if r['retention_passed']])}
    summary = {'date': '2026-10-23', 'design': manifest['design'], 'fixture_rule': manifest['fixture_rule'],
               'trial_count': len(rows), 'full_cycle_pass_count': sum(r['passed'] for r in rows),
               'retention_pass_count': sum(r['retention_passed'] for r in rows),
               'reference_repeat_passed': True, 'groups': groups, 'failed_check_counts': dict(failures), 'cases': rows,
               'manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
               'limitations': ['One-factor-at-a-time point tests; untested gaps and factor combinations are not established.',
                               'Known payload mass supplied to compensation; no mass-estimation or tactile-hardware validation.',
                               'Prepared poses, rigid contacts, ideal sensors, provisional motors/masses.',
                               'Full-cycle failures can come from fixture interference despite retained objects; no payload-capacity rating.']}
    (RESULTS/study/'grasp-sweep-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    lines = ['# Alpha grasp sensitivity sweep', '', 'Recorded: 23 October 2026 (Asia/Calcutta), Webots R2025a.', '',
             f"**{summary['full_cycle_pass_count']} of {len(rows)} whole-cycle trials passed.** Object retention/lift checks passed in {summary['retention_pass_count']} trials, including some that failed contact, balance, setup, or release requirements. Both reference runs passed.", '',
             '## Controlled experiment', '', manifest['design'], '', manifest['fixture_rule'], '',
             'Reference: 2 g, 20 mm sphere, friction 0.6, zero placement offset. Geometry, actuator limits, wheel gains, finger targets, sequence, and acceptance thresholds were held fixed. Each case lasted 48 simulated seconds unless the fall cutoff ended it earlier. The controller did not use object world position as balance feedback or change any object pose after scene setup.', '',
             'Changing diameter keeps the nominal sphere center fixed and adjusts stand height to support it. Placement moves the stand/tray with the object, while leaving the claw and the nominal payload-location estimate fixed. This tests prepared fixture misalignment; it is not an autonomous alignment test.', '',
             '## Tested points', '', '| Factor | Whole-cycle pass values | Whole-cycle fail values |', '| --- | --- | --- |']
    for factor, group in groups.items():
        lines.append(f"| {factor} | {', '.join(map(str, group['tested_full_cycle_pass_values']))} | {', '.join(map(str, group['tested_full_cycle_fail_values'])) or 'None at tested points'} |")
    lines += ['', 'These are individual tested points, not certified continuous operating ranges. A pass at two endpoints does not establish every value between them or a combination of the passing factors.', '',
              '## Failures and interpretation', '',
              'A whole-cycle pass includes initial support, actual additional arm lift of at least 6 mm, at least 5 s unsupported holding with at least two contacting fingers and slip under 3 mm, zero unintended arm contact, balance/drive limits, and release/settling into the tray. Retention alone is reported separately and never substitutes for a full pass. The end-settled flag checks absence of finger contact and a settled supported object; it can pass when a sphere was never picked up, so it does not by itself prove release.', '',
              'Higher-mass cases can retain the sphere while the robot contacts the stand. Contact examples on the stand surface identify this as an environment/motion-path limitation in those runs; it does not identify maximum finger force or payload capacity. Known mass compensation is favorable information, so these results do not cover unknown payloads.', '',
              'The zero-friction reference-size test is retained as a model check. Successful holding there suggests that geometric enclosure by the curled fingers contributes to retention; it does not establish that friction is irrelevant to other shapes, sizes, or contact conditions.', '',
              '| Case | Whole cycle | Retention/lift | Balance | End settled | Contact observation / failed checks |',
              '| --- | --- | --- | --- | --- | --- |']
    for row in rows:
        observation = row['contact_hint'] or ', '.join(row['failed_checks']) or 'All checks passed'
        lines.append(f"| {row['description']} | {'PASS' if row['passed'] else 'FAIL'} | {'PASS' if row['retention_passed'] else 'FAIL'} | {'PASS' if row['balance_passed'] else 'FAIL'} | {'PASS' if row['release_passed'] else 'FAIL'} | {observation} |")
    lines += ['', '## Evidence and next work', '',
              f'Each case has an input snapshot and generated setup/world under experiments/{study}/cases and worlds, plus a report, telemetry, logs, and actual renders under results/{study}. Source, input, and world hashes were verified. Expected criterion failures are saved outcomes; the batch rejects missing/stale reports or inconsistent simulator exit codes as infrastructure errors.', '',
              'The previous single-sphere milestone is preserved in archive/prepared-grasp-milestone. The reference and open-claw control are revalidated with the sweep-capable controller; the original 13-case base/empty-claw controller and robot parameters remain unchanged.', '',
              'Next: address fixture clearance under loaded motion and diagnose alignment failures before claiming a useful capture tolerance. Test combinations, uncertainty in payload estimates, and sensor/actuator effects later. No hardware was purchased.', '']
    (RESULTS/study/'GRASP_SWEEP_RESULTS.md').write_text('\n'.join(lines), encoding='utf-8')
    sys.path.insert(0, str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    values = [[r['setup_supported'], r['retention_passed'], r['contact_free'], r['balance_passed'], r['release_passed'], r['passed']] for r in rows]
    fig, ax = plt.subplots(figsize=(12, 11.5))
    ax.imshow(values, cmap=ListedColormap(['#f3d7d1', '#c8e9df']), vmin=0, vmax=1, aspect='auto')
    ax.set_yticks(range(len(rows)), [r['description'] for r in rows], fontsize=9)
    ax.set_xticks(range(6), ['Initially\nsupported', 'Retention\nand lift', 'No unintended\narm contact', 'Balance\nand return', 'End detached\nand settled', 'Complete\ncycle'], fontsize=9)
    ax.xaxis.tick_top()
    for i, row in enumerate(values):
        for j, passed in enumerate(row):
            ax.text(j, i, 'PASS' if passed else 'FAIL', ha='center', va='center', fontsize=8, color='#175c49' if passed else '#8d3425')
    for i in range(1, len(rows)):
        if rows[i]['factor']!=rows[i-1]['factor']:
            ax.axhline(i-.5, color='white', linewidth=3)
    ax.tick_params(length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.suptitle(f"Alpha grasp sensitivity: {summary['full_cycle_pass_count']}/{len(rows)} complete-cycle passes", fontsize=16, y=.99)
    fig.text(.5, .015, 'One factor at a time | known payload mass | prepared fixture | individual points, not a hardware rating', ha='center', fontsize=9, color='#555f68')
    fig.tight_layout(rect=(0, .035, 1, .96))
    fig.savefig(RESULTS/study/'grasp-sweep-outcomes.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'trials': len(rows), 'full_cycle_passes': summary['full_cycle_pass_count'], 'retention_passes': summary['retention_pass_count'], 'groups': groups}, indent=2))


if __name__ == '__main__':
    main()
