"""Verify the paired capture trials and distinguish grasps, refusals and misses."""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'


def read_study(phase):
    study = 'capture_alignment_'+phase
    manifest = json.loads((MODEL/'experiments'/study/'manifest.json').read_text(encoding='utf-8'))
    status = json.loads((RESULTS/study/'run-status.json').read_text(encoding='utf-8'))
    if status['infrastructure_errors'] or len(status['results'])!=len(manifest['cases']):
        raise ValueError('Incomplete study '+study)
    frozen = phase in ['baseline', 'corrected']
    snapshot = MODEL/'experiments'/study/'source_snapshot'
    rows = {}
    for entry in manifest['cases']:
        folder = RESULTS/study/entry['case']
        r = json.loads((folder/'report.json').read_text(encoding='utf-8'))
        for name, digest in r['source_sha256'].items():
            source = snapshot/name if frozen else WORKSPACE/name
            if hashlib.sha256(source.read_bytes()).hexdigest()!=digest:
                raise ValueError(f'Stale evidence {study}/{entry["case"]}: {name}')
        if hashlib.sha256((WORKSPACE/entry['world_path']).read_bytes()).hexdigest()!=r['world_sha256']:
            raise ValueError('Stale world '+entry['case'])
        if r['experiment']!=json.loads(((snapshot if frozen else WORKSPACE)/entry['experiment_path']).read_text(encoding='utf-8')):
            raise ValueError('Wrong experiment '+entry['case'])
        with (folder/'telemetry.csv').open(newline='', encoding='utf-8') as stream:
            data = list(csv.DictReader(stream))
        close = r.get('schedule_actual_s', r['experiment']['schedule_s'])['close']
        preclose = [row for row in data if float(row['time_s'])<close]
        initial = r['setup']['initial_object_position_m']
        r['preclose_object_displacement_m'] = max(math.dist(initial, [float(row[f'object_{a}_m']) for a in 'xyz']) for row in preclose)
        r['preclose_max_finger_contacts'] = max(int(row['finger_contact_count']) for row in preclose)
        rows[entry['case']] = r
    return rows


def main():
    studies = {phase: read_study(phase) for phase in ['baseline', 'corrected', 'validated', 'controls']}
    baseline, final, controls = studies['baseline'], studies['validated'], studies['controls']
    recovered = ['offset_x_minus10mm', 'offset_x_minus5mm', 'offset_x_plus10mm']
    for name in recovered:
        if baseline[name]['passed'] or not final[name]['passed']:
            raise ValueError('Failed paired recovery '+name)
        if final[name]['preclose_max_finger_contacts'] or final[name]['preclose_object_displacement_m']>.002:
            raise ValueError('Object disturbed before closure '+name)
    if not final['reference']['passed'] or not controls['aligned_x_plus10mm_repeat']['passed']:
        raise ValueError('Reference/repeat failed')
    guards = [final[name] for name in ['offset_y_minus10mm', 'offset_y_plus10mm']]
    guards += [controls[name] for name in ['height_plus10mm', 'distance_plus20mm']]
    if not all(r['alignment_guard_passed'] and not r['passed'] for r in guards):
        raise ValueError('Guard control failed')
    wrong = controls['wrong_target_x_plus10mm']
    if wrong['passed'] or wrong['checks']['object_lifted_above_stand'] or wrong['checks']['arm_motion_added_at_least_6_mm_lift']:
        raise ValueError('Wrong supplied target unexpectedly lifted object')
    regression = json.loads((RESULTS/'loaded-clearance-summary.json').read_text(encoding='utf-8'))
    if regression['accepted_grasp_passes']!=5 or regression['safe_rejections']!=2:
        raise ValueError('Load regression missing')
    reference = json.loads((RESULTS/'grasp-validation-summary.json').read_text(encoding='utf-8'))
    if not reference['prepared_grasp_passed'] or not reference['open_claw_control_passed']:
        raise ValueError('Reference/open-claw regression missing')
    # The regression summaries must refer to the same current controller source.
    for path in [RESULTS/'grasp/report.json', RESULTS/'grasp_negative/report.json', RESULTS/'loaded_clearance_validation/mass_10g/report.json']:
        report = json.loads(path.read_text(encoding='utf-8'))
        for name, digest in report['source_sha256'].items():
            if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest()!=digest:
                raise ValueError('Stale regression '+str(path))
    compact = lambda r: {k: r.get(k) for k in ['case', 'passed', 'alignment_rejected', 'alignment_failed_to_settle',
                    'alignment_guard_passed', 'time_s', 'alignment_wait_s', 'preclose_alignment_max_error_m',
                    'preclose_object_displacement_m', 'preclose_max_finger_contacts', 'unexpected_contact_samples',
                    'arm_added_lift_m', 'hold_time_s', 'min_hold_contacting_fingers', 'capture_plan']}
    summary = {'date': '2026-10-24', 'original_checkpoint_date': '2026-10-23', 'recovered_offset_cases': recovered, 'successful_alignment_grasps': 5,
               'guard_controls_passed': 4, 'wrong_target_expected_miss': True, 'physics_trials_this_study': 22,
               'baseline': [compact(r) for r in baseline.values()],
               'fixed_time_trial': [compact(r) for r in studies['corrected'].values()],
               'validated': [compact(r) for r in final.values()], 'controls': [compact(r) for r in controls.values()],
               'load_regression_and_reference_controls_passed': True}
    (RESULTS/'capture-alignment-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    lines = ['# Alpha supplied-target capture alignment', '', 'Original checkpoint: 23 October 2026; revalidated 24 October 2026, Webots R2025a.', '',
             '**Recovered three previously missed placements:** world X -10, -5, and +10 mm. Each now completes grasp, arm lift, unsupported hold, lowering and release with zero unintended arm contacts. The zero-offset reference and +10 mm repeat also pass.', '',
             '## What changed', '',
             'An optional supplied target offset commands a small longitudinal wheel translation while the arm remains in its previously tested raised/open pose. The controller uses pitch-corrected wheel odometry and gyro feedback. A smooth 2.5 s position reference is followed by a measured settling check: the last 0.5 s must stay within 0.5 mm odometry error and 3 mm/s speed before closure. If necessary the entire grasp schedule is delayed, up to a 26 s absolute alignment deadline; failure leaves the claw open.', '',
             'The offset is supplied as experiment input, separately from actual object placement. No live sphere position, camera detection, sphere relocation or artificial attachment drives the alignment. This validates local motion sequencing with favorable target information; it does not implement visual alignment or approach from the folded arm pose. Geometry, mass layout, arm path, torque/speed caps and load compensation remain unchanged.', '',
             'Alignment trials use the reference 2 g, 20 mm sphere and assumed friction 0.6. They do not combine the heavier passing mass points with placement offsets.', '',
             '## Paired results', '',
             '| Actual object offset | Previous no-alignment cycle | Revised outcome | Extra wait before grasp | Worst settled odometry error | Added arm lift |',
             '| --- | --- | --- | ---: | ---: | ---: |']
    for name in baseline:
        r = final[name]
        offset = r['experiment']['placement_offset_world_m']
        label = 'Reference' if name=='reference' else f"X {offset[0]*1000:+g} mm" if offset[0] else f"Y {offset[1]*1000:+g} mm"
        outcome = 'GRASP PASS' if r['passed'] else 'DECLINED; guard passed'
        lift = f"{r['arm_added_lift_m']*1000:.2f} mm" if r['passed'] else 'No pickup'
        error = f"{r['preclose_alignment_max_error_m']*1000:.3f} mm" if r['passed'] else 'Not attempted'
        lines.append(f"| {label} | {'PASS' if baseline[name]['passed'] else 'FAIL'} | {outcome} | {r['alignment_wait_s']:.2f} s | {error} | {lift} |")
    lines += ['', 'In recovered X cases the logged 64 ms preclosure samples showed zero finger/object contacts, and less than 0.001 mm sphere settling displacement on the stand in the idealized model. All three maintained three contacting digits during the unsupported hold, checked each 8 ms physics step. Requiring the settling window matters: the preserved first attempt used the old 6 s closing time, found 3.59–7.25 mm position error, and correctly stayed open.', '',
              '## Guard and negative controls', '',
              '- Y -10/+10 mm: declined because this local planner has no lateral/heading repositioning stage. Both remained balanced, open, and contact-free with the sphere on its stand.',
              '- Height +10 mm and X +20 mm: declined by the local admission limits; both guard tests passed without pickup.',
              '- Actual object X +10 mm with a deliberately incorrect supplied target of zero: the controller attempted the wrong position and failed the lift criteria, as expected. The simulator reported a failed grasp, not a success. This confirms that favorable target input matters and that the controller does not secretly recover from the live sphere position.',
              '- X +10 mm repeat: full cycle passed again.', '',
              'Admission limits of 15 mm X and 1 mm Y/Z are planning restrictions, not validated capture tolerance bands. Tests cover the listed individual points only. Sideways refusal does not mean Alpha can never reach those objects: a future differential-drive heading/repositioning plan can address them.', '',
              '## Regression, evidence and next work', '',
              'Revalidated all seven load/lean-guard cases with the final controller: five accepted grasps and two rejected-load controls. Revalidated the reference/open-claw control and robot-only grasp path audit. The original 13-case base/empty-claw source remains unchanged. See LOADED_CLEARANCE_RESULTS.md and GRASP_RESULTS.md for those checks.', '',
              'All 22 capture-study physics trials retain configurations, setups, worlds, reports, telemetry, logs and actual renders. Baseline and fixed-time trial source snapshots live under experiments/capture_alignment_*/source_snapshot; final reports are checked against current source/world hashes. The previous loaded-motion milestone and the offline arm-only IK screen are archived separately.', '',
              f'![Measured wheel alignment and closing times]({(RESULTS/"capture-alignment.png").as_posix()})', '',
              'Next: integrate a collision-checked transition from the resting arm pose and a broader base approach, then replace supplied target coordinates with an explicit sensing pipeline. Unknown payloads, simultaneous offsets, sensor noise/delay, arbitrary obstacles, continuous operating ranges and hardware transfer remain open. No parts purchased.', '']
    (RESULTS/'CAPTURE_ALIGNMENT_RESULTS.md').write_text('\n'.join(lines), encoding='utf-8')
    sys.path.insert(0, str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for ax, name in zip(axes, recovered):
        folder = RESULTS/'capture_alignment_validated'/name
        with (folder/'telemetry.csv').open(newline='', encoding='utf-8') as stream:
            data = list(csv.DictReader(stream))
        times = [float(r['time_s']) for r in data]
        ax.plot(times, [float(r['wheel_odometry_m'])*1000 for r in data], color='#13786b', label='Wheel odometry')
        ax.plot(times, [float(r['position_reference_m'])*1000 for r in data], '--', color='#a65a32', label='Supplied target')
        r = final[name]
        ax.axvline(r['schedule_actual_s']['close'], color='#556e9c', linestyle=':', label='Close after settling')
        ax.set_ylabel(f"X {r['experiment']['placement_offset_world_m'][0]*1000:+g} mm\nposition (mm)")
        ax.grid(alpha=.2)
        ax.legend(loc='upper right', fontsize=8)
    axes[-1].set_xlabel('Simulated time (s)')
    fig.suptitle('Alpha: align with the wheels, settle, then grasp')
    fig.text(.5, .015, 'Supplied coordinates and wheel odometry | prepared open arm | no camera alignment | simulation only', ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .04, 1, .96))
    fig.savefig(RESULTS/'capture-alignment.png', dpi=160)
    plt.close(fig)
    print(json.dumps({k: summary[k] for k in ['successful_alignment_grasps', 'guard_controls_passed', 'wrong_target_expected_miss', 'physics_trials_this_study']}, indent=2))


if __name__ == '__main__':
    main()
