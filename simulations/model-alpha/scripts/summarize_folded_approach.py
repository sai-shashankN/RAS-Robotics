"""Verify folded pickup missions, failure controls, clearance and regressions."""

from pathlib import Path
import csv
import hashlib
import json
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'


def verify(r, resolve):
    for name, digest in r['source_sha256'].items():
        if hashlib.sha256(resolve(name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Stale source '+name)
    if r.get('world_path'):
        world = str(Path(r['world_path']).relative_to(WORKSPACE))
        if hashlib.sha256(resolve(world).read_bytes()).hexdigest()!=r['world_sha256']:
            raise ValueError('Stale world '+world)


def archived(archive, name):
    path = Path(name)
    if path.parts[:2]==('simulations','common'):
        return archive/'common'/Path(*path.parts[2:])
    if path.parts[:2]==('simulations','model-alpha'):
        return archive/Path(*path.parts[2:])
    raise ValueError(name)


def main():
    study = 'folded_approach_validated'
    manifest = json.loads((MODEL/'experiments'/study/'manifest.json').read_text(encoding='utf-8'))
    status = json.loads((RESULTS/study/'run-status.json').read_text(encoding='utf-8'))
    if status['infrastructure_errors'] or len(status['results'])!=9:
        raise ValueError('Mission study incomplete')
    rows = {}
    for entry in manifest['cases']:
        r = json.loads((RESULTS/study/entry['case']/'report.json').read_text(encoding='utf-8'))
        verify(r, lambda name: WORKSPACE/name)
        if r['experiment']!=json.loads((WORKSPACE/entry['experiment_path']).read_text(encoding='utf-8')):
            raise ValueError('Wrong mission input '+entry['case'])
        rows[entry['case']] = r
    positives = ['reference_50mm','approach_40mm','approach_60mm','mass_5g_50mm','reference_repeat']
    if not all(rows[name]['passed'] for name in positives):
        raise ValueError('Positive folded mission failed')
    open_control = rows['open_claw_control']
    wrong = rows['wrong_target_control']
    load = rows['load_rejection_control']
    timeout = rows['alignment_timeout_control']
    if not open_control['negative_control_passed'] or open_control['passed']:
        raise ValueError('Open claw control failed')
    if wrong['passed'] or wrong['checks']['object_lifted_above_stand'] or wrong['checks']['arm_motion_added_at_least_6_mm_lift']:
        raise ValueError('Wrong target was not a missed grasp')
    if not load['load_guard_passed'] or load['passed'] or load['max_body_drift_m']>.002:
        raise ValueError('Load preflight did not stay folded')
    if not timeout['alignment_guard_passed'] or not timeout['alignment_failed_to_settle'] or timeout['passed']:
        raise ValueError('Alignment timeout control failed')
    with (RESULTS/study/'alignment_timeout_control/telemetry.csv').open(newline='',encoding='utf-8') as stream:
        data = list(csv.DictReader(stream))
    late = [r for r in data if float(r['time_s'])>=timeout['schedule_actual_s']['close']+1]
    if not late or any(abs(float(r['finger_0'])+.45)>.03 or abs(float(r['distal_0'])-.1)>.03 for r in late):
        raise ValueError('Timeout did not retain the open claw')
    if any(r['unexpected_contact_samples'] for r in rows.values()):
        raise ValueError('Unintended robot contact in mission/control')
    audits = {}
    for name in ['reference_50mm','approach_40mm','approach_60mm']:
        audit = json.loads((RESULTS/study/name/'clearance.json').read_text(encoding='utf-8'))
        verify(audit, lambda path: WORKSPACE/path)
        if not audit['all_robot_self_paths_clear'] or not audit['all_nominal_fixture_samples_separated']:
            raise ValueError('Clearance screen failed '+name)
        audits[name] = audit
    # Keep the first failed report and preceding seven-case pass sequence verifiable.
    trial = json.loads((RESULTS/'folded_approach_trial/reference/report.json').read_text(encoding='utf-8'))
    snapshot = MODEL/'experiments/folded_approach_trial/source_snapshot'
    verify(trial, lambda name: snapshot/name if (snapshot/name).exists() else WORKSPACE/name)
    archive = MODEL/'archive/folded-approach-seven-case-check'
    prior_count = 0
    for path in (archive/'results/folded_approach_validated').glob('*/report.json'):
        verify(json.loads(path.read_text(encoding='utf-8')), lambda name: archived(archive,name))
        prior_count += 1
    if prior_count!=7:
        raise ValueError('Seven-case checkpoint archive incomplete')
    # All active prepared-grasp regression reports must match final sources too.
    regression_count = 0
    for study_name in ['loaded_clearance_validation','capture_alignment_validated','capture_alignment_controls']:
        for path in (RESULTS/study_name).glob('*/report.json'):
            verify(json.loads(path.read_text(encoding='utf-8')), lambda name: WORKSPACE/name)
            regression_count += 1
    for name in ['grasp','grasp_negative']:
        verify(json.loads((RESULTS/name/'report.json').read_text(encoding='utf-8')),lambda path: WORKSPACE/path)
        regression_count += 1
    if regression_count!=19:
        raise ValueError('Prepared regression set incomplete')
    for name, required in [('grasp-validation-summary.json','prepared_grasp_passed'),
                           ('capture-alignment-summary.json','load_regression_and_reference_controls_passed')]:
        if not json.loads((RESULTS/name).read_text(encoding='utf-8'))[required]:
            raise ValueError('Regression summary failed '+name)
    load_summary = json.loads((RESULTS/'loaded-clearance-summary.json').read_text(encoding='utf-8'))
    if load_summary['accepted_grasp_passes']!=5 or load_summary['safe_rejections']!=2:
        raise ValueError('Load regression failed')
    compact = lambda r: {k:r.get(k) for k in ['case','passed','validation_passed','checks','time_s','alignment_wait_s',
                    'max_pitch_deg','max_pitch_error_deg','final_body_position_error_m','unexpected_contact_samples',
                    'deployment_tracking_error_rad','preclose_max_contacts','preclose_max_object_displacement_m',
                    'arm_added_lift_m','hold_time_s','min_hold_contacting_fingers','negative_control_passed',
                    'load_guard_passed','alignment_guard_passed']}
    summary = {'date':'2026-10-24','complete_mission_passes':5,'final_case_count':9,
               'open_claw_control_passed':True,'wrong_target_expected_miss':True,'load_rejection_passed':True,
               'alignment_timeout_passed':True,'prepared_regression_reports_verified':regression_count,
               'historical_trial_reports_verified':8,'cases':[compact(r) for r in rows.values()],
               'source_sha256':rows['reference_50mm']['source_sha256']}
    (RESULTS/'folded-approach-summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Alpha: folded start to straight approach and pickup','',
             'Recorded: 24 October 2026 (Asia/Calcutta), Webots R2025a.','',
             '**Five complete missions passed:** start folded, raise the closed claw, open, approach on the wheels, settle, grasp, lift, hold unsupported, lower and release. Tested straight approaches are 40/50/60 mm with the 2 g sphere, plus 5 g at 50 mm and a repeated 2 g/50 mm reference.','',
             '## Staging and motion','',
             'The robot starts at its folded-pose empty-COM lean. The free sphere is placed on the stand at the intended raised-claw capture location plus the specified approach distance; it is not placed at the initial folded claw. A 3 s initial hold precedes synchronized deployment. The claw is raised closed, allowed to settle, opened, and allowed to settle again. Wheel approach starts at 20.375 s.','',
             'A 6 s smooth longitudinal position reference uses the same wheel torque/speed caps and grasp feedback gains. Closure waits for the existing 0.5 mm / 3 mm/s measured settling window. The full grasp schedule shifts with that wait. The default alignment deadline is 40 s after staging; a shorter deadline is exercised in a failure control. The folded-mode command cap is 75 mm, an experimental admission restriction; only the listed 40/50/60 mm distances are validated. The robot finishes with the claw raised/open after release; returning to folded is not part of this mission.','',
             'Target coordinates and payload mass are supplied inputs. No camera, object position steering, object pose reset, attachment, arm-supported locomotion, or base heading plan is used. The reference object remains a rigid 20 mm sphere with assumed friction 0.6. Geometry, masses, arm/joint torque limits and the earlier base controller are unchanged.','',
             f'![Actual Webots frame during unsupported holding]({(RESULTS/study/"reference_50mm/24_lift_hold.png").as_posix()})','',
             '## Final cases','',
             '| Case | Outcome | Total simulated time | Peak pitch | Endpoint error | Added arm lift | Unintended robot contact samples |',
             '| --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for name,r in rows.items():
        outcome = 'MISSION PASS' if r['passed'] else 'OPEN-CLAW CONTROL PASS' if name=='open_claw_control' else 'EXPECTED MISS' if name=='wrong_target_control' else 'LOAD REJECTED' if name=='load_rejection_control' else 'TIMEOUT: OPEN CLAW'
        lift = f"{r['arm_added_lift_m']*1000:.2f} mm" if r['passed'] else 'No successful pickup'
        lines.append(f"| {name} | {outcome} | {r['time_s']:.2f} s | {r['max_pitch_deg']:.2f} deg | {r['final_body_position_error_m']*1000:.3f} mm | {lift} | {r['unexpected_contact_samples']} |")
    lines += ['', 'Positive missions require three tracked staging checkpoints within 0.03 rad, the object undisturbed with zero finger contact before closing (checked every physics step), and all preceding grip/lift/hold/release/balance checks. Endpoint error must be under 5 mm from the commanded destination; intentional approach travel is recorded separately from error. A plan rejected before deployment keeps its destination at the starting position. Unsupported hold requires at least two digits, positive stand clearance, at least 6 mm added arm lift and bounded slip.','',
              '## Contact and clearance checks','',
              'Mission monitoring covers all nine arm solids, the root body and both wheel solids. Intended finger/sphere contacts are allowed. Wheel-floor contact is allowed at/below the floor outside the known fixture projection, including normal solver penetration; fixture/body contacts remain failures. This contact witness is specific to the flat-floor fixture world, not a general collision classifier.','',
              'Nonadjacent self-clearance passes deployment and opening with the existing conservative motion bound. Three separate 40/50/60 mm fixture screens sample 501 poses each for deployment, opening and approach at nominal empty-COM lean. All sampled conservative envelopes separate from the stand, sphere and tray, with positive nonwheel floor gap. These world screens do not bound dynamic tracking or between-sample gaps; contact monitoring provides additional run-specific evidence. Direct joint neighbors still need CAD checks.','',
              '## Failure controls and preserved trials','',
              '- The open-claw mission deploys and approaches, but the sphere remains on its stand and positive lift checks fail.','- Actual 60 mm placement with a supplied 50 mm target produces a missed grasp, as expected; it is not counted as a pickup.','- A known 20 g load exceeds the existing lean plan, so the controller stays folded without driving or grasping.','- An intentionally shortened alignment deadline cancels grasping and leaves the raised claw open; it does not fold back into the nearby fixture. The base position loop retains its supplied destination, so this control is not an emergency drive stop.','',
              'The first trial completed the physical sequence but failed two measurement rules: normal penetrating wheel-floor contacts were counted as obstacles, and intended 50 mm travel exceeded a stationary-base drift check. Its original report and sources remain frozen. The corrected seven-case checkpoint is also archived before the final failure-control revision. None of these failed or intermediate reports is overwritten as a final pass.','',
              '## Regression and next work','',
              'All 19 current prepared-grasp reports were rerun and verified against final sources: seven load/lean-guard trials, ten alignment/control trials, and the reference/open-claw pair. Their expected pass/refusal/miss outcomes remain intact. The original 13-case base/empty-claw source hashes are unchanged. Reports, telemetry, actual renders, configurations, worlds, audits and historical sources are retained.','',
              f'![Folded deployment, approach and object lift]({(RESULTS/"folded-approach.png").as_posix()})','',
              'Next: validate retreat and folding after release, then add a heading/lateral approach stage and explicit real target sensing. Arbitrary obstacles, unknown loads, broad combined-factor robustness, actuator electrical/thermal behavior and hardware transfer remain open. No parts purchased.','']
    (RESULTS/'FOLDED_APPROACH_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ref = rows['reference_50mm']
    with (RESULTS/study/'reference_50mm/telemetry.csv').open(newline='',encoding='utf-8') as stream:
        data = list(csv.DictReader(stream))
    t = [float(r['time_s']) for r in data]
    fig,axes = plt.subplots(3,1,figsize=(11,8),sharex=True)
    for j in ['shoulder','elbow','wrist']:
        axes[0].plot(t,[float(r[j]) for r in data],label=j)
    axes[0].set_ylabel('Joint angle (rad)'); axes[0].legend(loc='upper left',ncol=3,fontsize=8)
    axes[1].plot(t,[float(r['wheel_odometry_m'])*1000 for r in data],label='Wheel odometry',color='#14776a')
    axes[1].plot(t,[float(r['position_reference_m'])*1000 for r in data],'--',label='Supplied target',color='#a95832')
    axes[1].set_ylabel('Base travel (mm)'); axes[1].legend(loc='upper left',fontsize=8)
    z0 = ref['setup']['initial_object_position_m'][2]
    axes[2].plot(t,[(float(r['object_z_m'])-z0)*1000 for r in data],label='Sphere rise from stand',color='#b57816')
    axes[2].set_ylabel('Object rise (mm)'); axes[2].set_xlabel('Simulated time (s)')
    for ax in axes:
        for when in [ref['approach_plan']['ready_s'],ref['schedule_actual_s']['close'],ref['schedule_actual_s']['release']]:
            ax.axvline(when,color='#667991',linestyle=':',linewidth=.9)
        ax.grid(alpha=.2)
    fig.suptitle('Alpha: folded start, raised claw, wheel approach, then pickup')
    fig.text(.5,.015,'Supplied target | reference 50 mm / 2 g mission | dotted lines: approach start, close, release | simulation only',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.04,1,.96)); fig.savefig(RESULTS/'folded-approach.png',dpi=160); plt.close(fig)
    print(json.dumps({k:summary[k] for k in ['complete_mission_passes','final_case_count','prepared_regression_reports_verified','historical_trial_reports_verified']},indent=2))


if __name__=='__main__':
    main()
