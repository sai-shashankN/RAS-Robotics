"""Join actual collision/finger evidence and render a standalone results plot."""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'


def main():
    if len(json.loads((MODEL/'parameters.json').read_text()).get('finger_angles_rad', [0, 1, 2]))==2:
        from summarize_two_finger import main as current_summary
        return current_summary()
    p = json.loads((MODEL/'parameters.json').read_text(encoding='utf-8'))
    sources = [WORKSPACE/'simulations/common'/name for name in ['robot_controller.py', 'generate_worlds.py', 'control_math.py', 'mass_properties.py']] + [MODEL/'parameters.json']
    hashes = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    scenarios = ['balance', 'tilt_positive', 'tilt_negative', 'push', 'steering', 'drive'] + list(p['dynamic_arm_targets']) + ['pose_guard', 'claw_cycle_folded', 'claw_cycle_raised']
    reports = {}
    for name in scenarios:
        folder = RESULTS/'dynamics' if name == 'balance' else RESULTS/'dynamics'/name
        r = json.loads((folder/'report.json').read_text())
        if r.get('source_sha256') != hashes:
            raise ValueError(f'Stale report: {name}')
        if not r['dynamics_validation_passed'] or not r['self_collision_enabled'] or r['max_arm_contact_count'] != 0:
            raise ValueError(f'Failed collision/dynamics test: {name}')
        reports[name] = r
    audit = json.loads((RESULTS/'clearance/report.json').read_text())
    for name, sha in audit['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest() != sha:
            raise ValueError(f'Stale clearance audit: {name}')
    if not audit['all_defined_paths_clear']:
        raise ValueError('Conservative clearance audit failed')
    geometry = json.loads((RESULTS/'geometry/report.json').read_text())
    if geometry.get('source_sha256') != hashes or not geometry['geometry_joint_and_fk_check_passed']:
        raise ValueError('Geometry/FK regression failed or stale')
    rows = []
    for name in ['claw_cycle_folded', 'claw_cycle_raised']:
        r = reports[name]
        rows.append({'scenario': name, 'duration_s': r['time_s'], 'checkpoints': len(r['checkpoints']),
                     'peak_pitch_deg': math.degrees(r['max_abs_pitch_rad']),
                     'final_drift_mm': 1000*r['final_drift_m'],
                     'max_joint_checkpoint_error_rad': max(max(c['joint_error_rad'].values()) for c in r['checkpoints']),
                     'max_finger_tip_fk_error_m': r['max_finger_tip_fk_error_m'],
                     'min_arm_floor_clearance_mm': 1000*r['min_arm_floor_clearance_m'],
                     'arm_contact_count': r['max_arm_contact_count'], 'criteria': r['dynamics_checks']})
    summary = {'date': '2026-10-23', 'all_claw_and_clearance_checks_passed': True, 'dynamic_cases_passed': len(reports),
               'source_sha256': hashes, 'claw_cases': rows, 'clearance_paths': audit['paths'],
               'direct_joint_pairs_excluded': audit['excluded_direct_joint_pairs'],
               'geometry_regression_passed': True}
    (RESULTS/'claw-validation-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    lines = ['# Alpha claw and clearance results', '', 'Recorded: 23 October 2026 (Asia/Calcutta), Webots R2025a.', '',
             '**Result:** 13 free-body cases passed with self-collision enabled and zero detected arm contacts. This includes the preceding 11 cases repeated with the revised geometry, plus two empty-claw cycling tests. The four-pose kinematic geometry/FK regression also passed.', '',
             '## Problems found and corrections', '',
             'The first self-collision-enabled run exposed forearm/finger-0 contact and collisions among the fingertips in the old folded pose. The robot remained balanced, but the contact check failed. That trial and its source/configuration are preserved in archive/self-collision-first-failure. Earlier collision-disabled runs remain historical control evidence; they did not establish a usable claw pose.', '',
             'Added a 15 mm wrist-to-palm standoff, changed the resting wrist pitch from 1.54 to 1.10 rad to point the claw farther forward, and reduced empty-claw closure from proximal/distal angles 0.15/0.50 rad to 0.00/0.25 rad. The first standoff-only conservative audit still flagged the shoulder/tips; that intermediate configuration is archived in archive/wrist-standoff-clearance-trial.', '',
             f"The hypothetical inner-body COM moved from 29.6 to {abs(p['body_com_x'])*1000:.1f} mm behind the axle to center the revised folded pose. Total modeled mass remains 1.223 kg, including the standoff in the assumed palm mass. No actuator torque limits were increased and no components were selected or bought.", '',
             '## Finger tests', '',
             'All three proximal and three distal hinges open and curl twice, first in the resting arm posture and then after deploying to the compact raised posture. Finger target speed is at most 0.30 rad/s. The raised test returns the arm to its resting posture.', '',
             '| Test | Duration (s) | Checkpoints | Peak lean (deg) | Final drift (mm) | Arm contacts |',
             '| --- | ---: | ---: | ---: | ---: | ---: |']
    for row in rows:
        lines.append(f"| {row['scenario']} | {row['duration_s']:.0f} | {row['checkpoints']} | {row['peak_pitch_deg']:.3f} | {row['final_drift_mm']:.2f} | {row['arm_contact_count']} |")
    lines += ['', 'Declared checks: every main/finger joint within 0.03 rad at each settled checkpoint; independently calculated distal endpoints within 2 mm of simulator positions; zero arm contacts; floor clearance over 1 cm; COM calculation within 1 mm; pitch-reference error below 5 degrees; peak body pitch below 20 degrees; return within 1 degree and 5 cm; no rejected pose command; wheel speeds within the existing limit. The raised run lasts 72 s and the resting run 48 s.', '',
              '## Conservative clearance audit', '',
              'A separate oriented-box separating-axis calculation encloses the body tube, wheels, camera/lower housing placeholders, links, palm/standoff, and fingers including the blue pads. Positive separating-axis gaps certify separation of those envelopes. A downstream-radius motion bound accounts for movement between joint-space samples. The reported bound is the sampled separating-axis gap minus that motion allowance, rather than an exact nearest-surface distance.', '',
              '| Defined path | Conservative nonadjacent clearance bound (mm) |', '| --- | ---: |']
    for name, row in audit['paths'].items():
        lines.append(f"| {name} | {1000*row['conservative_clearance_bound_m']:.2f} |")
    lines += ['', 'The six audited paths are folded-to-each-tested-arm-target and open/closed finger sweeps in the resting and raised postures. Reversing a path has the same geometric clearance. This covers these specified joint-space paths, not arbitrary commands.', '',
              'Webots omits collision response between directly joined solids even when selfCollision is enabled. The audit explicitly excludes those same owner pairs, including shoulder/body, elbow neighbors, wrist neighbors, and finger hinge neighbors. Joint pockets, bearing/motor housings, and mounting interference still need detailed CAD. [Official Webots R2025a behavior](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/robot.md).', '',
              '## Readiness for the next experiment', '',
              'Ready for a first controlled simulated grasp attempt in the compact workspace, using a light known object on a stand. Ground-level pickup and full forward extension are not established. The next experiment must verify real contact, object retention during a small lift, and release without attaching or teleporting the object.', '',
              'No payload was used in these results. Friction, pad compliance, finger coupling, sensing delay/noise, manufacturing tolerances, structural strength, motor electrical/thermal behavior, and hardware transfer remain unvalidated. The approximately 1.9 mm smallest envelope margin is a modeling result, not a hardware clearance allowance.', '',
              'Source hashes, acceptance criteria, telemetry, and actual simulator renders are retained with the JSON summaries. The prior complete empty-arm milestone is preserved under archive/empty-arm-milestone.', '']
    (RESULTS/'CLAW_AND_CLEARANCE_RESULTS.md').write_text('\n'.join(lines), encoding='utf-8')

    sys.path.insert(0, str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    for column, scenario in enumerate(['claw_cycle_folded', 'claw_cycle_raised']):
        with (RESULTS/'dynamics'/scenario/'telemetry.csv').open(newline='') as stream:
            data = list(csv.DictReader(stream))
        times = [float(row['time_s']) for row in data]
        for i, color in enumerate(['#2563a6', '#d76e28', '#23856e']):
            for part, style in [('finger', '-'), ('distal', '--')]:
                axes[0, column].plot(times, [math.degrees(float(r[f'{part}_{i}_rad'])) for r in data], style, color=color,
                                     label=f'{part} {i}', linewidth=1.4, alpha=.75)
        axes[0, column].set(title='Claw in '+('resting' if column == 0 else 'raised')+' posture', ylabel='Finger joint angle (degrees)')
        axes[0, column].legend(fontsize=8, ncol=2)
        axes[1, column].plot(times, [math.degrees(float(r['pitch_rad'])) for r in data], color='#2563a6', label='Simulated body lean')
        axes[1, column].plot(times, [math.degrees(float(r['lean_reference_rad'])) for r in data], '--', color='#d76e28', label='COM reference')
        axes[1, column].set(ylabel='Body pitch (degrees)')
        axes[1, column].legend(fontsize=8)
    for ax in axes.flat:
        ax.set_xlabel('Simulation time (s)')
        ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    fig.suptitle('Model Alpha: empty claw cycles with self-collision enabled', fontsize=15)
    fig.text(.5, .015, 'All six finger hinges logged | zero arm contacts detected | specified paths only | no object grasp yet', ha='center', fontsize=9, color='#555f68')
    fig.tight_layout(rect=(0, .04, 1, .95))
    fig.savefig(RESULTS/'claw-control-validation.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'dynamic_cases_passed': len(reports), 'clearance_paths_passed': len(audit['paths']), 'geometry_regression_passed': True}, indent=2))


if __name__ == '__main__':
    main()
