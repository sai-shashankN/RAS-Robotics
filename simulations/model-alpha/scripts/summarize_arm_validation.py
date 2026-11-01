"""Summarize recorded empty-arm runs; reject evidence from stale source/config."""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL / 'results'


def main():
    if len(json.loads((MODEL/'parameters.json').read_text()).get('finger_angles_rad', [0, 1, 2]))==2:
        from summarize_two_finger import main as current_summary
        return current_summary()
    p = json.loads((MODEL / 'parameters.json').read_text())
    paths = [WORKSPACE / 'simulations/common' / name for name in
             ['robot_controller.py', 'generate_worlds.py', 'control_math.py', 'mass_properties.py']] + [MODEL / 'parameters.json']
    hashes = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    rows = []
    for scenario in list(p['dynamic_arm_targets']) + ['pose_guard']:
        report = json.loads((RESULTS / 'dynamics' / scenario / 'report.json').read_text())
        if report.get('source_sha256') != hashes or report['scenario'] != scenario:
            raise ValueError(f'Stale or wrong report: {scenario}')
        if scenario == 'pose_guard':
            with (RESULTS / 'dynamics' / scenario / 'telemetry.csv').open(newline='') as stream:
                guard_data = list(csv.DictReader(stream))
            guard_error = max(abs(float(row[j+'_rad'])-p['poses']['folded'][j]) for row in guard_data for j in ['shoulder', 'elbow', 'wrist'])
            if guard_error >= 0.03:
                raise ValueError('Guard test moved away from folded during recorded telemetry')
        rows.append({'scenario': scenario, 'passed': report['dynamics_validation_passed'],
                     'duration_s': report['time_s'], 'peak_pitch_deg': math.degrees(report['max_abs_pitch_rad']),
                     'peak_pitch_error_deg': math.degrees(report['max_pitch_error_rad']),
                     'final_drift_mm': report['final_drift_m']*1000,
                     'min_arm_floor_clearance_mm': report['min_arm_floor_clearance_m']*1000,
                     'max_com_model_error_mm': report['max_com_model_error_m']*1000,
                     'peak_wheel_torque_nm': report['max_applied_wheel_torque_nm'], 'max_arm_contact_count': report['max_arm_contact_count'],
                     'criteria': report['dynamics_checks'], 'checkpoints': report['checkpoints']})
    sys.path.insert(0, str(WORKSPACE / 'simulations/common'))
    from mass_properties import mass_properties
    pose_leans = {name: math.degrees(mass_properties(p, name)['lean_to_align_com_rad']) for name in p['poses']}
    summary = {'date': '2026-10-23', 'all_arm_cases_passed': all(row['passed'] for row in rows),
               'cases': rows, 'source_sha256': hashes, 'geometry_pose_com_alignment_lean_deg': pose_leans,
               'targets_rad': p['dynamic_arm_targets'], 'main_joint_target_rate_rad_s': p['dynamic_arm_rate_rad_s'],
               'lean_guard_rad': p['dynamic_lean_limit_rad'], 'pose_guard_max_folded_joint_error_rad': guard_error}
    (RESULTS / 'arm-validation-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    reach_row = next(row for row in rows if row['scenario'] == 'arm_reach')
    raised_z, folded_z = [c['palm_position_m'][2] for c in reach_row['checkpoints']]
    lines = ['# Alpha empty-arm balance results', '', 'Recorded: 23 October 2026 (Asia/Calcutta), Webots R2025a.', '',
             'Four 48-second free-body tests move the shoulder, elbow, wrist, and then all three together. Each starts folded, deploys at 6 s, holds, and returns at 26 s. Main-joint targets move at at most 0.08 rad/s (4.6 degrees/s), using synchronized linear interpolation. Fingers retain their folded settings; there is no payload.', '',
             '| Test | Result | Peak body lean (deg) | Peak lean error (deg) | Final drift (mm) |',
             '| --- | --- | ---: | ---: | ---: |']
    for row in rows:
        lines.append(f"| {row['scenario']} | {'PASS' if row['passed'] else 'FAIL'} | {row['peak_pitch_deg']:.3f} | {row['peak_pitch_error_deg']:.3f} | {row['final_drift_mm']:.2f} |")
    lines += ['', '## What changed', '',
              'The controller calculates a quasi-static balance lean from measured joint angles and the assumed link masses. It follows that lean using the IMU, gyro, and wheel encoders, with reference-rate compensation. Simulator COM data is used only to check the mass calculation, not for balance feedback. The free-body root is never repositioned by the controller.', '',
              f'The coordinated case is a compact raised reach, not full forward extension. Its palm rises from approximately {folded_z:.3f} m to {raised_z:.3f} m above the floor while the body leans backward. Total mass and actuator limits remain unchanged. The wrist standoff, resting wrist/finger angles, and assumed rearward body COM were revised for clearance; see CLAW_AND_CLEARANCE_RESULTS.md.', '',
              '## Checks declared before the runs', '',
              'Complete the duration without falling; peak body pitch below 20 degrees; final pitch below 1 degree; final planar drift below 5 cm; wheel speed within 14 rad/s plus 0.5 rad/s numerical allowance. Moving-arm cases must reach both target/folded checkpoints within 0.03 rad per main joint, keep every generated arm collision primitive over 1 cm above the floor, keep pitch-reference error under 5 degrees, and match the simulator COM within 1 mm. Actual modeled lean must stay inside the 0.30 rad guard.', '',
              'A separate 30-second pose_guard test requests the geometry-only forward and lower poses, verifies that both are rejected, and checks that the arm remains folded. This tests the guard; it is not a failed physical reach trial.', '',
              '## Workspace limitation found', '',
              f"The current forward and lower geometry poses require calculated COM-alignment leans of {abs(pose_leans['reach_forward']):.1f} and {abs(pose_leans['reach_lower']):.1f} degrees with this assumed mass layout. They are outside the 17.2-degree command guard. Their earlier kinematic success never established dynamic reachability.", '',
              'The guard samples 101 points on the commanded joint-space path. It is a quasi-static lean check, not a collision checker or a guarantee of transient stability. Measured joint feedback and bounded wheel torques remain necessary.', '',
              '## Remaining work', '',
              '- Self-collision is now enabled for nonadjacent solids; these runs require zero detected arm contacts. Direct joint neighbors and a full arbitrary workspace still need checking.',
              '- No payload, finger opening/closing, object grasping, simultaneous driving with arm motion, or noisy/delayed sensing was tested here.',
              '- Masses and actuators are design assumptions. This does not establish real motor heating, backlash, electrical behavior, structural strength, or hardware readiness.', '',
              'Telemetry, screenshots, criteria, and source/configuration hashes are retained beside the summaries. The preceding folded-arm source/configuration and evidence are archived in ../archive/folded-base-control-milestone.', '']
    (RESULTS / 'ARM_CONTROL_RESULTS.md').write_text('\n'.join(lines), encoding='utf-8')

    packages = WORKSPACE / '.local-tools/plotting-packages'
    if packages.exists():
        sys.path.insert(0, str(packages))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for ax, scenario in zip(axes.flat, p['dynamic_arm_targets']):
        with (RESULTS / 'dynamics' / scenario / 'telemetry.csv').open(newline='') as stream:
            data = list(csv.DictReader(stream))
        times = [float(row['time_s']) for row in data]
        ax.plot(times, [math.degrees(float(row['pitch_rad'])) for row in data], color='#2563a6', label='Simulated body lean', linewidth=1.8)
        ax.plot(times, [math.degrees(float(row['lean_reference_rad'])) for row in data], '--', color='#d76e28', label='COM-based reference', linewidth=1.2)
        ax.axvline(6, color='#89939c', alpha=.4, linewidth=1)
        ax.axvline(26, color='#89939c', alpha=.4, linewidth=1)
        ax.set(title=scenario.replace('arm_', '').replace('_', ' ').capitalize() + (' (compact raised reach)' if scenario == 'arm_reach' else ''), xlabel='Simulation time (s)', ylabel='Pitch (degrees)')
        ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
        ax.legend(fontsize=8)
    fig.suptitle('Model Alpha: moving an empty arm while balancing', fontsize=15)
    fig.text(.5, .02, 'Deploy at 6 s; return at 26 s | provisional mass layout | no payload | self-collision enabled | specified paths only', ha='center', fontsize=9, color='#555f68')
    fig.tight_layout(rect=(0, .05, 1, .95))
    fig.savefig(RESULTS / 'arm-control-validation.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'all_arm_cases_passed': summary['all_arm_cases_passed'], 'tests': len(rows)}, indent=2))
    if not summary['all_arm_cases_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
