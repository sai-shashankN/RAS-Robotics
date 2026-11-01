"""Verify provenance and summarize the prepared-pose grasp and open-claw control."""

from pathlib import Path
import csv
import hashlib
import json
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'


def read_case(folder):
    report = json.loads((RESULTS/folder/'report.json').read_text(encoding='utf-8'))
    for name, sha in report['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest() != sha:
            raise ValueError(f'Stale evidence: {folder}, {name}')
    with (RESULTS/folder/'telemetry.csv').open(newline='') as stream:
        data = [{key: (value if key == 'stage' else float(value)) for key, value in row.items()} for row in csv.DictReader(stream)]
    return report, data


def main():
    report, data = read_case('grasp')
    control, control_data = read_case('grasp_negative')
    if not report['passed'] or not report['validation_passed'] or report['negative_control']:
        raise ValueError('Positive grasp test failed')
    if control['passed'] or not control['negative_control_passed'] or not control['negative_control']:
        raise ValueError('Open-claw control did not reject a non-grasp')
    if report['source_sha256'] != control['source_sha256']:
        raise ValueError('Positive/control source or configuration differs')
    audit = json.loads((RESULTS/'grasp/robot-clearance-audit.json').read_text())
    for name, sha in audit['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest() != sha:
            raise ValueError(f'Stale grasp-path clearance audit: {name}')
    if not audit['all_passed']:
        raise ValueError('Grasp-path robot clearance failed')
    # Existing base/empty-claw source is unchanged by this isolated experiment.
    baseline = json.loads((RESULTS/'claw-validation-summary.json').read_text())
    for name, sha in baseline['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest() != sha:
            raise ValueError(f'Baseline evidence needs updating: {name}')
    summary = {'date': '2026-10-24', 'original_checkpoint_date': '2026-10-23', 'prepared_grasp_passed': True, 'open_claw_control_passed': True,
               'baseline_13_case_source_unchanged': True, 'source_sha256': report['source_sha256'],
               'positive_checks': report['checks'], 'negative_expected_failures': [name for name, passed in control['checks'].items() if not passed],
               'object_mass_kg': report['experiment']['object_mass_kg'], 'object_radius_m': report['experiment']['object_radius_m'],
               'hold_time_s': report['hold_time_s'], 'min_stand_gap_m': report['min_hold_stand_gap_m'],
               'arm_added_lift_m': report['arm_added_lift_m'], 'minimum_contacting_fingers': report['min_hold_contacting_fingers'],
               'max_hold_relative_slip_m': report['max_hold_relative_slip_m'], 'final_body_drift_m': report['final_body_drift_m'],
               'max_body_pitch_deg': report['max_pitch_deg'], 'max_pitch_reference_error_deg': report['max_pitch_error_deg'],
               'unexpected_arm_contact_samples': report['unexpected_contact_samples'],
               'control_max_sphere_displacement_m': control['max_object_displacement_m'],
               'control_max_contacting_fingers': control['max_contacting_fingers'],
               'clearance_paths': audit['paths']}
    (RESULTS/'grasp-validation-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    text = f'''# Alpha: first prepared-pose contact grasp

Original checkpoint: 23 October 2026; revalidated 24 October 2026 (Asia/Calcutta), Webots R2025a.

**Passed:** grasp a free 2 g, 20 mm diameter sphere, lift, hold unsupported for {report['hold_time_s']:.1f} s, lower, and release into a catch tray while balancing. An identical arm-motion control with the claw left open produced no grasp or lift; the sphere stayed on its stand.

| Measurement | Recorded result |
| --- | ---: |
| Minimum sphere-bottom clearance above stand during hold | {report['min_hold_stand_gap_m']*1000:.2f} mm |
| Additional sphere rise from arm motion, measured after closure | {report['arm_added_lift_m']*1000:.2f} mm |
| Sustained unsupported hold | {report['hold_time_s']:.1f} s |
| Minimum contacting fingers throughout hold | {report['min_hold_contacting_fingers']} of 3 |
| Largest relative movement within claw during settled hold | {report['max_hold_relative_slip_m']*1000:.4f} mm |
| Peak body pitch | {report['max_pitch_deg']:.2f} deg |
| Peak pitch-reference error | {report['max_pitch_error_deg']:.2f} deg |
| Final chassis drift | {report['final_body_drift_m']*1000:.2f} mm |
| Unintended arm contact samples | {report['unexpected_contact_samples']} |

The exceptionally small settled slip is a result of this idealized rigid-body simulation, not a real gripper precision claim. The object rises partly while the fingers curl; the additional arm-motion metric separates that effect from the later arm lift.

## Experiment setup

Alpha starts with its raised, open claw already aligned around the sphere. The sphere rests on a narrow static stand above a tray with low retaining walls. The claw closes at 6 s, arm lift starts at 12 s, settled hold is checked throughout 20–26 s, lowering starts at 28 s, and release starts at 36 s. Both runs last 48 s.

The sphere is a standalone Solid with its own Physics node. After initial scene placement, the controller only reads its position, velocity, and contact points. There is no object pose reset, artificial attachment, or applied object force. Robot motion comes from the existing bounded wheel torques and joint motors, with self-collision enabled.

The lift changes elbow angle from 1.50 to 1.25 rad while retaining 1.89 rad wrist pitch. The separate robot-envelope audit passes closure, lift/lower, and the open-claw control path. Object/stand contacts are checked dynamically. Direct-joint mounting pockets remain excluded, as in the earlier audit.

## Balance and contact assumptions

The robot mass layout and actuator limits are unchanged. Grasp-specific position/velocity gains are now 0.16/0.24, and the known-load ramp is 0.1 s, following the loaded-motion study in LOADED_CLEARANCE_RESULTS.md. Known-payload compensation adds the specified 2 g mass at an estimated palm-relative location when at least two fingers contact the unsupported sphere. It ramps out on release. A nominal full-load closure/lift preflight must fit the existing 0.30 rad lean limit; this 2 g reference is admitted. This uses simulator contact information as a stand-in for future sensing; it does not demonstrate a physical tactile sensor or payload estimator. Object world position is telemetry, not balance feedback.

Finger/sphere Coulomb friction is assumed to be 0.6. The simulation treats the collision surfaces as rigid and uses the existing 0.06 N m finger motor limits. It does not model a selected actuator, silicone deformation, real electrical behavior, or heating.

Contact-point queries can be empty for settled bodies in this run. Support checks therefore also recognize a sphere at rest within 0.1 mm of the known stand/tray support face. Unsupported holding requires a positive vertical gap of at least 6 mm above the stand, continuous finger contacts, and bounded relative slip; absence of a contact record alone cannot establish a lift.

## Acceptance checks and control

The positive case must complete without falling, stay below 20 degrees body pitch and 5 degrees reference error, return within 1 degree reference error and 5 cm chassis drift, remain within wheel speed limits, have no unintended arm contacts, start supported, gain at least 6 mm from actual arm motion, hold at least 5 s with at least two fingers and no fixture support, keep settled slip below 3 mm, and finish released, supported, and settled for the last 3 s.

The open-claw control must complete safely with no unintended arm contact, zero finger/object contact throughout, sphere movement under 2 mm, and failure of the positive lift/retention checks. It passed those control criteria. The sphere's largest movement was {control['max_object_displacement_m']*1000:.4f} mm.

## Failures preserved

- archive/grasp-first-trial: sphere retention worked, but a fingertip brushed the stand; the initial static-support contact classifier was also inadequate.
- archive/grasp-payload-compensation-trial: known-load compensation reduced chassis movement, but the original lift bent the wrist into the forearm/finger envelope.
- archive/grasp-stand-clearance-trial: holding wrist angle fixed cleared the robot itself; a remaining brief stand brush still failed the strict contact check. Moving the prepared object/stand lower within the open claw resolved it.

These are failed whole-test outcomes even though parts of the grasp worked. The prior complete empty-claw milestone is preserved in archive/claw-clearance-milestone. Its 13-case controller/model source remains unchanged and its existing evidence hashes still match.

## Limits and next work

This is one prepared-pose sphere experiment with known mass and assumed friction. It establishes neither autonomous approach, perception, ground pickup, precise placement back on the stand, arbitrary objects, nor a payload capacity rating. Release is into the tray rather than precise replacement on the narrow stand.

Next: vary mass, surface friction, sphere diameter, and placement error; record successful ranges and failures before integrating approach from the resting arm pose. Hardware dimensions, candidate actuators, compliance, and measured behavior still need to replace assumptions. No hardware was bought.

The separate grasp controller/world generator preserve the earlier base and empty-claw experiments. Source hashes, per-step hold checks, 64 ms telemetry, actual simulator renders, and the open-claw control are retained in the results folders.
'''
    (RESULTS/'GRASP_RESULTS.md').write_text(text, encoding='utf-8')

    sys.path.insert(0, str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    times = [row['time_s'] for row in data]
    axes[0, 0].plot(times, [1000*r['object_z_m'] for r in data], color='#d68c00', label='Grasp: sphere center')
    axes[0, 0].plot([r['time_s'] for r in control_data], [1000*r['object_z_m'] for r in control_data], '--', color='#75838a', label='Open-claw control')
    axes[0, 0].axhline(report['prelift_object_z_m']*1000, color='#b5bdc2', linewidth=1, label='After closure, before arm lift')
    axes[0, 0].set(title='Physical object motion', ylabel='Sphere center height (mm)')
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].step(times, [r['contacting_fingers'] for r in data], color='#2563a6', where='post')
    axes[0, 1].set(title='Finger contact through lift and hold', ylabel='Contacting fingers', yticks=[0, 1, 2, 3])
    axes[1, 0].plot(times, [r['pitch_rad']*180/3.141592653589793 for r in data], label='Measured', color='#2563a6')
    axes[1, 0].plot(times, [r['reference_rad']*180/3.141592653589793 for r in data], '--', label='COM reference + known load', color='#c46c22')
    axes[1, 0].set(title='Balancing during the grasp', ylabel='Body pitch (degrees)')
    axes[1, 0].legend(fontsize=8)
    axes[1, 1].plot(times, [r['body_x_m']*1000 for r in data], color='#2563a6')
    axes[1, 1].set(title='Chassis movement', ylabel='Forward displacement (mm)')
    for ax in axes.flat:
        ax.axvspan(20, 26, color='#23856e', alpha=.10)
        for when in [6, 12, 28, 36]:
            ax.axvline(when, color='#89939c', alpha=.3, linewidth=.8)
        ax.set_xlabel('Simulation time (s)')
        ax.grid(alpha=.2)
        ax.spines[['top', 'right']].set_visible(False)
    fig.suptitle('Model Alpha: 2 g sphere grasp, lift, hold, and release', fontsize=15)
    fig.text(.5, .015, 'Prepared pose | free object and physical contacts | shaded: 6 s hold | provisional contact and actuator assumptions', ha='center', fontsize=9, color='#555f68')
    fig.tight_layout(rect=(0, .04, 1, .95))
    fig.savefig(RESULTS/'grasp-validation.png', dpi=160)
    plt.close(fig)
    print(json.dumps({'prepared_grasp_passed': True, 'open_claw_control_passed': True,
                      'arm_added_lift_mm': 1000*report['arm_added_lift_m'], 'hold_s': report['hold_time_s']}, indent=2))


if __name__ == '__main__':
    main()
