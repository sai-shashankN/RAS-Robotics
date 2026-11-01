"""Verify and summarize current two-finger geometry and empty dynamics evidence."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import math
import re
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
RESULTS = MODEL/'results'


def main():
    p = json.loads((MODEL/'parameters.json').read_text())
    if p['finger_angles_rad'] != [0,math.pi] or p['has_sleeve'] or p['wrist_roll_plan']['required']:
        raise ValueError('Unexpected Alpha mechanism')
    sources = [MODEL.parent/'common'/name for name in ['robot_controller.py','generate_worlds.py','control_math.py','mass_properties.py']]+[MODEL/'parameters.json']
    hashes = {str(path.relative_to(WORKSPACE)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    previous_argv = list(sys.argv)
    try:
        sys.argv = [str(MODEL.parent/'common/generate_worlds.py'),str(MODEL)]
        spec = importlib.util.spec_from_file_location('current_generator',MODEL.parent/'common/generate_worlds.py')
        generator = importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    finally:
        sys.argv = previous_argv
    cases = [('geometry','balance')]+[('dynamics',s) for s in ['balance','tilt_positive','tilt_negative','push','steering','drive',*p['dynamic_arm_targets'],'pose_guard','claw_cycle_folded','claw_cycle_raised']]
    rows = []
    for mode,scenario in cases:
        folder = RESULTS/mode
        if scenario!='balance':folder /= scenario
        report_path = folder/'report.json'
        r = json.loads(report_path.read_text())
        if r['source_sha256']!=hashes or r['finger_count']!=2 or r['wrist_roll_present']:
            raise ValueError('Stale/wrong evidence: '+str((mode,scenario)))
        key = 'geometry_joint_and_fk_check_passed' if mode=='geometry' else 'dynamics_validation_passed'
        if not r[key] or (mode=='dynamics' and (not r['self_collision_enabled'] or r['max_arm_contact_count'])):
            raise ValueError('Failed check: '+str((mode,scenario)))
        suffix = '_'+scenario if scenario!='balance' else ''
        world = MODEL/'worlds'/f'alpha_{mode}{suffix}_validation.wbt'
        text = world.read_text(encoding='utf-8')
        if text!=generator.world(mode,True,scenario):
            raise ValueError('World does not match current generator: '+str(world))
        if set(re.findall(r'RotationalMotor \{ name "((?:finger|distal)_\d+)"',text))!={'finger_0','finger_1','distal_0','distal_1'}:
            raise ValueError('Wrong finger motors')
        if mode=='geometry' and ('physics Physics' in text or r['max_displacement_m']>1e-9 or not r['geometry_direct_joint_positioning']):
            raise ValueError('Geometry fixture is not stationary/massless')
        rows.append({'mode':mode,'scenario':scenario,'passed':True,'duration_s':r['time_s'],
                     'peak_pitch_deg':math.degrees(r['max_abs_pitch_rad']),'final_drift_mm':r['final_drift_m']*1000,
                     'peak_pitch_error_deg':math.degrees(r['max_pitch_error_rad']),
                     'arm_contacts':r['max_arm_contact_count'],'finger_tip_fk_error_m':r['max_finger_tip_fk_error_m'],
                     'report_sha256':hashlib.sha256(report_path.read_bytes()).hexdigest(),
                     'world_sha256':hashlib.sha256(world.read_bytes()).hexdigest()})
    audit_path = RESULTS/'clearance/report.json'
    audit = json.loads(audit_path.read_text())
    if not audit['all_defined_paths_clear']:
        raise ValueError('Clearance failed')
    for name,sha in audit['source_sha256'].items():
        if hashlib.sha256((WORKSPACE/name).read_bytes()).hexdigest()!=sha:
            raise ValueError('Stale clearance: '+name)
    summary = {'date':'2026-10-24','finger_count':2,'finger_hinges':4,'pinch_axis':'palm-local X; front/back in reference grasp',
               'alpha_wrist_roll':False,'beta_x_wrist_roll':'Required future implementation, not simulated here',
               'mass_kg':r['mass_model']['mass_kg'],'source_sha256':hashes,'all_passed':True,'cases':rows,
               'clearance_sha256':hashlib.sha256(audit_path.read_bytes()).hexdigest(),
               'clearance_paths':audit['paths'],'grasp_or_placement_validated':False,
               'limitations':['Specified empty paths only; geometry uses prescribed joint positioning, no motor/contact claim.',
                              'Dynamics uses provisional rigid bodies, sensors/actuators, masses and COM; no hardware validation.',
                              'Direct joint pockets, contact seating, precision placement, compliance/coupling and uncertainty remain open.']}
    (RESULTS/'two-finger-summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Alpha two-finger verification','', 'Recorded 24 October 2026; Webots R2025a and Python 3.13.','',
             '**The two-finger model passes four prescribed geometry poses, thirteen empty free-body cases and six specified conservative clearance paths. No two-finger object pickup or placement is validated.**','',
             'Finger bases are opposite on palm-local X (0/pi), giving a front/back pinch in the reference downward grasp. Each curls at proximal/distal hinges. Alpha has no claw roll; Beta/X require that additional approach-axis rotation, deferred to their implementation.','',
             'One finger pair was removed: modeled mass is 1.207 kg. The provisional 650 g body COM moves from -31.665 to -28.887 mm to center the folded pose. Body mass, gains, motor caps and main-arm dimensions remain unchanged. This is a hypothetical component layout.','',
             '| Empty dynamics case | Peak body pitch (deg) | Final drift (mm) | Arm contacts |','| --- | ---: | ---: | ---: |']
    for row in rows:
        if row['mode']=='dynamics':
            lines.append(f"| {row['scenario']} | {row['peak_pitch_deg']:.3f} | {row['final_drift_mm']:.3f} | {row['arm_contacts']} |")
    lines += ['', 'All dynamics cases retain their detailed checks and source hashes. Finger cycles check all four finger hinges within 0.03 rad at settled checkpoints and both distal endpoints against independent FK within 2 mm. COM agreement, lean guard, balance/drive limits and nonadjacent contacts are checked in the applicable runs.','',
              'Geometry contains no Physics nodes and disables self-collision. It directly positions prescribed joints and checks all main/finger angles and FK; the root stays fixed. This demonstrates geometry, not motor tracking or collision freedom. Expanded checks caught fingers not settling in the previous motor-driven massless fixture. That failed check and the initial thirteen passing dynamics runs are retained in archive/two-finger-first-validation. Disabling fixture self-collision removed the old root drift; direct kinematic positioning corrected the fixture. Final dynamics was repeated with matching sources and remains motor/torque driven without body/joint state writes.','',
              '## Specified robot clearances','', '| Path | Conservative bound (mm) |','| --- | ---: |']
    for name,path in audit['paths'].items():
        lines.append(f"| {name} | {path['conservative_clearance_bound_m']*1000:.3f} |")
    lines += ['', 'The audit uses conservative oriented boxes and a between-sample motion allowance. Directly connected joint neighbors are excluded; mounting pockets, tolerances, arbitrary poses, obstacles and loaded manipulation remain unvalidated. Positive bounds are not hardware clearance allowances.','',
              '## Next contact work','', 'Inspect first contact, closure-induced object movement, seating, retention, supported setdown and release separately. The user correctly identified that fingers move the object inside the hand; returning the wrist does not return the object to its original point. The archived three-finger supported-setdown diagnostic still failed. Old grasp/approach/return passes do not transfer to this two-finger claw.','',
              'Beta/X claw roll, X ground-support redesign, real sensing, coupling/compliance, uncertainty and hardware tests remain open. No hardware purchased.','',
              '![Current two-finger raised geometry](geometry/open_above.png)','',
              '![Empty claw telemetry](two-finger-validation.png)','',
              'See [accepted revision](../../../project-notes/DESIGN_REVISION_2026-10-24.md), [prior checkpoint](../archive/three-finger-2026-10-24/README.md), and two-finger-summary.json for current machine-readable evidence.','']
    (RESULTS/'TWO_FINGER_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,2,figsize=(11,7))
    for column,scenario in enumerate(['claw_cycle_folded','claw_cycle_raised']):
        data = list(csv.DictReader((RESULTS/'dynamics'/scenario/'telemetry.csv').open()))
        times = [float(row['time_s']) for row in data]
        for i,color in enumerate(['#236ab5','#d66a29']):
            for part,style in [('finger','-'),('distal','--')]:
                axes[0,column].plot(times,[math.degrees(float(row[f'{part}_{i}_rad'])) for row in data],style,color=color,label=f'{part} {i}')
        axes[0,column].set(title='Folded' if column==0 else 'Raised',ylabel='Finger joint angle (deg)')
        axes[0,column].legend(fontsize=8,ncol=2)
        axes[1,column].plot(times,[math.degrees(float(row['pitch_rad'])) for row in data],label='Measured pitch')
        axes[1,column].plot(times,[math.degrees(float(row['lean_reference_rad'])) for row in data],'--',label='COM reference')
        axes[1,column].set(ylabel='Body pitch (deg)');axes[1,column].legend(fontsize=8)
    for ax in axes.flat:
        ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2)
    fig.suptitle('Alpha: two opposing curling fingers — empty cycles')
    fig.tight_layout();fig.savefig(RESULTS/'two-finger-validation.png',dpi=160);plt.close(fig)
    print(json.dumps({'geometry_poses':4,'empty_dynamics_cases':13,'clearance_paths':6,'object_grasp_validated':False}),flush=True)


if __name__=='__main__':
    main()
