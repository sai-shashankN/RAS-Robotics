"""Verify paired fixed-axis evidence without transferring success between variants."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_sources(report):
    for name,value in report['source_sha256'].items():
        if sha(WORKSPACE/name)!=value:
            raise ValueError('Stale evidence: '+name)


def variant(model):
    p = read(model/'parameters.json')
    original = list(sys.argv)
    try:
        sys.argv = ['generator',str(model)]
        spec = importlib.util.spec_from_file_location('generator_'+p['model'],WORKSPACE/'simulations/common/generate_worlds.py')
        g = importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    finally:
        sys.argv = original
    cases = [('geometry','balance')]+[('dynamics',name) for name in ['balance','tilt_positive','tilt_negative','push','steering','drive',*p['dynamic_arm_targets'],'pose_guard','claw_cycle_folded','claw_cycle_raised']]
    rows = []
    for mode,name in cases:
        folder = model/'results'/mode
        if name!='balance':folder /= name
        path = folder/'report.json';r = read(path);verify_sources(r)
        if r['model']!=p['model'] or r['finger_angles_rad']!=p['finger_angles_rad'] or r['finger_count']!=2 or r['wrist_roll_present']:
            raise ValueError('Wrong mechanism: '+str(path))
        passed = r['geometry_joint_and_fk_check_passed'] if mode=='geometry' else r['dynamics_validation_passed']
        if not passed or (mode=='dynamics' and (not r['self_collision_enabled'] or r['max_arm_contact_count']!=0)):
            raise ValueError('Failed empty validation: '+str(path))
        suffix = '_'+name if name!='balance' else ''
        world = model/'worlds'/f'{p["model"].lower()}_{mode}{suffix}_validation.wbt'
        if world.read_text(encoding='utf-8')!=g.world(mode,True,name):
            raise ValueError('World differs from current generator: '+str(world))
        if mode=='geometry' and (r['max_displacement_m']>1e-9 or not r['geometry_direct_joint_positioning']):
            raise ValueError('Geometry fixture moved')
        rows.append({'mode':mode,'scenario':name,'passed':True,'time_s':r['time_s'],
                     'peak_pitch_deg':math.degrees(r['max_abs_pitch_rad']),'final_drift_mm':1000*r['final_drift_m'],
                     'contacts':r['max_arm_contact_count'],'max_com_model_error_mm':1000*r['max_com_model_error_m'],
                     'max_finger_fk_error_mm':1000*r['max_finger_tip_fk_error_m'],
                     'report_path':str(path.relative_to(WORKSPACE)),'report_sha256':sha(path),'world_sha256':sha(world)})
    audit_path = model/'results/clearance/report.json';audit = read(audit_path);verify_sources(audit)
    if not audit['all_defined_paths_clear']:
        raise ValueError('Failed clearance audit: '+str(model))
    return {'label':p['model'],'parameters_sha256':sha(model/'parameters.json'),'finger_angles_rad':p['finger_angles_rad'],
            'mass_kg':r['mass_model']['mass_kg'],'cases':rows,'clearance_paths':audit['paths'],
            'audit_sha256':sha(audit_path),'all_empty_checks_passed':True}


def main():
    a,b = MODEL,MODEL.parent/'model-alpha-b'
    comparison = read(b/'comparison-setup.json')
    if sha(a/'parameters.json')!=comparison['baseline_sha256'] or sha(b/'parameters.json')!=comparison['variant_sha256']:
        raise ValueError('A/B baseline changed')
    pa,pb = read(a/'parameters.json'),read(b/'parameters.json')
    differences = sorted(k for k in set(pa)|set(pb) if pa.get(k)!=pb.get(k))
    if differences!=['finger_angles_rad','gripper_design','model']:
        raise ValueError('Unmatched physical inputs')
    variants = {name:variant(model) for name,model in [('Alpha-A',a),('Alpha-B',b)]}
    summary = {'date':'2026-10-24','only_physical_parameter_changed':'finger_angles_rad',
               'metadata_differences':['model','gripper_design'],'variants':variants,
               'all_empty_checks_passed':True,'powered_roll_tested':False,
               'limitation':'Two fixed orientations and specified empty paths only; no object/grasp, full roll, hardware or X support validation.'}
    output = a/'results'
    (output/'axis-comparison-summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    (b/'results/axis-empty-summary.json').write_text(json.dumps(variants['Alpha-B'],indent=2)+'\n',encoding='utf-8')
    lines = ['# Alpha-A / Alpha-B: fixed-axis empty validation','', 'Recorded 24 October 2026, Webots R2025a and Python 3.13.','',
             '**Both fixed orientations pass four prescribed geometry poses, thirteen empty dynamics cases and six specified robot-clearance paths.**','',
             'Alpha-A is the original reference-X/front-back claw. Alpha-B is the reference-Y/left-right claw, with finger bases rotated 90 degrees around the palm approach axis. Both have two curling fingers, four finger hinges, fixed front shoulder and no sleeve or powered claw roll. Their axes follow the arm/base rather than remaining fixed in the world.','',
             'The only physical parameter changed is finger_angles_rad. Palm shape, link/body geometry, masses/COM, gains, motor limits and joint targets are identical; model/gripper description labels also differ. The 1.207 kg mass model and circular palm are retained. No feedback or motor limits were retuned. Alpha-A evidence was verified without unnecessary reruns because its controller/model hashes are unchanged.','',
             '| Empty dynamics case | A peak pitch (deg) | B peak pitch (deg) | A final drift (mm) | B final drift (mm) | A/B arm contacts |','| --- | ---: | ---: | ---: | ---: | ---: |']
    a_rows = {row['scenario']:row for row in variants['Alpha-A']['cases'] if row['mode']=='dynamics'}
    b_rows = {row['scenario']:row for row in variants['Alpha-B']['cases'] if row['mode']=='dynamics'}
    for name,ar in a_rows.items():
        br = b_rows[name]
        lines.append(f"| {name} | {ar['peak_pitch_deg']:.3f} | {br['peak_pitch_deg']:.3f} | {ar['final_drift_mm']:.3f} | {br['final_drift_mm']:.3f} | {ar['contacts']}/{br['contacts']} |")
    lines += ['', 'Geometry is a massless fixed fixture with prescribed direct joint positioning; it checks main/finger angles and independent FK, not motor/contact performance. Dynamics uses Physics, bounded motor commands and nonadjacent self-collision with no body/joint pose writes. Applicable cases check COM agreement, lean/reference tracking, floor clearance, finger tracking/FK, guard refusals and drive behavior. Detailed criteria remain in each report.','',
              '| Specified path | A conservative bound (mm) | B conservative bound (mm) |','| --- | ---: | ---: |']
    for name,ar in variants['Alpha-A']['clearance_paths'].items():
        br = variants['Alpha-B']['clearance_paths'][name]
        lines.append(f"| {name} | {ar['conservative_clearance_bound_m']*1000:.3f} | {br['conservative_clearance_bound_m']*1000:.3f} |")
    lines += ['', 'These are conservative separating-axis bounds for specified nonadjacent robot-envelope paths, with a between-sample motion allowance. B has more margin on some of these paths; this is not an overall grasping or hardware advantage. Direct-joint mounting pockets, arbitrary poses, fixtures, object contacts, tolerances and flex remain unvalidated by this audit.','',
              'Next: matched object-contact investigations of closure seating/displacement, retention and supported release, then directional objects and palm-shape comparisons. Two fixed orientations do not validate Beta intermediate angles or its added roll-joint dynamics/wiring. Useful-task tests and movement-limit mapping remain separate. X support/lifting claw design stays deferred.','',
              '![A/B empty-motion comparison](axis-comparison.png)','',
              '[Alpha-A actual geometry](geometry/open_above.png) · [Alpha-B actual geometry](../../model-alpha-b/results/geometry/open_above.png)','',
              'Machine-readable evidence: axis-comparison-summary.json; B comparison-setup.json; each report/world/source hash and clearance audit.','']
    (output/'AXIS_COMPARISON_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(1,2,figsize=(12,4.5))
    for name,model,color in [('Alpha-A',a,'#2167ac'),('Alpha-B',b,'#d76b28')]:
        data = list(csv.DictReader((model/'results/dynamics/claw_cycle_raised/telemetry.csv').open()))
        axes[0].plot([float(row['time_s']) for row in data],[math.degrees(float(row['pitch_rad'])) for row in data],label=name,color=color)
    axes[0].set(xlabel='Simulation time (s)',ylabel='Body pitch (deg)',title='Raised empty claw cycles');axes[0].legend()
    paths = list(variants['Alpha-A']['clearance_paths'])
    for name,offset,color in [('Alpha-A',-.18,'#2167ac'),('Alpha-B',.18,'#d76b28')]:
        axes[1].barh([i+offset for i in range(len(paths))],[variants[name]['clearance_paths'][key]['conservative_clearance_bound_m']*1000 for key in paths],height=.34,label=name,color=color)
    axes[1].set(yticks=range(len(paths)),yticklabels=paths,xlabel='Conservative bound (mm)',title='Specified robot-envelope paths');axes[1].legend()
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(output/'axis-comparison.png',dpi=160);plt.close(fig)
    print(json.dumps({'variants_verified':2,'geometry_poses_each':4,'empty_dynamics_cases_each':13,'clearance_paths_each':6}),flush=True)


if __name__=='__main__':
    main()
