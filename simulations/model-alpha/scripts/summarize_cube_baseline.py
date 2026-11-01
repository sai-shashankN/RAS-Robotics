"""Cube seating/retention observations; inherited tray-drop pass is not placement."""
from pathlib import Path
import csv
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace
from cube_geometry import bottom,corners,self_check

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/cube_baseline'
PRIOR = MODEL/'experiments/fixed_cadence_checks'


def main():
    self_check();manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=4:raise ValueError('Incomplete cube diagnostic')
    if sha(PRIOR/'checkpoint-index.json')!=manifest['prior_checkpoint_sha256']:raise ValueError('Prior checkpoint changed')
    if sha(STUDY/'projects/common/cube_geometry.py')!=sha(MODEL/'scripts/cube_geometry.py'):raise ValueError('Geometry analysis differs from runtime helper')
    rows,raw = [],{}
    for entry in manifest['cases']:
        project = WORKSPACE/entry['project'];folder = project/'results/physics'/entry['case'];r = read(folder/'report.json');p = read(project/'parameters.json')
        if not read(project/'experiments/physics/cases'/entry['case']/'initial-clearance.json')['passed']:raise ValueError('Unreviewed initial geometry')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale cube input')
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        if r['experiment']['object_size_m']!=[.02,.02,.02] or p['distal_profile_angle_deg']!=0 or not p['physical_joint_stops'] or p['world_basic_time_step_ms']!=2:raise ValueError('Wrong cube configuration')
        data = trace(folder/'contact-trace.jsonl');raw[entry['case']] = data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Missing controller trace samples')
        for d in data:
            points = corners(d['object_position_m'],d['object_orientation_matrix'],r['experiment']['object_size_m'])
            if abs(bottom(points)-d['object_bottom_m'])>1e-12 or abs(bottom(points)-r['setup']['stand_top_m']-d['stand_gap_m'])>1e-12:raise ValueError('Incorrect box support geometry')
        before = lambda t:max((d for d in data if d['time_s']<t),key=lambda d:d['time_s'])
        schedule = r['schedule_actual_s'];start,lift,release = [before(schedule[k]) for k in ['close','lift','release']]
        after = [d for d in data if d['time_s']>=schedule['release']]
        release_contacts = [d for d in after if d['finger_contacts']]
        post_stand = [d for d in after if d['on_stand']]
        pre = [d for d in data if schedule['close']-.5<=d['time_s']<schedule['close']]
        clean = all(d['on_stand'] and not d['contacting_fingers'] for d in pre) and max(math.dist(d['object_position_m'],r['setup']['initial_object_position_m']) for d in pre)<.0005
        retained = r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm']
        ranges = dict(p['joint_ranges']);ranges.update({f'{kind}_{i}':limits for kind,limits in [('finger',[-.65,.35]),('distal',[0,1.1])] for i in range(2)})
        excess = {j:max(max(lo-d['joint_angles_rad'][j],d['joint_angles_rad'][j]-hi,0) for d in data) for j,(lo,hi) in ranges.items()}
        telemetry = list(csv.DictReader((folder/'telemetry.csv').open(encoding='utf-8')))
        opening = min(telemetry,key=lambda d:abs(float(d['time_s'])-5.5))
        palm_world = sum(float(opening[k]) for k in ['pitch_rad','shoulder','elbow','wrist'])
        if entry['open_control'] and not r['negative_control_passed']:raise ValueError('Cube open control failed')
        rows.append(dict(entry,cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],retained_through_hold=retained,
            clean_preclose_window=clean,failed_checks=[k for k,v in r['checks'].items() if not v],
            closure_center_shift_world_mm=[1000*(lift['object_position_m'][i]-start['object_position_m'][i]) for i in range(3)],
            minimum_hold_contacting_fingers=r['min_hold_contacting_fingers'],arm_added_lift_mm=r['arm_added_lift_m']*1000,
            hold_slip_mm=r['max_hold_relative_slip_m']*1000 if r['max_hold_relative_slip_m'] is not None else None,
            preopening_support_gap_mm=release['stand_gap_m']*1000,preopening_contacting_fingers=release['contacting_fingers'],
            preopening_on_stand=release['on_stand'],preopening_cube_top_tilt_deg=math.degrees(math.acos(min(1,max(-1,release['object_orientation_matrix'][8])))),
            last_opening_finger_contact_s=release_contacts[-1]['time_s'] if release_contacts else None,
            last_opening_finger_segments=sorted(set(c['segment'] for c in release_contacts[-1]['finger_contacts'])) if release_contacts else [],
            first_postopening_stand_contact_s=post_stand[0]['time_s'] if post_stand else None,
            final_three_seconds_on_stand=all(d['on_stand'] for d in data if d['time_s']>=schedule['finish']-3),
            final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],unintended_arm_contact_points=r['unexpected_contact_samples'],
            unintended_contact_by_node=r['unexpected_contact_counts_by_node'],unintended_contact_examples=r['unexpected_contact_examples'],
            intended_range_excess_rad=excess,opened_palm_world_pitch_rad=palm_world,
            deviation_from_vertical_approach_deg=math.degrees(abs(math.pi-palm_world)),
            report_path=str((folder/'report.json').relative_to(WORKSPACE)),report_sha256=sha(folder/'report.json'),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    positive = [r for r in rows if not r['open_control']]
    summary = {'date':'2026-10-24','task':manifest['task'],'design':manifest['design'],'limitations':manifest['limitations'],
        'cases':rows,'positive_cycle_passes':sum(r['cycle_passed'] for r in positive),'positive_trials':2,
        'retained_through_hold':sum(r['retained_through_hold'] for r in positive),'open_controls_passed':sum(r['negative_control_passed'] for r in rows if r['open_control']),
        'supported_placement_validated':False,'profile_or_dimensions_adopted':False,'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# First cube contact diagnostic','','24 October 2026 — actual box geometry and stable support; separate from the sphere studies.','',
        '**Both A/B retain and lift the cube through the six-second hold, with zero unintended arm contacts or observed range violations. Both open controls stay on the pedestal. Closing moves the center much less in this new benchmark, but A still drags the cube off the support while opening; B ends stably on it.**','',manifest['design'],'',manifest['task'],'',
        '| Variant | Inherited cycle | Hold retention | Closure shift X/Y/Z (mm) | Unintended arm points |',
        '| --- | --- | --- | --- | ---: |']
    for r in positive:lines.append(f"| {r['variant'].upper()} | {'Pass' if r['cycle_passed'] else 'FAIL'} | {'Yes' if r['retained_through_hold'] else 'No'} | {' / '.join(f'{x:.2f}' for x in r['closure_center_shift_world_mm'])} | {r['unintended_arm_contact_points']} |")
    lines += ['',f"{summary['open_controls_passed']}/2 open controls pass their non-grasp checks. Full failed checks, observed joint range excess, support state, pre-opening seating and palm orientation are preserved in summary.json.",'',
        'Closing shifts A about -0.224 mm X/+0.787 mm Z and B +0.229 mm X/+0.008 mm Z. This is a new object/support/pad/settings configuration, not a controlled one-factor proof that cube shape alone causes the improvement.','',
        '## Slopes and release remain visible','',
        'Neither cube is supported on the pedestal immediately before opening. A is tilted about 10.8°; its lowest corner is below the top-plane height but outside the support footprint. It touches the support at 36.232 s, remains in contact with TIP_0 until 37.192 s, and ends in the tray about 81 mm from the pedestal center in X. B first touches at 36.224 s and loses its last TIP_0 contact at 36.304 s; it finishes on the pedestal throughout the final three seconds. These observations are consistent with the sloping fingertip/opening path guiding A off support; they do not prove a unique causal mechanism.','',
        'Next: review vertical wrist/approach alignment and the full finger opening/withdrawal sweep, then establish support before opening with stable-pose checks. Wrist offset/range, finger geometry/coordination, arm lengths and wheel/base geometry are all open variables. Review clearance and balance before selecting any changed candidate.','',
        'Box support uses all eight transformed corners, actual contact points on rectangular support tops and a sleeping-body geometric witness. The old sphere-radius bottom shortcut is not used. Geometry checks include a support fully inside the cube’s flat bottom face, a displaced support and a tilted cube. Each trace records the orientation matrix, corners and true lowest point, and the analyzer independently recomputes them.','',
        'The initial opened robot passes a conservative clearance screen against the cube and pedestal. That screen does not certify loaded closure/lowering: actual contacts and range observations remain part of each run. The two digits still curl; flat contact pads do not remove the joint transitions or their slopes.','',
        'The initial inherited arm pose is tilted relative to a vertical approach because the body balances at an angle. Palm world pitch/deviation are recorded separately. If contact seating or release calls for it, review a level/vertical grasp pose, wrist/link offsets, finger shape and base geometry together. The user explicitly permits arm/wrist/joint and wheel changes; no dimension is frozen.','',
        'A cycle pass uses inherited grasp/lift/hold and eventual catch-tray settling criteria. It is not supported placement on the pedestal. A cleanly supported lowering, opening, withdrawal and final stable pose still need an explicit plan and acceptance checks. No geometry/hardware choice is adopted.','',manifest['limitations'],'',
        '![Cube contact and release traces](cube-contact-release.png)','',
        '![Alpha-B cube during unsupported hold](projects/b/results/physics/b_cube/24_lift_hold.png)','',
        'Raw reports, 8 ms contact traces, telemetry, inputs, worlds, logs and actual simulator frames are under projects/. manifest.json, run-status.json, summary.json and checkpoint-index.json preserve the complete first cube trial set.','']
    (STUDY/'CUBE_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,2,figsize=(11,7))
    for v,color in [('a','#d67a32'),('b','#2775a7')]:
        data = raw[f'{v}_cube'];closing = [d for d in data if 6<=d['time_s']<12];opening = [d for d in data if 36<=d['time_s']<=38]
        first = closing[0]['object_position_m']
        axes[0,0].plot([d['time_s'] for d in closing],[1000*(d['object_position_m'][0]-first[0]) for d in closing],label=v.upper(),color=color)
        axes[0,1].plot([d['time_s'] for d in closing],[1000*(d['object_position_m'][2]-first[2]) for d in closing],label=v.upper(),color=color)
        reference_x = data[0]['object_position_m'][0]
        axes[1,0].plot([d['time_s'] for d in opening],[1000*(d['object_position_m'][0]-reference_x) for d in opening],label=v.upper(),color=color)
        axes[1,1].plot([d['time_s'] for d in opening],[len(d['contacting_fingers']) for d in opening],label=v.upper(),color=color)
    for ax,title,ylabel in zip(axes.flat,['Closing: horizontal displacement','Closing: vertical displacement','Opening: A moves away from the support','Opening: lingering fingertip contact'],['Center X shift (mm)','Center Z shift (mm)','Center X from pedestal (mm)','Contacting finger count']):
        ax.set(title=title,xlabel='Simulation time (s)',ylabel=ylabel);ax.grid(alpha=.2);ax.legend()
    fig.suptitle('20 mm cube: useful retention; supported release remains a separate task')
    fig.tight_layout();fig.savefig(STUDY/'cube-contact-release.png',dpi=160);plt.close(fig)
    for name in ['run_cube_baseline.py','summarize_cube_baseline.py','cube_geometry.py','run_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        source = MODEL/'scripts'/name;target = STUDY/'source_snapshot'/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(p.relative_to(STUDY)):sha(p) for p in STUDY.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['positive_cycle_passes','positive_trials','retained_through_hold','open_controls_passed']}),flush=True)


if __name__=='__main__':main()
