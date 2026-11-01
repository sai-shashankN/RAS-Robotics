"""Verify the nominal vertical-path diagnostic against observed free-body motion."""
from pathlib import Path
import argparse
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace
from palm_leveling import angle_error

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--study',choices=['cube_cartesian','cube_cartesian_gain'],default='cube_cartesian');args=parser.parse_args()
    STUDY=MODEL/'experiments'/args.study
    m,status=read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=4:raise ValueError('Incomplete study')
    prior=MODEL/'experiments'/m.get('prior_study','cube_leveling_v2')
    if sha(prior/'checkpoint-index.json')!=m['prior_checkpoint_sha256']:raise ValueError('Prior checkpoint changed')
    rows=[];raw={}
    for entry in m['cases']:
        project=WORKSPACE/entry['project'];folder=project/'results/physics'/entry['case'];r=read(folder/'report.json');p=read(project/'parameters.json');plan=read(project/'cartesian-plan.json')
        if args.study=='cube_cartesian_gain':
            old=prior/'projects'/entry['variant']
            for name in ['parameters.json','cartesian-plan.json','scripts/generate_grasp_worlds.py']:
                if sha(project/name)!=sha(old/name):raise ValueError('Non-gain model/path change')
            for path in (STUDY/'projects/common').glob('*.py'):
                if sha(path)!=sha(prior/'projects/common'/path.name):raise ValueError('Controller/helper changed')
            expected=read(old/'experiments/physics/cases'/entry['case']/'experiment.json')
            expected['grasp_balance_gains']['position']=.64
            if r['experiment']!=expected:raise ValueError('Other experiment change')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Source changed: '+name)
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace changed')
        data=trace(folder/'contact-trace.jsonl');raw[entry['case']]=data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Trace gap')
        speed=lambda d:math.sqrt(sum(v*v for v in d['object_velocity'][:3]))
        tilt=lambda d:math.degrees(math.acos(max(-1,min(1,d['object_orientation_matrix'][8]))))
        initial=r['setup']['initial_object_position_m'];center=lambda d:math.hypot(d['object_position_m'][0]-initial[0],d['object_position_m'][1]-initial[1])
        release=r['cartesian_cycle']['released_at_s'];pre=[d for d in data if release is not None and release-.5<=d['time_s']<release]
        final=[d for d in data if d['time_s']>=r['schedule_actual_s']['finish']-3]
        support=bool(pre) and all(d['on_stand'] and speed(d)<.005 and tilt(d)<2 for d in pre)
        settled=bool(final) and all(d['on_stand'] and not d['contacting_fingers'] and speed(d)<.005 and tilt(d)<2 and center(d)<.002 for d in final)
        limits=dict(p['joint_ranges']);limits.update({f'{kind}_{i}':bounds for kind,bounds in [('finger',[-.65,.35]),('distal',[0,1.1])] for i in range(2)})
        excess=max(max(lo-d['joint_angles_rad'][j],d['joint_angles_rad'][j]-hi,0) for d in data for j,(lo,hi) in limits.items())
        lift=[d for d in data if 12<=d['time_s']<=28];start=min(lift,key=lambda d:d['time_s'])
        pathx=max(abs(d['palm_world_position_m'][0]-start['palm_world_position_m'][0]) for d in lift)
        pathz=max(abs(d['palm_world_position_m'][2]-start['palm_world_position_m'][2]-d['planned_palm_height_m']) for d in lift)
        anchorx=max(abs(d['palm_world_position_m'][0]-plan['anchor_world_xz_m'][0]) for d in data)
        opening=[d for d in data if release is not None and d['time_s']>=release and d['finger_contacts']]
        held=r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm']
        checks=dict(inherited_cycle_and_attitude=r['passed'],two_finger_unsupported_hold=held,stable_half_second_support_before_open=support,
            stable_centered_level_final_three_seconds=settled,observed_range_excess_under_0p03_rad=excess<.03)
        if entry['open_control']:checks={}
        rows.append(dict(entry,cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],retained_through_hold=held,
            release_s=release,support_blocked=r['cartesian_cycle']['support_blocked'],max_palm_attitude_error_deg=r['max_palm_attitude_error_deg'],
            added_object_lift_mm=r['arm_added_lift_m']*1000,min_hold_bottom_gap_mm=r['min_hold_stand_gap_m']*1000 if r['min_hold_stand_gap_m'] is not None else None,
            lift_palm_x_excursion_mm=pathx*1000,lift_palm_height_tracking_error_mm=pathz*1000,max_palm_anchor_x_error_mm=anchorx*1000,
            max_body_drift_mm=r['max_body_drift_m']*1000,max_contacting_fingers=r['max_contacting_fingers'],object_displacement_mm=r['max_object_displacement_m']*1000,
            unintended_arm_contact_points=r['unexpected_contact_samples'],unintended_by_node=r['unexpected_contact_counts_by_node'],range_excess_rad=excess,
            support_before_open_passed=support,stable_final_placement_passed=settled,final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],
            final_cube_tilt_deg=tilt(data[-1]),final_center_error_mm=center(data[-1])*1000,
            last_opening_contact_s=opening[-1]['time_s'] if opening else None,placement_checks=checks,supported_placement_passed=bool(checks) and all(checks.values()),
            failed_checks=[k for k,v in r['checks'].items() if not v],report_path=str((folder/'report.json').relative_to(WORKSPACE)),report_sha256=sha(folder/'report.json')))
    positive=[r for r in rows if not r['open_control']];controls=[r for r in rows if r['open_control']]
    summary=dict(date='2026-10-24',cases=rows,positive_cycle_passes=sum(r['cycle_passed'] for r in positive),positive_trials=2,
        open_controls_passed=sum(r['negative_control_passed'] for r in controls),open_controls=2,supported_placement_passes=sum(r['supported_placement_passed'] for r in positive),configuration_adopted=False,
        interpretation='Nominal balanced IK is distinct from actual ground-referenced hand motion on a balancing base. Path errors are measured, not hidden by a claim of vertical execution.',analysis_sha256=sha(Path(__file__)))
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines=['# Cartesian cube diagnostic','',m['question'],'',m.get('revision','First nominal Cartesian path and support-gated release candidate.'),'',m['shared_geometry'],'',m['limitations'],'',
        '| Variant | Cube retained through hold | Added object lift (mm) | Palm X excursion during lift (mm) | Opening time | Supported placement |',
        '| --- | --- | ---: | ---: | ---: | --- |']
    for r in positive:lines.append(f"| {r['variant'].upper()} | {r['retained_through_hold']} | {r['added_object_lift_mm']:.2f} | {r['lift_palm_x_excursion_mm']:.2f} | {r['release_s']} | {r['supported_placement_passed']} |")
    if args.study=='cube_cartesian_gain':
        lines+=['','The stronger position feedback is rejected: both loaded trials exceed 5° pitch error and all four trials record unintended arm contacts. A no longer sustains the required unsupported hold. B still retains through hold, but both positives eventually open and end in the tray. The ideal support/speed gate does not require cube tilt below 2°, so its permission to open is not a strict placement pass. No gain is promoted.','']
    lines+=['',f"{summary['positive_cycle_passes']}/2 positive cycles pass; {summary['open_controls_passed']}/2 open controls pass their non-grasp criteria; {summary['supported_placement_passes']}/2 positives meet all strict supported-placement criteria. This candidate is not adopted.",'',
        'The waypoint grid reproduces its intended 8 mm vertical line to floating-point precision in forward kinematics. A has only 1.116 mm minimum sampled nonadjacent self-envelope separation; B 9.762 mm. These screens exclude direct joint neighbors and do not certify all dynamic paths. The open-only nominal cube/pedestal sweeps pass, while actual free-body runs remain authoritative. A 10 mm balanced lift has a nominal solution; 15/20 mm proposals have none within the sampled/bisected equilibrium and joint bounds. This is a limitation of this prepared pose, geometry and balance guard, not a universal arm-reach proof.','',
        'Runtime motion interpolates precomputed empty/full-load and open/closed families. IMU pitch and encoders level the wrist at the original speed cap. The plan assumes zero base translation; actual wrist position and cube motion are logged at 8 ms to expose that assumption. The wider opening and 32 mm neck are provisional changes shared by A/B; fixed lumped neck/palm mass remains an assumption.','',
        'Opening is allowed only after ideal simulated pedestal support and speed below 5 mm/s persist for 0.6 s. This gate does not require cube tilt/centering and cannot certify level stable support transfer. Support seeking is limited to 2 mm at 0.5 mm/s; if no gate is satisfied by 41 s, the hand stays closed and the trial is reported blocked. This diagnostic contact witness needs a realizable sensor strategy before hardware use. Object pose never supplies a joint target and no attachments or object force/pose writes are used.','',
        'Strict supported placement retains the prior requirements: cycle/attitude and range checks, two-finger unsupported hold, level stable pedestal support for 0.5 s before opening, then final three seconds finger-free, supported, under 2° tilt, 5 mm/s speed and 2 mm center error. Support confirmation alone is not sufficient. Path tracking is reported separately without retroactively choosing a passing threshold.','',
        '## Retained outcomes','']
    for r in rows:lines.append(f"- {r['case']}: cycle {r['cycle_passed']}; non-grasp control {r['negative_control_passed']}; support blocked {r['support_blocked']}; arm contact points {r['unintended_arm_contact_points']}; failed cycle checks: {', '.join(r['failed_checks'])}.")
    lines+=['','Open controls intentionally fail pickup/hold checks. Finger contact, unwanted arm contact or excessive object disturbance invalidates the separate non-grasp control. All failures are retained.','',
        '![Measured palm trajectory](cartesian-path.png)','',
        '![A actual hold](projects/a/results/physics/a_vertical/24_lift_hold.png)','',
        '## Next work','',
        'Plan coordinated base/arm motion using odometry and IMU with explicit swept pedestal clearance, rather than increasing one gain further. Check the effect of contact-constrained finger angles versus the nominal open/closed mass-layout families and payload transfer. Add attitude/centering to the placement gate and stop or retract on unintended arm contact. Revalidate A/B and empty controls before broader objects. Wider jaw opening alone cannot validate supported release. No Beta roll workaround, hardware purchase or main-model promotion.','']
    (STUDY/'CARTESIAN_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for i,v in enumerate(['a','b']):
        for suffix,label,color in [('', 'loaded','#2874a5'),('_open','open control','#d67a32')]:
            data=raw[v+'_vertical'+suffix]
            axes[i,0].plot([d['time_s'] for d in data],[1000*(d['palm_world_position_m'][0]-.0815567916714127) for d in data],label=label,color=color)
            start=min(data,key=lambda d:abs(d['time_s']-12))
            axes[i,1].plot([d['time_s'] for d in data],[1000*(d['palm_world_position_m'][2]-start['palm_world_position_m'][2]) for d in data],label=label,color=color)
            axes[i,1].plot([d['time_s'] for d in data],[1000*d['planned_palm_height_m'] for d in data],color=color,ls='--',label=label+' plan',alpha=.7)
        axes[i,0].set(title=v.upper()+': actual palm X relative to anchor',ylabel='X error (mm)');axes[i,1].set(title=v.upper()+': actual palm height change',ylabel='Height (mm)')
    for ax in axes.flat:ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(STUDY/'cartesian-path.png',dpi=160);plt.close(fig)
    for name in ['run_cube_cartesian.py','run_cube_cartesian_gain.py','summarize_cube_cartesian.py','cartesian_pick_place.py','palm_leveling.py','run_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        path=MODEL/'scripts'/name;target=STUDY/'source_snapshot'/path.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    files={str(p.relative_to(STUDY)):sha(p) for p in STUDY.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps(dict(date='2026-10-24',files=files),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
