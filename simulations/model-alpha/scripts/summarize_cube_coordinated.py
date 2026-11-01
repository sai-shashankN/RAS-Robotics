"""Keep coordination, fingertip alignment, controls and placement claims separate."""
from pathlib import Path
import argparse
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--study',choices=['cube_coordinated','cube_parallel_pads'],default='cube_coordinated');args=parser.parse_args()
    study=MODEL/'experiments'/args.study;m=read(study/'manifest.json');status=read(study/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=4:raise ValueError('Incomplete physical study')
    prior=MODEL/'experiments'/m.get('prior_study','cube_cartesian')
    if sha(prior/'checkpoint-index.json')!=m['prior_checkpoint_sha256']:raise ValueError('Previous evidence changed')
    rows=[];raw={}
    for entry in m['cases']:
        project=WORKSPACE/entry['project'];folder=project/'results/physics'/entry['case'];r=read(folder/'report.json');p=read(project/'parameters.json')
        for path,digest in r['source_sha256'].items():
            if sha(WORKSPACE/path)!=digest:raise ValueError('Changed physical inputs')
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace changed')
        data=trace(folder/'contact-trace.jsonl');raw[entry['case']]=data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Trace gaps')
        speed=lambda d:math.sqrt(sum(v*v for v in d['object_velocity'][:3]))
        tilt=lambda d:math.degrees(math.acos(max(-1,min(1,d['object_orientation_matrix'][8]))))
        initial=r['setup']['initial_object_position_m'];center=lambda d:math.hypot(d['object_position_m'][0]-initial[0],d['object_position_m'][1]-initial[1])
        release=r['cartesian_cycle']['released_at_s'];pre=[d for d in data if release is not None and release-.5<=d['time_s']<release]
        final=[d for d in data if d['time_s']>=r['schedule_actual_s']['finish']-3]
        support=bool(pre) and all(d['on_stand'] and speed(d)<.005 and tilt(d)<2 for d in pre)
        settled=bool(final) and all(d['on_stand'] and not d['contacting_fingers'] and speed(d)<.005 and tilt(d)<2 and center(d)<.002 for d in final)
        limits=dict(p['joint_ranges']);limits.update({f'{kind}_{i}':bounds for kind,bounds in [('finger',[-.65,.35]),('distal',p.get('distal_joint_range_rad',[0,1.1]))] for i in range(2)})
        excess=max(max(lo-d['joint_angles_rad'][j],d['joint_angles_rad'][j]-hi,0) for d in data for j,(lo,hi) in limits.items())
        hold=[d for d in data if 20<=d['time_s']<=26];lift=[d for d in data if 12<=d['time_s']<=28]
        start=lift[0] if lift else data[0]
        palm_excursion=max((abs(d['palm_world_position_m'][0]-start['palm_world_position_m'][0]) for d in lift),default=0)*1000
        pad_angles=[math.degrees(d['joint_angles_rad'][f'finger_{i}']+d['joint_angles_rad'][f'distal_{i}']) for d in hold for i in range(2)]
        checks=dict(cycle_and_attitude=r['passed'],two_finger_unsupported_hold=r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm'],stable_half_second_support_before_open=support,
            stable_centered_level_final_three_seconds=settled,observed_range_excess_under_0p03_rad=excess<.03,no_control_abort=r['abort_reason'] is None)
        if entry['open_control']:checks={}
        rows.append(dict(entry,cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],validation_passed=r['validation_passed'],abort_reason=r['abort_reason'],
            release_s=release,support_blocked=r['cartesian_cycle']['support_blocked'],max_palm_attitude_error_deg=r['max_palm_attitude_error_deg'],
            retained_through_hold=r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm'],
            added_object_lift_mm=r['arm_added_lift_m']*1000,lift_palm_x_excursion_mm=palm_excursion,max_body_drift_mm=r['max_body_drift_m']*1000,
            max_abs_hold_pad_angle_to_palm_deg=max(map(abs,pad_angles),default=None),max_hold_cube_tilt_deg=max((tilt(d) for d in hold),default=None),
            final_cube_tilt_deg=tilt(data[-1]),final_center_error_mm=center(data[-1])*1000,final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],
            unintended_arm_contact_points=r['unexpected_contact_samples'],observed_range_excess_rad=excess,placement_checks=checks,
            supported_placement_passed=bool(checks) and all(checks.values()),failed_checks=[k for k,v in r['checks'].items() if not v],
            report_sha256=sha(folder/'report.json'),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    positives=[r for r in rows if not r['open_control']];controls=[r for r in rows if r['open_control']]
    summary=dict(date='2026-10-24',cases=rows,positive_cycle_passes=sum(r['cycle_passed'] for r in positives),positive_trials=2,
        supported_placement_passes=sum(r['supported_placement_passed'] for r in positives),open_controls_passed=sum(r['negative_control_passed'] for r in controls),open_controls=2,
        configuration_adopted=False,analysis_sha256=sha(Path(__file__)))
    (study/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines=['# Cube coordinated-motion diagnostic','',m['question'],'',m['changes'],'',m['limitations'],'',
        '| Axis | Held unsupported | Added lift (mm) | Palm X excursion in lift (mm) | Max hold cube tilt (°) | Opening time | Strict placement |',
        '| --- | --- | ---: | ---: | ---: | ---: | --- |']
    for r in positives:lines.append(f"| {r['variant'].upper()} | {r['retained_through_hold']} | {r['added_object_lift_mm']:.2f} | {r['lift_palm_x_excursion_mm']:.2f} | {r['max_hold_cube_tilt_deg']} | {r['release_s']} | {r['supported_placement_passed']} |")
    lines+=['',f"{summary['positive_cycle_passes']}/2 positive cycles pass, {summary['open_controls_passed']}/2 open controls pass, and {summary['supported_placement_passes']}/2 strict supported placements pass. Geometry/control remains a private diagnostic candidate.",'',
        '## Interpretation','',
        'The controller fixes the shoulder, uses measured pitch to command elbow height, and shapes a wheel-position reference from encoder-derived palm reach so the hand stays over the fixed horizontal target. Original wheel gains and motor limits remain. Measured simulator palm XYZ is observational only; actual object state only qualifies release. The height controller assumes a planar, fixed wheel-height root and no-slip wheel odometry.','',
        'Opening needs 0.6 s ideal simulated pedestal support, speed below 5 mm/s, cube tilt below 2° and centering within 2 mm. Support seek stops as soon as support is detected, even if tilt/centering fail; there is no blind downward push to force a placement pass. Unintended arm contact or an unreachable height target halts new arm commands and holds the current base position for three seconds. A refusal or abort is recorded separately from task success. No safety or deployable sensing certification.','',
        'Strict placement retains the previous acceptance: cycle/attitude, two-finger unsupported hold, observed range excess below 0.03 rad, continuous level stable pedestal support for the last 0.5 s before opening, then three final seconds supported, finger-free, tilt below 2°, speed below 5 mm/s and center error below 2 mm. No thresholds relaxed.','']
    if args.study=='cube_coordinated':
        lines+=['Both open controls are clean and both positives retain/lift the cube, but neither opens. A palm pitch error falls to 0.138° and its lift X excursion to 5.97 mm, versus 11.06 mm in the fixed-base nominal plan. B lift X excursion rises from 1.64 to 4.50 mm and setdown still drifts. This is a mixed result, not a universal tracking improvement.','',
            'Measured proximal angles settle asymmetrically while distal angles remain near 0.25 rad. The retained cube tilts about 3.8° even with the level palm, so the stricter gate correctly refuses release. Distal-face pitch relative to the palm can be much larger than cube tilt. A level wrist does not make the contact faces parallel. Next test coordinates each distal joint against its measured proximal angle; any required joint-range changes must be modeled and re-screened explicitly.','']
    else:
        lines+=['Encoder-compensated distal faces reduce maximum hold cube tilt to 0.014° in A and 0.475° in B. Both retain through hold and add 7.81/7.83 mm lift. Both open controls pass, and all four runs record zero unintended arm contacts and zero observed range excess. This supports contact-face alignment as a useful task-specific change; it does not validate the complete mechanism.','',
            'Setdown still fails. The load-bearing contact transition disturbs the balancing base, with maximum palm attitude errors 4.41°/5.09° and maximum body drift 60.8/68.4 mm. Both hands remain closed because centering/stability is unconfirmed. Final cube centers are near the pedestal edge and bottom corners lie below its top; a contact-at-top-height witness alone is not proof of a centered load-bearing placement. Strict placement remains failed.','',
            'Next: controlled/compliant support transfer and a latched first-contact stop, plus force/load-transfer observability and physics checks. An RL environment is proposed separately to learn setdown/release against the same unchanged acceptance criteria, with explicit guards against contact-reward farming, staying at the initial support, and simulator penetration exploits. No RL implementation/training is claimed by this diagnostic.','']
    for r in rows:lines.append(f"- {r['case']}: abort {r['abort_reason']}; unwanted arm points {r['unintended_arm_contact_points']}; observed range excess {r['observed_range_excess_rad']:.6f} rad; failed cycle checks: {', '.join(r['failed_checks'])}.")
    lines+=['','Open controls are expected to fail pickup/hold checks. Their non-grasp validity, contacts, displacement and abort status are evaluated separately.','',
        '![Actual hand motion and cube tilt](coordination-contacts.png)','',
        'All four free-body runs, inputs, source hashes, raw contacts, telemetry and actual simulator frames are preserved in projects/. Source snapshot and checkpoint-index.json freeze the diagnostic. Main A/B, Beta and X remain unchanged; no hardware purchased.','']
    (study/'COORDINATION_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for i,v in enumerate(['a','b']):
        entry=next(r for r in positives if r['variant']==v);data=raw[entry['case']];anchor=read(WORKSPACE/entry['project']/'cartesian-plan.json')['anchor_world_xz_m']
        time=[d['time_s'] for d in data]
        axes[i,0].plot(time,[1000*(d['palm_world_position_m'][0]-anchor[0]) for d in data],label='Palm X error')
        axes[i,1].plot(time,[math.degrees(math.acos(max(-1,min(1,d['object_orientation_matrix'][8])))) for d in data],label='Cube tilt')
        axes[i,1].axhline(2,color='#b33',ls='--',label='Placement limit')
        axes[i,0].set(title=v.upper()+': actual world hand tracking',ylabel='X error (mm)');axes[i,1].set(title=v.upper()+': retained object attitude',ylabel='Cube tilt (°)')
    for ax in axes.flat:ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(study/'coordination-contacts.png',dpi=160);plt.close(fig)
    for name in ['run_cube_coordinated.py','summarize_cube_coordinated.py','coordinated_pick_place.py','cartesian_pick_place.py','run_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py','run_cube_parallel_pads.py']:
        path=MODEL/'scripts'/name
        if not path.exists():continue
        target=study/'source_snapshot'/path.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    files={str(p.relative_to(study)):sha(p) for p in study.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (study/'checkpoint-index.json').write_text(json.dumps(dict(date='2026-10-24',files=files),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
