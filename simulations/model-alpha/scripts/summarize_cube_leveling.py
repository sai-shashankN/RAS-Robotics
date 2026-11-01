"""Separate pitch tracking, grasping, control validity and supported placement."""
from pathlib import Path
import argparse
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace
from palm_leveling import angle_error

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--study',choices=['cube_leveling','cube_leveling_v2'],default='cube_leveling_v2');args=parser.parse_args()
    study=MODEL/'experiments'/args.study;manifest,status=read(study/'manifest.json'),read(study/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=8:raise ValueError('Incomplete attitude comparison')
    if sha(MODEL/'experiments/cube_baseline/checkpoint-index.json')!=manifest['prior_checkpoint_sha256']:raise ValueError('Original cube checkpoint changed')
    for name,digest in manifest.get('previous_reports_sha256',{}).items():
        if sha(WORKSPACE/name)!=digest:raise ValueError('First-controller evidence changed')
    rows,raw=[],{}
    for entry in manifest['cases']:
        project=WORKSPACE/entry['project'];folder=project/'results/physics'/entry['case'];r=read(folder/'report.json');p=read(project/'parameters.json')
        baseline=read(MODEL/'experiments/cube_baseline/projects'/entry['variant']/'parameters.json')
        baseline['wrist_standoff']=.035;baseline['finger_targets']['claw_open']['finger']=-.2
        baseline['dynamic_arm_targets']['arm_reach']['wrist']=p['dynamic_arm_targets']['arm_reach']['wrist']
        if p!=baseline:raise ValueError('Unlisted model input changed')
        if args.study=='cube_leveling_v2':
            previous=MODEL/'experiments/cube_leveling/projects'/project.name
            if sha(project/'parameters.json')!=sha(previous/'parameters.json'):raise ValueError('Geometry changed with wrist-slew revision')
            if r['experiment']!=read(previous/'results/physics'/entry['case']/'report.json')['experiment']:raise ValueError('Experiment changed with wrist-slew revision')
            for path in (study/'projects/common').glob('*.py'):
                if path.name!='grasp_controller.py' and sha(path)!=sha(MODEL/'experiments/cube_leveling/projects/common'/path.name):raise ValueError('Other helper changed with revision')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale attitude input')
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        data=trace(folder/'contact-trace.jsonl');raw[entry['case']]=data;s=r['schedule_actual_s']
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Missing trace samples')
        before=max((d for d in data if d['time_s']<s['release']),key=lambda d:d['time_s'])
        pre=[d for d in data if s['release']-.5<=d['time_s']<s['release']]
        final=[d for d in data if d['time_s']>=s['finish']-3];opening=[d for d in data if d['time_s']>=s['release']]
        contacts=[d for d in opening if d['finger_contacts']];stand=[d for d in opening if d['on_stand']]
        speed=lambda d:math.sqrt(sum(v*v for v in d['object_velocity'][:3]))
        tilt=lambda d:math.degrees(math.acos(max(-1,min(1,d['object_orientation_matrix'][8]))))
        initial=r['setup']['initial_object_position_m'];position=lambda d:math.hypot(d['object_position_m'][0]-initial[0],d['object_position_m'][1]-initial[1])
        support_before=bool(pre) and all(d['on_stand'] and speed(d)<.005 and tilt(d)<2 for d in pre)
        final_stable=bool(final) and all(d['on_stand'] and not d['contacting_fingers'] and speed(d)<.005 and tilt(d)<2 and position(d)<.002 for d in final)
        retained=r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm']
        limits=dict(p['joint_ranges']);limits.update({f'{kind}_{i}':bounds for kind,bounds in [('finger',[-.65,.35]),('distal',[0,1.1])] for i in range(2)})
        excess=max(max(lo-d['joint_angles_rad'][j],d['joint_angles_rad'][j]-hi,0) for d in data for j,(lo,hi) in limits.items())
        checks={'inherited_cycle_and_attitude_passed':r['passed'],'two_finger_hold_retained':retained,'support_stable_half_second_before_open':support_before,
            'stable_centered_2mm_level_2deg_final_three_seconds':final_stable,'observed_joint_range_excess_under_0p03_rad':excess<.03}
        if entry['open_control']:checks={}
        window=[d for d in data if s['close']-.5<=d['time_s']<=s['release']+2]
        rows.append(dict(entry,cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],retained_through_hold=retained,
            attitude_tracking_passed=r['checks']['palm_attitude_within_2_deg'],max_selected_frame_attitude_error_deg=r['max_palm_attitude_error_deg'],
            max_world_pitch_deviation_deg=max(math.degrees(abs(angle_error(d['palm_world_pitch_rad']))) for d in window),
            preopening_world_pitch_deviation_deg=math.degrees(abs(angle_error(before['palm_world_pitch_rad']))),
            wrist_goal_clip_samples=r['wrist_goal_clip_samples'],max_contacting_fingers=r['max_contacting_fingers'],max_object_displacement_mm=r['max_object_displacement_m']*1000,
            preopening_cube_tilt_deg=tilt(before),preopening_on_stand=before['on_stand'],preopening_gap_mm=before['stand_gap_m']*1000,
            last_opening_finger_contact_s=contacts[-1]['time_s'] if contacts else None,
            last_opening_finger_segments=sorted(set(c['segment'] for c in contacts[-1]['finger_contacts'])) if contacts else [],
            first_postopening_stand_contact_s=stand[0]['time_s'] if stand else None,
            final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],final_center_error_mm=position(data[-1])*1000,
            support_before_open_passed=support_before,stable_final_placement_passed=final_stable,placement_checks=checks,
            supported_placement_passed=bool(checks) and all(checks.values()),unintended_arm_contact_points=r['unexpected_contact_samples'],observed_range_excess_rad=excess,
            failed_checks=[k for k,v in r['checks'].items() if not v],initial_object_world_m=initial,
            report_path=str((folder/'report.json').relative_to(WORKSPACE)),report_sha256=sha(folder/'report.json'),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    positive=[r for r in rows if not r['open_control']];controls=[r for r in rows if r['open_control']]
    summary={'date':'2026-10-24','question':manifest['question'],'revision':manifest.get('revision','First coupled-interpolation attitude controller.'),'cases':rows,
        'positive_cycle_passes':sum(r['cycle_passed'] for r in positive),'positive_trials':4,
        'open_controls_passed':sum(r['negative_control_passed'] for r in controls),'open_controls':4,
        'supported_placement_passes':sum(r['supported_placement_passed'] for r in positive),'configuration_adopted':False,
        'analysis_sha256':sha(Path(__file__))}
    (study/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines=['# Cube palm-attitude comparison','',manifest['question'],'',summary['revision'],'',manifest['shared_geometry'],'',manifest['comparison'],'',manifest['limitations'],'',
        '| Variant/frame | Cycle incl. attitude | Max frame attitude error (°) | Cube held | Support before open | Stable final placement | Last finger contact after opening (s) |',
        '| --- | --- | ---: | --- | --- | --- | ---: |']
    for row in positive:
        last=f"{row['last_opening_finger_contact_s']-36:.3f}" if row['last_opening_finger_contact_s'] is not None else 'None'
        lines.append(f"| {row['variant'].upper()} / {row['frame']} | {'Pass' if row['cycle_passed'] else 'FAIL'} | {row['max_selected_frame_attitude_error_deg']:.3f} | {'Yes' if row['retained_through_hold'] else 'No'} | {'Yes' if row['support_before_open_passed'] else 'No'} | {'Yes' if row['stable_final_placement_passed'] else 'No'} | {last} |")
    lines+=['',f"{summary['positive_cycle_passes']}/4 positive cycles pass including the unchanged two-degree attitude threshold; {summary['open_controls_passed']}/4 open controls meet their non-grasp criteria. {summary['supported_placement_passes']}/4 positives meet all separately defined supported-placement checks.",'',
        '## Control validity and retained failures','']
    if args.study=='cube_leveling_v2':
        aw=next(r for r in positive if r['case']=='a_world');ab=next(r for r in positive if r['case']=='a_body')
        lines += [f"The revised loaded A world-frame run holds palm pitch within {aw['max_selected_frame_attitude_error_deg']:.3f}°. A cube tilt just before opening falls from {ab['preopening_cube_tilt_deg']:.2f}° in the body-frame trial to {aw['preopening_cube_tilt_deg']:.2f}° in the world-frame trial. Nevertheless, A still ends with fingertip contact and fails release-settling. Accurate palm attitude alone does not establish a clear release.",'',
            'Both revised A positives retain the cube through hold but fail release-settling; both B positives pass the cycle. Only B/body has a successful open control. A open controls touch the stationary cube, and B/world/open records arm-base/support contact during the motion. This configuration is rejected as a validated task baseline; the failed controls cannot be bypassed by selecting B or by relaxing acceptance.','']
    for row in rows:
        if row['failed_checks'] and (not row['open_control'] or not row['negative_control_passed']):lines.append(f"- {row['case']}: {', '.join(row['failed_checks'])}; max finger count {row['max_contacting_fingers']}; object displacement {row['max_object_displacement_mm']:.3f} mm.")
    lines+=['','Open controls are expected to fail pickup/hold checks. Any accidental finger contact or displacement remains a failed non-grasp control. Attitude tracking is recorded independently, so an open control with poor attitude is not proof of a valid level-grip comparison.','',
        '## Placement acceptance and interpretation','',
        'A supported-placement pass requires the inherited cycle and attitude checks, two-finger unsupported hold, observed range excess below 0.03 rad, continuous pedestal support with cube tilt under 2° and speed under 5 mm/s for the last 0.5 s before opening, and final three seconds on the pedestal with no finger contact, tilt under 2°, speed under 5 mm/s and center error under 2 mm. These are explicit prototype criteria; they are not a hardware accuracy rating. Support is observed for acceptance, not used to attach/reposition the object or to trigger opening in this fixed schedule.','',
        'World mode compensates pitch using IMU and shoulder/elbow encoders. It levels palm pitch; it does not add a roll actuator or certify arbitrary 3D orientation control. Holding attitude level is separate from moving along a vertical Cartesian line: the unchanged shoulder/elbow lift still follows a joint-space arc. Curling fingertip slopes and support transfer remain possible causes of drag even with correct attitude.','',
        'The original 15 mm wrist standoff failed the level-pose envelope screen for A. Merely lengthening the neck caused the flared open hand to approach the upper arm; a 35 mm standoff and smaller -0.2 rad opening provide nominal clearance. Rejected screens and pose surveys are retained. Fixed provisional palm/neck mass is an assumption to revisit in actual CAD, not free hardware mass.','',
        'The user explicitly asked to resolve A rather than rely on future Beta roll to switch to B. Neither frame is adopted by this diagnostic. Next changes must address any remaining dynamic tracking, open-control collision, Cartesian path and support-transfer failures; preserve A/B conditions and rerun affected checks.','',
        '![Palm pitch and release contact comparison](attitude-release.png)','',
        'Measured A/world loaded hold below. This shows the pitch-level hand, not successful supported placement.','',
        '![A world-frame cube hold](projects/a_world/results/physics/a_world/24_lift_hold.png)','',
        'All outcomes, raw reports/traces/telemetry, actual renders, sources and setup/world hashes are under projects/. checkpoint-index.json and source_snapshot/ preserve the study.','']
    (study/'LEVELING_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for i,v in enumerate(['a','b']):
        for frame,color in [('body','#d67a32'),('world','#2874a5')]:
            data=raw[f'{v}_{frame}'];window=[d for d in data if 5.5<=d['time_s']<=38];opening=[d for d in data if 36<=d['time_s']<=39]
            axes[i,0].plot([d['time_s'] for d in window],[math.degrees(angle_error(d['palm_world_pitch_rad'])) for d in window],label=frame,color=color)
            axes[i,1].plot([d['time_s'] for d in opening],[len(d['contacting_fingers']) for d in opening],label=frame,color=color)
        axes[i,0].set(title=f'{v.upper()}: world pitch relative to level',ylabel='Pitch error (°)');axes[i,0].axhline(0,color='#666',linewidth=.7)
        axes[i,1].set(title=f'{v.upper()}: finger contacts during opening',ylabel='Contacting fingers',yticks=[0,1,2])
    for ax in axes.flat:ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(study/'attitude-release.png',dpi=160);plt.close(fig)
    for name in ['run_cube_leveling.py','run_cube_leveling_v2.py','summarize_cube_leveling.py','palm_leveling.py','run_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        path=MODEL/'scripts'/name;target=study/'source_snapshot'/path.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    hashes={str(p.relative_to(study)):sha(p) for p in study.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (study/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['revision','positive_cycle_passes','open_controls_passed','supported_placement_passes']}),flush=True)


if __name__=='__main__':main()
