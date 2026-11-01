"""Verify one-factor candidate trials; do not turn a tested grid into a guarantee."""
from pathlib import Path
import json
import math
import shutil
import sys
from run_profile_robustness import conditions
from summarize_contact_width_study import sha,read,trace

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/profile_robustness'
PREVIOUS = MODEL/'experiments/contact_profile_study'


def main():
    manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=30:raise ValueError('Incomplete study')
    if {r['case'] for r in status['results']}!={r['case'] for r in manifest['cases']}:raise ValueError('Wrong run set')
    for group in ['baseline_source_sha256','inherited_source_sha256']:
        for name,digest in manifest[group].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Protected source changed: '+name)
    expected = dict(conditions());base = read(MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json')
    rows,raw = [],{}
    for entry in manifest['cases']:
        if entry['changes']!=expected[entry['condition']]:raise ValueError('Study condition changed')
        project = WORKSPACE/entry['project'];prior = PREVIOUS/'projects'/f"{entry['variant']}_10deg"
        if sha(project/'parameters.json')!=sha(prior/'parameters.json'):raise ValueError('Candidate model changed')
        for path in (STUDY/'projects/common').glob('*.py'):
            if sha(path)!=sha(PREVIOUS/'projects/common'/path.name):raise ValueError('Common helper changed')
        inheritance = read(project/'clearance-inheritance.json')
        prior_screen = WORKSPACE/inheritance['prior_screen']
        if sha(prior_screen)!=inheritance['prior_screen_sha256'] or not read(prior_screen)['all_passed']:raise ValueError('Clearance inheritance changed')
        for name,digest in read(prior_screen)['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale prior clearance source')
        folder = project/'results/robustness'/entry['case'];report_path = folder/'report.json';r = read(report_path)
        if r['experiment']!=dict(base,**entry['changes']):raise ValueError('Other experiment input changed')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale input: '+name)
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        run = next(row for row in status['results'] if row['case']==entry['case'])
        if run['passed']!=r['passed'] or run['validation_passed']!=r['validation_passed'] or run['exit_code']!=(0 if r['validation_passed'] else 1):raise ValueError('Run/report mismatch')
        data = trace(folder/'contact-trace.jsonl');raw[entry['case']] = data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Missing trace samples')
        schedule = r['schedule_actual_s']
        before = lambda t:max((d for d in data if d['time_s']<t),key=lambda d:d['time_s'])
        start,lift,release = [before(schedule[name]) for name in ['close','lift','release']]
        preclose = [d for d in data if schedule['close']-.5<=d['time_s']<schedule['close']]
        preclose_contacts = sum(bool(d['contacting_fingers']) for d in preclose)
        preclose_displacement = max(math.dist(d['object_position_m'],r['setup']['initial_object_position_m']) for d in preclose)
        clean_preclose = bool(preclose) and preclose_contacts==0 and all(d['on_stand'] for d in preclose) and preclose_displacement<.0005
        contact = [d for d in data if schedule['close']<=d['time_s']<schedule['lift'] and d['finger_contacts']]
        p = read(project/'parameters.json')
        limits = dict(p['joint_ranges'])
        for i in range(2):limits.update({f'finger_{i}':[-.65,.35],f'distal_{i}':[0,1.1]})
        excess = {name:max(max(lo-d['joint_angles_rad'][name],d['joint_angles_rad'][name]-hi,0) for d in data)
                  for name,(lo,hi) in limits.items() if name in data[0]['joint_angles_rad']}
        final = data[-1];initial = r['setup']['initial_object_position_m'];radius = r['experiment']['object_radius_m']
        inside_tray = all(abs(final['object_position_m'][i]-initial[i])<bound-radius for i,bound in [(0,.10),(1,.08)])
        penetration = max(0,r['setup']['tray_top_m']-(final['object_position_m'][2]-radius))*1000 if inside_tray else 0
        physics_screen = max(excess.values())<.03 and penetration<1
        retained = r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm']
        if entry['open_control'] and not r['negative_control_passed']:raise ValueError('Open control failed')
        rows.append(dict(entry,inherited_cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],
            retained_through_hold=retained,failed_checks=[k for k,v in r['checks'].items() if not v],
            clean_preclose_window=clean_preclose,preclose_trace_contact_samples=preclose_contacts,
            preclose_max_displacement_mm=1000*preclose_displacement,
            intended_range_excess_rad_by_joint=excess,final_tray_interpenetration_mm=penetration,
            observational_physics_screen_passed=physics_screen,
            first_closing_contact_s=contact[0]['time_s'] if contact else None,last_closing_contact_s=contact[-1]['time_s'] if contact else None,
            last_closing_contact_local_y_mm=[1000*c['point_segment_local_m'][1] for c in contact[-1]['finger_contacts']] if contact else [],
            last_closing_contact_object_palm_m=contact[-1]['object_palm_m'] if contact else None,
            closure_net_world_mm=[1000*(lift['object_position_m'][i]-start['object_position_m'][i]) for i in range(3)],
            arm_added_lift_mm=1000*r['arm_added_lift_m'],min_hold_contacting_fingers=r['min_hold_contacting_fingers'],
            max_hold_relative_slip_mm=1000*r['max_hold_relative_slip_m'] if r['max_hold_relative_slip_m'] is not None else None,
            hold_window_s=r['hold_time_s'],preopening_stand_gap_mm=1000*release['stand_gap_m'],
            unintended_arm_contact_samples=sum(d['unintended_arm_contact_points']>0 for d in data),
            unintended_arm_contact_points_total=r['unexpected_contact_samples'],unintended_contact_by_node=r['unexpected_contact_counts_by_node'],
            unintended_contact_examples=r['unexpected_contact_examples'],max_pitch_deg=r['max_pitch_deg'],max_pitch_error_deg=r['max_pitch_error_deg'],
            final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],final_position_m=data[-1]['object_position_m'],
            report_path=str(report_path.relative_to(WORKSPACE)),report_sha256=sha(report_path),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    reproduction = {}
    for v in ['a','b']:
        old = {round(d['time_s']*1000):d for d in trace(PREVIOUS/'projects'/f'{v}_10deg/results/profile/{v}_10deg/contact-trace.jsonl')}
        reproduction[v] = {}
        for suffix in ['reference','reference_repeat']:
            new = {round(d['time_s']*1000):d for d in raw[f'{v}_{suffix}']}
            if old.keys()!=new.keys():raise ValueError('Reference trace time mismatch')
            error = 1000*max(math.dist(old[k]['object_position_m'],new[k]['object_position_m']) for k in old)
            if error>1e-6:raise ValueError('Candidate reference changed trajectory')
            reproduction[v][suffix] = {'matched_samples':len(old),'max_position_difference_mm':error}
    positives = [r for r in rows if not r['open_control']];controls = [r for r in rows if r['open_control']]
    summary = {'date':'2026-10-24','candidate':manifest['candidate'],'design':manifest['design'],'held_fixed':manifest['held_fixed'],
        'limitations':manifest['limitations'],'cases':rows,'reference_reproduction':reproduction,
        'positive_cycle_passes':sum(r['inherited_cycle_passed'] for r in positives),'positive_trials':len(positives),
        'retained_through_hold_count':sum(r['retained_through_hold'] for r in positives),
        'clean_preclose_windows':sum(r['clean_preclose_window'] for r in rows),
        'cycle_and_observational_physics_passes':sum(r['inherited_cycle_passed'] and r['observational_physics_screen_passed'] and r['clean_preclose_window'] for r in positives),
        'physics_screen_flagged_cases':[r['case'] for r in rows if not r['observational_physics_screen_passed']],
        'open_controls_passed':sum(r['negative_control_passed'] for r in controls),
        'unintended_contact_cases':[r['case'] for r in rows if r['unintended_arm_contact_points_total']],
        'by_variant':{v:{'positive_trials':sum(r['variant']==v for r in positives),
            'cycle_passes':sum(r['variant']==v and r['inherited_cycle_passed'] for r in positives),
            'retained_through_hold':sum(r['variant']==v and r['retained_through_hold'] for r in positives)} for v in ['a','b']},
        'profile_adopted':False,'precise_supported_placement_validated':False,'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# 10-degree V profile: one-factor neighborhood','','24 October 2026 — separate study; previous checkpoints and main models unchanged.','',
        f"**{summary['positive_cycle_passes']}/{len(positives)} positive trials pass the inherited grip/lift/drop cycle; {summary['retained_through_hold_count']} retain through hold. Both orientations have a reference repeat and open-claw control.**",'',
        manifest['candidate'],'',manifest['design'],'',manifest['held_fixed'],'',manifest['limitations'],'',
        '| Condition | A cycle | B cycle | A/B retained through hold | A/B unintended arm-contact samples |',
        '| --- | --- | --- | --- | ---: |']
    for condition,_ in conditions():
        a,b = [next(r for r in rows if r['variant']==v and r['condition']==condition) for v in ['a','b']]
        label = lambda r:('Control pass' if r['negative_control_passed'] else 'Control FAIL') if r['open_control'] else ('Pass' if r['inherited_cycle_passed'] else 'FAIL')
        lines.append(f"| {condition} | {label(a)} | {label(b)} | {'Yes' if a['retained_through_hold'] else 'No'} / {'Yes' if b['retained_through_hold'] else 'No'} | {a['unintended_arm_contact_samples']} / {b['unintended_arm_contact_samples']} |")
    lines += ['', 'Pass refers to the inherited retained-grasp, arm-lift, hold, balance/contact and eventual release-settling checks. It is not precise pole placement. Open controls are successful non-grasps, not additional pickups. A retained object can still fail other task checks; raw failed_checks are preserved. Contact-sample columns count trace timestamps with unintended contacts; full-monitor point counts are saved separately.','',
        '## Failures and contacts','']
    failures = [r for r in positives if not r['inherited_cycle_passed']]
    if failures:
        for r in failures:
            lines.append(f"- **{r['case']}**: {', '.join(r['failed_checks'])}. Last closing contact: {r['last_closing_contact_s']} s; minimum hold fingers: {r['min_hold_contacting_fingers']}; unintended arm-contact samples: {r['unintended_arm_contact_samples']}.")
    else:lines.append('No positive cycle failures in this sampled grid. This does not establish a continuous tolerance envelope or performance on combined variations.')
    lines += ['', 'Unintended contact examples and per-node counts are in summary.json; none are silently excluded from cycle acceptance. Prepared monitoring covers arm nodes, not whole robot body and wheels. Each result includes a 48 s run or an explicitly reported early failure.','',
        '## Simulation fidelity observations','',
        f"{summary['cycle_and_observational_physics_passes']}/{len(positives)} positive trials also pass an observational screen for joint range excess below 0.03 rad and final tray penetration below 1 mm, plus the pre-close screen. Flagged physics cases: {', '.join(summary['physics_screen_flagged_cases']) or 'none'}. These diagnostic thresholds are not hardware tolerances or new controller guards.",'',
        'The inherited generator sets motor target limits but does not set HingeJointParameters minStop/maxStop; those fields exist separately in the installed Webots node definitions. Contact loads can therefore move a digit outside the intended mechanical range. Also, the 8 ms step and thin tray require a resolution check when a released sphere penetrates a support. Preserve these observations before adding stops or changing the step in a separate diagnostic; do not silently turn an embedded sphere into a successful settled release.','',
        '## Reference checks and interpretation','',
        'Fresh reference and repeat runs for both axes reproduce the earlier 10° candidate sphere-position trajectory to numerical precision at all matched timestamps. Repeating a deterministic simulation checks reproducibility, not a statistical success rate. Every trial has an 8 ms observational contact trace; there is no object attachment, repositioning after setup or live object-pose steering.','',
        f"Additional observational pre-close screen: {summary['clean_preclose_windows']}/30 trials remain on-pole with no finger contact and less than 0.5 mm displacement from setup throughout the final 0.5 s before closing. This screen is separate from inherited acceptance checks and does not cover all earlier setup time.",'',
        'The robot-only clearance screen is inherited because the geometry and commanded paths are identical to the checked 10° candidate. Sphere diameter and position changes also move the pole surface so the object begins supported; fixture interactions are evaluated dynamically rather than certified by the robot-only screen. The nominal target used by control remains fixed.','',
        'Closure displacement, added arm lift, slip, pre-opening pole gap, final stand/tray state and balance metrics are recorded separately. A catch-tray finish does not validate the user’s requested supported release or exact object positioning. No profile is promoted by this study. Alpha/Beta remain manipulation experiments; X support/lifting and powered Beta roll are separate.','',
        '![Paired one-factor trial outcomes](robustness-outcomes.png)','',
        'manifest.json, run-status.json, summary.json and checkpoint-index.json preserve the complete trial set. projects/ contains private sources, worlds, initial setups, raw reports, telemetry, traces, simulator renders and logs. source_snapshot/ retains runner/analysis code.','']
    (STUDY/'ROBUSTNESS_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    names = [name for name,_ in conditions()];matrix = []
    for name in names:
        matrix.append([1 if next(r for r in rows if r['condition']==name and r['variant']==v)['negative_control_passed' if name=='open_control' else 'inherited_cycle_passed'] else 0 for v in ['a','b']])
    fig,ax = plt.subplots(figsize=(7,8));ax.imshow(np.array(matrix),cmap=matplotlib.colors.ListedColormap(['#e28c82','#8fc4a6']),vmin=0,vmax=1,aspect='auto')
    ax.set(xticks=[0,1],xticklabels=['Alpha-A','Alpha-B'],yticks=range(len(names)),yticklabels=names)
    for i,row in enumerate(matrix):
        for j,value in enumerate(row):ax.text(j,i,('CONTROL ' if names[i]=='open_control' else '')+('PASS' if value else 'FAIL'),ha='center',va='center',fontsize=9)
    ax.set_title('10° V profile: paired one-factor trials\nCycle outcomes; open controls are non-grasps')
    fig.tight_layout();fig.savefig(STUDY/'robustness-outcomes.png',dpi=160);plt.close(fig)
    for name in ['run_profile_robustness.py','summarize_profile_robustness.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        source = MODEL/'scripts'/name;target = STUDY/'source_snapshot'/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(p.relative_to(STUDY)):sha(p) for p in STUDY.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes,'note':'All one-factor outcomes and private inputs preserved; no profile adopted.'},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['positive_cycle_passes','positive_trials','retained_through_hold_count','open_controls_passed','unintended_contact_cases','by_variant']}),flush=True)


if __name__=='__main__':main()
