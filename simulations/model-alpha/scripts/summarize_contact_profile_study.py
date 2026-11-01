"""Verify profile inputs/evidence and separate retention from supported placement."""
from pathlib import Path
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/contact_profile_study'


def main():
    manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors']:raise ValueError('Infrastructure errors need review')
    for name,value in manifest['baseline_source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Authoritative baseline changed: '+name)
    raw,rows = {},[]
    reference = read(MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json')
    for entry in manifest['cases']:
        project = WORKSPACE/entry['project'];audit = read(project/'robot-clearance.json')
        for name,value in audit['source_sha256'].items():
            if sha(WORKSPACE/name)!=value:raise ValueError('Stale geometry source: '+name)
        if not entry['admitted_by_robot_clearance']:
            if audit['all_passed']:raise ValueError('Inconsistent clearance skip')
            rows.append(dict(entry,skipped=True));continue
        folder = project/'results/profile'/entry['case'];report_path = folder/'report.json';r = read(report_path)
        for name,value in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=value:raise ValueError('Stale source: '+name)
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        p = read(project/'parameters.json')
        baseline = MODEL if entry['variant']=='a' else MODEL.parent/'model-alpha-b'
        if {k:v for k,v in p.items() if k!='distal_profile_angle_deg'}!=read(baseline/'parameters.json'):raise ValueError('Other model parameter changed')
        if p['distal_profile_angle_deg']!=entry['profile_angle_deg']:raise ValueError('Wrong profile')
        if dict(r['experiment'],open_claw_control=False)!=reference:raise ValueError('Other experiment input changed')
        if not audit['all_passed']:raise ValueError('Unadmitted run')
        data = trace(folder/'contact-trace.jsonl');raw[entry['case']] = data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:raise ValueError('Missing trace samples')
        schedule = r['schedule_actual_s']
        closing = [d for d in data if schedule['close']<=d['time_s']<schedule['lift'] and d['finger_contacts']]
        before = lambda time:max((d for d in data if d['time_s']<time),key=lambda d:d['time_s'])
        start,lift,release = [before(schedule[name]) for name in ['close','lift','release']]
        if entry['open_control'] and not r['negative_control_passed']:raise ValueError('Failed non-grasp control')
        retained = r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm']
        rows.append(dict(entry,skipped=False,inherited_cycle_passed=r['passed'],negative_control_passed=r['negative_control_passed'],
            retained_through_hold=retained,failed_checks=[k for k,v in r['checks'].items() if not v],
            first_closing_contact_s=closing[0]['time_s'] if closing else None,last_closing_contact_s=closing[-1]['time_s'] if closing else None,
            last_contact_local_y_mm=[1000*c['point_segment_local_m'][1] for c in closing[-1]['finger_contacts']] if closing else [],
            closure_net_world_mm=[1000*(lift['object_position_m'][i]-start['object_position_m'][i]) for i in range(3)],
            min_hold_contacting_fingers=r['min_hold_contacting_fingers'],arm_added_lift_mm=1000*r['arm_added_lift_m'],hold_window_s=r['hold_time_s'],
            preopening_stand_gap_mm=1000*release['stand_gap_m'],unintended_arm_contact_samples=r['unexpected_contact_samples'],
            unintended_contact_by_node=r['unexpected_contact_counts_by_node'],unintended_contact_examples=r['unexpected_contact_examples'],
            final_on_stand=data[-1]['on_stand'],final_on_tray=data[-1]['on_tray'],precise_supported_placement_demonstrated=False,
            minimum_robot_clearance_bound_mm=1000*min(row['conservative_clearance_bound_m'] for row in audit['paths'].values()),
            report_path=str(report_path.relative_to(WORKSPACE)),report_sha256=sha(report_path),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    reproduced = {}
    for v,baseline in [('a',MODEL),('b',MODEL.parent/'model-alpha-b')]:
        old = {round(d['time_s']*1000):d for d in trace(baseline/'results/axis_contact_probe'/f'{v}_reference/contact-trace.jsonl')}
        new = {round(d['time_s']*1000):d for d in raw[f'{v}_0deg']}
        if old.keys()!=new.keys():raise ValueError('Baseline time mismatch')
        difference = 1000*max(math.dist(old[k]['object_position_m'],new[k]['object_position_m']) for k in old)
        if difference>1e-6:raise ValueError('Flat control changed baseline trajectory')
        reproduced[v] = {'matched_samples':len(old),'max_object_position_difference_mm':difference}
    positive = [r for r in rows if not r['open_control'] and not r['skipped']]
    controls = [r for r in rows if r['open_control'] and not r['skipped']]
    summary = {'date':'2026-10-24','factor':manifest['factor'],'held_fixed':manifest['held_fixed'],'caveat':manifest['caveat'],
        'cases':rows,'baseline_reproduction':reproduced,'positive_cycle_passes':sum(r['inherited_cycle_passed'] for r in positive),
        'positive_trials':len(positive),'open_controls_passed':sum(r['negative_control_passed'] for r in controls),
        'precise_placement_validated':False,'profile_promoted_to_authoritative_model':False,
        'next_candidate':'10-degree V: smallest tested nonzero profile; exploratory robustness candidate only.',
        'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Shallow V contact-profile comparison','','24 October 2026 — isolated A/B diagnostic; original models and evidence unchanged.','',
        '**Both 10° and 15° profiles retain the sphere in A and B through the six-second unsupported hold. Flat B reproduces its earlier closure escape. This is a grip-retention improvement in one prepared scene; precise placement remains unresolved.**','',
        manifest['factor'],'',manifest['held_fixed'],'',manifest['caveat'],'',
        '## Recorded outcomes','',
        '| Variant | V facet angle | Retained through hold | Last closing contact (s) | Inherited cycle | Unintended arm-contact samples |',
        '| --- | ---: | --- | ---: | --- | ---: |']
    for r in positive:
        last = f"{r['last_closing_contact_s']:.3f}" if r['last_closing_contact_s'] is not None else 'None'
        lines.append(f"| {r['variant'].upper()} | {r['profile_angle_deg']}° | {'Yes' if r['retained_through_hold'] else 'No'} | {last} | {'Pass' if r['inherited_cycle_passed'] else 'Fail'} | {r['unintended_arm_contact_samples']} |")
    lines += ['',f"{len(positive)} positive trials completed, with {summary['positive_cycle_passes']} inherited cycle passes. {summary['open_controls_passed']}/{len(controls)} open-claw controls pass their non-grasp criteria; these are not successful pickups. {len(status['clearance_skips'])} clearance skips and {len(status['infrastructure_errors'])} infrastructure errors.",'',
        'All eight runs record zero unintended arm-contact samples. Both open controls leave the sphere on its pole with no finger contact or added lift. Every positive run ends in the catch tray after release, including the successful retained grasps.','',
        'The new profiles still move the object while closing: A changes by about -6.02/+18.04 mm (world X/Z) at 10° and -6.64/+18.76 mm at 15°; B by -6.44/+17.98 mm and -6.62/+18.76 mm. Just before opening, the retained spheres remain about 18.0–18.8 mm above the pole. The improved contact shape does not solve support transfer or exact pickup position.','',
        'Closing begins at 6 s and arm lift at 12 s; the scheduled unsupported hold runs for six seconds. Closing itself can displace the sphere before arm motion. Inspect closure_net_world_mm and preopening_stand_gap_mm in summary.json rather than treating palm pose as object pose.','',
        'The profile consists of two separate convex half-prisms, so a convex hull cannot fill the V recess. Its inner center stays at local x=-6.5 mm; the edges extend inward by 0.882/1.340 mm at 10/15 degrees. The shape is unchanged along the 24 mm fingertip length. This tests a tangential channel rather than a complete spherical socket. Flat controls emit the original collision box and original visual pad.','',
        'Both fresh flat controls reproduce the original recorded sphere-position trajectory to numerical precision; matched sample counts and maximum differences are in summary.json. Conservative clearance envelopes enclose all new mesh vertices. Closure, lift/lower and open-control-lift paths pass for admitted geometries. This screen excludes direct joint neighbors and does not certify arbitrary workspace, detailed joint pockets or fixture clearance.','',
        'No new profile is promoted to the main models. Any retention improvement is evidence for this prepared 2 g/20 mm sphere scene only. Gain, friction, speed/torque limits, timing and finger trajectories are unchanged. Fixed 8 g segment mass/COM with automatic inertia is a diagnostic assumption requiring a real material/actuator layout before hardware selection. Prepared-scene unintended-contact monitoring covers the arm, not root body and wheels.','',
        'Precise supported placement is not validated: the inherited release criterion permits eventual catch-tray support. Offset/size/friction sensitivity, directional objects, repeated grasp trials, supported release and candidate empty-motion checks remain separate work. Alpha-A/B remain fixed axes; Beta powered roll and X support/lifting redesign stay deferred.','',
        '## Next experiment','',
        'Use the 10° profile, the smallest tested retaining angle, as an isolated robustness candidate. First test a matched A/B offset, sphere-size and friction neighborhood without retuning the controller, retaining misses and open controls. Separately test directional objects and contact-aware supported lowering/release; then run appropriate empty-motion regressions before adopting any geometry. This is a research choice, not a claim that 10° is optimal.','',
        '![Profile cross sections](profile-shapes.png)','',
        '![Profile contact traces](profile-comparison.png)','',
        'Actual simulator frame: B, 10° profile, at 24 s during unsupported hold. The inherited image heading says Model Alpha; case b_10deg and the saved model/world hashes identify the B orientation.','',
        '![B 10-degree unsupported sphere hold](projects/b_10deg/results/profile/b_10deg/24_lift_hold.png)','',
        'manifest.json, run-status.json, summary.json and checkpoint-index.json retain provenance. projects/ contains the private helpers, geometry screens, worlds, experiment inputs, reports, contact traces, telemetry, logs and actual simulator renders. source_snapshot/ preserves the study scripts.','']
    (STUDY/'PROFILE_STUDY_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,2,figsize=(12,7));colors = {0:'#2467ad',10:'#dc7628',15:'#238875'}
    for i,v in enumerate(['a','b']):
        for angle,color in colors.items():
            key = f'{v}_{angle}deg'
            if key not in raw:continue
            data = [d for d in raw[key] if 6<=d['time_s']<=26]
            axes[i,0].plot([d['time_s'] for d in data],[d['stand_gap_m']*1000 for d in data],color=color,label=f'{angle}°')
            axes[i,1].plot([d['time_s'] for d in data],[len(d['contacting_fingers']) for d in data],color=color,label=f'{angle}°')
        axes[i,0].set(title=f'{v.upper()}: sphere support gap',ylabel='Sphere bottom gap over pole (mm)')
        axes[i,1].set(title=f'{v.upper()}: contacting fingers',ylabel='Finger count',yticks=[0,1,2])
    for ax in axes.flat:
        ax.axvline(12,color='#777',linestyle='--',linewidth=.8,label='Arm lift')
        ax.axvspan(20,26,color='#888',alpha=.08);ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Shallow V profile at original width: observed contact, with hold window shaded')
    fig.tight_layout();fig.savefig(STUDY/'profile-comparison.png',dpi=160);plt.close(fig)
    sys.path.insert(0,str(STUDY/'projects/common'))
    from contact_profile import section
    fig,axes = plt.subplots(1,3,figsize=(11,4),sharex=True,sharey=True)
    for ax,angle in zip(axes,[0,10,15]):
        for polygon in section(angle):
            xy = [(y*1000,x*1000) for x,y in polygon]
            ax.fill([p[0] for p in xy],[p[1] for p in xy],color=colors[angle],alpha=.55,edgecolor='#16344a')
        ax.set(title=f'{angle}° per side',xlabel='Tangential y (mm)',aspect='equal',ylim=(-9,8))
        ax.axhline(-6.5,color='#666',linestyle='--',linewidth=.7)
        ax.text(0,-8.4,'Object side',ha='center',fontsize=9);ax.grid(alpha=.15)
    axes[0].set_ylabel('Radial x (mm)')
    fig.suptitle('Distal contact cross section: same 10 mm width, inward retaining edges')
    fig.tight_layout();fig.savefig(STUDY/'profile-shapes.png',dpi=160);plt.close(fig)
    for source in [Path(__file__),MODEL/'scripts/run_contact_profile_study.py',MODEL/'scripts/run_contact_width_study.py',MODEL/'scripts/summarize_contact_width_study.py']:
        target = STUDY/'source_snapshot'/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(path.relative_to(STUDY)):sha(path) for path in STUDY.rglob('*') if path.is_file() and path.suffix in ['.py','.json','.wbt','.wbproj','.csv','.jsonl','.png','.ini','.md','.log'] and path.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes,'note':'Private study inputs and all outcomes; original A/B unchanged.'},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:summary[k] for k in ['positive_cycle_passes','positive_trials','open_controls_passed','baseline_reproduction']}),flush=True)


if __name__=='__main__':main()
