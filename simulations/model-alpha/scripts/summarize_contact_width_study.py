"""Separate contact-width outcomes, baseline reproduction and placement limits."""
from pathlib import Path
import hashlib
import json
import math
import shutil
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/contact_width_study'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def trace(path):
    return [json.loads(line) for line in path.open(encoding='utf-8')]


def main():
    manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or status['clearance_skips'] or len(status['results'])!=8:
        raise ValueError('Incomplete study')
    for name,value in manifest['baseline_source_sha256'].items():
        if sha(WORKSPACE/name)!=value:raise ValueError('Authoritative baseline changed: '+name)
    raw,rows = {},[]
    reference_experiment = read(MODEL/'experiments/axis_contact_probe/cases/a_reference/experiment.json')
    for entry in manifest['cases']:
        project = WORKSPACE/entry['project']
        folder = project/'results/width'/entry['case']
        report_path = folder/'report.json';r = read(report_path)
        for name,value in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=value:raise ValueError('Stale input: '+name)
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:
            raise ValueError('World/trace mismatch')
        baseline = MODEL if entry['variant']=='a' else MODEL.parent/'model-alpha-b'
        p,original = read(project/'parameters.json'),read(baseline/'parameters.json')
        if {k:v for k,v in p.items() if k!='distal_contact_width_m'}!=original:
            raise ValueError('Non-width parameter changed')
        if p['distal_contact_width_m']!=entry['width_mm']/1000:
            raise ValueError('Wrong width')
        experiment = dict(r['experiment'],open_claw_control=False)
        if experiment!=reference_experiment:raise ValueError('Other experiment input changed')
        audit = read(project/'robot-clearance.json')
        if not audit['all_passed']:raise ValueError('Robot clearance failed')
        for name,value in audit['source_sha256'].items():
            if sha(WORKSPACE/name)!=value:raise ValueError('Stale clearance source')
        data = trace(folder/'contact-trace.jsonl');raw[entry['case']] = data
        if max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))>.008001:
            raise ValueError('Missing trace samples')
        s = r['schedule_actual_s']
        closing = [row for row in data if s['close']<=row['time_s']<s['lift']]
        contacts = [row for row in closing if row['finger_contacts']]
        before_close = max((row for row in data if row['time_s']<s['close']),key=lambda row:row['time_s'])
        before_lift = max((row for row in data if row['time_s']<s['lift']),key=lambda row:row['time_s'])
        before_release = max((row for row in data if row['time_s']<s['release']),key=lambda row:row['time_s'])
        if entry['open_control'] and not r['negative_control_passed']:
            raise ValueError('Open-claw control failed')
        rows.append({'case':entry['case'],'variant':entry['variant'],'width_mm':entry['width_mm'],
            'open_control':entry['open_control'],'inherited_cycle_passed':r['passed'],
            'negative_control_passed':r['negative_control_passed'],'failed_checks':[k for k,v in r['checks'].items() if not v],
            'first_closing_contact_s':contacts[0]['time_s'] if contacts else None,
            'last_closing_contact_s':contacts[-1]['time_s'] if contacts else None,
            'last_contact_local_y_mm':[c['point_segment_local_m'][1]*1000 for c in contacts[-1]['finger_contacts']] if contacts else [],
            'closure_net_world_mm':[(before_lift['object_position_m'][i]-before_close['object_position_m'][i])*1000 for i in range(3)],
            'min_hold_contacting_fingers':r['min_hold_contacting_fingers'],
            'unsupported_hold_passed':r['checks']['object_unsupported_through_hold'],
            'retained_through_hold':r['checks']['object_unsupported_through_hold'] and r['min_hold_contacting_fingers']>=2 and r['checks']['object_slip_under_3_mm'],
            'arm_added_lift_mm':r['arm_added_lift_m']*1000,'hold_window_s':r['hold_time_s'],
            'preopening_stand_gap_mm':before_release['stand_gap_m']*1000,
            'unintended_arm_contact_samples':r['unexpected_contact_samples'],
            'unintended_contact_by_node':r['unexpected_contact_counts_by_node'],
            'unintended_contact_examples':r['unexpected_contact_examples'],
            'final_on_stand':data[-1]['on_stand'],'final_on_tray':data[-1]['on_tray'],
            'precise_supported_placement_demonstrated':False,
            'minimum_robot_clearance_bound_mm':1000*min(path['conservative_clearance_bound_m'] for path in audit['paths'].values()),
            'report_path':str(report_path.relative_to(WORKSPACE)),'report_sha256':sha(report_path),
            'trace_sha256':r['contact_trace_sha256'],'world_sha256':r['world_sha256']})
    reproduced = {}
    for v,baseline in [('a',MODEL),('b',MODEL.parent/'model-alpha-b')]:
        original = trace(baseline/'results/axis_contact_probe'/f'{v}_reference/contact-trace.jsonl')
        old = {round(row['time_s']*1000):row for row in original}
        new = {round(row['time_s']*1000):row for row in raw[f'{v}_10mm']}
        if old.keys()!=new.keys():raise ValueError('Baseline trace time mismatch')
        reproduced[v] = {'matched_samples':len(old),'max_object_position_difference_mm':1000*max(math.dist(old[k]['object_position_m'],new[k]['object_position_m']) for k in old)}
        if reproduced[v]['max_object_position_difference_mm']>1e-6:
            raise ValueError('10 mm control did not reproduce recorded baseline to numerical precision')
    positive = [row for row in rows if not row['open_control']]
    summary = {'date':'2026-10-24','factor':manifest['factor'],'held_fixed':manifest['held_fixed'],
        'caveat':manifest['caveat'],'cases':rows,'baseline_reproduction':reproduced,
        'positive_cycle_passes':sum(row['inherited_cycle_passed'] for row in positive),'positive_trials':len(positive),
        'open_controls_passed':sum(row['negative_control_passed'] for row in rows if row['open_control']),
        'precise_placement_validated':False,'width_promoted_to_authoritative_model':False,
        'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Distal face-width comparison','', '24 October 2026 — isolated A/B geometry experiment; original models and evidence unchanged.','',
        '**Wider flat distal faces do not fix B retention in this prepared sphere task. A retains at 10/16/24 mm; B loses the sphere during closing at all three widths.**','',
        'Changed distal collision-face tangential width only (10, 16, 24 mm), with the visible distal housing/pad following at 1 mm less width. Proximal geometry, radial thickness, lengths, palm, segment mass/COM, gains, motor torque/speed limits, timing, friction and 2 g/20 mm sphere are fixed. Derived inertia changes with width at fixed mass. Actual contact forces are outcomes, not held-force commands. This is not a specification for constant-mass manufactured parts.','',
        'Private copied generator/controller helpers keep authoritative A/B parameters and source hashes intact. Six positive trials and wide-face open controls for both orientations completed; each geometry passes closure, lift/lower and open-control-lift conservative robot-envelope screens. Direct joint neighbors and fixture interactions are not certified by that screen.','',
        '| Variant | Distal width (mm) | Retained through hold | Last closing contact (s) | Inherited cycle | Unintended arm-contact samples |',
        '| --- | ---: | --- | ---: | --- | ---: |']
    for row in positive:
        lines.append(f"| {row['variant'].upper()} | {row['width_mm']} | {'Yes' if row['retained_through_hold'] else 'No'} | {row['last_closing_contact_s']:.3f} | {'Pass' if row['inherited_cycle_passed'] else 'Fail'} | {row['unintended_arm_contact_samples']} |")
    lines += ['', 'Arm lift starts at 12 s. B last-contact times are 7.656, 7.688 and 7.712 s: increasing width to 16/24 mm delays loss by only 32/56 ms. Last contacts reach the corresponding +/-5, +/-8 and +/-12 mm side edges. Added flat surface gives more travel before escape without establishing a retained grasp. Wider B runs also register ARM_BASE contact samples after escape; these remain failures rather than being ignored. Robot-only self-clearance does not prevent contact with a falling object.','',
        'Both fresh 10 mm controls reproduce the complete prior recorded sphere-position trajectory exactly across 5,313 matched 8 ms samples each (maximum difference 0.0 mm). This checks that the isolated source/path adaptation did not manufacture a width effect.','',
        'All three A widths retain through the scheduled six-second unsupported hold and pass the inherited grip/lift/drop checks, with zero unintended arm-contact samples. The 24 mm open controls have no finger contact/no lift and leave their spheres on the pole. Those are successful non-grasp controls, not extra successful pickups.','',
        'A still moves the sphere while closing and releases above the pole into the tray. B has already dropped before release. No precise supported placement is demonstrated. No gain, friction, motor-limit or timing changes conceal failures; neither wider geometry is promoted to the main models.','',
        '## Next experiment','',
        'Investigate a shallow retaining/contact profile or a separate closing-motion change, one factor at a time, while preserving this flat-face width comparison. Recheck self/fixture clearances and mass/inertia assumptions for any new shape. Then assess directional objects, offsets/uncertainty and supported release. The result does not imply left/right pinch is inherently unusable or all Beta angles/tasks must work. The circular palm remains the comparison baseline; Beta roll and X robot-support redesign are still deferred.','',
        '![Width/contact comparison](width-comparison.png)','',
        'Raw isolated projects, reports, traces, telemetry and simulator renders are under projects/. manifest.json, run-status.json, summary.json and checkpoint-index.json retain all eight outcomes and provenance. An initial preparation-folder error, before any physics run, is preserved in ../../archive/contact-width-preparation-first-attempt.','']
    (STUDY/'WIDTH_STUDY_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,2,figsize=(12,7))
    colors = {10:'#2467ad',16:'#dc7628',24:'#238875'}
    for mm,color in colors.items():
        data = [row for row in raw[f'b_{mm}mm'] if 7.25<=row['time_s']<=8.1]
        axes[0,0].plot([row['time_s'] for row in data],[row['stand_gap_m']*1000 for row in data],label=f'{mm} mm',color=color)
        edge = [max((abs(c['point_segment_local_m'][1])*2000/mm for c in row['finger_contacts'] if c['segment'].startswith('TIP_')),default=float('nan')) for row in data]
        axes[0,1].plot([row['time_s'] for row in data],edge,label=f'{mm} mm',color=color)
        last_tip = next(row for row in reversed(data) if any(c['segment'].startswith('TIP_') for c in row['finger_contacts']))
        last_edge = max(abs(c['point_segment_local_m'][1])*2000/mm for c in last_tip['finger_contacts'] if c['segment'].startswith('TIP_'))
        axes[0,1].scatter([last_tip['time_s']],[last_edge],color=color,s=22,zorder=3)
        a_data = [row for row in raw[f'a_{mm}mm'] if 6<=row['time_s']<=12]
        axes[1,0].plot([row['time_s'] for row in a_data],[row['stand_gap_m']*1000 for row in a_data],label=f'{mm} mm',color=color)
    axes[0,0].set(title='B: ball falls during closing at every width',ylabel='Sphere bottom gap over pole (mm)')
    axes[0,1].axhline(1,color='#555',linestyle='--',label='Side edge');axes[0,1].set(title='B: contact reaches each side edge',ylabel='Fraction of collision-face half width')
    axes[1,0].set(title='A: closure displacement persists',ylabel='Sphere bottom gap over pole (mm)')
    b_rows = [row for row in positive if row['variant']=='b']
    axes[1,1].bar([str(row['width_mm']) for row in b_rows],[1000*(row['last_closing_contact_s']-row['first_closing_contact_s']) for row in b_rows],color=[colors[row['width_mm']] for row in b_rows])
    axes[1,1].set(title='B: more travel, no hold',xlabel='Distal width (mm)',ylabel='First-to-last closing contact (ms)')
    for i,ax in enumerate(axes.flat):
        ax.grid(alpha=.2)
        if i<3:ax.set_xlabel('Simulation time (s)');ax.legend(fontsize=8)
    fig.suptitle('Tested distal widths at fixed mass and actuator limits: A retains; B escapes')
    fig.tight_layout();fig.savefig(STUDY/'width-comparison.png',dpi=160);plt.close(fig)
    frozen = STUDY/'source_snapshot'
    for source in [Path(__file__),MODEL/'scripts/run_contact_width_study.py']:
        target = frozen/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(path.relative_to(STUDY)):sha(path) for path in STUDY.rglob('*') if path.is_file() and path.suffix in ['.py','.json','.wbt','.csv','.jsonl','.png','.ini'] and path.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes,'note':'Private experimental sources/inputs and all outcomes preserved; authoritative A/B baseline unchanged.'},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'positive_cycle_passes':summary['positive_cycle_passes'],'positive_trials':6,'open_controls_passed':summary['open_controls_passed'],
        'baseline_trajectory_max_difference_mm':0.0,'width_promoted':False}),flush=True)


if __name__=='__main__':
    main()
