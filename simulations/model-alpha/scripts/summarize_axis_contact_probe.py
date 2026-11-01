"""Report seating, retention and release separately; retain the B failure."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = 'axis_contact_probe'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    output = MODEL/'results'/STUDY
    manifest = read(output/'manifest.json')
    status = read(output/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=4:
        raise ValueError('Incomplete physics probes')
    results,raw = {},{}
    experiments = {}
    for entry in manifest['cases']:
        model = WORKSPACE/entry['model_path']
        folder = model/'results'/STUDY/entry['case']
        report_path = folder/'report.json';r = read(report_path)
        for name,value in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=value:
                raise ValueError('Stale source/input: '+name)
        if sha(WORKSPACE/entry['world_path'])!=r['world_sha256']:
            raise ValueError('Stale world')
        trace_path = folder/'contact-trace.jsonl'
        if sha(trace_path)!=r['contact_trace_sha256']:
            raise ValueError('Trace hash mismatch')
        trace = [json.loads(line) for line in trace_path.open(encoding='utf-8')]
        if not trace or max(b['time_s']-a['time_s'] for a,b in zip(trace,trace[1:]))>.008001:
            raise ValueError('Missing 8 ms trace samples')
        raw[entry['case']] = trace
        experiments[entry['case']] = r['experiment']
        schedule = r['schedule_actual_s']
        before_close = max((row for row in trace if row['time_s']<schedule['close']),key=lambda row:row['time_s'])
        before_lift = max((row for row in trace if row['time_s']<schedule['lift']),key=lambda row:row['time_s'])
        before_release = max((row for row in trace if row['time_s']<schedule['release']),key=lambda row:row['time_s'])
        closing = [row for row in trace if schedule['close']<=row['time_s']<schedule['lift']]
        contacts = [row for row in closing if row['finger_contacts']]
        last = contacts[-1] if contacts else None
        is_control = entry['case'].endswith('open_control')
        if is_control and not r['negative_control_passed']:
            raise ValueError('Open-claw control failed: '+entry['case'])
        result = {'case':entry['case'],'variant':entry['variant'],'negative_control':is_control,
                  'inherited_grip_lift_drop_checks_passed':r['passed'],
                  'negative_control_passed':r['negative_control_passed'],
                  'failed_checks':[key for key,value in r['checks'].items() if not value],
                  'closure_net_displacement_world_mm':[(before_lift['object_position_m'][i]-before_close['object_position_m'][i])*1000 for i in range(3)],
                  'max_closure_stand_gap_mm':max(row['stand_gap_m'] for row in closing)*1000,
                  'first_closing_contact_s':contacts[0]['time_s'] if contacts else None,
                  'last_closing_contact_s':last['time_s'] if last else None,
                  'last_closing_contacts':last['finger_contacts'] if last else [],
                  'last_contact_object_palm_m':last['object_palm_m'] if last else None,
                  'closure_contact_segments':dict(Counter(c['segment'] for row in closing for c in row['finger_contacts'])),
                  'min_hold_contacting_fingers':r['min_hold_contacting_fingers'],
                  'unsupported_hold_check_passed':r['checks']['object_unsupported_through_hold'],
                  'hold_window_s':r['hold_time_s'],'arm_added_lift_mm':r['arm_added_lift_m']*1000,
                  'hold_slip_mm':r['max_hold_relative_slip_m']*1000 if r['max_hold_relative_slip_m'] is not None else None,
                  'preopening_stand_gap_mm':before_release['stand_gap_m']*1000,
                  'final_on_stand':trace[-1]['on_stand'],'final_on_tray':trace[-1]['on_tray'],
                  'max_contacting_fingers':r['max_contacting_fingers'],'unintended_arm_contact_samples':r['unexpected_contact_samples'],
                  'max_pitch_deg':r['max_pitch_deg'],'precise_supported_placement_demonstrated':False,
                  'report_path':str(report_path.relative_to(WORKSPACE)),'report_sha256':sha(report_path),
                  'trace_sha256':sha(trace_path),'world_sha256':r['world_sha256']}
        results[entry['case']] = result
    if experiments['a_reference']!=experiments['b_reference'] or experiments['a_open_control']!=experiments['b_open_control']:
        raise ValueError('A/B experiment inputs differ')
    summary = {'date':'2026-10-24','study':STUDY,'cases':results,
               'comparison_inputs':'Same 2 g, 20 mm sphere, prepared frame, friction 0.6, targets, schedule and known-load feedback. Only finger orientation/model labels differ.',
               'infrastructure_errors':0,'physics_runs':4,'positive_probe_passes':sum(results[k]['inherited_grip_lift_drop_checks_passed'] for k in ['a_reference','b_reference']),
               'matched_negative_controls_passed':2,'precise_placement_validated':False,
               'analysis_sha256':sha(Path(__file__)),
               'limitation':'One deterministic sphere trial per orientation plus open controls. Rigid contact and ideal simulator sensing/actuation; no hardware, directional-object or full-roll operating envelope.'}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    a,b = results['a_reference'],results['b_reference']
    lines = ['# A/B initial sphere-contact diagnostic','', '24 October 2026 — actual Webots runs, with full 8 ms contact/state traces.','',
             '**Alpha-A retains the sphere through the six-second unsupported hold; Alpha-B loses it during closing, before arm lift. Both open-claw controls leave the sphere on the pole. Neither positive probe demonstrates precise supported placement.**','',
             'The prepared sphere is 2 g and 20 mm diameter, with finger friction 0.6. Both variants use the same circular palm, mass/COM, gains, motor caps, main/finger targets, initial intended capture frame and timing. Known payload mass/location and simulator contacts feed the existing balance compensation. State/contact traces are observations only; no new object feedback, object force, attachment or pose reset is used.','',
             '| Measurement | Alpha-A: front/back | Alpha-B: left/right |','| --- | ---: | ---: |',
             f"| Inherited grip/lift/drop checks | {'Pass' if a['inherited_grip_lift_drop_checks_passed'] else 'Fail'} | {'Pass' if b['inherited_grip_lift_drop_checks_passed'] else 'Fail'} |",
             f"| First finger contact (s) | {a['first_closing_contact_s']:.3f} | {b['first_closing_contact_s']:.3f} |",
             f"| Last closing contact (s); arm lift starts at 12 s | {a['last_closing_contact_s']:.3f} | {b['last_closing_contact_s']:.3f} |",
             f"| Closure net X displacement before arm lift (mm) | {a['closure_net_displacement_world_mm'][0]:.3f} | {b['closure_net_displacement_world_mm'][0]:.3f} |",
             f"| Closure net Z displacement before arm lift (mm) | {a['closure_net_displacement_world_mm'][2]:.3f} | {b['closure_net_displacement_world_mm'][2]:.3f} |",
             f"| Minimum contacting digits during scheduled hold | {a['min_hold_contacting_fingers']} | {b['min_hold_contacting_fingers']} |",
             f"| Added arm lift (mm) | {a['arm_added_lift_mm']:.3f} | {b['arm_added_lift_mm']:.3f} |",
             f"| Gap over pole when opening starts (mm) | {a['preopening_stand_gap_mm']:.3f} | {b['preopening_stand_gap_mm']:.3f} |",
             '| Final location | Catch tray | Catch tray |',
             f"| Unintended arm contact samples | {a['unintended_arm_contact_samples']} | {b['unintended_arm_contact_samples']} |",'',
             'B initially touches the sphere with both distal segments at 7.360 s. Its last closing contacts at 7.656 s lie at local tangential coordinates -5/+5 mm: the side edges of the 10 mm-wide distal collision faces. At that time the sphere center has moved about 7.28 mm in palm-local X. This supports lateral escape across the narrow contact faces as a failure mechanism; contact forces, compliance and individual causal contributions were not measured. No extra actuator torque or friction was added to conceal the failure.','',
             'B briefly lifts the sphere during closure, then it drops into the tray before the 12 s arm-lift command. Its final negative Z displacement in the table includes that fall, not successful seating. The scheduled hold window still lasts six seconds, but has zero finger contacts and tray support; it is not a grasp. Small relative-motion/slip numbers alone would be misleading.','',
             'A closes around the sphere and moves it approximately -5.84 mm in world X and +17.43 mm in Z before arm lift. It retains the object during the six-second unsupported hold, but opening begins about 17.42 mm above the pole and the sphere finishes in the tray. This is the inherited grip/lift/drop task, not an exact pick-and-place demonstration. The user\'s concern about seating and release remains relevant.','',
             'The two matching open-claw controls have no finger/object contact or lift and keep the sphere on its pole; their positive grip criteria fail as expected. The B failed positive probe remains failed, with its original checks, exit status, source/input/world hashes, trace and renders. All four runs completed without infrastructure errors.','',
             '## Next controlled comparison','',
             'Preserve this baseline. Investigate lateral containment/contact-face geometry and closure sequencing with one factor changed at a time, using matched A/B conditions. Do not infer that changing only the palm from circular to rectangular fixes a finger-contact failure. Then add directional objects, offsets, friction/load uncertainty and supported setdown/release checks. B empty-motion success is distinct from object retention; these observations inform Beta angle-dependent limits, not a verdict that left/right pinching is inherently unusable.','',
             'A sphere is an initial seating diagnostic only. No continuous tolerance, hardware payload rating, directional-object preference, full 360-degree roll or X robot-weight support capability is established.','',
             '![Closing contact and release traces](contact-comparison.png)','',
             'See summary.json, manifest.json and run-status.json for the four original cases and provenance.','']
    (output/'CONTACT_PROBE_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,2,figsize=(12,7))
    for case,label,color in [('a_reference','Alpha-A','#2167ac'),('b_reference','Alpha-B','#d76b28')]:
        data = raw[case]
        closing = [row for row in data if 5.9<=row['time_s']<=12]
        axes[0,0].plot([row['time_s'] for row in closing],[row['stand_gap_m']*1000 for row in closing],label=label,color=color)
        face = [row for row in data if 7.25<=row['time_s']<=8]
        offsets = [max((abs(c['point_segment_local_m'][1])*1000 for c in row['finger_contacts'] if c['segment'].startswith('TIP_')),default=float('nan')) for row in face]
        axes[0,1].plot([row['time_s'] for row in face],offsets,label=label,color=color)
        axes[1,0].step([row['time_s'] for row in data],[len(row['contacting_fingers']) for row in data],where='post',label=label,color=color)
        release = [row for row in data if 35.8<=row['time_s']<=37.5]
        axes[1,1].plot([row['time_s'] for row in release],[row['stand_gap_m']*1000 for row in release],label=label,color=color)
    axes[0,0].set(title='Closing: sphere bottom relative to pole',ylabel='Gap (mm)')
    axes[0,1].axhline(5,color='#555',linestyle='--',label='Distal collision-face side edge')
    axes[0,1].set(title='Closing: contact moves across distal face',ylabel='Absolute local tangential offset (mm)')
    axes[1,0].set(title='Contacts through scheduled lift/hold/release',ylabel='Contacting digits',yticks=[0,1,2])
    axes[1,1].set(title='Opening: no supported pole placement',ylabel='Gap (mm)')
    for ax in axes.flat:
        ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('A/B sphere probe: retention, seating and release are separate outcomes')
    fig.tight_layout();fig.savefig(output/'contact-comparison.png',dpi=160);plt.close(fig)
    print(json.dumps({'positive_probe_passes':summary['positive_probe_passes'],'positive_probes':2,
                      'open_controls_passed':2,'precise_placement_validated':False}),flush=True)


if __name__=='__main__':
    main()
