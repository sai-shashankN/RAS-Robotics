"""Separate observed stop/rate fixes from the unchanged 30-run experiment."""
from pathlib import Path
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/profile_robustness'
STUDY = MODEL/'experiments/profile_physics_checks'


def metrics(report,data,p):
    limits = dict(p['joint_ranges'])
    for i in range(2):limits.update({f'finger_{i}':[-.65,.35],f'distal_{i}':[0,1.1]})
    excess = {name:max(max(lo-d['joint_angles_rad'][name],d['joint_angles_rad'][name]-hi,0) for d in data)
              for name,(lo,hi) in limits.items() if name in data[0]['joint_angles_rad']}
    end = data[-1];initial = report['setup']['initial_object_position_m'];radius = report['experiment']['object_radius_m']
    inside = all(abs(end['object_position_m'][i]-initial[i])<bound-radius for i,bound in [(0,.10),(1,.08)])
    penetration = max(0,report['setup']['tray_top_m']-(end['object_position_m'][2]-radius))*1000 if inside else 0
    return {'cycle_passed':report['passed'],'negative_control_passed':report['negative_control_passed'],
        'retained_through_hold':report['checks']['object_unsupported_through_hold'] and report['min_hold_contacting_fingers']>=2 and report['checks']['object_slip_under_3_mm'],
        'failed_checks':[k for k,v in report['checks'].items() if not v],
        'max_intended_joint_range_excess_rad':max(excess.values()),'range_excess_by_joint_rad':excess,
        'unintended_arm_contact_points':report['unexpected_contact_samples'],
        'unintended_arm_contact_trace_samples':sum(d['unintended_arm_contact_points']>0 for d in data),
        'final_tray_interpenetration_mm':penetration,'final_on_stand':end['on_stand'],'final_on_tray':end['on_tray'],
        'max_pitch_deg':report['max_pitch_deg'],'arm_added_lift_mm':1000*report['arm_added_lift_m'],
        'trace_samples':len(data),'maximum_trace_step_ms':1000*max(b['time_s']-a['time_s'] for a,b in zip(data,data[1:]))}


def main():
    manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=6:raise ValueError('Incomplete physics checks')
    if sha(PRIOR/'checkpoint-index.json')!=manifest['prior_checkpoint_sha256']:raise ValueError('Prior checkpoint changed')
    rows = []
    for entry in manifest['cases']:
        project = WORKSPACE/entry['project'];folder = project/'results/physics'/entry['case'];r = read(folder/'report.json')
        p = read(project/'parameters.json');original = read(PRIOR/'projects'/entry['variant']/'parameters.json')
        if {k:v for k,v in p.items() if k not in ['physical_joint_stops','world_basic_time_step_ms']}!=original:raise ValueError('Other model input changed')
        old_path = WORKSPACE/entry['comparison_report'];old = read(old_path)
        if r['experiment']!=old['experiment']:raise ValueError('Experiment changed')
        for path in (STUDY/'projects/common').glob('*.py'):
            if path.name!='generate_worlds.py' and sha(path)!=sha(PRIOR/'projects/common'/path.name):raise ValueError('Control helper changed')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale diagnostic source')
        world = WORKSPACE/entry['world']
        if sha(world)!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        source = world.read_text(encoding='utf-8')
        if source.count('minStop ')!=(7 if entry['physical_stops'] else 0) or f"basicTimeStep {entry['basic_time_step_ms']} " not in source:raise ValueError('Wrong stop/step settings')
        data = trace(folder/'contact-trace.jsonl');old_data = trace(old_path.parent/'contact-trace.jsonl')
        current,baseline = metrics(r,data,p),metrics(old,old_data,original)
        if current['maximum_trace_step_ms']>entry['basic_time_step_ms']+.0001:raise ValueError('Missing trace samples')
        if entry['open_control'] and not r['negative_control_passed']:raise ValueError('Stop control failed')
        rows.append(dict(entry,current=current,baseline=baseline,report_sha256=sha(folder/'report.json'),
            trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    a = next(r for r in rows if r['case']=='a_y_pos4_stops8');b = next(r for r in rows if r['case']=='b_16mm_step2')
    summary = {'date':'2026-10-24','cases':rows,'changes':manifest['changes'],'scope':manifest['scope'],
        'step_cadence_caveat':'The existing controller derives its update period from basicTimeStep; reducing it to 2 ms changes physics, control and observation cadence together. This is not a fixed-control-rate solver isolation.',
        'stops_limit_observed_excursion':a['current']['max_intended_joint_range_excess_rad']<.03,
        'stop_case_has_no_unintended_arm_contacts':a['current']['unintended_arm_contact_points']==0,
        'finer_step_resolves_observed_embedding':b['current']['final_tray_interpenetration_mm']<1 and b['current']['final_on_tray'],
        'profile_promoted':False,'complete_convergence_validated':False,'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Focused simulation-physics checks','','24 October 2026 — separate from the unchanged 30-run candidate study.','',manifest['changes'],'',manifest['scope'],'',
        '| Case | Cycle / control | Max range excess, old → new (rad) | Unintended arm points, old → new | Final tray penetration, old → new (mm) |',
        '| --- | --- | ---: | ---: | ---: |']
    for row in rows:
        new,old = row['current'],row['baseline'];passed = new['negative_control_passed'] if row['open_control'] else new['cycle_passed']
        lines.append(f"| {row['case']} | {'Control pass' if row['open_control'] and passed else 'Pass' if passed else 'FAIL'} | {old['max_intended_joint_range_excess_rad']:.6f} → {new['max_intended_joint_range_excess_rad']:.6f} | {old['unintended_arm_contact_points']} → {new['unintended_arm_contact_points']} | {old['final_tray_interpenetration_mm']:.4f} → {new['final_tray_interpenetration_mm']:.4f} |")
    lines += ['', '## Interpretation','',
        f"A +Y4 at 8 ms: joint stops constrain the observed excursion below 0.03 rad: {summary['stops_limit_observed_excursion']}; no unintended arm contact points: {summary['stop_case_has_no_unintended_arm_contacts']}. Its recorded cycle and failed checks remain in summary.json; bounding an excursion is not itself a successful pickup.",'',
        f"B 16 mm with only the world-step input reduced to 2 ms: the observed final embedding is resolved (on tray, under 1 mm penetration): {summary['finer_step_resolves_observed_embedding']}. Retention and full-cycle checks are recorded independently. Two resolutions in one case are not a convergence study or proof that every contact artifact is eliminated.",'',
        summary['step_cadence_caveat'],'',
        'The installed R2025a node definitions and [official JointParameters documentation](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/jointparameters.md) distinguish physical minStop/maxStop from motor minPosition/maxPosition. The old generator set only motor limits. The private corrected generator now supports explicit stops matching the existing commanded ranges on seven arm/finger hinges; wheel rotation stays continuous. Stop stiffness/load capacity and real mechanical stop design remain unmodeled hardware questions.','',
        'Reference cycles and matching open controls with stops are checked at the original 8 ms. No controller, motor torque, friction, timing or contact-profile changes are used to mask failed grasps. A/B comparison models and all earlier studies remain unchanged.','',
        'Next: check finer physics steps while keeping control at a fixed 8 ms, then use a separate candidate with explicit stops and a selected step. Repeat affected cases and appropriate reference/control/empty checks before revalidating the matched neighborhood. Supported release and directional objects still need their own acceptance checks. Do not transfer the old 24/28 count to a corrected model without rerunning it.','',
        'Raw reports, traces, telemetry, world/setup inputs, logs and actual frames are in projects/. manifest.json, summary.json and checkpoint-index.json preserve the comparison and code.','']
    (STUDY/'PHYSICS_CHECKS.md').write_text('\n'.join(lines),encoding='utf-8')
    for name in ['run_profile_physics_checks.py','summarize_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        source = MODEL/'scripts'/name;target = STUDY/'source_snapshot'/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(p.relative_to(STUDY)):sha(p) for p in STUDY.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['cases','changes','scope']}),flush=True)


if __name__=='__main__':main()
