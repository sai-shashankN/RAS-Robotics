"""Compare finer physics while keeping controller/sensors at 8 ms."""
from pathlib import Path
import json
import shutil
from summarize_contact_width_study import sha,read,trace
from summarize_profile_physics_checks import metrics

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
PRIOR = MODEL/'experiments/profile_physics_checks'
STUDY = MODEL/'experiments/fixed_cadence_checks'


def main():
    manifest,status = read(STUDY/'manifest.json'),read(STUDY/'run-status.json')
    if status['infrastructure_errors'] or len(status['results'])!=4:raise ValueError('Incomplete fixed-cadence diagnostic')
    if sha(PRIOR/'checkpoint-index.json')!=manifest['prior_checkpoint_sha256']:raise ValueError('Prior checkpoint changed')
    old_source = (PRIOR/'projects/common/grasp_controller.py').read_text(encoding='utf-8')
    expected = old_source.replace('step = int(robot.getBasicTimeStep())','step = 8  # fixed control/sensor period; physics may substep at 2 ms',1)
    if (STUDY/'projects/common/grasp_controller.py').read_text(encoding='utf-8')!=expected:raise ValueError('Other controller code changed')
    rows = []
    for entry in manifest['cases']:
        project = WORKSPACE/entry['project'];folder = project/'results/physics'/entry['case'];r = read(folder/'report.json')
        old_path = WORKSPACE/entry['prior_report'];old = read(old_path);p = read(project/'parameters.json')
        prior_p = read(old_path.parents[3]/'parameters.json')
        if dict(prior_p,world_basic_time_step_ms=2)!=p or r['experiment']!=old['experiment']:raise ValueError('Other input changed')
        for path in (STUDY/'projects/common').glob('*.py'):
            if path.name!='grasp_controller.py' and sha(path)!=sha(PRIOR/'projects/common'/path.name):raise ValueError('Other helper changed')
        for name,digest in r['source_sha256'].items():
            if sha(WORKSPACE/name)!=digest:raise ValueError('Stale diagnostic input')
        if sha(WORKSPACE/entry['world'])!=r['world_sha256'] or sha(folder/'contact-trace.jsonl')!=r['contact_trace_sha256']:raise ValueError('World/trace mismatch')
        data = trace(folder/'contact-trace.jsonl')
        current = metrics(r,data,p)
        if not 7.999<current['maximum_trace_step_ms']<8.001:raise ValueError('Control/trace cadence changed')
        rows.append(dict(entry,current=current,baseline=metrics(old,trace(old_path.parent/'contact-trace.jsonl'),prior_p),
            report_sha256=sha(folder/'report.json'),trace_sha256=r['contact_trace_sha256'],world_sha256=r['world_sha256']))
    summary = {'date':'2026-10-24','cases':rows,'change':manifest['change'],'limitations':manifest['limitations'],'profile_adopted':False,
        'analysis_sha256':sha(Path(__file__))}
    (STUDY/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    lines = ['# Fixed-control-cadence physics checks','','24 October 2026.','',manifest['change'],'',manifest['limitations'],'',
        '| Case | Cycle | Range excess (rad) | Unintended arm points | Final tray penetration (mm) |',
        '| --- | --- | ---: | ---: | ---: |']
    for row in rows:
        c = row['current'];lines.append(f"| {row['prior_case']} | {'Pass' if c['cycle_passed'] else 'FAIL'} | {c['max_intended_joint_range_excess_rad']:.6f} | {c['unintended_arm_contact_points']} | {c['final_tray_interpenetration_mm']:.4f} |")
    lines += ['', 'Failed checks, baseline comparisons and all metrics are in summary.json. Observed range extrema are sampled every 8 ms, so they do not bound excursions between observations. Stops matching commanded ranges are provisional ideal constraints, not a physical stop specification. No failed case is relabeled successful merely because it is better behaved.','',
        'This separates world-physics resolution from feedback cadence. The stopped A/B reference and A +Y4 cases compare physics 8 → 2 ms at unchanged 8 ms control. The B 16 mm case compares control 2 → 8 ms at unchanged 2 ms physics, and can also be compared to the original robustness run (physics/control both 8 ms, no stops).','',
        'Next: resolve any remaining contact/range/settling issue before certifying a corrected baseline; validate reference, open controls and empty motions, then rerun the affected neighborhood. Supported pole placement and directional objects remain unvalidated. Main A/B files and all earlier studies remain unchanged.','',
        'A pre-physics preparation anchor error and its partial source copy are preserved in ../../archive/fixed-cadence-preparation-first-attempt. No simulator evidence was overwritten.','']
    (STUDY/'FIXED_CADENCE_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    for name in ['run_fixed_cadence_checks.py','summarize_fixed_cadence_checks.py','run_profile_physics_checks.py','summarize_profile_physics_checks.py','run_contact_width_study.py','summarize_contact_width_study.py']:
        source = MODEL/'scripts'/name;target = STUDY/'source_snapshot'/source.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    hashes = {str(p.relative_to(STUDY)):sha(p) for p in STUDY.rglob('*') if p.is_file() and p.suffix in ['.py','.json','.wbt','.wbproj','.ini','.csv','.jsonl','.png','.md','.log'] and p.name!='checkpoint-index.json'}
    (STUDY/'checkpoint-index.json').write_text(json.dumps({'date':'2026-10-24','files':hashes},indent=2)+'\n',encoding='utf-8')
    print(json.dumps([{'case':r['case'],'metrics':r['current']} for r in rows]),flush=True)


if __name__=='__main__':main()
