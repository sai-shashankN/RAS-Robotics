"""Cross-study result and verification without changing either frozen study."""
from pathlib import Path
import json
import math
import shutil
import sys
from summarize_contact_width_study import sha,read,trace
from cartesian_pick_place import VerticalCycle

MODEL=Path(__file__).resolve().parents[1]
WORKSPACE=MODEL.parents[1]
OUT=MODEL/'results/cube_cartesian_review'


def main():
    OUT.mkdir(exist_ok=True)
    studies={name:MODEL/'experiments'/name for name in ['cube_cartesian','cube_cartesian_gain']}
    hashes={};summaries={};counts={}
    for name,study in studies.items():
        index=read(study/'checkpoint-index.json');counts[name]=len(index['files'])
        for path,digest in index['files'].items():
            if sha(study/path)!=digest:raise ValueError('Changed frozen evidence: '+str(study/path))
        summaries[name]=read(study/'summary.json');hashes[str((study/'checkpoint-index.json').relative_to(WORKSPACE))]=sha(study/'checkpoint-index.json')
    plan=read(studies['cube_cartesian']/'projects/a/cartesian-plan.json')
    checks={}
    for label,supported,speed,expected_block in [('unsupported',False,0,True),('stable_support',True,0,False),('moving_support',True,.006,True)]:
        cycle=VerticalCycle(plan,False,.008);schedule=dict(release=36.,finish=48.)
        for i in range(1,6626):cycle.update(i*.008,1,supported,speed,schedule)
        checks[label]=(cycle.blocked==expected_block and (cycle.released_at is None)==expected_block and cycle.height>=plan['minimum_seek_m'])
    if not all(checks.values()):raise ValueError('Gate regression')
    review=dict(date='2026-10-24',study_index_sha256=hashes,verified_file_counts=counts,gate_checks=checks,
        gate_check_scope='Pure controller checks, separate from physics. Support absent and moving support block; stable support permits opening. Does not establish real sensing or tilt/centering.',
        baseline_parameters_sha256=sha(MODEL/'parameters.json'),review_script_sha256=sha(Path(__file__)),helper_sha256=sha(MODEL/'scripts/cartesian_pick_place.py'))
    sys.path.insert(0,str(WORKSPACE/'.local-tools/plotting-packages'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for i,v in enumerate(['a','b']):
        for name,label,color in [('cube_cartesian','position gain 0.16','#2874a5'),('cube_cartesian_gain','position gain 0.64','#d67a32')]:
            data=trace(studies[name]/'projects'/v/'results/physics'/f'{v}_vertical/contact-trace.jsonl')
            anchor=read(studies[name]/'projects'/v/'cartesian-plan.json')['anchor_world_xz_m'];start=min(data,key=lambda d:abs(d['time_s']-12))
            time=[d['time_s'] for d in data]
            axes[i,0].plot(time,[1000*(d['palm_world_position_m'][0]-anchor[0]) for d in data],label=label,color=color)
            axes[i,1].plot(time,[1000*(d['palm_world_position_m'][2]-start['palm_world_position_m'][2]) for d in data],label=label+' actual',color=color)
            axes[i,1].plot(time,[1000*d['planned_palm_height_m'] for d in data],label=label+' plan',color=color,ls='--',alpha=.6)
        axes[i,0].set(title=v.upper()+': loaded palm X error',ylabel='X error (mm)')
        axes[i,1].set(title=v.upper()+': loaded palm height change',ylabel='Height (mm)')
    for ax in axes.flat:ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(OUT/'base-arm-comparison.png',dpi=160);plt.close(fig)
    lines=['# Cube vertical-path and base-control review','',
        'Eight physics runs complete: four loaded attempts and four matching open-claw controls across two wheel-position gains. No loaded attempt passes strict supported placement. One of four open controls passes. Both candidates remain rejected; main A/B models are unchanged.','',
        '| Position gain | Axis | Required unsupported hold | Added object lift (mm) | Palm X excursion during lift (mm) | Max palm pitch error (°) | Result |',
        '| ---: | --- | --- | ---: | ---: | ---: | --- |']
    for name,s in summaries.items():
        for r in s['cases']:
            if r['open_control']:continue
            result='Support gate blocks opening' if r['support_blocked'] else 'Opens, finishes in tray; rejected'
            lines.append(f"| {'.16' if name=='cube_cartesian' else '.64'} | {r['variant'].upper()} | {'Pass' if r['retained_through_hold'] else 'FAIL'} | {r['added_object_lift_mm']:.2f} | {r['lift_palm_x_excursion_mm']:.2f} | {r['max_palm_attitude_error_deg']:.2f} | {result} |")
    lines+=['','## What the trials resolve','',
        'A level palm and a vertical nominal joint plan are different from vertical motion on a balancing robot. The first candidate grips, lifts by about 8 mm and holds in both axes, with loaded palm pitch within 1.4°. Its A palm nevertheless travels 11.06 mm sideways during the lift. B lift is much straighter (1.64 mm excursion), then deviates during setdown. Wheel odometry follows measured body X closely in the sampled telemetry; base translation and contact-constrained arm motion must be included in the task controller.','',
        'The initial gain retains both cubes but cannot confirm stable support on return within the 2 mm bounded seek, so both hands stay closed. A records no loaded unwanted arm contacts, but its open control brushes a finger against the cube and records two arm-base/pedestal contact points. B open control passes. No strict supported-placement pass is claimed.','',
        'Changing only wheel position gain from 0.16 to 0.64 fails to solve the coupled motion. Both loaded trials exceed 5° pitch error, all four runs record unwanted arm contacts, and A loses the required unsupported hold. Both positives eventually open and finish in the tray. Stable contact plus low speed is insufficient as a placement gate: it does not require a level or centered cube. Preserve this as a rejected gain experiment, not an improvement.','',
        'All eight runs have zero observed joint-range excess at the 8 ms trace cadence, and no simulator infrastructure errors. This does not prove absence of substep transients. No perception, real force/tactile feedback, ground pickup, full workspace or hardware validation.','',
        '## Geometry and plan scope','',
        'The private candidate uses a 32 mm wrist standoff and wider -0.3 rad proximal opening with the existing flat two-finger claw. Same 20 mm/2 g cube, 12×12 mm pedestal, motor caps, speeds and 2 ms physics/8 ms control are retained. Neck/palm mass remains a provisional lumped 45 g despite changing geometry.','',
        'Balanced nominal inverse-kinematics families span -2 to +8 mm in 0.2 mm steps. Independent forward kinematics agrees to less than 1e-8 m. The 8 mm lift is intentionally small and above the existing 6 mm criterion. A 10 mm nominal branch exists; the bounded equilibrium search finds no admissible 15/20 mm proposal for this pose, geometry and lean guard. This is not a universal reach bound. Minimum sampled nonadjacent envelope separation is only 1.116 mm for A and 9.762 mm for B. Direct joint neighbors are excluded. Initial and nominal open-only fixture sweeps pass; failed dynamic controls show the limits of those assumptions.','',
        'The support gate uses ideal simulator contact and speed, seeks at 0.5 mm/s up to 2 mm, waits for 0.6 s stable contact, and blocks opening at 41 s if unconfirmed. No object pose targets, attachments, resets or force writes. Its pure regression checks cover absent support, stable support and moving support. Sensor implementation, cube attitude/centering and contact-abort behavior remain work.','',
        '## Next implementation','',
        '1. Plan base and hand together using odometry/IMU, actual joint tracking and a pedestal clearance corridor. Inspect contact-constrained finger angles and known-load transfer instead of assuming commanded finger geometry.','2. Make unintended arm contact stop/retract the trial, and require the placement attitude/centering conditions before allowing release. Test refusal cases explicitly.','3. Revalidate A/B pickup, supported lowering, full opening/withdrawal, and both open controls. Keep arm, wrist, fingers, base mass layout and wheel changes available; do not bypass A with future Beta roll.','',
        '![Loaded hand motion and rejected gain comparison](base-arm-comparison.png)','',
        '[First Cartesian study](../../experiments/cube_cartesian/CARTESIAN_RESULTS.md) · [Rejected position-gain study](../../experiments/cube_cartesian_gain/CARTESIAN_RESULTS.md) · [Verification](verification.json)','',
        'Plot clarification: the first study’s frozen cartesian-path.png has one dashed nominal-height curve taken from its open control; its loaded hands remain at the blocked seek height instead. The comparison above plots each loaded run’s own intended height, preserving the original study unchanged. All raw reports, traces, actual simulator frames, sources and hashes are retained in the linked studies.','']
    (OUT/'CARTESIAN_REVIEW.md').write_text('\n'.join(lines),encoding='utf-8')
    (OUT/'verification.json').write_text(json.dumps(review,indent=2)+'\n',encoding='utf-8')
    snapshot=OUT/'source_snapshot';snapshot.mkdir(exist_ok=True)
    shutil.copy2(Path(__file__),snapshot/Path(__file__).name)
    probe=WORKSPACE/'.local-tools/cartesian-plan-probe.json'
    if probe.exists():shutil.copy2(probe,OUT/'preparation-plan-probe.json')
    files={str(p.relative_to(OUT)):sha(p) for p in OUT.rglob('*') if p.is_file() and p.name!='checkpoint-index.json'}
    (OUT/'checkpoint-index.json').write_text(json.dumps(dict(files=files),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(review),flush=True)


if __name__=='__main__':main()
