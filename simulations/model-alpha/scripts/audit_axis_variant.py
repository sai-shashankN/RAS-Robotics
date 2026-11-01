"""Use the same robot-envelope audit for an independently specified Alpha variant."""
from pathlib import Path
import argparse
import hashlib
import json

import audit_clearance as geometry

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model',type=Path,default=MODEL.parent/'model-alpha-b')
    args = parser.parse_args()
    model = args.model.resolve()
    p = json.loads((model/'parameters.json').read_text())
    folded = p['poses']['folded']
    paths = {name:(folded,target) for name,target in p['dynamic_arm_targets'].items()}
    for posture,base in [('folded',folded),('raised',p['dynamic_arm_targets']['arm_reach'])]:
        paths['fingers_'+posture] = (dict(base,**p['finger_targets']['claw_closed']),dict(base,**p['finger_targets']['claw_open']))
    rows = {name:geometry.audit_path(p,start,end) for name,(start,end) in paths.items()}
    sources = [Path(__file__),Path(geometry.__file__),model/'parameters.json',WORKSPACE/'simulations/common/control_math.py',WORKSPACE/'simulations/common/generate_worlds.py']
    report = {'model':p['model'],'finger_angles_rad':p['finger_angles_rad'],
              'method':'Same conservative OBB envelopes and between-sample joint-motion allowance as Alpha-A.',
              'excluded_direct_joint_pairs':[sorted(pair) for pair in geometry.EXCLUDED],
              'limitations':['Specified robot-only paths; no object/fixture or arbitrary-workspace guarantee.',
                             'Direct joint neighbors excluded; mounting pockets, tolerances and real hardware remain open.'],
              'source_sha256':{str(path.relative_to(WORKSPACE)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              'paths':rows,'all_defined_paths_clear':all(row['conservative_nonadjacent_clearance_passed'] for row in rows.values())}
    out = model/'results/clearance';out.mkdir(parents=True,exist_ok=True)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'model':p['model'],'all_defined_paths_clear':report['all_defined_paths_clear'],
                      'bounds_mm':{name:row['conservative_clearance_bound_m']*1000 for name,row in rows.items()}}),flush=True)
    if not report['all_defined_paths_clear']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
