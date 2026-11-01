"""Reproduce robot-only conservative clearance checks for the grasp sequence."""

from pathlib import Path
import hashlib
import json

from audit_clearance import audit_path

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def main():
    p = json.loads((MODEL/'parameters.json').read_text())
    e = json.loads((MODEL/'grasp_parameters.json').read_text())
    opened = dict(p['dynamic_arm_targets']['arm_reach'], **p['finger_targets']['claw_open'])
    closed = dict(opened, **p['finger_targets']['claw_closed'])
    lifted = e['lift_pose']
    lift_open = dict(lifted, **p['finger_targets']['claw_open'])
    paths = {'close': audit_path(p, opened, closed), 'lift': audit_path(p, closed, lifted),
             'open_control_lift': audit_path(p, opened, lift_open)}
    sources = [Path(__file__), MODEL/'scripts/audit_clearance.py', MODEL/'parameters.json', MODEL/'grasp_parameters.json']
    report = {'paths': paths, 'all_passed': all(row['conservative_nonadjacent_clearance_passed'] for row in paths.values()),
              'source_sha256': {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              'limitations': 'Robot nonadjacent envelopes only; fixture/object interactions checked by dynamic contact monitoring.'}
    (MODEL/'results/grasp/robot-clearance-audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({name: row['conservative_clearance_bound_m']*1000 for name, row in paths.items()}, indent=2))
    if not report['all_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
