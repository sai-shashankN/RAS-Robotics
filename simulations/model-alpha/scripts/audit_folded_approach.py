"""Robot self-clearance and sampled nominal staging clearance to the fixture."""

from pathlib import Path
import argparse
import hashlib
import json
import math
import sys

from audit_clearance import audit_path, boxes, gap, ry
import numpy as np

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
sys.path.insert(0, str(MODEL.parent/'common'))
from approach_planning import plan_approach
from grasp_planning import loaded_reference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', default='reference_50mm')
    args = parser.parse_args()
    if not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.case):
        raise ValueError('Invalid case name')
    config = MODEL/'experiments/folded_approach_validated/cases'/args.case/'experiment.json'
    setup_path = config.parent/'setup.json'
    p = json.loads((MODEL/'parameters.json').read_text(encoding='utf-8'))
    e = json.loads(config.read_text(encoding='utf-8'))
    setup = json.loads(setup_path.read_text(encoding='utf-8'))
    plan = plan_approach(p, e)
    paths = [('deploy', plan['folded_pose'], plan['raised_closed_pose']), ('open', plan['raised_closed_pose'], plan['open_pose'])]
    self_audit = {name: audit_path(p, start, end) for name, start, end in paths}
    x, y, z = setup['initial_object_position_m']
    bottom, top = setup['tray_top_m'], setup['stand_top_m']
    identity = np.eye(3)
    def box(name, center, size):
        return (name, 'fixture', np.array(center), identity, np.array(size)/2)
    fixtures = [box('sphere envelope', [x, y, z], [2*e['object_radius_m']]*3),
                box('stand envelope', [x,y,(bottom+top)/2], [2*e['stand_radius_m']]*2+[top-bottom]),
                box('tray floor', [x,y,.002], [.20,.16,.004])]
    for dx, dy, size in [(.102,0,[.004,.168,.020]),(-.102,0,[.004,.168,.020]),(0,.082,[.20,.004,.020]),(0,-.082,[.20,.004,.020])]:
        fixtures.append(box('tray wall', [x+dx,y+dy,.014], size))
    nominal = {}
    # Straight translation at the already tested open pose is checked separately.
    for name, start, end in paths+[('approach', plan['open_pose'], plan['open_pose'])]:
        minimum, closest, floor = float('inf'), None, float('inf')
        for i in range(501):
            u = i/500
            pose = {j: start[j]+(end[j]-start[j])*u for j in start}
            lean = loaded_reference(p, e, pose, 0)
            rotation = ry(lean)
            base = np.array([e['alignment_target_offset_world_m'][0]*u if name=='approach' else 0,0,p['wheel_radius']])
            for shape in boxes(p, pose):
                world = (shape[0], shape[1], base+rotation@shape[2], rotation@shape[3], shape[4])
                if not shape[1].startswith('wheel'):
                    floor = min(floor, float(world[2][2]-np.abs(world[3][2])@world[4]))
                for fixture in fixtures:
                    distance = gap(world, fixture)
                    if distance<minimum:
                        minimum, closest = distance, [shape[0],fixture[0]]
        nominal[name] = {'minimum_sampled_separating_gap_m': minimum, 'closest_pair': closest,
                         'minimum_sampled_nonwheel_floor_gap_m': floor, 'samples': 501}
    sources = [Path(__file__), MODEL/'scripts/audit_clearance.py', MODEL/'scripts/generate_grasp_worlds.py',
               MODEL.parent/'common/generate_worlds.py', MODEL/'parameters.json', config, setup_path]
    sources += [MODEL.parent/'common'/name for name in ['approach_planning.py','grasp_planning.py','control_math.py','mass_properties.py']]
    report = {'self_clearance': self_audit, 'nominal_fixture_clearance': nominal,
              'all_nominal_fixture_samples_separated': all(row['minimum_sampled_separating_gap_m']>0 and row['minimum_sampled_nonwheel_floor_gap_m']>0 for row in nominal.values()),
              'all_robot_self_paths_clear': all(r['conservative_nonadjacent_clearance_passed'] for r in self_audit.values()),
              'source_sha256': {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              'limitations': 'Fixture gaps enclose cylinders/sphere in boxes and sample nominal empty-COM lean, zero yaw and nominal base position. No bound on dynamic tracking or gaps between world samples; physical contact checks remain required. Self-clearance has the existing between-sample joint-motion bound; direct joint neighbors are excluded.'}
    out = MODEL/'results/folded_approach_validated'/args.case/'clearance.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'self_clear': report['all_robot_self_paths_clear'], 'nominal_fixture': nominal}, indent=2))
    if not report['all_robot_self_paths_clear']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
