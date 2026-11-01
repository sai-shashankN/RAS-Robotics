"""Conservative OBB clearance audit of specified Alpha joint-space paths.

Cylinders are enclosed in boxes. A positive separating-axis gap certifies
separation; overlapping envelopes are candidates, not proof of real collision.
Directly joined solids are excluded explicitly (mounting pockets need CAD).
"""

from pathlib import Path
import hashlib
import itertools
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
sys.path.insert(0, str(WORKSPACE / '.local-tools/plotting-packages'))
import numpy as np
sys.path.insert(0,str(WORKSPACE/'simulations/common'))
from control_math import finger_angles


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def boxes(p, pose):
    out = []
    identity = np.eye(3)

    def add(name, owner, center, rotation, size):
        out.append((name, owner, np.array(center), rotation, np.array(size)/2))

    add('tube envelope', 'body', [0, 0, 0], identity, [2*p['body_radius'], p['body_length'], 2*p['body_radius']])
    add('lower housing', 'body', [0, 0, -.024], identity, [.07, .15, .025])
    for sign in [-1, 1]:
        add(f'camera {sign}', 'body', [.040, sign*.085, .008], identity, [.012, .025, .022])
        add(f'wheel {sign}', f'wheel{sign}', [0, sign*p['wheel_track']/2, 0], identity, [2*p['wheel_radius'], p['wheel_width']+.002, 2*p['wheel_radius']])
    q, e, w = pose['shoulder'], pose['elbow'], pose['wrist']
    mount = np.array(p['arm_mount_offset'])
    upper_r, fore_r, palm_r = ry(q), ry(q+e), ry(q+e+w)
    elbow = mount + upper_r @ np.array([0, 0, p['upper_length']])
    wrist = elbow + fore_r @ np.array([0, 0, p['forearm_length']])
    neck = p.get('wrist_standoff', 0)
    palm = wrist + palm_r @ np.array([0, 0, neck])
    add('shoulder collar', 'upper', mount, upper_r, [.036, .034, .036])
    add('upper link', 'upper', mount+upper_r @ np.array([0, 0, p['upper_length']/2]), upper_r, [.025, .024, p['upper_length']])
    add('forearm', 'forearm', elbow+fore_r @ np.array([0, 0, p['forearm_length']/2]), fore_r, [.020, .023, p['forearm_length']])
    add('palm envelope', 'palm', palm, palm_r, [2*p['palm_radius'], 2*p['palm_radius'], .016])
    if neck:
        add('wrist standoff', 'palm', wrist+palm_r @ np.array([0, 0, neck/2]), palm_r, [.012, .012, neck])
    for i,phi in enumerate(finger_angles(p)):
        radial_r = palm_r @ rz(phi)
        base = palm + radial_r @ np.array([.018, 0, .009])
        proximal_r = radial_r @ ry(-pose['finger'])
        distal_r = radial_r @ ry(-pose['finger']-pose['distal'])
        tip_base = base + proximal_r @ np.array([0, 0, p['finger_proximal_length']])
        # Covers both collision boxes and the slightly protruding blue pads.
        add(f'finger {i}', f'finger{i}', base+proximal_r @ np.array([0, 0, p['finger_proximal_length']/2]), proximal_r, [.0145, .011, p['finger_proximal_length']])
        add(f'tip {i}', f'tip{i}', tip_base+distal_r @ np.array([0, 0, p['finger_distal_length']/2]), distal_r, [.0145, .010, p['finger_distal_length']])
    return out


EXCLUDED = {frozenset(pair) for pair in [('body', 'upper'), ('upper', 'forearm'), ('forearm', 'palm'), ('body', 'wheel-1'), ('body', 'wheel1')]
            + [('palm', f'finger{i}') for i in range(2)] + [(f'finger{i}', f'tip{i}') for i in range(2)]}


def gap(a, b):
    axes = np.concatenate((a[3].T, b[3].T, np.cross(a[3].T[:, None, :], b[3].T[None, :, :]).reshape(-1, 3)))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-9] / norms[norms > 1e-9, None]
    projected_distance = np.abs(axes @ (b[2]-a[2]))
    radii = np.abs(axes @ a[3]) @ a[4] + np.abs(axes @ b[3]) @ b[4]
    return float(np.max(projected_distance-radii))


def audit_path(p, start, target):
    delta = {j: target[j]-start[j] for j in start}
    steps = max(1, math.ceil(max(abs(v) for v in delta.values())/.002))
    # Upper bounds on distance from each joint axis to any downstream envelope.
    radii = {'shoulder': .34, 'elbow': .25, 'wrist': .16, 'finger': .08, 'distal': .04}
    # Each point moves at most sum(r*dq) per interval; relative pair motion is
    # at most twice that. The nearest sampled endpoint is <= half an interval.
    motion_allowance = sum(radii[j]*abs(delta[j])/steps for j in delta)
    min_gap, closest = float('inf'), None
    candidates = {}
    for index in range(steps+1):
        pose = {j: start[j]+delta[j]*index/steps for j in start}
        shapes = boxes(p, pose)
        for a, b in itertools.combinations(shapes, 2):
            if a[1] == b[1] or frozenset((a[1], b[1])) in EXCLUDED:
                continue
            clearance = gap(a, b)
            if clearance < min_gap:
                min_gap, closest = clearance, [a[0], b[0]]
            if clearance <= motion_allowance:
                pair = a[0]+' / '+b[0]
                candidates.setdefault(pair, {'first_fraction': index/steps, 'min_axis_gap_m': clearance})
                candidates[pair]['min_axis_gap_m'] = min(candidates[pair]['min_axis_gap_m'], clearance)
    return {'samples': steps+1, 'minimum_sampled_axis_gap_m': min_gap, 'closest_pair': closest,
            'between_sample_motion_allowance_m': motion_allowance,
            'conservative_clearance_bound_m': min_gap-motion_allowance,
            'conservative_nonadjacent_clearance_passed': min_gap > motion_allowance,
            'candidate_overlaps_or_small_margins': candidates}


def main():
    p = json.loads((MODEL/'parameters.json').read_text())
    folded = p['poses']['folded']
    paths = {name: (folded, target) for name, target in p['dynamic_arm_targets'].items()}
    for posture, base in [('folded', folded), ('raised', p['dynamic_arm_targets']['arm_reach'])]:
        paths['fingers_'+posture] = (dict(base, **p['finger_targets']['claw_closed']), dict(base, **p['finger_targets']['claw_open']))
    results = {name: audit_path(p, start, target) for name, (start, target) in paths.items()}
    sources = [Path(__file__), MODEL/'parameters.json', WORKSPACE/'simulations/common/generate_worlds.py',WORKSPACE/'simulations/common/control_math.py']
    report = {'method': 'Conservative oriented-box SAT envelopes plus a joint-motion bound between samples; Alpha only.',
              'excluded_direct_joint_pairs': [sorted(pair) for pair in EXCLUDED],
              'limitations': ['Direct joint neighbors/mounting pockets excluded and need detailed CAD.', 'Only specified linear joint-space paths; no arbitrary-workspace guarantee.', 'Cylinders and visual pads conservatively enclosed; candidate overlaps may be false positives.', 'Provisional dimensions without flex, wiring, tolerances, or real actuator housings.'],
              'source_sha256': {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              'paths': results, 'all_defined_paths_clear': all(row['conservative_nonadjacent_clearance_passed'] for row in results.values())}
    out = MODEL/'results/clearance'
    out.mkdir(exist_ok=True)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'all_defined_paths_clear': report['all_defined_paths_clear'], 'paths': {name: {'bound_mm': r['conservative_clearance_bound_m']*1000, 'closest_pair': r['closest_pair'], 'candidates': list(r['candidate_overlaps_or_small_margins'])} for name, r in results.items()}}, indent=2))
    if not report['all_defined_paths_clear']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
