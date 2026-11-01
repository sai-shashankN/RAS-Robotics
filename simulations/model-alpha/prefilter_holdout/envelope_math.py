import math,itertools
import numpy as np
finger_angles=lambda p:p['finger_angles_rad']

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
    forward=p.get('wrist_forward_offset_m',0.);palm = wrist + palm_r @ np.array([-forward, 0, neck])
    add('shoulder collar', 'upper', mount, upper_r, [.036, .034, .036])
    add('upper link', 'upper', mount+upper_r @ np.array([0, 0, p['upper_length']/2]), upper_r, [.025, .024, p['upper_length']])
    add('forearm', 'forearm', elbow+fore_r @ np.array([0, 0, p['forearm_length']/2]), fore_r, [.020, .023, p['forearm_length']])
    add('palm envelope', 'palm', palm, palm_r, [2*p['palm_radius'], 2*p['palm_radius'], .016])
    if neck:
        add('wrist standoff', 'palm', wrist+palm_r @ np.array([-forward/2, 0, neck/2]), palm_r @ ry(-math.atan2(forward,neck)), [.012, .012, math.hypot(neck,forward)])
    for i,phi in enumerate(finger_angles(p)):
        radial_r = palm_r @ rz(phi)
        base = palm + radial_r @ np.array([.018, 0, .009])
        proximal_r = radial_r @ ry(-pose.get('finger_'+str(i),pose['finger']))
        distal_r = radial_r @ ry(-pose.get('finger_'+str(i),pose['finger'])-pose.get('distal_'+str(i),pose['distal']))
        tip_base = base + proximal_r @ np.array([0, 0, p['finger_proximal_length']])
        # Covers both collision boxes and the slightly protruding blue pads.
        add(f'finger {i}', f'finger{i}', base+proximal_r @ np.array([0, 0, p['finger_proximal_length']/2]), proximal_r, [.0145, .011, p['finger_proximal_length']])
        add(f'tip {i}', f'tip{i}', tip_base+distal_r @ np.array([0, 0, p['finger_distal_length']/2]), distal_r, [.0145, .010, p['finger_distal_length']])
    return out

def gap(a, b):
    axes = np.concatenate((a[3].T, b[3].T, np.cross(a[3].T[:, None, :], b[3].T[None, :, :]).reshape(-1, 3)))
    norms = np.linalg.norm(axes, axis=1)
    axes = axes[norms > 1e-9] / norms[norms > 1e-9, None]
    projected_distance = np.abs(axes @ (b[2]-a[2]))
    radii = np.abs(axes @ a[3]) @ a[4] + np.abs(axes @ b[3]) @ b[4]
    return float(np.max(projected_distance-radii))