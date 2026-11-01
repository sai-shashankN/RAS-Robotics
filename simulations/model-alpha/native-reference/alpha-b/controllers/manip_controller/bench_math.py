"""Supported planar arm paths. No simulator state mutations or object attachments."""
import math

MAIN = ('shoulder', 'elbow', 'wrist')
DT = .008


def ik(p, x, z, root_z=.072):
    """World palm X/Z, downward palm local +Z, positive-elbow branch."""
    dx = x-p['arm_mount_offset'][0]
    dz = z-root_z-p['arm_mount_offset'][2]+p['wrist_standoff']
    a,b=p['upper_length'],p['forearm_length']
    c=(dx*dx+dz*dz-a*a-b*b)/(2*a*b)
    if abs(c)>1:raise ValueError('Unreachable palm target')
    e=math.acos(c)
    q=math.atan2(dx,dz)-math.atan2(b*math.sin(e),a+b*math.cos(e))
    pose=dict(shoulder=q,elbow=e,wrist=math.pi-q-e)
    if any(not p['joint_ranges'][j][0]<=pose[j]<=p['joint_ranges'][j][1] for j in MAIN):
        raise ValueError('IK exceeds joint ranges')
    return pose


def smooth(u):
    u=max(0.,min(1.,u));return u*u*(3-2*u)


def target_between(start,end,t,duration):
    u=smooth(t/duration)
    return [a+(b-a)*u for a,b in zip(start,end)]


def configuration(p):
    q,e=-.25,1.5
    x=p['arm_mount_offset'][0]+p['upper_length']*math.sin(q)+p['forearm_length']*math.sin(q+e)
    z=.072+p['arm_mount_offset'][2]+p['upper_length']*math.cos(q)+p['forearm_length']*math.cos(q+e)-p['wrist_standoff']
    return dict(root_z_m=.072,source_palm_xz_m=[x,z],destination_palm_xz_m=[x+.050,z],
                object_size_m=[.020]*3,object_mass_kg=.002,object_local_z_m=.058,
                platform_size_m=[.040,.040],platform_top_m=z-.068,
                source_object_xyz_m=[x,0,z-.058],destination_object_xyz_m=[x+.050,0,z-.058],
                lift_m=.010,approach_clearance_m=.015,transfer_m=.050,
                control_period_ms=8,physics_period_ms=2,
                gates=dict(center_error_m=.003,cube_tilt_rad=math.radians(3),support_speed_m_s=.005,
                           support_gap_m=.0003,pre_open_s=.6,clear_stable_s=3.,hold_s=6.),
                assumptions=['Body root has no Physics node: explicit infinite-stiffness fixture; articulated links retain Physics.',
                             'No fixture reaction-force sensor is modeled. Anchor loads are not measured.',
                             'Four independent finger motors and extended negative distal travel remain unverified hardware assumptions.',
                             'Object/support/contact observations are ideal simulator witnesses, not hardware sensors.',
                             'Two 40 mm platforms require more than 40 mm separation: selected 50 mm after reach screening.'])


def motion_segments(c):
    x,z=c['source_palm_xz_m'];xd,_=c['destination_palm_xz_m']
    return [('approach',[x,z+.015],[x,z],10.,-.3,.3,-.3,.3),
            ('close',[x,z],[x,z],5.,-.3,.3,.12,-.12),
            ('lift',[x,z],[x,z+.010],6.,.12,-.12,.12,-.12),
            ('hold',[x,z+.010],[x,z+.010],6.,.12,-.12,.12,-.12),
            ('transfer',[x,z+.010],[xd,z+.010],18.,.12,-.12,.12,-.12),
            ('lower',[xd,z+.010],[xd,z-.002],10.,.12,-.12,.12,-.12),
            ('open',[xd,z-.002],[xd,z-.002],5.,.12,-.12,-.3,.3),
            ('withdraw',[xd,z-.002],[xd,z+.015],10.,-.3,.3,-.3,.3)]


def settled_support(s,c,which='destination'):
    return (s[which+'_support'] and abs(s[which+'_gap_m'])<=c['gates']['support_gap_m']
            and s[which+'_error_m']<=c['gates']['center_error_m']
            and s['tilt_rad']<=c['gates']['cube_tilt_rad'] and s['speed_m_s']<c['gates']['support_speed_m_s'])


class Dwell:
    """Elapsed intervals; first qualifying sample earns zero elapsed time."""
    def __init__(self):self.start=None
    def update(self,t,valid):
        if not valid:self.start=None;return 0.
        if self.start is None:self.start=t
        return max(0.,t-self.start)
