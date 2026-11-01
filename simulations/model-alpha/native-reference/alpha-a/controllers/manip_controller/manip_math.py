"""Individual-finger mass model and bounded equilibrium-reference governor."""
import math
from mass_properties import mass_properties
from control_math import palm_position
from path_math import world_ik
MAIN=['shoulder','elbow','wrist'];DT=.008


def robot_mass(p,a):
    pose={j:a[j] for j in MAIN};pose.update(finger=0.,distal=0.);model=mass_properties(p,joint_angles=pose);parts=[part for part in model['parts'] if part['name'] not in ['proximal_fingers','distal_fingers']];palm=palm_position(p,pose);phi=sum(pose[j] for j in MAIN);cp,sp=math.cos(phi),math.sin(phi)
    for i,azimuth in enumerate(p['finger_angles_rad']):
        f,d=a[f'finger_{i}'],a[f'distal_{i}'];ca,sa=math.cos(azimuth),math.sin(azimuth);lp,ld=p['finger_proximal_length'],p['finger_distal_length']
        for name,x,z in [('prox',.018-lp/2*math.sin(f),.009+lp/2*math.cos(f)),('tip',.018-lp*math.sin(f)-ld/2*math.sin(f+d),.009+lp*math.cos(f)+ld/2*math.cos(f+d))]:
            rx,ry=x*ca,x*sa;point=[palm[0]+cp*rx+sp*z,ry,palm[2]-sp*rx+cp*z];parts.append(dict(name=f'{name}_{i}',mass_kg=p['finger_segment_mass'],com_body_m=point))
    total=sum(part['mass_kg'] for part in parts);com=[sum(part['mass_kg']*part['com_body_m'][i] for part in parts)/total for i in range(3)]
    return dict(mass_kg=total,com_body_m=com)


def equilibrium(p,hand,a,payload,seat_dx,guess=.09):
    def evaluate(u):
        pose=world_ik(p,u,hand[1],0.,0.,0.,1)
        if pose is None:return None
        angles=dict(a,**{j:pose[j] for j in MAIN});m=robot_mass(p,angles);return (m['mass_kg']*m['com_body_m'][0]+payload*(u+seat_dx))/(m['mass_kg']+payload)
    u=guess
    for _ in range(8):
        f=evaluate(u)
        if f is None:break
        if abs(f)<1e-9:return hand[0]-u,u,f
        lo,hi=evaluate(u-.00005),evaluate(u+.00005)
        if lo is None or hi is None or abs(hi-lo)<1e-12:break
        step=max(-.01,min(.01,f/((hi-lo)/.0001)));u=max(.02,min(.15,u-step))
    last=None
    for i in range(131):
        u=.02+i*.001;f=evaluate(u)
        if f is not None and last is not None and f*last[1]<=0:
            lo,hi=last[0],u;flo=last[1]
            for _ in range(32):
                mid=(lo+hi)/2;fm=evaluate(mid)
                if fm is None:raise ValueError('Equilibrium branch discontinuity')
                if fm*flo<=0:hi=mid
                else:lo=mid;flo=fm
            u=(lo+hi)/2;return hand[0]-u,u,evaluate(u)
        last=(u,f) if f is not None else None
    raise ValueError('No admitted zero-pitch equilibrium')


class Governor:
    def __init__(self,x):self.x=x;self.velocity=0.
    def update(self,target):
        desired=max(-.003,min(.003,(target-self.x)/.1));self.velocity+=max(-.01*DT,min(.01*DT,desired-self.velocity));self.x+=self.velocity*DT;return self.x,self.velocity


def smooth(u):u=max(0.,min(1.,u));return u*u*(3-2*u)
