"""World-level palm IK and static COM roots at fixed body pitch."""
import math
MAIN=['shoulder','elbow','wrist']


def world_ik(p,relative_x,z,theta,finger,distal,branch=1):
    c,s=math.cos(theta),math.sin(theta);height=z-p['wheel_radius'];bx,bz=c*relative_x-s*height,s*relative_x+c*height;phi=branch*math.pi-theta
    dx=bx-p['arm_mount_offset'][0]-p['wrist_standoff']*math.sin(phi)+p.get('wrist_forward_offset_m',0.)*math.cos(phi);dz=bz-p['arm_mount_offset'][2]-p['wrist_standoff']*math.cos(phi)-p.get('wrist_forward_offset_m',0.)*math.sin(phi)
    a,b=p['upper_length'],p['forearm_length'];cosine=(dx*dx+dz*dz-a*a-b*b)/(2*a*b)
    if abs(cosine)>1:return None
    e=branch*math.acos(cosine);q=math.atan2(dx,dz)-math.atan2(b*math.sin(e),a+b*math.cos(e));pose=dict(shoulder=q,elbow=e,wrist=phi-q-e,finger=finger,distal=distal)
    if any(not p['joint_ranges'][j][0]+.03<=pose[j]<=p['joint_ranges'][j][1]-.03 for j in MAIN) or abs(math.sin(e))<.2:return None
    return pose


def com(p,pose,theta,mass,payload):
    from control_math import palm_position
    m=mass.mass_properties(p,joint_angles=pose);palm=palm_position(p,pose);phi=sum(pose[j] for j in MAIN);point=[palm[0]+.058*math.sin(phi),0.,palm[2]+.058*math.cos(phi)];total=m['mass_kg']+payload
    bx=(m['mass_kg']*m['com_body_m'][0]+payload*point[0])/total;bz=(m['mass_kg']*m['com_body_m'][2]+payload*point[2])/total
    return bx*math.cos(theta)+bz*math.sin(theta)


def roots(p,z,theta,finger,distal,payload,mass,branch=1):
    def evaluate(x):
        pose=world_ik(p,x,z,theta,finger,distal,branch)
        return None if pose is None else (com(p,pose,theta,mass,payload),pose)
    results=[];last=None
    for i in range(131):
        x=.02+i*.001;v=evaluate(x)
        if v is not None and last is not None and v[0]*last[1][0]<=0:
            lo,hi=last[0],x;flo=last[1][0]
            for _ in range(35):
                mid=(lo+hi)/2;value=evaluate(mid)
                if value is None:break
                if value[0]*flo<=0:hi=mid
                else:lo=mid;flo=value[0]
            u=(lo+hi)/2;r=evaluate(u)
            if r is not None and abs(r[0])<1e-9 and (not results or abs(u-results[-1]['relative_x_m'])>1e-6):results.append(dict(relative_x_m=u,pose=r[1],com_residual_m=r[0]))
        last=(x,v) if v is not None else None
    return results


def segments(c,offset):
    x,z=c['source_palm_xz_m'];z+=offset;xd=c['destination_palm_xz_m'][0];top=c['platform_top_m']+offset;cube0=[x,0,top+.01];cube1=[xd,0,top+.01]
    # The 2 mm contact-seeking envelope moves the hand, while the supported cube stays at the top.
    return [dict(name=name,start=a,end=b,duration_s=duration,finger_start=f0,finger_end=f1,distal_start=d0,distal_end=d1,payload_start_kg=m0,payload_end_kg=m1,object_mode=mode) for name,a,b,duration,f0,d0,f1,d1,m0,m1,mode in [
        ('approach',[x,z+.015],[x,z],10.,-.3,.3,-.3,.3,0.,0.,'source'),
        ('close',[x,z],[x,z],5.,-.3,.3,.12,-.12,0.,.002,'source'),
        ('lift',[x,z],[x,z+.01],6.,.12,-.12,.12,-.12,.002,.002,'held'),
        ('hold',[x,z+.01],[x,z+.01],6.,.12,-.12,.12,-.12,.002,.002,'held'),
        ('transfer',[x,z+.01],[xd,z+.01],18.,.12,-.12,.12,-.12,.002,.002,'held'),
        ('lower',[xd,z+.01],[xd,z],10.,.12,-.12,.12,-.12,.002,.002,'held'),
        ('seek',[xd,z],[xd,z-.002],3.,.12,-.12,.12,-.12,.002,0.,'destination'),
        ('open',[xd,z-.002],[xd,z-.002],5.,.12,-.12,-.3,.3,0.,0.,'destination'),
        ('withdraw',[xd,z-.002],[xd,z+.015],10.,-.3,.3,-.3,.3,0.,0.,'destination')]]


def target(segment,u):
    f=u*u*(3-2*u)
    return dict(hand=[a+(b-a)*f for a,b in zip(segment['start'],segment['end'])],finger=segment['finger_start']+(segment['finger_end']-segment['finger_start'])*f,distal=segment['distal_start']+(segment['distal_end']-segment['distal_start'])*f,payload_kg=segment['payload_start_kg']+(segment['payload_end_kg']-segment['payload_start_kg'])*f)
