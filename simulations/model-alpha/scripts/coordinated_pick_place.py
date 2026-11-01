"""Planar height control plus odometry-based hand-X wheel reference.

Object pose is not a kinematic target. A separate ideal placement observer may
permit opening; hardware sensing, slip and full 3D remain unvalidated.
"""
import math
from cartesian_pick_place import rotate


def height_pose(p,z,pitch,shoulder,finger,distal):
    mx,_,mz=p['arm_mount_offset'];a,b=p['upper_length'],p['forearm_length']
    cosine=(z-p['wheel_radius']+p['wrist_standoff']+mx*math.sin(pitch)-mz*math.cos(pitch)-a*math.cos(shoulder+pitch))/b
    if not -1<=cosine<=1:return None
    elbow=math.acos(cosine)-shoulder-pitch
    pose=dict(shoulder=shoulder,elbow=elbow,wrist=math.pi-pitch-shoulder-elbow,finger=finger,distal=distal)
    if any(not p['joint_ranges'][j][0]<=pose[j]<=p['joint_ranges'][j][1] for j in ['shoulder','elbow','wrist']):return None
    return pose


def nominal_plan(p,e,anchor,lift=.008):
    from control_math import palm_position
    from grasp_planning import loaded_reference
    shoulder=p['dynamic_arm_targets']['arm_reach']['shoulder'];families={};limit=p['dynamic_lean_limit_rad']
    opened,closed=p['finger_targets']['claw_open'],p['finger_targets']['claw_closed']
    for name,f,d,load in [('open',opened['finger'],opened['distal'],0),('closed_empty',closed['finger'],closed['distal'],0),('closed_full',closed['finger'],closed['distal'],1)]:
        rows=[];previous_pitch=-.2
        for i in range(51):
            h=round(-.002+i*.0002,7)
            def residual(t):
                pose=height_pose(p,anchor[1]+h,t,shoulder,f,d)
                return None if pose is None else (loaded_reference(p,e,pose,load)-t,pose)
            roots=[];previous=None
            for k in range(181):
                t=-limit+2*limit*k/180;r=residual(t)
                if r and previous and r[0]*previous[1][0]<=0:
                    lo,hi=previous[0],t
                    for _ in range(35):
                        mid=(lo+hi)/2
                        if residual(lo)[0]*residual(mid)[0]<=0:hi=mid
                        else:lo=mid
                    t0=(lo+hi)/2;roots.append((t0,residual(t0)[1]))
                previous=(t,r) if r else None
            if not roots:raise ValueError('No fixed-shoulder nominal equilibrium: '+str((name,h)))
            pitch,pose=min(roots,key=lambda r:abs(r[0]-previous_pitch));previous_pitch=pitch
            palm=palm_position(p,pose);point=rotate(palm[0],palm[2],pitch)
            rows.append(dict(height_m=h,pose=pose,pitch_rad=pitch,base_x_m=anchor[0]-point[0]))
        families[name]=rows
    return dict(anchor_world_xz_m=anchor,lift_m=lift,minimum_seek_m=-.002,height_step_m=.0002,families=families,
        load_plan=dict(within_lean_limit=True,peak_nominal_loaded_lean_rad=max(abs(r['pitch_rad']) for rows in families.values() for r in rows),lean_limit_rad=limit,samples=153,
            assumption='Fixed shoulder, nominal equilibria with planned wheel translation. Runtime pitch/encoders determine elbow height and wheel reference; contact-constrained geometry and tracking must be checked.'))


class HeightBaseCoordinator:
    def __init__(self,p,anchor,dt):
        self.p,self.anchor,self.dt=p,anchor,dt
        self.shoulder=p['dynamic_arm_targets']['arm_reach']['shoulder']
        self.position=0.;self.velocity=0.;self.invalid_samples=0;self.max_reference=0.
    def update(self,height,pitch,angles,target):
        from control_math import palm_position
        pose=height_pose(self.p,self.anchor[1]+height,pitch,self.shoulder,target['finger'],target['distal'])
        if pose is None:self.invalid_samples+=1;return None,self.position,self.velocity
        palm=palm_position(self.p,angles);point=rotate(palm[0],palm[2],pitch)
        desired=self.anchor[0]-point[0]
        # Reference shaping, not higher wheel gains or motor speed/torque caps.
        wanted=max(-.006,min(.006,(desired-self.position)/.15))
        self.velocity+=max(-.01*self.dt,min(.01*self.dt,wanted-self.velocity))
        self.position+=self.velocity*self.dt;self.max_reference=max(self.max_reference,abs(self.position))
        return pose,self.position,self.velocity
    def report(self):return dict(invalid_height_ik_samples=self.invalid_samples,max_abs_wheel_position_reference_m=self.max_reference,
        reference_speed_cap_m_s=.006,reference_acceleration_cap_m_s2=.01,
        sensing='Wheel encoders + IMU pitch + arm encoders; fixed root-height and planar/no-slip assumptions. Object pose is not used for arm or wheel targets.')


def placement_ready(supported,speed,tilt_rad,center_error):
    return bool(supported and speed<.005 and tilt_rad<math.radians(2) and center_error<.002)
