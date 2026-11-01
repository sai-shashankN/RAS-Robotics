"""Two-station travel references only; this module has no simulator interface."""
import math
import numpy as np

PHASES=('backout','backout_settle','turn','turned_hold','approach','arrival_settle')
def rz(a):
    c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
def cubic(t,duration,start,end):
    u=max(0.,min(1.,t/duration));d=end-start
    return start+d*u*u*(3-2*u),d*6*u*(1-u)/duration,d*(6-12*u)/duration**2

class DestinationTransit:
    def __init__(self,approach,backout=.1,back_seconds=50.,turn_seconds=60.,approach_seconds=150.,dwell_seconds=4.,direction=1):
        values=[approach,backout,back_seconds,turn_seconds,approach_seconds,dwell_seconds]
        if not all(math.isfinite(v) and v>0 for v in values) or direction not in [-1,1]:raise ValueError('Positive finite dimensions/times and turn direction required')
        self.approach=approach;self.backout=backout;self.times=(back_seconds,dwell_seconds,turn_seconds,dwell_seconds,approach_seconds,dwell_seconds);self.direction=direction;self.duration=sum(self.times)
    def sample(self,t):
        if not math.isfinite(t) or t<0:raise ValueError('Finite nonnegative time required')
        clock=0.;d,l=self.backout,self.approach
        for name,duration in zip(PHASES,self.times):
            if t<clock+duration or name=='arrival_settle':
                u=min(duration,t-clock);path=-d;world_x=-d;speed=accel=world_speed=world_accel=angle=omega=alpha=0.
                if name=='backout':path,speed,accel=cubic(u,duration,0.,-d);world_x=path;world_speed=speed;world_accel=accel
                elif name=='turn':angle,omega,alpha=cubic(u,duration,0.,self.direction*math.pi)
                elif name in ['turned_hold','approach','arrival_settle']:angle=self.direction*math.pi
                if name=='approach':travel,speed,accel=cubic(u,duration,0.,l);path=-d+travel;world_x=-d-travel;world_speed=-speed;world_accel=-accel
                elif name=='arrival_settle':path=-d+l;world_x=-d-l
                return dict(stage=name,path_m=path,speed_m_s=speed,acceleration_m_s2=accel,world_delta_initial_m=np.array([world_x,0.,0.]),world_velocity_initial_m_s=np.array([world_speed,0.,0.]),world_acceleration_initial_m_s2=np.array([world_accel,0.,0.]),heading_rad=angle,heading_rate_rad_s=omega,heading_acceleration_rad_s2=alpha)
            clock+=duration

class DestinationTask:
    def __init__(self,origin,heading,palm,object_point,fixed_destination_held_point,backout=.1,direction=1,lateral_tolerance=.002):
        points=[np.array(p,dtype=float) for p in [origin,palm,object_point,fixed_destination_held_point]]
        if not all(p.shape==(3,) and np.all(np.isfinite(p)) for p in points) or not math.isfinite(heading) or not math.isfinite(lateral_tolerance) or lateral_tolerance<0:raise ValueError('Finite3Dpoints,heading and lateral tolerance required')
        self.origin,self.palm_start,self.object_start,self.destination=points;self.heading_start=heading;self.heading_goal=heading+direction*math.pi;self.direction=direction;Q=rz(heading);self.palm_local=Q.T@(self.palm_start-self.origin);self.object_local=Q.T@(self.object_start-self.origin)
        turn_root=self.origin+Q@[-backout,0.,0.];end_Q=rz(heading+direction*math.pi);turn_object=turn_root+end_Q@self.object_local;need=end_Q.T@(self.destination-turn_object);self.lateral_residual=float(need[1]);self.vertical_residual=float(need[2])
        if abs(self.lateral_residual)>lateral_tolerance:raise ValueError('Destination requires lateral base alignment')
        if abs(self.vertical_residual)>1e-6:raise ValueError('Fixed carry pose requires equal held-target height')
        self.reference=DestinationTransit(float(need[0]),backout=backout,direction=direction)
    def sample(self,t):
        s=self.reference.sample(t);origin=self.origin+rz(self.heading_start)@s['world_delta_initial_m'];heading=self.heading_start+s['heading_rad'];Q=rz(heading)
        return s,origin,origin+Q@self.palm_local,origin+Q@self.object_local,heading
    def arrival_anchor(self,path_start):
        s,origin,_,_,heading=self.sample(self.reference.duration)
        return dict(world_anchor=origin.tolist(),path_anchor=path_start+s['path_m'],heading=heading,lateral_residual_m=self.lateral_residual,vertical_residual_m=self.vertical_residual)
    def project_orientation(self,initial_world_rotation,t):
        R=np.array(initial_world_rotation,dtype=float)
        if R.shape!=(3,3) or not np.all(np.isfinite(R)) or np.max(abs(R.T@R-np.eye(3)))>1e-9 or abs(np.linalg.det(R)-1)>1e-9:raise ValueError('Proper3Drotation required')
        return rz(self.reference.sample(t)['heading_rad'])@R

def placement_palm_target(fixed_object_goal,seat_palm_local,palm_rotation_at_grasp,direction=1):
    goal=np.array(fixed_object_goal,dtype=float);seat=np.array(seat_palm_local,dtype=float);R=np.array(palm_rotation_at_grasp,dtype=float)
    if goal.shape!=(3,) or seat.shape!=(3,) or R.shape!=(3,3) or not np.all(np.isfinite(np.r_[goal,seat,R.ravel()])) or direction not in [-1,1]:raise ValueError('Finite fixed target,seat,rotation and direction required')
    if np.max(abs(R.T@R-np.eye(3)))>1e-9 or abs(np.linalg.det(R)-1)>1e-9:raise ValueError('Proper palm rotation required')
    final_R=rz(direction*math.pi)@R;return goal-final_R@seat,final_R
