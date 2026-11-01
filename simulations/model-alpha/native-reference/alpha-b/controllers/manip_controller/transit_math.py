"""Reusable coordinate/reference interface; this module does not command motors."""
import math
import numpy as np
from dynamics_core import rx,ry,rz
from payload_math import point_jacobian

def rotation(rpy):return rz(rpy[2])@ry(rpy[1])@rx(rpy[0])
def to_world(origin,R,local):return np.array(origin)+R@np.array(local)
def to_local(origin,R,world):return R.T@(np.array(world)-np.array(origin))

def euler_rates(body_gyro,roll,pitch):
    x,y,z=body_gyro;sr,cr=math.sin(roll),math.cos(roll);c=math.cos(pitch);mix=sr*y+cr*z;return np.array([x+math.tan(pitch)*mix,cr*y-sr*z,mix/c])

def known_load_gravity(model,q,mass,seat_palm_local):
    J,_=point_jacobian(model,q,seat_palm_local);return mass*9.81*J[2]

def cubic(t,duration,start,end):
    u=max(0.,min(1.,t/duration));delta=end-start;return start+delta*(3*u*u-2*u*u*u),delta*6*u*(1-u)/duration,delta*(6-12*u)/duration**2

class TransitReference:
    """Backout100mm,turn180,turn back,return.4s dwell at each stop."""
    def __init__(self,distance=.1,back_seconds=50.,turn_seconds=36.,dwell_seconds=4.):
        if distance<=0 or min(back_seconds,turn_seconds,dwell_seconds)<=0:raise ValueError('Positive transit dimensions required')
        self.distance=distance;self.back_seconds=back_seconds;self.turn_seconds=turn_seconds;self.dwell_seconds=dwell_seconds;self.duration=2*back_seconds+2*turn_seconds+4*dwell_seconds
    def sample(self,t):
        if t<0:raise ValueError('Transit time must be nonnegative')
        b,r,w,d=self.back_seconds,self.turn_seconds,self.dwell_seconds,self.distance;clock=0.
        for name,duration in [('backout',b),('backout_settle',w),('turn',r),('turned_hold',w),('turn_back',r),('heading_settle',w),('return',b),('final_settle',w)]:
            if t<clock+duration or name=='final_settle':
                u=min(duration,t-clock);s=-d if name not in ['backout','return','final_settle'] else 0.;v=a=yaw=omega=alpha=0.
                if name=='backout':s,v,a=cubic(u,b,0.,-d)
                elif name=='turn':yaw,omega,alpha=cubic(u,r,0.,math.pi)
                elif name=='turned_hold':yaw=math.pi
                elif name=='turn_back':yaw,omega,alpha=cubic(u,r,math.pi,0.)
                elif name=='return':s,v,a=cubic(u,b,-d,0.)
                return dict(stage=name,path_m=s,speed_m_s=v,acceleration_m_s2=a,heading_rad=yaw,heading_rate_rad_s=omega,heading_acceleration_rad_s2=alpha)
            clock+=duration

def world_reference(sample,base_start,heading_start,point_body,body_roll=0.,body_pitch=0.):
    origin=np.array(base_start)+rz(heading_start)@np.array([sample['path_m'],0.,0.]);rpy=[body_roll,body_pitch,heading_start+sample['heading_rad']];R=rotation(rpy);return origin,R,to_world(origin,R,point_body)
