"""Signed wheel-path reference and continuous yaw; no simulator dependencies."""
import math
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
def smooth(u):u=max(0.,min(1.,u));return u*u*(3-2*u)
def transition(t,start,end,duration):
    u=max(0.,min(1.,t/duration));return start+(end-start)*smooth(u),(end-start)*6*u*(1-u)/duration
def schedule(t,c):
    a=c['backout_m'];b=c['forward_m']
    if t<5:return 'hold',0.,0.,0.,0.
    if t<30:s,v=transition(t-5,0.,a,25);return 'backout',s,v,0.,0.
    if t<34:return 'backout_settle',a,0.,0.,0.
    if t<70:y,w=transition(t-34,0.,math.pi,36);return 'turn',a,0.,y,w
    if t<74:return 'turn_settle',a,0.,math.pi,0.
    if t<99:s,v=transition(t-74,a,a+b,25);return 'forward',s,v,math.pi,0.
    return 'verify',a+b,0.,math.pi,0.

class Odometry:
    def __init__(self,x,y,yaw):self.x=x;self.y=y;self.yaw=yaw;self.absolute=None;self.raw_yaw=None;self.s=0.
    def update(self,radius,left,right,pitch,raw_yaw):
        value=radius*((left+right)/2+pitch)
        if self.absolute is None:self.absolute=value;self.raw_yaw=raw_yaw;return self.x,self.y,self.yaw,self.s,0.
        dy=wrap(raw_yaw-self.raw_yaw);ds=value-self.absolute;mid=self.yaw+dy/2;self.x+=ds*math.cos(mid);self.y+=ds*math.sin(mid);self.yaw+=dy;self.s+=ds;self.absolute=value;self.raw_yaw=raw_yaw
        return self.x,self.y,self.yaw,self.s,ds/.008

