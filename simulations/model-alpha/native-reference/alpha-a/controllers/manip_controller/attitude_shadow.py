"""Offline gyro/gravity attitude baseline. Inputs are six-axis snapshots only."""
import math
from dataclasses import dataclass
from measurement import I,vector,rotation,cross,transpose_apply

def norm(v):return math.sqrt(sum(x*x for x in v))
def normalize(q):
    n=norm(q)
    if not math.isfinite(n) or n<1e-15:raise ValueError('degenerate quaternion')
    return tuple(x/n for x in q)
def multiply(a,b):
    w,x,y,z=a;v,p,q,r=b
    return (w*v-x*p-y*q-z*r,w*p+x*v+y*r-z*q,w*q-x*r+y*v+z*p,w*r+x*q-y*p+z*v)
def matrix(q):
    w,x,y,z=normalize(q)
    return (1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))
def from_gravity(a):
    u=tuple(x/norm(a) for x in a);z=(0.,0.,1.);axis=cross(u,z)
    if u[2]<-1+1e-12:return (0.,1.,0.,0.)
    return normalize((1+u[2],)+axis)
def step_quaternion(q,w,dt):
    angle=norm(w)*dt
    if angle<1e-12:dq=(1.,)+tuple(x*dt/2 for x in w)
    else:dq=(math.cos(angle/2),)+tuple(x*math.sin(angle/2)/norm(w) for x in w)
    return normalize(multiply(q,dq))
def forward(R,v):return tuple(sum(R[3*i+j]*v[j] for j in range(3)) for i in range(3))

@dataclass(frozen=True)
class Config:
    dt_s:float=.008
    gravity_m_s2:float=9.81
    gravity_norm_band_m_s2:float=.3
    correction_gain_per_s:float=2.
    gyro_bias_rad_s:tuple=(0.,0.,0.)
    R_BS:tuple=I

class ShadowAttitude:
    """Read-clock replay with relative yaw; no pose/velocity/witness input.

    Accelerometer norm screening cannot separate sustained horizontal acceleration
    from tilt. Bias is configured, not learned by assuming the robot is stationary.
    A malformed/gapped stream latches unavailable and never silently reinitializes.
    """
    def __init__(self,cfg=Config()):
        if not all(math.isfinite(x) and x>0 for x in [cfg.dt_s,cfg.gravity_m_s2,cfg.gravity_norm_band_m_s2]) or not math.isfinite(cfg.correction_gain_per_s) or cfg.correction_gain_per_s<0:raise ValueError('configuration')
        self.c=cfg;self.mount=rotation(cfg.R_BS);self.bias=vector(cfg.gyro_bias_rad_s);self.q=None;self.last_t=None;self.last_w=None;self.fault=None
    def result(self,t,status,gravity_assumption=False):
        t=t if isinstance(t,(float,int)) and not isinstance(t,bool) and math.isfinite(t) else None
        return dict(read_s=t,status=status,quaternion_WB=None if self.q is None else list(self.q),gravity_assumption_applied=gravity_assumption,yaw_absolute_observed=False,usable_for_feedback=False,hardware_measurement=False,fault=self.fault)
    def update(self,sample):
        if not isinstance(sample,dict):self.fault='ValueError:sample dictionary required';return self.result(None,'latched_unavailable')
        t=sample.get('read_s')
        if self.fault is not None:return self.result(t,'latched_unavailable')
        try:
            if not isinstance(t,(float,int)) or isinstance(t,bool) or not math.isfinite(t):raise ValueError('read clock')
            if self.last_t is not None and abs(t-self.last_t-self.c.dt_s)>1e-9:raise ValueError('consecutive sample required')
            if sample.get('valid') is not True or sample.get('sampling_period_ms')!=round(self.c.dt_s*1000) or sample.get('frame')!='sensor_axes':raise ValueError('sample status/frame/period')
            if sample.get('hardware_measurement',False) is not False:raise ValueError('offline native or synthetic replay only')
            a=forward(self.mount,vector(sample['specific_force']));raw=forward(self.mount,vector(sample['angular_rate']));w=tuple(x-b for x,b in zip(raw,self.bias));m=norm(a);trusted=abs(m-self.c.gravity_m_s2)<=self.c.gravity_norm_band_m_s2
            if self.q is None:
                if trusted:self.q=from_gravity(a)
                self.last_t=t;self.last_w=w;return self.result(t,'initialized_from_gravity' if self.q is not None else 'waiting_for_gravity',trusted)
            avg=tuple((x+y)/2 for x,y in zip(self.last_w,w));error=(0.,0.,0.)
            if trusted:
                measured=tuple(x/m for x in a);predicted=transpose_apply(matrix(self.q),(0.,0.,1.));error=cross(measured,predicted)
            corrected=tuple(x+self.c.correction_gain_per_s*e for x,e in zip(avg,error));self.q=step_quaternion(self.q,corrected,self.c.dt_s);self.last_t=t;self.last_w=w;return self.result(t,'shadow_attitude',trusted)
        except (ValueError,TypeError,KeyError) as error:
            self.fault=type(error).__name__+':'+str(error);return self.result(t,'latched_unavailable')
