"""Bounded yaw PI and independently specified task references."""
import math
CASES=['rate','heading_pos','heading_neg','zero']
def clip(x,a,b):return max(a,min(b,x))
def transition(t,a,b,duration):
    u=clip(t/duration,0,1);return a+(b-a)*u*u*(3-2*u),(b-a)*6*u*(1-u)/duration
def reference(t,case):
    if case=='zero':return dict(stage='zero',rate=0.,heading=None,duration=59.)
    if case.startswith('heading'):
        goal=math.pi*(1 if case=='heading_pos' else -1);angle,rate=transition(t-5,0,goal,36)
        return dict(stage='settle' if t<5 else 'turn' if t<41 else 'settle_heading' if t<45 else 'verify',rate=rate,heading=angle,duration=53.)
    if t<5:stage,w='settle',0.
    elif t<11:stage='ramp_positive';w=transition(t-5,0,.1,6)[0]
    elif t<19:stage,w='positive',.1
    elif t<25:stage='ramp_zero';w=transition(t-19,.1,0,6)[0]
    elif t<31:stage,w='pause',0.
    elif t<37:stage='ramp_negative';w=transition(t-31,0,-.08,6)[0]
    elif t<45:stage,w='negative',-.08
    elif t<51:stage='ramp_stop';w=transition(t-45,-.08,0,6)[0]
    else:stage,w='recover',0.
    return dict(stage=stage,rate=w,heading=None,duration=59.)
class PI:
    def __init__(self,kp=.35,ki=.175,integral_limit=.06):self.kp=kp;self.ki=ki;self.limit=integral_limit;self.integral=0.;self.last_direction=0;self.reset_event=False
    def update(self,request,measured,cap,dt=.008,fault='none',motion_reference=0.):
        error=request-measured
        self.reset_event=False
        if fault=='no_yaw':self.integral=0.;return 0.,error,0.,False
        if fault=='no_integral':self.integral=0.;raw=self.kp*error;return clip(raw,-cap,cap),error,raw,abs(raw)>cap
        direction=1 if motion_reference>.01 else -1 if motion_reference<-.01 else 0
        if direction and self.last_direction and direction!=self.last_direction and direction*self.integral<0:
            self.integral=0.;self.reset_event=True
        if direction:self.last_direction=direction
        candidate=clip(self.integral+self.ki*error*dt,-self.limit,self.limit);raw=self.kp*error+candidate
        # Conditional integration: only accumulate into saturation when the
        # error would unwind it. The hard integral bound is separate.
        if abs(raw)<=cap or raw*error<=0:self.integral=candidate
        raw=self.kp*error+self.integral
        return clip(raw,-cap,cap),error,raw,abs(raw)>cap
