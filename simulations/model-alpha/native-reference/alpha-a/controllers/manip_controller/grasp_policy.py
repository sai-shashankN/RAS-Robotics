"""Pure port of the qualified supported closure; no simulator state writes."""
from policy import CAP,advance
class Dwell:
    def __init__(self):self.start=None
    def update(self,t,valid):
        if not valid:self.start=None;return 0.
        if self.start is None:self.start=t
        return t-self.start
def smooth(q):q=max(0.,min(1.,q));return q*q*(3-2*q)
class GraspClose:
    def __init__(self):
        self.anchors=[None,None];self.goals=[-.3,-.3];self.preload_time=None;self.preload_from=None;self.aligned_dwell=Dwell();self.grasp_dwell=Dwell()
    def update(self,t,finger,a):
        events=[];aligned=all(abs(a[f'finger_{i}']+a[f'distal_{i}'])<=.001 for i in range(2));ready=False
        for i in range(2):
            if i in finger and self.anchors[i] is None:
                self.anchors[i]=a[f'finger_{i}'];self.goals[i]=self.anchors[i];events.append(dict(time_s=t,event='first_contact_rebase',finger=i,q=self.anchors[i]))
            elif self.preload_time is None and i not in finger:self.goals[i]=advance(self.goals[i],self.anchors[i] is not None,.008)
        if self.preload_time is None and self.aligned_dwell.update(t,len(finger)==2 and aligned)>=.4:
            self.preload_time=t;self.preload_from=list(self.goals);events.append(dict(time_s=t,event='settled_bilateral_preload',goals=self.goals[:]))
        if self.preload_time is not None:
            self.goals=[min(CAP,g+.002*smooth((t-self.preload_time)/.5)) for g in self.preload_from]
            ready=t-self.preload_time>=.5 and self.grasp_dwell.update(t,len(finger)==2 and aligned)>=1
        return bool(ready),events
