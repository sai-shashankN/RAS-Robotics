"""Reference feedforward schedule, separate from observed grasp membership.

The free cube stays physical. This is neither measured carried weight nor a
contact-force estimator. Safety/fault membership clearing remains independent.
"""
import math

class SupportReference:
    def __init__(self,duration=1.):
        if not math.isfinite(duration) or duration<=0:raise ValueError('Positive finite duration required')
        self.duration=duration;self.started=None;self.mass=0.
    def update(self,t,stage,payload,stand_contact,stable,failure=False):
        if failure:return 0.,False
        if not math.isfinite(t) or not math.isfinite(payload) or payload<0:raise ValueError('Finite nonnegative reference input required')
        if self.started is None and stage=='open':
            if not(stand_contact and stable and payload>0):return 0.,True
            self.started=t;self.mass=payload
        if self.started is None:return payload,False
        if stage=='open' and not stand_contact:return 0.,True
        u=max(0.,min(1.,(t-self.started)/self.duration));return self.mass*(1-u*u*(3-2*u)),False
