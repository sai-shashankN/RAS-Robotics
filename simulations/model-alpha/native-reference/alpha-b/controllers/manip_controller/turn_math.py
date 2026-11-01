import math
import numpy as np
from dynamics_core import rz
from transit_math import TransitReference,world_reference
PHASES=['backout','backout_settle','turn','turned_hold','turn_back','heading_settle','return','final_settle']
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi
class TurnTask:
    def __init__(self,origin,heading,palm,object_point):
        self.reference=TransitReference(turn_seconds=60.);self.origin=np.array(origin);self.heading=heading;R=rz(heading);self.palm_local=R.T@(np.array(palm)-self.origin);self.object_local=R.T@(np.array(object_point)-self.origin)
    def sample(self,t):
        s=self.reference.sample(t);origin,R,palm=world_reference(s,self.origin,self.heading,self.palm_local);obj=origin+R@self.object_local
        return s,palm,obj,self.heading+s['heading_rad']


def heading_request(rate,error):return max(-.15,min(.15,rate+3.*error))
def recovery_reference(actual,path,names):
    return float(path),float(path),0.,0.,{n:float(actual[n]) for n in names}
