"""Ideal observation contract; no simulator, sensor or actuator access."""
from dataclasses import dataclass
import math
import numpy as np
MAIN=('shoulder','elbow','wrist')
FINGERS=('finger_0','finger_1','distal_0','distal_1')
JOINTS=MAIN+FINGERS

@dataclass(frozen=True)
class FixtureContacts:
    time_s: float
    top: tuple=(0,)*8
    other: tuple=(0,)*8
    valid: bool=True
    def validate(self,t):
        if not math.isfinite(self.time_s) or abs(self.time_s-t)>1e-9:raise ValueError('Fixture observation must match the current control time')
        if len(self.top)!=8 or len(self.other)!=8 or any(not isinstance(v,int) or isinstance(v,bool) or v<0 for v in tuple(self.top)+tuple(self.other)):raise ValueError('Eight nonnegative integer fixture counts required')
        if not isinstance(self.valid,bool):raise ValueError('Explicit fixture observation validity required')
    def total(self):return sum(self.top)+sum(self.other)
    def intended_top(self,fixture=4):return self.valid and self.top[fixture]>0 and sum(self.other)==0 and sum(v for i,v in enumerate(self.top) if i!=fixture)==0
    def wrong_contact(self,fixture=4):return sum(self.other)>0 or any(v for i,v in enumerate(self.top) if i!=fixture)

@dataclass(frozen=True)
class Observation:
    time_s: float
    root_world: tuple
    root_rpy: tuple
    root_velocity: tuple
    root_angular_velocity: tuple
    path_m: float
    palm_world: tuple
    palm_rotation: tuple
    cube_world: tuple
    cube_rotation: tuple
    cube_velocity: tuple
    finger_count: int
    joints: dict
    contacts: FixtureContacts
    forbidden_contact: bool=False
    penetration: bool=False
    def validate(self):
        vectors=[self.root_world,self.root_rpy,self.root_velocity,self.root_angular_velocity,self.palm_world,self.cube_world,self.cube_velocity]
        if any(len(v)!=3 for v in vectors) or not all(math.isfinite(float(v)) for values in vectors for v in values):raise ValueError('Finite3Dobservations required')
        if not math.isfinite(self.time_s) or not math.isfinite(self.path_m):raise ValueError('Finite time/path required')
        if set(self.joints)!=set(JOINTS) or not all(math.isfinite(v) for v in self.joints.values()):raise ValueError('All seven finite measured joints required')
        if not isinstance(self.finger_count,int) or isinstance(self.finger_count,bool) or not 0<=self.finger_count<=2:raise ValueError('Actual finger count must be0..2')
        for flat in [self.palm_rotation,self.cube_rotation]:
            if len(flat)!=9 or not all(math.isfinite(v) for v in flat):raise ValueError('Finite3Drotations required')
            R=np.array(flat).reshape(3,3)
            if np.max(abs(R.T@R-np.eye(3)))>1e-8 or abs(np.linalg.det(R)-1)>1e-8:raise ValueError('Proper measured rotation required')
        self.contacts.validate(self.time_s)

@dataclass(frozen=True)
class Destination:
    fixture_id: int
    center_xy: tuple
    top_m: float
    half_xy: tuple
    def __post_init__(self):
        if self.fixture_id!=4 or len(self.center_xy)!=2 or len(self.half_xy)!=2 or not all(math.isfinite(v) for v in [*self.center_xy,self.top_m,*self.half_xy]) or any(v<=0 for v in self.half_xy):raise ValueError('Fixed finite fixture4destination required')
    @classmethod
    def from_profile(cls,profile):
        if len(profile['fixtures'])!=8:raise ValueError('Qualified eight-fixture profile required')
        v=profile['fixtures'][4];c=v['center_m'];half=v['half_m'];R=np.array(v['rotation'])
        if v['name']!='destination top' or len(c)!=3 or len(half)!=3 or R.shape!=(3,3) or not np.all(np.isfinite(R)) or np.max(abs(R-np.eye(3)))>1e-10:raise ValueError('Fixture4must be the actual horizontal destination top')
        if not all(math.isfinite(x) for x in c+half) or any(x<=0 for x in half):raise ValueError('Finite positive destination box required')
        return cls(4,tuple(c[:2]),c[2]+half[2],tuple(half[:2]))
    def object_goal(self,size):return np.array([*self.center_xy,self.top_m+size[2]/2])
