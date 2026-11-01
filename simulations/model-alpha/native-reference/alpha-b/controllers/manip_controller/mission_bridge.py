"""Wire ideal observations and fixed mission frames; no simulator/motor calls."""
import math
import numpy as np
from mission_reference import DestinationTask,PHASES
from observations import Observation,FixtureContacts,Destination
from planning import PlacementPlanner
from controller_core import PlacementController

def observation(t,root,rpy,speed,rates,path,palm,palm_R,cube,cube_R,cube_v,fingers,joints,packet,bad,penetration):
    if abs(packet['end_s']-t)>1e-9 or packet['valid']!=1:raise ValueError('Current valid live contact packet required')
    contacts=FixtureContacts(t,tuple(int(packet[f'fixture_{i}_top_points']) for i in range(8)),tuple(int(packet[f'fixture_{i}_other_points']) for i in range(8)),True)
    o=Observation(t,tuple(root),tuple(rpy),(speed*math.cos(rpy[2]),speed*math.sin(rpy[2]),0.),tuple(rates),path,tuple(palm),tuple(np.array(palm_R).reshape(-1)),tuple(cube),tuple(cube_R),tuple(cube_v),fingers,dict(joints),contacts,bool(bad or packet['forbidden_points'] or packet['unknown_points']),bool(penetration or packet['max_cube_static_depth_m']>.001 or packet['max_forbidden_depth_m']>.001))
    o.validate();return o

class MissionTask:
    def __init__(self,params,profile,setup,origin,heading,palm,cube,path,seat_local,palm_R):
        self.params=params;self.destination=Destination.from_profile(profile);self.size=setup['object_size_m'];held=self.destination.object_goal(self.size);held[2]=cube[2]
        self.task=DestinationTask(origin,heading,palm,cube,held);self.reference=self.task.reference;self.heading=self.task.heading_goal;self.path_start=float(path);self.seat_local=np.array(seat_local).copy();self.palm_R=np.array(palm_R).copy();self.placement=None
    def sample(self,t):
        s,root,palm,cube,heading=self.task.sample(t);return s,palm,cube,heading
    def arrive(self,elapsed,obs,commands,grip_goals,membership):
        if self.placement is not None:raise ValueError('Arrival may be admitted only once')
        planner=PlacementPlanner(self.params,self.task.arrival_anchor(self.path_start),self.destination,self.size,self.seat_local,self.palm_R)
        self.placement=PlacementController(planner,self.task,self.path_start,elapsed,obs,commands,grip_goals,membership)
        return self.placement
