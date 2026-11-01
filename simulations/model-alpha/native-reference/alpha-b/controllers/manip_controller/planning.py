"""Fixed world destination and heading-frame planning; no actuator calls."""
import math
import numpy as np
from frame_math import StationFrame
from manip_math import Governor
from mission_reference import placement_palm_target

def wrap(a):return math.atan2(math.sin(a),math.cos(a))

class PlacementPlanner:
    def __init__(self,params,arrival_anchor,destination,object_size,seat_local,palm_rotation_at_grasp,direction=1):
        self.params=params;self.destination=destination;self.size=tuple(object_size)
        if len(self.size)!=3 or not all(math.isfinite(x) and x>0 for x in self.size):raise ValueError('Positive finite object dimensions required')
        self.frame=StationFrame(arrival_anchor['world_anchor'],arrival_anchor['path_anchor'],arrival_anchor['heading'])
        self.object_goal=destination.object_goal(self.size)
        self.rest_palm,self.palm_goal_rotation=placement_palm_target(self.object_goal,seat_local,palm_rotation_at_grasp,direction)
        _,_,lateral=self.frame.task(self.rest_palm)
        if abs(lateral)>.002:raise ValueError('Fixed destination requires lateral base alignment')
        if any(self.size[i]/2>destination.half_xy[i] for i in range(2)):raise ValueError('Object footprint does not fit destination top')
        self.local_seat=np.array(seat_local,dtype=float);self.world_seat=self.palm_goal_rotation@self.local_seat;self.governor=Governor(self.frame.path);self.guess=.1
    def static(self,target_world,actual,reference_mass):
        result=self.frame.balance(self.params,target_world,actual,reference_mass,self.world_seat,self.guess)
        pose,lateral=self.frame.ik(self.params,target_world,result['base_path'])
        return result,pose,lateral
    def step(self,target_world,actual,reference_mass):
        result=self.frame.balance(self.params,target_world,actual,reference_mass,self.world_seat,self.guess);self.guess=result['relative_palm_x']
        path,speed=self.governor.update(result['base_path']);pose,lateral=self.frame.ik(self.params,target_world,path)
        scalar,height,_=self.frame.task(target_world)
        return dict(base_goal_path_m=float(result['base_path']),base_reference_path_m=float(path),base_reference_speed_m_s=float(speed),main_target={n:float(pose[n]) for n in ['shoulder','elbow','wrist']},target_world_xyz=[float(v) for v in target_world],projected_target_world_xyz=[float(v) for v in self.frame.world(scalar,height)],scalar_target_path_m=float(scalar),lateral_residual_m=float(lateral),heading_reference_rad=self.frame.heading)

def arrival_checks(task,path_start,transit_elapsed,obs,destination,object_size):
    obs.validate();sample,root,palm,cube,heading=task.sample(task.reference.duration);anchor=task.arrival_anchor(path_start);top=destination.top_m
    from cube_geometry import corners,bottom
    gap=bottom(corners(obs.cube_world,obs.cube_rotation,object_size))-top
    root_error=math.dist(root,obs.root_world);path_error=abs(obs.path_m-anchor['path_anchor']);Q=np.array([[-math.sin(heading),math.cos(heading),0.]])[0];lateral=abs(float(Q@(np.array(obs.root_world)-root)))
    cube_tilt=math.acos(max(-1.,min(1.,obs.cube_rotation[8])))
    return dict(complete_reference=transit_elapsed>=task.reference.duration,root_world3mm=root_error<=.003,path3mm=path_error<=.003,lateral2mm=lateral<=.002,heading02rad=abs(wrap(obs.root_rpy[2]-heading))<=.02,palm_world3mm=math.dist(palm,obs.palm_world)<=.003,cube_world3mm=math.dist(cube,obs.cube_world)<=.003,fixed_destination_center3mm=math.dist(obs.cube_world[:2],destination.center_xy)<=.003,unsupported_lift8mm=gap>=.008,bilateral_grasp=obs.finger_count==2,zero_fixture_contacts=obs.contacts.valid and obs.contacts.total()==0,body_settled=max(abs(v) for v in obs.root_rpy[:2])<math.radians(1) and math.hypot(*obs.root_velocity[:2])<.01 and max(abs(v) for v in obs.root_angular_velocity)<.01,cube_tilt3deg=cube_tilt<=math.radians(3),no_forbidden_contact=not obs.forbidden_contact,no_penetration=not obs.penetration)
