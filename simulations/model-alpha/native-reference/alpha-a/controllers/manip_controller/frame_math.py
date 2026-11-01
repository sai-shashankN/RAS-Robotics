"""Fixed-heading planar manipulation coordinates, separate from world XY."""
import math
from path_math import world_ik
from manip_math import equilibrium
class StationFrame:
    def __init__(self,world_anchor,path_anchor,heading):
        if len(world_anchor)!=3 or not all(math.isfinite(float(v)) for v in [*world_anchor,path_anchor,heading]):raise ValueError('Finite3Danchor,path and heading required')
        self.origin=tuple(float(v) for v in world_anchor);self.path=float(path_anchor);self.heading=float(heading)
    def world(self,path,height,lateral=0.):
        x=path-self.path;c,s=math.cos(self.heading),math.sin(self.heading)
        return [self.origin[0]+c*x-s*lateral,self.origin[1]+s*x+c*lateral,float(height)]
    def task(self,world):
        x,y=world[0]-self.origin[0],world[1]-self.origin[1];c,s=math.cos(self.heading),math.sin(self.heading)
        return self.path+c*x+s*y,float(world[2]),-s*x+c*y
    def vector(self,world_vector):
        x,y,z=world_vector;c,s=math.cos(self.heading),math.sin(self.heading)
        return [c*x+s*y,-s*x+c*y,z]
    def ik(self,p,world_palm,base_path,pitch=0.,lateral_tolerance=.002):
        path,height,lateral=self.task(world_palm)
        if abs(lateral)>lateral_tolerance:raise ValueError('Target needs lateral base alignment')
        pose=world_ik(p,path-base_path,height,pitch,0.,0.,1)
        if pose is None:raise ValueError('Planar target unreachable')
        return pose,lateral
    def balance(self,p,world_palm,actual,payload,seat_world,guess=.1,lateral_tolerance=.002):
        path,height,lateral=self.task(world_palm)
        if abs(lateral)>lateral_tolerance:raise ValueError('Target needs lateral base alignment')
        base,u,residual=equilibrium(p,[path,height],actual,payload,self.vector(seat_world)[0],guess)
        return dict(base_path=base,relative_palm_x=u,residual_m=residual,lateral_error_m=lateral,world_root=self.world(base,p['wheel_radius']))
