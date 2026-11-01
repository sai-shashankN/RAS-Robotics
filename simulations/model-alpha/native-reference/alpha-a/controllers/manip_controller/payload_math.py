"""Known carried-load gravity/inertia estimate; never a physical attachment."""
import numpy as np

def point_jacobian(model,q,seat_local=(0.,0.,.058)):
    palm,Jv,Jw=model.palm_jacobian(q);R=model.frames(q)[1][3];offset=R@np.array(seat_local);J=Jv+np.cross(Jw.T,offset).T
    return J,Jw

def payload_gravity(model,q,mass,seat_world,actual_palm_pitch):
    c,s=np.cos(actual_palm_pitch),np.sin(actual_palm_pitch);local=(c*seat_world[0]-s*seat_world[2],seat_world[1],s*seat_world[0]+c*seat_world[2]);J,Jw=point_jacobian(model,q,local);return mass*9.81*J[2]

class LoadedModel:
    def __init__(self,model,mass):self.model=model;self.mass=mass;self.p=model.p
    def reduction(self):return self.model.reduction()
    def mass_gravity(self,q):
        M,G=self.model.mass_gravity(q);J,Jw=point_jacobian(self.model,q);return M+self.mass*(J.T@J)+self.mass*.02**2/6*(Jw.T@Jw),G+self.mass*9.81*J[2]
