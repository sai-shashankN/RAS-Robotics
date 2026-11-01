"""Virtual optical geometry,not image generation or a perception model."""
import math
import numpy as np
def camera_rotation(direction):
    forward=np.array(direction,dtype=float);forward/=np.linalg.norm(forward);right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right);down=np.cross(forward,right);return np.column_stack([right,down,forward])
def project(points,origin,R,width=640,height=480,hfov=math.radians(70)):
    points=np.array(points,dtype=float);local=(points-origin)@R;focal=width/(2*math.tan(hfov/2));z=local[:,2];safe=np.where(abs(z)>1e-12,z,np.nan);pixels=local[:,:2]/safe[:,None]*focal+[width/2,height/2];inside=(z>0)&(pixels[:,0]>=0)&(pixels[:,0]<width)&(pixels[:,1]>=0)&(pixels[:,1]<height);return pixels,inside,z
def blocked(points,origin,shape):
    _,_,c,R,h=shape;o=(origin-c)@R;d=(np.array(points)-origin)@R;parallel=abs(d)<1e-12;low=np.full_like(d,-np.inf);high=np.full_like(d,np.inf)
    for i in range(3):
        moving=~parallel[:,i];a=np.zeros(len(d));b=np.zeros(len(d));np.divide(-h[i]-o[i],d[:,i],out=a,where=moving);np.divide(h[i]-o[i],d[:,i],out=b,where=moving);low[moving,i]=np.minimum(a[moving],b[moving]);high[moving,i]=np.maximum(a[moving],b[moving]);outside=parallel[:,i] & (abs(o[i])>h[i]);low[outside,i]=np.inf;high[outside,i]=-np.inf
    enter=low.max(axis=1);leave=high.min(axis=1);return (enter<=leave)&(leave>1e-9)&(enter<1-1e-7)
def surface(center,R,size,resolution=5):
    points=[];normals=[];half=np.array(size)/2
    for axis in range(3):
        other=[i for i in range(3) if i!=axis]
        for sign in [-1,1]:
            n=np.zeros(3);n[axis]=sign
            for u in np.linspace(-.8,.8,resolution):
                for v in np.linspace(-.8,.8,resolution):
                    q=np.zeros(3);q[axis]=sign*half[axis];q[other[0]]=u*half[other[0]];q[other[1]]=v*half[other[1]];points.append(center+R@q);normals.append(R@n)
    return np.array(points),np.array(normals)
def visibility(points,normals,origin,R,shapes):
    pixels,inside,z=project(points,origin,R);facing=np.einsum('ij,ij->i',normals,origin-points)>1e-12;occluded=np.zeros(len(points),dtype=bool)
    for shape in shapes:occluded |= blocked(points,origin,shape)
    visible=facing&inside&~occluded;return dict(facing_points=int(facing.sum()),in_frame_points=int((facing&inside).sum()),visible_points=int(visible.sum()),visible_fraction=float(visible.sum()/max(1,facing.sum())),visible=visible,pixels=pixels)
