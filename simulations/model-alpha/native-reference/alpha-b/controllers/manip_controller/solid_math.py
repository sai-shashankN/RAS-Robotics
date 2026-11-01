"""Actual oriented-box penetration; no infinite platform-top proxy."""
import math

def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def columns(R):return [[R[j][i] for j in range(3)] for i in range(3)]
def gap(ca,Ra,ha,cb,Rb,hb):
    A,B=columns(Ra),columns(Rb);delta=[b-a for a,b in zip(ca,cb)];best=-float('inf')
    for axis in A+B+[cross(a,b) for a in A for b in B]:
        length=math.sqrt(dot(axis,axis))
        if length<1e-9:continue
        axis=[x/length for x in axis];value=abs(dot(delta,axis))-sum(h*abs(dot(axis,v)) for h,v in zip(ha,A))-sum(h*abs(dot(axis,v)) for h,v in zip(hb,B));best=max(best,value)
    return best

def measure(center,rotation_flat,size,profile):
    R=[rotation_flat[i:i+3] for i in [0,3,6]];half=[v/2 for v in size];gaps=[gap(center,R,half,v['center_m'],v['rotation'],v['half_m']) for v in profile['fixtures']];minimum=min(gaps);bottom=center[2]-sum(abs(R[2][i])*half[i] for i in range(3));tol=profile['tolerance_m'];return dict(minimum_fixture_gap_m=minimum,floor_bottom_m=bottom,violated=minimum < -tol or bottom < profile['floor_z_m']-tol)
