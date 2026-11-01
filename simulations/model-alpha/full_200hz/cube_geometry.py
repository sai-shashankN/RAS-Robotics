"""Orientation-aware box corners and rectangular support witnesses (observation only)."""
import itertools
import math


def corners(center,rotation,size):
    return [[center[i]+sum(rotation[3*i+j]*sign[j]*size[j]/2 for j in range(3)) for i in range(3)]
            for sign in itertools.product([-1,1],repeat=3)]


def bottom(points):return min(point[2] for point in points)


def point_on_top(point,center_xy,half_size,top,tolerance=.0001):
    return abs(point[2]-top)<tolerance and all(abs(point[i]-center_xy[i])<=half_size[i]+tolerance for i in [0,1])


def resting_on_top(points,center_xy,half_size,top,speed,tolerance=.0001):
    """Sleeping-body witness: lowest box feature intersects the support rectangle."""
    low = bottom(points)
    if abs(low-top)>=tolerance or speed>=.005:return False
    feature = [p[:2] for p in points if p[2]-low<tolerance]
    cx,cy = [sum(p[i] for p in feature)/len(feature) for i in [0,1]]
    feature.sort(key=lambda p:math.atan2(p[1]-cy,p[0]-cx))
    rect = [[center_xy[0]+sx*half_size[0],center_xy[1]+sy*half_size[1]] for sx,sy in itertools.product([-1,1],repeat=2)]
    axes = [[1,0],[0,1]]
    for a,b in zip(feature,feature[1:]+feature[:1]):
        dx,dy = b[0]-a[0],b[1]-a[1];length = math.hypot(dx,dy)
        if length>1e-12:axes.append([-dy/length,dx/length])
    for axis in axes:
        fa = [sum(p[i]*axis[i] for i in [0,1]) for p in feature]
        ra = [sum(p[i]*axis[i] for i in [0,1]) for p in rect]
        if max(fa)<min(ra)-tolerance or max(ra)<min(fa)-tolerance:return False
    return True


def self_check():
    identity = [1,0,0,0,1,0,0,0,1];size = [.02,.02,.02]
    points = corners([0,0,.02],identity,size)
    assert abs(bottom(points)-.01)<1e-12
    assert resting_on_top(points,[0,0],[.006,.006],.01,0)  # support inside a larger bottom face
    assert not resting_on_top(points,[.04,0],[.006,.006],.01,0)
    assert not resting_on_top(points,[0,0],[.006,.006],.009,0)
    a = math.pi/4;c,s = math.cos(a),math.sin(a)
    tilted = corners([0,0,.02],[c,0,s,0,1,0,-s,0,c],size)
    assert abs(bottom(tilted)-(.02-.01*math.sqrt(2)))<1e-12
    assert not resting_on_top(tilted,[0,0],[.006,.006],.01,0)
    assert point_on_top([0,0,.01],[0,0],[.006,.006],.01)
    assert not point_on_top([.02,0,.01],[0,0],[.006,.006],.01)


if __name__=='__main__':self_check()
