"""Reference witnesses only; no device data, pose feedback or acquisition clock grant."""
import math
from measurement import vector,rotation,cross,add,transpose_apply

def forward(R,v):return tuple(sum(R[3*i+j]*v[j] for j in range(3)) for i in range(3))
def point_velocity(row,mount):
    # Fixture reference origin is its COM. Alpha must independently establish that origin.
    R=rotation(row['orientation']);v=vector(row['velocity'][:3]);w=vector(row['velocity'][3:]);r=vector(mount['r_B']);return add(v,cross(w,forward(R,r)))

def sampled_reference(previous,current,mount,expected_period_s=.008,gravity_W=(0.,0.,-9.81)):
    a,b=previous['time_s'],current['time_s'];dt=b-a
    if not all(math.isfinite(t) for t in [a,b,expected_period_s]) or dt<=0 or abs(dt-expected_period_s)>1e-9:raise ValueError('full consecutive declared sampling interval required')
    vp,vc=point_velocity(previous,mount),point_velocity(current,mount);g=vector(gravity_W);aw=tuple((y-x)/dt-g[i] for i,(x,y) in enumerate(zip(vp,vc)));R=rotation(current['orientation']);M=rotation(mount['R_BS'])
    return dict(interval_start_s=a,interval_end_s=b,interval_s=dt,specific_force=transpose_apply(M,transpose_apply(R,aw)),angular_rate=transpose_apply(M,transpose_apply(R,vector(current['velocity'][3:]))))
