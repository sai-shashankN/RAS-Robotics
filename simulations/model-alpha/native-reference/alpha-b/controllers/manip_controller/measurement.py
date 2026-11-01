"""Reference equations and a passive six-axis read boundary. No state estimator."""
import math,numbers

I=(1.,0.,0.,0.,1.,0.,0.,0.,1.)
def vector(v):
    if len(v)!=3 or any(isinstance(x,bool) or not isinstance(x,numbers.Real) or not math.isfinite(x) for x in v):raise ValueError('finite three-vector')
    return tuple(float(x) for x in v)
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def add(*vs):return tuple(sum(v[i] for v in vs) for i in range(3))
def transpose_apply(R,v):return tuple(sum(R[3*j+i]*v[j] for j in range(3)) for i in range(3))
def rotation(R):
    if len(R)!=9 or any(not math.isfinite(x) for x in R):raise ValueError('rotation shape/finite')
    if max(abs(sum(R[3*k+i]*R[3*k+j] for k in range(3))-(i==j)) for i in range(3) for j in range(3))>1e-10:raise ValueError('orthogonal rotation')
    if abs(sum(R[i]*cross(R[3:6],R[6:9])[i] for i in range(3))-1)>1e-10:raise ValueError('right handed rotation')
    return tuple(R)

def reference(R_WB,a_origin_W,omega_B,alpha_B,r_B=(0.,0.,0.),R_BS=I,gravity_W=(0.,0.,-9.81)):
    """Rigid sensor: f_S=R_BS^T[R_WB^T(a_origin_W-g_W)+alpha×r+omega×(omega×r)].

    R_WB maps body to world; R_BS maps sensor to body; r_B is fixed sensor
    offset from the acceleration reference origin. Inputs are reference witnesses
    for validation only. They are not a real sensor's available control inputs.
    """
    R_WB=rotation(R_WB);R_BS=rotation(R_BS);a_origin_W=vector(a_origin_W);omega_B=vector(omega_B);alpha_B=vector(alpha_B);r_B=vector(r_B);gravity_W=vector(gravity_W)
    f_B=add(transpose_apply(R_WB,tuple(a-b for a,b in zip(a_origin_W,gravity_W))),cross(alpha_B,r_B),cross(omega_B,cross(omega_B,r_B)))
    return dict(specific_force_S_m_s2=transpose_apply(R_BS,f_B),angular_rate_S_rad_s=transpose_apply(R_BS,omega_B))

class PassiveCapture:
    """Snapshot API values; read time is not a measured hardware acquisition time."""
    def __init__(self,accelerometer,gyro,period_ms=8,enabled_s=0.):
        if type(period_ms) is not int or period_ms<=0 or not math.isfinite(enabled_s):raise ValueError('sampling configuration')
        self.accel=accelerometer;self.gyro=gyro;self.period=period_ms;self.enabled=enabled_s;self.previous=None
    def read(self,read_s):
        if not math.isfinite(read_s) or read_s<self.enabled or (self.previous is not None and read_s<=self.previous):raise ValueError('monotonic read clock')
        self.previous=read_s;row=dict(schema=1,read_s=read_s,acquisition_s=None,sampling_period_ms=self.period,frame='sensor_axes',accel_units='m/s^2_specific_force',gyro_units='rad/s_body_rates',hardware_measurement=False,usable_for_feedback=False,valid=False,specific_force=None,angular_rate=None)
        if read_s+1e-12<self.enabled+self.period*.001:row['reason']='first_sample_not_due';return row
        try:
            if any(s.getSamplingPeriod()!=self.period for s in [self.accel,self.gyro]):raise ValueError('sampling_period_mismatch')
            a=vector(self.accel.getValues());w=vector(self.gyro.getValues())
        except (ValueError,TypeError,RuntimeError,AttributeError) as ex:row['reason']=type(ex).__name__+':'+str(ex);return row
        row.update(valid=True,reason='finite_api_snapshot_only',specific_force=a,angular_rate=w);return row
