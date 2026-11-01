import math,sys
from context import ROOT
sys.path.insert(0,str(ROOT/'.local-tools/plotting-packages'))
import numpy as np
from dynamics_core import rx,ry,rz

def assess(data):
    palm_error=rotation_error=seat_error=0.
    for v in data:
        native_palm=np.array([v['palm_'+k+'_m'] for k in 'xyz']);estimated=np.array([v['palm_est_'+k+'_m'] for k in 'xyz']);R=np.array([v['palm_r'+str(i)] for i in range(9)]).reshape(3,3);expected=rz(v['yaw_rad'])@ry(v['pitch_rad'])@rx(v['roll_rad'])@ry(v['shoulder']+v['elbow']+v['wrist']);native_seat=R.T@(np.array([v['object_'+k+'_m'] for k in 'xyz'])-native_palm);palm_error=max(palm_error,float(np.linalg.norm(estimated-native_palm)));rotation_error=max(rotation_error,float(np.max(abs(R-expected))));seat_error=max(seat_error,float(np.linalg.norm(native_seat-[v['seat_local_est_'+k+'_m'] for k in 'xyz'])))
    checks=dict(palm_estimate=palm_error<.002,native_palm_orientation=rotation_error<1e-4,local_seat_estimate=seat_error<.002,finite_coordinates=all(math.isfinite(v[k]) for v in data for k in ['odom_x_m','odom_y_m','odom_yaw_rad','path_s_m','control_pitch_rate_rad_s','control_heading_rate_rad_s']));return dict(passed=all(checks.values()),checks=checks,max_palm_estimate_error_m=palm_error,max_palm_rotation_matrix_error=rotation_error,max_local_seat_estimate_error_m=seat_error)
