import unittest,math
from projection import project
def quat(roll,pitch,yaw):
    r,p,y=[v/2 for v in [roll,pitch,yaw]];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]
def inputs(r=0.,p=0.,y=0.):return dict(read_s=.016,state_read_s=.016,status='shadow_attitude',fault=None,quaternion_WB=quat(r,p,y)),dict(read_s=.016,valid=True,sampling_period_ms=8,frame='sensor_axes',hardware_measurement=False,angular_rate=[.1,.2,.3])
class ProjectionControls(unittest.TestCase):
    def test_signed_angles_and_Euler_rate_with_nonzero_roll(self):
        for sign in [-1,1]:
            r,p,y=[math.radians(sign*v) for v in [20,30,125]];s,a=inputs(r,p,y);v=project(s,a);self.assertEqual(v['status'],'shadow_projection');self.assertAlmostEqual(v['roll_rad'],r);self.assertAlmostEqual(v['pitch_rad'],p);self.assertAlmostEqual(v['pitch_rate_rad_s'],math.cos(r)*.2-math.sin(r)*.3);self.assertNotAlmostEqual(v['pitch_rate_rad_s'],.2)
    def test_unknown_world_yaw_does_not_change_pitch_projection(self):
        values=[project(*inputs(.2,-.3,y)) for y in [-2.7,0.,1.9]]
        for v in values:self.assertAlmostEqual(v['roll_rad'],.2);self.assertAlmostEqual(v['pitch_rad'],-.3);self.assertAlmostEqual(v['pitch_rate_rad_s'],values[0]['pitch_rate_rad_s']);self.assertFalse(v['yaw_absolute_observed']);self.assertFalse(v['usable_for_feedback'])
    def test_waiting_fault_and_stale_state_have_no_projected_signal(self):
        for kind in ['waiting','fault','stale']:
            s,a=inputs()
            if kind=='waiting':s['quaternion_WB']=None;s['state_read_s']=None
            elif kind=='fault':s['fault']='latched';s['state_read_s']=.008
            else:s['state_read_s']=.008
            v=project(s,a)
            for k in ['roll_rad','pitch_rad','pitch_rate_rad_s','state_read_s','acquisition_s']:self.assertIsNone(v[k])
            self.assertFalse(v['usable_for_feedback'])
    def test_declared_Euler_singularity_is_explicitly_unavailable(self):
        for p in [math.radians(70),math.pi/2,-math.pi/2]:
            v=project(*inputs(0,p,0));self.assertEqual(v['status'],'projection_unavailable');self.assertIn('singularity',v['reason']);self.assertIsNone(v['pitch_rad'])
    def test_nonfinite_malformed_status_and_raw_frame_rejected(self):
        for field,value in [('angular_rate',[float('nan'),0,0]),('frame','world_axes'),('valid',False),('read_s',True)]:
            s,a=inputs();a[field]=value;self.assertEqual(project(s,a)['status'],'projection_unavailable')
        s,a=inputs();s['quaternion_WB']=[2,0,0,0];self.assertEqual(project(s,a)['status'],'projection_unavailable')
    def test_sensor_to_body_transform_and_no_input_mutation(self):
        s,a=inputs();before=repr((s,a));R=[0,-1,0,1,0,0,0,0,1];v=project(s,a,R);self.assertAlmostEqual(v['pitch_rate_rad_s'],.1);self.assertEqual(repr((s,a)),before);self.assertFalse(v['hardware_measurement'])
if __name__=='__main__':unittest.main()
