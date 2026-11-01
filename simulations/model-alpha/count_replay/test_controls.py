import copy,unittest
from replay import validate,scales,header
def sample():
    s=dict(schema=1,read_s=.005,sampling_period_ms=5,frame='sensor_axes',accel_units='m/s^2_specific_force',gyro_units='rad/s_body_rates',hardware_measurement=False,usable_for_feedback=False,valid=True,acquisition_s=None,specific_force=[0.,0.,9.81],angular_rate=[0.,0.,0.])
    return dict(producer_read_s=.005,samples=dict(identity=dict(snapshot=s,accel_period_ms=5,gyro_period_ms=5,gyro_api=[0.,0.,0.])))
class Controls(unittest.TestCase):
    def test_explicit_nine_scales_and_all_axes(self):
        self.assertEqual(len(scales()),9);self.assertEqual(len(header()),28);self.assertEqual([r['range_value'] for r in scales()],[2,4,8,16,2000,1000,500,250,125]);validate(sample(),1)
    def test_clock_units_frame_and_feedback_scope_rejections(self):
        for key,value in [('read_s',.006),('frame','body_axes'),('accel_units','g'),('gyro_units','deg/s'),('acquisition_s',.005),('hardware_measurement',True),('usable_for_feedback',True),('valid',False),('specific_force',[0.,0.,float('nan')]),('angular_rate',[True,0,0])]:
            row=sample();row['samples']['identity']['snapshot'][key]=value
            with self.assertRaises(ValueError):validate(row,1)
        with self.assertRaises(ValueError):validate(sample(),2)
    def test_actual_pair_and_complete_gyro_agreement_required(self):
        r=sample();r['samples']['identity']['gyro_api']=[1.,0.,0.]
        with self.assertRaises(ValueError):validate(r,1)
        r=sample();r['samples']['identity']['gyro_period_ms']=8
        with self.assertRaises(ValueError):validate(r,1)
if __name__=='__main__':unittest.main()
