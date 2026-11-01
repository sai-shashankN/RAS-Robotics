import math,unittest
from history200 import ApiHistory200
from consumer200 import HistoryConsumer200
from projection200 import project
from replay import profiles,score

def sample(t,rate=(0.,0.,0.)):
    return dict(read_s=t,valid=True,sampling_period_ms=5,frame='sensor_axes',hardware_measurement=False,usable_for_feedback=False,specific_force=[0.,0.,9.81],angular_rate=list(rate),acquisition_s=None)
class Controls(unittest.TestCase):
    def test_predeclared_profiles_are_distinct_and_explicit(self):
        self.assertEqual([(p['acc_code'],p['gyro_code']) for p in profiles()],[(0,4),(1,3),(3,0)])
    def test_first_snapshot_quarantine_then_original5msstate(self):
        h=ApiHistory200();c=HistoryConsumer200()
        for i in range(1,4):
            s=sample(i*.005);m=h.update(s,dict(read_s=s['read_s'],accel_period_ms=5,gyro_period_ms=5));r=c.update(s,m)
            self.assertEqual(r['state_read_s'],None if i==1 else s['read_s']);self.assertFalse(r['usable_for_feedback']);self.assertIsNone(r['acquisition_s'])
    def test_time_gap_latches_and_later_good_sample_cannot_reset(self):
        h=ApiHistory200();c=HistoryConsumer200()
        for t in [.005,.01,.02,.025]:
            s=sample(t);m=h.update(s,dict(read_s=t,accel_period_ms=5,gyro_period_ms=5));r=c.update(s,m)
        self.assertIsNotNone(r['fault']);self.assertEqual(r['state_read_s'],.01)
    def test_known_pitch_and_yaw_scoring_control(self):
        # Known 0.1rad pitch and 0.2rad/s pitch rate against identity witness.
        p=.1;q=[math.cos(p/2),0,math.sin(p/2),0];s=sample(.01,(0,.2,0));state=dict(read_s=.01,state_read_s=.01,status='shadow_attitude',fault=None,quaternion_WB=q)
        projection=project(state,s);ref=dict(orientation=[1.,0,0,0,1.,0,0,0,1.],velocity=[0.]*6);values,initial=score(state,projection,ref,None)
        self.assertAlmostEqual(values['tilt_deg'],math.degrees(p));self.assertAlmostEqual(values['pitch_deg'],math.degrees(p));self.assertAlmostEqual(values['pitch_rate_rad_s'],.2);self.assertEqual(values['relative_yaw_deg'],0.)
if __name__=='__main__':unittest.main()
