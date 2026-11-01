import json,tempfile,unittest
from pathlib import Path
from full200 import FullObservedAlpha200
class Device:
    def __init__(self,values):self.period=0;self.values=values
    def getSamplingPeriod(self):return self.period
    def enable(self,p):self.period=p
    def getValues(self):return self.values
class Robot:
    def __init__(self):self.t=0.;self.devices={'raw200_accel':Device([0.,0.,9.81]),'raw200_gyro':Device([0.,.02,0.])}
    def getTime(self):return self.t
    def getDevice(self,n):return self.devices[n]
class Body:
    def __init__(self):self.x=0.
    def getPosition(self):return [self.x,0.,0.]
    def getOrientation(self):return [1.,0.,0.,0.,1.,0.,0.,0.,1.]
    def getVelocity(self):return [0.,0.,0.,0.,.02,0.]
def rows(p):return [json.loads(l) for l in p.read_text().splitlines()]
class ClockControls(unittest.TestCase):
    def test_nonaligned_endpoints_have_actual_witness_and_old_state_clock(self):
        for end in [48,64]:
            with self.subTest(end=end),tempfile.TemporaryDirectory() as tmp:
                p=Path(tmp);r=Robot();b=Body();c=FullObservedAlpha200(r,b,p)
                for ms in range(1,end+1):r.t=ms*.001;c.tick(r.t)
                b.x=1.25;c.close(r.t);raw=rows(p/'raw200-producer.jsonl');refs=rows(p/'raw200-reference.jsonl');host=rows(p/'raw200-host.jsonl');observed=rows(p/'raw200-host-observer.jsonl');endpoint=json.loads((p/'raw200-endpoint.json').read_text());last=observed[-1]['outputs'][-1]
                self.assertEqual(endpoint['time_s'],end*.001);self.assertEqual(endpoint['position'],[1.25,0.,0.]);self.assertEqual(refs[-1]['position'],[0.,0.,0.]);self.assertEqual(round(raw[-1]['producer_read_s']*1000),(end//5)*5);self.assertEqual(last['state_read_s'],raw[-1]['producer_read_s']);self.assertAlmostEqual(last['state_api_age_s'],end%5*.001);self.assertEqual(len(raw),sum(len(v['batch']) for v in host));self.assertEqual(json.loads((p/'raw200-completion.json').read_text())['pending'],0)
    def test_first5mswaiting_and15msstate_delivered16ms_without_retiming(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);r=Robot();c=FullObservedAlpha200(r,Body(),p)
            for ms in range(1,41):r.t=ms*.001;c.tick(r.t)
            c.close(r.t);out=rows(p/'raw200-host-observer.jsonl');self.assertIsNone(out[0]['outputs'][0]['state_read_s']);self.assertEqual([o['state_read_s'] for o in out[1]['outputs']],[.010,.015]);self.assertTrue(all(o['host_read_s']==.016 for o in out[1]['outputs']))
if __name__=='__main__':unittest.main()
