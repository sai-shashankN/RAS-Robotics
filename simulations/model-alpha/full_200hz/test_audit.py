import copy,math,unittest
from context import ROOT,STUDY,read
from mission_audit import reference,support_features
from solid_math import measure
class AuditChecks(unittest.TestCase):
    def sample(self):
        e=read(STUDY/'manifest.json')['entries'][0];p=ROOT/e['project'];profile=read(p/'collision-fixtures.json');setup=read(p/'setup.json');f=profile['fixtures'][4];top=f['center_m'][2]+f['half_m'][2];v={}
        for i,r in enumerate([1,0,0,0,1,0,0,0,1]):v['object_r'+str(i)]=r
        for k,x in zip('xyz',[*f['center_m'][:2],top+setup['object_size_m'][2]/2]):v['object_'+k+'_m']=x;v['object_v'+k+'_m_s']=0.
        face={f'fixture_{i}_{kind}_points':0 for i in range(8) for kind in ['top','other']};face['fixture_4_top_points']=4;raw=dict(object_points_m=[[*f['center_m'][:2],top]])
        return v,raw,face,setup,profile
    def test_destination_top_stable_positive(self):self.assertTrue(support_features(*self.sample())['stable'])
    def test_source_contact_does_not_count_as_destination(self):
        v,r,f,s,p=self.sample();f['fixture_0_top_points']=4;f['fixture_4_top_points']=0;self.assertFalse(support_features(v,r,f,s,p)['stable'])
    def test_correct_and_wrong_contact_together_rejected(self):
        for key in ['fixture_0_top_points','fixture_4_other_points','fixture_5_other_points','fixture_7_other_points']:
            v,r,f,s,p=self.sample();f[key]=1;self.assertFalse(support_features(v,r,f,s,p)['stable'])
    def test_native_truth_cannot_replace_raw_contact(self):
        v,r,f,s,p=self.sample();r['object_points_m']=[];self.assertFalse(support_features(v,r,f,s,p)['stable'])
    def test_contact_near_top_cannot_override_pose_and_speed(self):
        for field,value in [('object_z_m',.1),('object_x_m',.1),('object_vz_m_s',.01)]:
            v,r,f,s,p=self.sample();v[field]+=value;self.assertFalse(support_features(v,r,f,s,p)['stable'])
    def test_signed_path_world_separate_after_turn(self):
        phase,path,world,yaw=reference(272,.282);self.assertEqual(phase,6);self.assertAlmostEqual(path,.182);self.assertAlmostEqual(world,-.382);self.assertAlmostEqual(yaw,math.pi);self.assertGreater(abs(path-world),.5)
    def test_full_independent_schedule_and_endpoints(self):
        self.assertEqual([reference(t,.282)[0] for t in [0,50,54,114,118,268,272]],[1,2,3,4,5,6,6]);self.assertAlmostEqual(reference(84,.282)[3],math.pi/2);self.assertAlmostEqual(reference(193,.282)[1],-.1+.282/2)
    def test_contained_cube_detected_without_contact_points(self):
        v,r,f,s,p=self.sample();c=p['fixtures'][4]['center_m'];state=measure(c,[1,0,0,0,1,0,0,0,1],s['object_size_m'],p);self.assertTrue(state['violated'])
if __name__=='__main__':unittest.main()
