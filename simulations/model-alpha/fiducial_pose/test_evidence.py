import unittest,math
from pathlib import Path
import cv2,numpy as np
from context import ROOT,SOURCE,STUDY,read,sha
from core import detect,square_points,hypotheses,compose_body_object,FAMILY

class PoseChecks(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[457.,0,320],[0,457.,240],[0,0,1.]]);self.rv=np.array([2.8,.1,.05]);self.t=np.array([.001,-.002,.12]);self.R=cv2.Rodrigues(self.rv)[0];self.corners=cv2.projectPoints(square_points(.016),self.rv,self.t,self.K,np.zeros(5))[0].reshape(4,2)
    def test_exact_known_pose_and_positive_depth_hypotheses(self):
        h=hypotheses(self.corners,self.K,.016);self.assertEqual(len(h),2);np.testing.assert_allclose(h[0]['translation_camera_tag_m'],self.t,atol=1e-12);np.testing.assert_allclose(h[0]['rotation_camera_tag'],self.R,atol=1e-10);self.assertLess(h[0]['reprojection_rms_px'],1e-9)
        for p in h:self.assertGreater(min((square_points(.016)@np.array(p['rotation_camera_tag']).T+np.array(p['translation_camera_tag_m']))[:,2]),0)
    def test_reversed_or_missing_corners_rejected(self):
        for c in [self.corners[::-1],self.corners[:3]]:
            with self.assertRaises(ValueError):hypotheses(c,self.K,.016)
    def test_nonfinite_and_invalid_calibration_rejected(self):
        c=self.corners.copy();c[0,0]=float('nan')
        with self.assertRaises(ValueError):hypotheses(c,self.K,.016)
        K=self.K.copy();K[0,0]=-1
        with self.assertRaises(ValueError):hypotheses(self.corners,K,.016)
        K=self.K.copy();K[2,2]=0
        with self.assertRaises(ValueError):hypotheses(self.corners,K,.016)
    def test_cyclic_corner_relabeling_can_hide_wrong_orientation(self):
        h=hypotheses(np.roll(self.corners,1,axis=0),self.K,.016)[0];self.assertLess(h['reprojection_rms_px'],1e-8);R=np.array(h['rotation_camera_tag']);angle=math.acos(np.clip((np.trace(R.T@self.R)-1)/2,-1,1));self.assertGreater(angle,1.)
    def test_small_corner_noise_returns_finite_hypotheses_not_confidence(self):
        h=hypotheses(self.corners+np.array([[.2,-.2],[-.2,.2],[.2,.2],[-.2,-.2]]),self.K,.016);self.assertTrue(h)
        for v in h:self.assertTrue(np.isfinite(v['reprojection_rms_px']));self.assertNotIn('confidence',v);self.assertNotIn('sigma',v)
    def test_tag_size_is_explicit_and_changes_translation_scale(self):
        h=hypotheses(self.corners,self.K,.032)[0];np.testing.assert_allclose(h['translation_camera_tag_m'],self.t*2,atol=1e-12)
        with self.assertRaises(ValueError):square_points(0)
    def test_full_frame_chain_includes_object_center_offset(self):
        BC=cv2.Rodrigues(np.array([.1,.2,-.3]))[0];bp=np.array([.047,-.085,.008]);TO=cv2.Rodrigues(np.array([0,.5,0]))[0];tp=np.array([0,0,-.01]);h=hypotheses(self.corners,self.K,.016)[0];R,p=compose_body_object(BC,bp,h,TO,tp);np.testing.assert_allclose(R,BC@self.R@TO,atol=1e-10);np.testing.assert_allclose(p,BC@(self.R@tp+self.t)+bp,atol=1e-12);self.assertGreater(np.linalg.norm(np.array(p)-(BC@self.t+bp)),.009)
    def test_generated_tag_rotates_but_retains_decoded_id(self):
        image=cv2.cvtColor(cv2.imread(str(STUDY/'manufactured-tag.png')),cv2.COLOR_BGR2RGB)
        for k in [0,1,2,3]:
            a=np.ascontiguousarray(np.rot90(image,k));d=detect(a.tobytes(),384,384);self.assertEqual([v['id'] for v in d['detections']],[7])
    def test_missing_tag_is_explicit(self):self.assertEqual(detect(bytes([255])*384*384*3,384,384)['detections'],[])

class EvidenceChecks(unittest.TestCase):
    def test_counts_and_scope_only_manufactured_controls(self):
        s=read(STUDY/'summary.json');self.assertEqual(s['simulator_runs'],0);self.assertEqual(len(s['results']),3);self.assertTrue(all(v['kind'].startswith('manufactured_') for v in s['results']));self.assertFalse(s['camera_perception_admitted']);self.assertFalse(s['sensed_controller_admitted']);self.assertFalse(s['full_m2_admitted'])
    def test_installed_dependency_and_prior_results_bound(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['dependency']['opencv_version'],cv2.__version__);self.assertEqual(cv2.__version__,'4.11.0');self.assertTrue(all(sha(Path(n))==h for n,h in p['dependency']['binary_sha256'].items()));self.assertEqual(p['prior_summary_sha256'],sha(STUDY.parent/'rgb_baseline_v1/summary.json'))
    def test_no_detector_oracle_image_inputs(self):
        import inspect
        self.assertEqual(list(inspect.signature(detect).parameters),['rgb','width','height']);self.assertEqual(list(inspect.signature(hypotheses).parameters),['corners_px','K','size_m','distortion'])

if __name__=='__main__':unittest.main()
