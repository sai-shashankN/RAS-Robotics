import ast,unittest,json
from context import ROOT,SOURCE,STUDY,read,sha
from test_transport import TransportChecks
class EvidenceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=read(STUDY/'summary.json')
    def test_counts_and_no_native_or_control_admission(self):self.assertEqual(self.s['simulator_runs'],0);self.assertEqual(self.s['actual_saved_image_worker_jobs'],14);self.assertEqual(self.s['explicit_static_native_record_reuses'],14);self.assertTrue(self.s['functional_worker_benchmark_admitted']);self.assertFalse(self.s['runtime_deadline_admitted']);self.assertFalse(self.s['controller_integrated']);self.assertFalse(self.s['camera_perception_admitted']);self.assertFalse(self.s['full_m2_admitted'])
    def test_actual_child_matches_all_saved_image_outputs(self):
        for r in self.s['results']:self.assertNotEqual(r['worker_pid'],r['producer_pid']);self.assertTrue(r['exact_pose']);self.assertTrue(r['exact_status']);self.assertTrue(r['rgb_sha256_matches']);self.assertFalse(r['controller_usable']);self.assertAlmostEqual(r['age_sim_s'],.088);self.assertEqual(r['source_estimates_sha256'],sha(ROOT/r['source_directory']/'image-estimates.json'))
    def test_no_estimator_or_actuator_calls_on_producer_path(self):
        tree=ast.parse((SOURCE/'broker.py').read_text());imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]+[a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names];self.assertNotIn('cv2',imports);self.assertNotIn('numpy',imports);self.assertNotIn('controller',imports);self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in {'setTorque','setPosition','setVelocity','getPosition'} for n in ast.walk(tree)))
    def test_worker_estimator_and_calibration_source_bound(self):
        prior=STUDY.parent/'prefilter_holdout_v1';self.assertEqual(read(STUDY/'test-plan.json')['source_index_sha256'],sha(prior/'checkpoint-index.json'))
        for n in ['pose_estimator.py','pose_core.py','pinhole.py']:self.assertEqual(sha(SOURCE/n),sha(prior/'source_snapshot'/n))
if __name__=='__main__':unittest.main()
