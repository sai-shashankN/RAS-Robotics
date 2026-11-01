import unittest,math
from context import ROOT,SOURCE,STUDY,read,sha
class Checks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=read(STUDY/'summary.json')
    def test_native_counts_partial_scope_not_controller(self):self.assertEqual(self.s['simulator_runs'],14);self.assertEqual(self.s['dynamic_robot_trials'],0);self.assertTrue(self.s['static_benchmark_complete']);self.assertFalse(self.s['selected_pose_admitted']);self.assertFalse(self.s['camera_perception_admitted']);self.assertFalse(self.s['full_m2_admitted'])
    def test_all_twelve_cases_all_variants_meet_fixed_diagnostics(self):
        self.assertEqual(len(self.s['metrics']),4)
        for m in self.s['metrics']:self.assertEqual(m['task_cases'],12);self.assertEqual(m['returned'],12);self.assertEqual(m['returned_within_limits'],12);self.assertEqual(m['missing_or_rejected'],0);self.assertLess(m['max_center_error_m'],.00025);self.assertLess(m['max_rotation_error_rad'],math.radians(1))
    def test_frozen_algorithm_and_raw_launch_provenance(self):
        f=read(STUDY/'frozen-estimator.json');self.assertTrue(all(sha(SOURCE/n)==h for n,h in f['sha256'].items()));self.assertEqual(f['camera_height_m'],.055)
        for r in self.s['results']:
            out=STUDY/r['directory'];self.assertTrue(all(sha(out/n)==h for n,h in r['input_sha256'].items()));self.assertEqual(read(out/'audit.json'),r);l=read(out/'launch-inputs.json');self.assertEqual(l['plan_sha256'],sha(STUDY/'test-plan.json'));self.assertTrue(all(sha(out/'sources'/n)==h for n,h in l['sha256'].items()))
    def test_pose_selection_does_not_use_oracle_and_has_no_uncertainty(self):
        for r in self.s['results']:
            for e in read(STUDY/r['directory']/'image-estimates.json').values():
                self.assertFalse(e['controller_usable']);self.assertIsNone(e['calibrated_uncertainty'])
                if e['chosen']:self.assertEqual(e['chosen'],min(e['solutions'],key=lambda v:v['robust_cost']))
    def test_positive_and_occluded_controls(self):
        a=next(r for r in self.s['results'] if r['entry']['role']=='positive');b=next(r for r in self.s['results'] if r['entry']['role']=='negative');self.assertTrue(a['checks']['positive_clean_pose']);self.assertTrue(b['checks']['negative_no_pose'])
    def test_no_renamed_old_native_pose_images(self):
        previous=set()
        for name in ['tagged_surface_v1','joint_pose_holdout_v1']:
            p=STUDY.parent/name
            for r in read(p/'summary.json')['results']:
                scene=read(p/r['directory']/'sources/scene.json')
                if 'source_time_s' in scene:previous.add((scene['axis'],scene['source_time_s']))
        for r in self.s['results']:
            scene=read(STUDY/r['directory']/'sources/scene.json')
            if r['entry']['role']=='task':self.assertNotIn((scene['axis'],scene['source_time_s']),previous)
    def test_profiling_is_reused_software_invocations_not_native_or_deadline(self):
        p=read(STUDY/'timing-profile.json');self.assertEqual(p['simulator_runs'],0);self.assertEqual(p['estimator_invocations'],60);self.assertTrue(p['all_outputs_exact']);self.assertFalse(p['runtime_deadline_admitted']);self.assertTrue(all(s['wall_ms']>0 and s['cpu_ms']>0 for s in p['samples']))
if __name__=='__main__':unittest.main()
