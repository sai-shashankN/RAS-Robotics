import math, unittest
from fractions import Fraction
from context import ROOT,SOURCE,STUDY,read,sha

class EvidenceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.s=read(STUDY/'summary.json');cls.c=read(STUDY/'refinement-results.json');cls.prior=STUDY.parent/'impact_probe_v1'
    def test_eight_new_native_observations_not_missions(self):
        self.assertEqual(self.s['simulator_runs'],8);self.assertEqual(len(self.s['results']),8);self.assertTrue(self.s['all_isolated_observations_valid']);self.assertFalse(self.s['full_m2_admitted'])
    def test_complete_four_state_two_step_matrix(self):
        self.assertEqual({(r['entry']['axis'],r['entry']['seed_kind'],r['entry']['physics_ms']) for r in self.s['results']},{(a,s,h) for a in 'ab' for s in ['coarse','fine'] for h in [.0625,.03125]})
    def test_actual_initial_state_and_native_contact_integrity(self):
        for r in self.s['results']:
            self.assertTrue(r['passed']);self.assertTrue(all(r['checks'].values()));self.assertLessEqual(r['maximum_depth_m'],.001);self.assertLess(r['initial_errors']['native_state'],1e-12)
    def test_raw_and_launch_input_bindings(self):
        for r in self.s['results']:
            out=STUDY/r['directory'];self.assertTrue(all(sha(out/n)==h for n,h in r['input_sha256'].items()));launch=read(out/'launch-inputs.json');self.assertTrue(all(sha(out/'sources'/n)==h for n,h in launch['sha256'].items()));self.assertEqual(launch['plan_sha256'],sha(STUDY/'test-plan.json'))
    def test_same_four_physical_initial_states_and_materials(self):
        for r in self.s['results']:
            out=STUDY/r['directory'];old=ROOT/r['entry']['runtime_source_project'];self.assertEqual(sha(out/'sources/seed.json'),sha(old/'seed.json'));cfg=read(out/'sources/normalization.json');h=Fraction(str(r['entry']['physics_ms']))/1000
            self.assertEqual(cfg['contact_erp'],float(h*800000/(h*800000+800)));self.assertEqual(cfg['contact_cfm'],float(1/(h*800000+800)))
            self.assertEqual(sha(out/'sources/plugins/physics/impact_probe/impact_probe.dll'),sha(old/'plugins/physics/impact_probe/impact_probe.dll'))
    def test_all_consecutive_pairs_original_absolute_time_limits(self):
        self.assertEqual(len(self.c['step_pairs']),12)
        for r in self.c['step_pairs']:
            self.assertEqual(r['passed'],r['max_position_difference_m']<=.0005 and r['max_rotation_difference_rad']<=math.radians(.5))
        self.assertFalse(self.s['finest_pair_pose_convergence'])
    def test_different_initial_state_observations_remain_separate(self):
        self.assertEqual(len(self.c['initial_state_sensitivity']),8);self.assertTrue(all(r['comparison_kind']=='different_initial_states_sensitivity_not_convergence' for r in self.c['initial_state_sensitivity']))
    def test_comparison_and_prior_index_bindings(self):
        self.assertEqual(self.s['refinement_results_sha256'],sha(STUDY/'refinement-results.json'));self.assertEqual(read(STUDY/'test-plan.json')['prior_index_sha256'],sha(self.prior/'checkpoint-index.json'))
    def test_observers_unchanged_and_read_only(self):
        for n in ['impact_probe.c','observe_controller.py']:
            self.assertEqual(sha(SOURCE/n),sha(self.prior/'source_snapshot'/n))

if __name__=='__main__':unittest.main()
