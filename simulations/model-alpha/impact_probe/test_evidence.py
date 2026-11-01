import ast,math,unittest
from context import ROOT,SOURCE,STUDY,read,sha
class Checks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=read(STUDY/'summary.json');cls.c=read(STUDY/'comparison-results.json')
    def test_truthful_short_native_counts_no_mission_admission(self):self.assertEqual(self.s['simulator_runs'],16);self.assertEqual(len(self.s['results']),16);self.assertTrue(self.s['all_isolated_observations_valid']);self.assertFalse(self.s['full_m2_admitted'])
    def test_complete_factorial_matrix(self):self.assertEqual({(r['entry']['axis'],r['entry']['seed_kind'],r['entry']['model'],r['entry']['physics_ms']) for r in self.s['results']},{(a,k,m,h) for a in 'ab' for k in ['coarse','fine'] for m in ['default','normalized'] for h in [.25,.125]})
    def test_all_native_initial_mass_state_and_contact_gates(self):
        for r in self.s['results']:self.assertTrue(r['passed']);self.assertTrue(all(r['checks'].values()));self.assertLessEqual(r['maximum_depth_m'],.001);self.assertLess(r['initial_errors']['native_state'],1e-12)
    def test_raw_sources_and_input_hashes_immutable(self):
        for r in self.s['results']:
            out=STUDY/r['directory'];self.assertTrue(all(sha(out/n)==h for n,h in r['input_sha256'].items()));launch=read(out/'launch-inputs.json');self.assertTrue(all(sha(out/'sources'/n)==h for n,h in launch['sha256'].items()));self.assertEqual(launch['plan_sha256'],sha(STUDY/'test-plan.json'))
    def test_exact_coarse_anchor_all_four_states(self):self.assertEqual(len(self.c['coarse_anchor_checks']),4);self.assertTrue(self.s['coarse_anchor_exact']);self.assertTrue(all(r['telemetry_exact'] and r['native_motion_exact'] and r['native_contacts_exact'] for r in self.c['coarse_anchor_checks']))
    def test_failed_convergence_retained_original_limits(self):
        self.assertEqual(len(self.c['step_pairs']),8)
        for r in self.c['step_pairs']:self.assertEqual(r['passed'],r['max_position_difference_m']<=.0005 and r['max_rotation_difference_rad']<=math.radians(.5))
        self.assertFalse(self.s['normalized_common_state_pose_convergence']);self.assertFalse(self.s['default_common_state_pose_convergence'])
    def test_different_seed_sensitivity_not_called_convergence(self):self.assertEqual(len(self.c['initial_state_sensitivity']),8);self.assertTrue(all(r['comparison_kind']=='different_initial_states_sensitivity_not_convergence' for r in self.c['initial_state_sensitivity']))
    def test_no_runtime_state_or_force_writes(self):
        tree=ast.parse((SOURCE/'observe_controller.py').read_text());self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in {'setVelocity','setSFVec3f','setSFRotation','addForce','addTorque','resetPhysics'} for n in ast.walk(tree)));s=(SOURCE/'impact_probe.c').read_text();self.assertFalse(any(v in s for v in ['dBodySet','dBodyAdd','dJointCreate','dJointSet','dGeomSet']))
    def test_native_compiler_and_diagnostic_source_bound(self):
        b=read(SOURCE/'build/build.json');self.assertEqual(b['returncode'],0);self.assertEqual(b['source_sha256'],sha(SOURCE/'impact_probe.c'));self.assertEqual(b['dll_sha256'],sha(SOURCE/'build/impact_probe.dll'));self.assertEqual(read(STUDY/'test-plan.json')['diagnosis_index_sha256'],sha(STUDY.parent/'m2_drop_diagnosis_v1/checkpoint-index.json'))
    def test_comparison_result_and_seed_origin_bound(self):
        self.assertEqual(self.s['comparison_results_sha256'],sha(STUDY/'comparison-results.json'))
        for s in read(STUDY/'seeds.json'):self.assertEqual(s['source_sha256'],sha(ROOT/s['source_directory']/'telemetry.csv'));self.assertEqual(s['source_time_s'],157.648)
if __name__=='__main__':unittest.main()
