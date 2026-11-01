import math,unittest
from context import ROOT,SOURCE,STUDY,read,sha

class EvidenceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.s=read(STUDY/'summary.json')
    def test_complete_pairings_with_no_new_trials(self):
        s=self.s;self.assertEqual(s['simulator_runs'],0);self.assertEqual(s['explicit_coarse_reuses'],8);self.assertEqual(s['explicit_fine_reuses'],8);self.assertEqual(s['distinct_reused_native_records'],16);self.assertEqual({(r['axis'],r['case']) for r in s['results']},{(a,c) for a in 'ab' for c in ['no_grasp','drop_transit','transit_stop','blocked_open']})
    def test_physical_recovery_and_root_evidence_not_used_to_admit_cube(self):
        self.assertEqual(self.s['fine_physical_recovery_pass_count'],8);self.assertEqual(self.s['root_pose_pair_pass_count'],8);self.assertEqual(self.s['full_pose_pair_pass_count'],4);self.assertFalse(self.s['fault_fine_matrix_admitted']);self.assertFalse(self.s['full_m2_admitted'])
        for r in self.s['results']:self.assertTrue(r['measured_recovery_reference_passed']);self.assertGreaterEqual(r['recovery_elapsed_s'],5-1e-8);self.assertLessEqual(r['recovery_elapsed_s'],5.008+1e-8)
    def test_original_failed_types_and_limits_preserved(self):
        for r in self.s['results']:
            self.assertEqual(r['original_full_comparison_passed'],r['case'] in {'no_grasp','blocked_open'})
            for label,v in r['comparison']['metrics'].items():self.assertEqual(r['comparison']['checks'][label+'_full_pose'],v['max_position_difference_m']<=.0005 and v['max_rotation_difference_rad']<=math.radians(.5))
    def test_exact_sealed_native_sources_and_qualified_audits(self):
        for n,h in read(STUDY/'test-plan.json')['source_indices'].items():self.assertEqual(sha(STUDY.parent/n/'checkpoint-index.json'),h)
        for r in self.s['results']:self.assertEqual(sha(ROOT/r['fine_directory']/r['fine_audit']),r['fine_audit_sha256'])
        for o in read(STUDY/'reused-records.json'):
            p=ROOT/o['directory'];self.assertTrue(all(sha(p/n)==h for n,h in o['input_sha256'].items()));self.assertTrue(all(sha(p/'sources'/n)==h for n,h in o['launch_sources_sha256'].items()))
    def test_original_stopped_queue_not_rewritten_as_complete(self):
        p=STUDY.parent/'m2_fault_fine_v1';self.assertEqual(read(p/'matrix-stop.json')['stage'],'drop_transit');self.assertFalse((p/'transit_stop_status.json').exists());self.assertFalse(read(p/'summary.json')['fault_fine_matrix_admitted'])

if __name__=='__main__':unittest.main()
