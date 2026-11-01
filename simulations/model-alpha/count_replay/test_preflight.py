import unittest
from context import ROOT,SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_frozen_sources_and_prior_indices(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['prerequisite_index_sha256'],sha(STUDY.parent/'count_si_v1/checkpoint-index.json'));self.assertEqual(p['native_source_index_sha256'],sha(STUDY.parent/'full_200hz_v1/checkpoint-index.json'));self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'))
        self.assertEqual(sha(SOURCE/'scale.py'),sha(STUDY.parent/'count_si_v1/source_snapshot/scale.py'))
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
    def test_four_explicit_complete_histories_and_zero_new_trials(self):
        entries=read(STUDY/'manifest.json')['entries'];p=read(STUDY/'test-plan.json');self.assertEqual(len(entries),4);self.assertEqual(sum(e['expected_rows'] for e in entries),271498)
        for e in entries:self.assertEqual(sha(ROOT/e['input']),p['input_sha256'][e['id']]);self.assertTrue(e['prior_physical_case_passed']);self.assertEqual(e['source_checkpoint'],226)
        self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['new_hardware_calls_declared'],0)
if __name__=='__main__':unittest.main()
