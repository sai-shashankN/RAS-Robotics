import unittest
from context import ROOT,SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_sources_plan_and_all_prior_inputs_are_bound(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'))
        for n,v in p['source_indices'].items():self.assertEqual(sha(STUDY.parent/n/'checkpoint-index.json'),v)
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
        for values in p['input_sha256'].values():
            for n,v in values.items():self.assertEqual(sha(ROOT/n),v)
    def test_exact_original_observer_source_limits_and_zero_native_declaration(self):
        for n in ['history200.py','consumer200.py','timed_shadow.py','attitude_shadow.py','measurement.py','projection200.py']:self.assertEqual(sha(SOURCE/n),sha(STUDY.parent/'full_200hz_v1/source_snapshot'/n))
        self.assertEqual(sha(SOURCE/'scale.py'),sha(STUDY.parent/'count_si_v1/source_snapshot/scale.py'))
        p=read(STUDY/'test-plan.json');self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['new_hardware_calls_declared'],0)
        old=read(STUDY.parent/'full_200hz_v1/test-plan.json')['limits'];self.assertEqual(p['original_observer_limits'],{k:old['observer200_'+k] for k in p['original_observer_limits']})
if __name__=='__main__':unittest.main()
