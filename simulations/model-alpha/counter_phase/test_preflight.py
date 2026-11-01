import unittest
from context import SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_frozen_sources_plan_and_prior_index(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['prerequisite_index_sha256'],sha(STUDY.parent/'fifo_burst_epoch_v1/checkpoint-index.json'));self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'))
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
    def test_exact_previous_grammar_owner_and_zero_native_declaration(self):
        for n in ['frames.py','owner.py','model.py']:self.assertEqual(sha(SOURCE/n),sha(STUDY.parent/'fifo_burst_epoch_v1/source_snapshot'/n))
        p=read(STUDY/'test-plan.json');self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['new_hardware_calls_declared'],0)
if __name__=='__main__':unittest.main()
