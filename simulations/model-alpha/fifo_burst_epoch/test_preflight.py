import unittest
from context import SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_frozen_sources_plan_and_prior_index(self):
        p=read(STUDY/'test-plan.json');prior=STUDY.parent/'bmi_fifo_contract_v1';self.assertEqual(p['prerequisite_index_sha256'],sha(prior/'checkpoint-index.json'));self.assertEqual(p['prior_manufactured_corpus_sha256'],sha(prior/'manufactured-differential-cases.jsonl'));self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'))
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
    def test_software_only_declaration_and_primary_basis(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['new_hardware_calls_declared'],0);self.assertEqual(p['primary_reference']['revision'],'1.6');self.assertEqual(p['primary_reference']['printed_pages'],[35,36,37])
if __name__=='__main__':unittest.main()
