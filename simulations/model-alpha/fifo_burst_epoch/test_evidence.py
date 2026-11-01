import json,unittest
from context import STUDY,read
from evaluate import corpus,execute_trace
from test_controls import Controls
from test_preflight import Preflight
class Evidence(unittest.TestCase):
    def test_complete_existing_corpus_classification_reproduces(self):
        with(STUDY/'existing-corpus-classification.jsonl').open() as f:actual=[json.loads(l) for l in f]
        self.assertEqual(actual,corpus());self.assertEqual(len(actual),268)
    def test_complete_manufactured_owner_traces_reproduce(self):
        cases=read(STUDY/'manufactured-owner-traces.json');self.assertEqual(len(cases),6)
        for c in cases:self.assertEqual(c['outputs'],execute_trace(c['actions']))
        self.assertTrue(cases[0]['outputs'][-1]['result']['model_owner_bytes_match']);self.assertEqual(cases[0]['outputs'][-1]['result']['regular_frames'],0);self.assertEqual(cases[1]['outputs'][-1]['result']['current_model_epoch'],2)
        for c in cases[2:]:self.assertIsNotNone(c['outputs'][-1]['snapshot']['fault'])
    def test_counts_and_no_native_hardware_or_clock_grants(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['fifo_burst_epoch_source_contract_prepared']);self.assertEqual(s['selected_frame_fields_reconciled_with_saved_vendor'],266);self.assertEqual(s['old_bare_overread_intentionally_rejected'],2);self.assertEqual(s['grammar_compatible_corpus_cases']+s['grammar_rejected_corpus_cases'],268)
        for k in ['simulator_runs','new_native_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['conversion_freshness_admitted','hardware_configuration_qualified','hardware_driver_executed','host_device_clock_mapping_qualified','acquisition_timestamp_mapping_admitted','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
        for c in read(STUDY/'manufactured-owner-traces.json'):
            for a in c['outputs']:
                if a['result'] is not None:
                    r=a['result'];self.assertIsNone(r['acquisition_s']);self.assertFalse(r['usable_for_feedback']);self.assertFalse(r['hardware_configuration_qualified']);self.assertFalse(r['consumer_reset_authorized'])
if __name__=='__main__':unittest.main()
