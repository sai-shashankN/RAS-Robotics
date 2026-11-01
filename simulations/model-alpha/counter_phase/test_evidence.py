import json,unittest
from context import STUDY,read
from evaluate import replay,oracle_records,composed_trace
from test_controls import Controls
from test_preflight import Preflight
class Evidence(unittest.TestCase):
    def test_all_manufactured_schedule_and_composition_traces_reproduce(self):
        cases=read(STUDY/'manufactured-counter-traces.json');self.assertEqual(len(cases),6)
        for c in cases:self.assertEqual(c['outputs'],replay(c))
        self.assertEqual(read(STUDY/'manufactured-burst-composition.json'),composed_trace())
    def test_all_independent_event_list_oracle_records_reproduce(self):
        with(STUDY/'event-list-oracle.jsonl').open() as f:saved=[json.loads(l) for l in f]
        self.assertEqual(saved,oracle_records());self.assertEqual(len(saved),2500)
    def test_counts_and_all_acquisition_hardware_feedback_grants_remain_false(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['counter_phase_source_contract_prepared']);self.assertEqual(s['independent_event_list_episodes'],100);self.assertEqual(s['independent_event_list_observations'],2500)
        for k in ['simulator_runs','new_native_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['conversion_freshness_admitted','host_device_clock_mapping_qualified','acquisition_timestamp_mapping_admitted','output_event_phase_physically_qualified','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
        cases=read(STUDY/'manufactured-counter-traces.json');self.assertIn(96,cases[0]['outputs'][0]['candidate_phases_ticks']);self.assertEqual(cases[1]['outputs'][-1]['candidate_phases_ticks'],[32])
        for c in cases:
            for r in c['outputs']:self.assertIsNone(r['acquisition_s']);self.assertFalse(r['usable_for_feedback']);self.assertFalse(r['hardware_driver_executed']);self.assertFalse(r['consumer_reset_authorized'])
if __name__=='__main__':unittest.main()
