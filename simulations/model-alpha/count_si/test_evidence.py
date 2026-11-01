import unittest
from context import STUDY,read
from evaluate import exhaustive_records,random_bounds,conversion_examples
from test_controls import Controls
from test_preflight import Preflight
class Evidence(unittest.TestCase):
    def test_complete_exhaustive_code_domain_reproduces(self):
        saved=read(STUDY/'exhaustive-counts.json');self.assertEqual(saved,exhaustive_records());self.assertEqual(sum(r['codes_checked'] for r in saved),589824)
        self.assertTrue(all(r['roundtrip_failures']==0 and r['rail_codes']==2 for r in saved))
    def test_seeded_bounds_and_manufactured_pair_examples_reproduce(self):
        self.assertEqual(read(STUDY/'random-quantization-bounds.json'),random_bounds());self.assertEqual(read(STUDY/'manufactured-conversions.json'),conversion_examples())
    def test_scope_counts_and_no_sensor_feedback_grants(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['counts_to_SI_source_contract_prepared']);self.assertEqual(s['exhaustive_signed_code_cases'],589824);self.assertEqual(s['random_bound_cases'],9000)
        for k in ['simulator_runs','new_native_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['calibrated_SI_accuracy_qualified','hardware_configuration_qualified','conversion_freshness_admitted','host_device_clock_mapping_qualified','acquisition_timestamp_mapping_admitted','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
if __name__=='__main__':unittest.main()
