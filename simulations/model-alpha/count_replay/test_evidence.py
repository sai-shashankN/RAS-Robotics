import math,unittest
from context import STUDY,read,sha
from replay import verify_saved
from test_controls import Controls
from test_preflight import Preflight
class Evidence(unittest.TestCase):
    def test_every_saved_count_and_original_producer_clock_independent_oracle(self):
        for r in read(STUDY/'summary.json')['results']:
            p=STUDY/r['saved_counts'];self.assertEqual(sha(p),r['saved_counts_sha256']);actual=verify_saved(r['entry'],p);self.assertEqual(actual,r['independent_oracle'])
            for k,v in actual.items():self.assertEqual(r[k],v)
    def test_complete_scalar_evidence_and_finite_error_diagnostics(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['counts_quantization_replay_complete']);self.assertEqual(s['producer_rows'],271498);self.assertEqual(s['scalar_quantizations'],7330446)
        for r in s['results']:
            self.assertTrue(r['passed']);self.assertEqual(len(r['scale_results']),9)
            for st in r['scale_results']:
                self.assertEqual(st['bound_violations'],0)
                for k in ['max_absolute_input_SI','max_absolute_error_SI','rms_error_SI']:self.assertTrue(all(math.isfinite(v) and v>=0 for v in st[k]))
                # The half-LSB bound applies only where the ideal input fits.
                for axis in range(3):
                    if st['inputs_outside_representable_interval'][axis]==0:self.assertLessEqual(st['max_absolute_error_SI'][axis],.50000001*st['scale'])
    def test_replay_does_not_count_as_native_calibration_or_feedback(self):
        s=read(STUDY/'summary.json');self.assertEqual(s['explicit_existing_native_histories'],4)
        for k in ['simulator_runs','new_native_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['hardware_configuration_qualified','calibrated_SI_accuracy_qualified','conversion_freshness_admitted','acquisition_timestamp_mapping_admitted','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
if __name__=='__main__':unittest.main()
