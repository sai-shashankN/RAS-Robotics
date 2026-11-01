import hashlib,re,unittest
from context import ROOT,SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_frozen_sources_plan_and_prior_index(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['prerequisite_index_sha256'],sha(STUDY.parent/'counter_phase_v1/checkpoint-index.json'));self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'))
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
        self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['new_hardware_calls_declared'],0)

    def test_exact_primary_source_bindings_and_conversion_convention(self):
        ledger=read(SOURCE/'vendor/sources.json')
        for r in ledger['files']:
            b=(SOURCE/'vendor'/r['name']).read_bytes();self.assertEqual(sha(SOURCE/'vendor'/r['name']),r['sha256']);self.assertEqual(hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest(),r['git_blob_sha'])
        defs=(SOURCE/'vendor/bmi2_defs.h').read_text();example=(SOURCE/'vendor/accel_gyro_hw_int.c').read_text()
        for name,value in [('ACC_RANGE_MASK',3),('GYR_RANGE_MASK',7),('GYR_OIS_RANGE_MASK',8),('ACC_RANGE_2G',0),('ACC_RANGE_16G',3),('GYR_RANGE_2000',0),('GYR_RANGE_125',4)]:
            m=re.search(r'#define BMI2_'+name+r'\s+UINT8_C\((0x[0-9A-Fa-f]+)\)',defs);self.assertIsNotNone(m);self.assertEqual(int(m[1],16),value)
        self.assertIn('(9.80665f)',example);self.assertIn('(GRAVITY_EARTH * val * g_range) / half_scale',example);self.assertIn('(dps / (half_scale)) * (val)',example)
        self.assertFalse(ledger['hardware_driver_executed'])
if __name__=='__main__':unittest.main()
