import json,unittest
from context import STUDY,read
from harness import VendorParser
from decoder import HeaderBurstDecoder
from test_controls import Controls
from test_preflight import Preflight
class Evidence(unittest.TestCase):
    def test_saved_manufactured_cases_reproduce_in_both_parsers(self):
        count=0;vendor=VendorParser()
        with(STUDY/'manufactured-differential-cases.jsonl').open() as f:
            for line in f:
                c=json.loads(line);data=bytes.fromhex(c['hex']);self.assertEqual(HeaderBurstDecoder().decode(data),c['candidate']);self.assertEqual(vendor.decode(data),c['vendor']);self.assertEqual(c['vendor']['forbidden_io_calls'],0);count+=1
        self.assertEqual(count,read(STUDY/'summary.json')['manufactured_vendor_memory_cases'])
    def test_actual_software_only_counts_and_no_grants(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['fifo_header_source_contract_prepared']);self.assertEqual(s['manufactured_vendor_memory_cases'],268)
        for k in ['simulator_runs','vendor_io_calls','new_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['conversion_freshness_admitted','hardware_driver_executed','acquisition_timestamp_mapping_admitted','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
if __name__=='__main__':unittest.main()
