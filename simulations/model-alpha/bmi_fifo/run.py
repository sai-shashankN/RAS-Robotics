import io,unittest,json
from context import STUDY,write
import test_controls,test_preflight
def main():
    assert not(STUDY/'summary.json').exists();log=io.StringIO();test_controls.CASES.clear();suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(m) for m in [test_controls,test_preflight]);r=unittest.TextTestRunner(stream=log).run(suite)
    with(STUDY/'manufactured-differential-cases.jsonl').open('w',encoding='utf-8') as f:
        for case in test_controls.CASES:f.write(json.dumps(case,allow_nan=False)+'\n')
    io_calls=sum(c['vendor']['forbidden_io_calls'] for c in test_controls.CASES);write(STUDY/'summary.json',dict(simulator_runs=0,results=[dict(id='manufactured_memory_parser_contract',passed=r.wasSuccessful())],fifo_header_source_contract_prepared=r.wasSuccessful() and io_calls==0,manufactured_vendor_memory_cases=len(test_controls.CASES),vendor_io_calls=io_calls,new_robot_trials=0,new_hardware_driver_executions=0,conversion_freshness_admitted=False,hardware_driver_executed=False,acquisition_timestamp_mapping_admitted=False,raw_imu_feedback_admitted=False,full_m2_admitted=False));write(STUDY/'preflight.json',dict(tests=r.testsRun,passed=r.wasSuccessful(),log=log.getvalue()));print(log.getvalue());assert r.wasSuccessful()
if __name__=='__main__':main()
