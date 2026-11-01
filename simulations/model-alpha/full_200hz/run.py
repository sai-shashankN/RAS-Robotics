import io,unittest
from concurrent.futures import ThreadPoolExecutor
from context import ROOT,STUDY,read,write
from launcher import one
from evaluate import compare
import test_preflight,test_clock_controls
def main():
    assert not(STUDY/'summary.json').exists();stream=io.StringIO();suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(m) for m in [test_preflight,test_clock_controls]);tests=unittest.TextTestRunner(stream=stream).run(suite);write(STUDY/'preflight.json',dict(tests=tests.testsRun,passed=tests.wasSuccessful(),log=stream.getvalue()));assert tests.wasSuccessful(),stream.getvalue();results=[];comparisons=[]
    def save(status,running):
        admitted=len(results)==4 and len(comparisons)==2 and all(r['passed'] for r in results+comparisons)
        write(STUDY/'summary.json',dict(simulator_runs=len(results),results=results,comparisons=comparisons,full200_nominal_observation_admitted=admitted,new_complete_nominal_missions=sum(r.get('new_complete_nominal_mission',False) for r in results),explicit_old_native_trial_reuses=0,hardware_driver_executed=False,conversion_freshness_admitted=False,raw_imu_feedback_admitted=False,realistic_sensing_admitted=False,full_m2_admitted=False))
        state=dict(status=status,study=str(STUDY.relative_to(ROOT)),latest_sealed_checkpoint=225,native_trials_completed=len(results),native_trials_declared=4,native_queue_running=running,software_verification_running=False,background_continuation_running=False,max_native_workers=2,alpha200Hzintegration_implemented=True,alpha200Hzintegration_admitted=True,owned200Hzintegration_admitted=True,full200Hzintegration_implemented=True,full200Hzintegration_admitted=admitted,hardware_driver_executed=False,conversion_freshness_admitted=False,raw_imu_feedback_admitted=False,realistic_sensing_admitted=False,full_m2_admitted=False,programme_complete=False,scope='Four fresh complete prepared20mm/2gnominal tasks with passive200Hzraw/consumer and unchanged legacy control;not hardware/feedback/fullM2fault envelope.',next='Seal actual complete200Hznominal outcome;then realistic measurement/timing/bias and gravity-confidence work before any rawfeedback proposal. Preserve unresolvedM2fault/impact,Beta/Xandhardware scope.')
        write(STUDY/'queue-state.json',state);write(ROOT/'project-notes/CONTINUATION_STATE.json',state)
    save('running',True)
    with ThreadPoolExecutor(max_workers=2) as pool:
        for h in [.25,.125]:
            pair=list(pool.map(one,[e for e in read(STUDY/'manifest.json')['entries'] if e['physics_ms']==h]));results+=pair
            if h==.125 and all(r['passed'] for r in pair):comparisons=[compare([r for r in results if r['entry']['axis']==a]) for a in 'ab']
            okay=all(r['passed'] for r in pair+comparisons);complete=len(results)==4;save('declared_queue_complete' if okay and complete else 'stopped_on_error' if not okay else 'running',okay and not complete)
            if not okay:write(STUDY/'matrix-stop.json',dict(first_failed=next(r for r in pair+comparisons if not r['passed'])));return
if __name__=='__main__':main()
