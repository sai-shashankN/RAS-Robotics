import contextlib,csv,gzip,math,unittest
from itertools import zip_longest
from context import ROOT,STUDY,read,sha
from replay import METRICS,stream,flattened,wrap
from test_controls import Controls
from test_preflight import Preflight

def witness_errors(row,ref,gyro,initial):
    # Recompute from saved quaternion and original independent physical witness.
    q=[float(row[k]) for k in ['qw','qx','qy','qz']];norm=math.sqrt(sum(v*v for v in q));w,x,y,z=[v/norm for v in q]
    R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];rr=ref['orientation']
    a=R[6:9];b=rr[6:9];tilt=math.degrees(math.acos(max(-1.,min(1.,sum(u*v for u,v in zip(a,b))))));yaw=math.atan2(R[3],R[0]);true_yaw=math.atan2(rr[3],rr[0]);roll=math.atan2(R[7],R[8]);pitch=math.atan2(-R[6],math.hypot(R[7],R[8]));tr=math.atan2(rr[7],rr[8]);tp=math.atan2(-rr[6],math.hypot(rr[7],rr[8]))
    if initial is None:initial=(yaw,true_yaw)
    actual_rates=[sum(rr[3*j+i]*ref['velocity'][3+j] for j in range(3)) for i in range(3)];rate=math.cos(roll)*gyro[1]-math.sin(roll)*gyro[2];true_rate=math.cos(tr)*actual_rates[1]-math.sin(tr)*actual_rates[2]
    if abs(roll-float(row['roll_rad']))>1e-12 or abs(pitch-float(row['pitch_rad']))>1e-12 or abs(rate-float(row['projected_pitch_rate_rad_s']))>1e-12:raise ValueError('saved quaternion/projection mismatch')
    return dict(tilt_deg=tilt,relative_yaw_deg=math.degrees(abs(wrap(wrap(yaw-initial[0])-wrap(true_yaw-initial[1])))),roll_deg=math.degrees(abs(wrap(roll-tr))),pitch_deg=math.degrees(abs(wrap(pitch-tp))),pitch_rate_rad_s=abs(rate-true_rate)),initial

class Evidence(unittest.TestCase):
    def test_all_saved_quantized_states_rejoin_witness_codes_and_actual_host_clocks(self):
        for result in read(STUDY/'summary.json')['results']:
            e=result['entry'];native=ROOT/e['native_directory'];profiles=[p for p in result['profiles'] if p['profile']!='ideal_reference']
            with contextlib.ExitStack() as stack:
                readers=[];maxima={p['profile']:{k:0. for k in METRICS} for p in profiles};initial={p['profile']:None for p in profiles};n=0
                for p in profiles:
                    path=STUDY/p['saved_trace'];self.assertEqual(sha(path),p['saved_trace_sha256']);readers.append(csv.DictReader(stack.enter_context(gzip.open(path,'rt',newline='',encoding='utf-8'))))
                refs=stream(native/'raw200-reference.jsonl');next(refs);next(refs);codes=csv.DictReader(stack.enter_context(gzip.open(ROOT/e['counts_path'],'rt',newline='',encoding='utf-8')));next(codes);actual=flattened(native/'raw200-host-observer.jsonl');next(actual)
                for values in zip_longest(refs,codes,actual,*readers):
                    self.assertNotIn(None,values);ref,code,host,*rows=values;n+=1
                    for p,row in zip(profiles,rows):
                        t=float(row['original_producer_read_s']);self.assertEqual(t,ref['time_s']);self.assertEqual(t,float(row['state_read_s']));self.assertEqual(t,float(code['original_simulation_producer_read_s']));self.assertEqual(t,host['producer_read_s']);self.assertEqual(float(row['original_host_read_s']),host['host_read_s'])
                        d=p['range_profile'];gyro=[int(code[f"gyro_{(2000,1000,500,250,125).index(d['range_dps'])}_{a}_count"])*d['gyro_lsb_rad_s'] for a in 'xyz'];errors,initial[p['profile']]=witness_errors(row,ref,gyro,initial[p['profile']])
                        for k,v in errors.items():self.assertAlmostEqual(float(row[k]),v,delta=2e-6 if k=='tilt_deg' else 1e-12);maxima[p['profile']][k]=max(maxima[p['profile']][k],float(row[k]))
                self.assertEqual(n,e['expected_rows']-1)
                for p in profiles:self.assertEqual(maxima[p['profile']],p['max_errors'])

    def test_all_ideal_states_and_original_error_maxima_are_preserved(self):
        for r in read(STUDY/'summary.json')['results']:
            self.assertTrue(r['ideal_observer_every_state_exact_to_actual226']);ideal=next(p for p in r['profiles'] if p['profile']=='ideal_reference');old=read(ROOT/r['entry']['native_directory']/'full200-audit.json')['observer200'];self.assertEqual(ideal['max_errors'],old['max_errors']);self.assertEqual(ideal['accepted_states'],old['accepted_states'])

    def test_evaluation_completeness_and_old_limits_reported_without_hardware_grants(self):
        s=read(STUDY/'summary.json');self.assertTrue(s['observer_quantization_evaluation_complete']);self.assertEqual(s['explicit_existing_native_histories'],4);self.assertEqual(s['quantized_profile_replays'],12);limits=read(STUDY/'test-plan.json')['original_observer_limits']
        for r in s['results']:
            for p in r['profiles']:
                self.assertTrue(p['source_integrity_passed']);self.assertEqual(p['original_error_gates'],{k:v<=limits[k] for k,v in p['max_errors'].items()})
        self.assertEqual(s['all_quantization_profiles_meet_old_numerical_limits'],all(p['original_error_gates_passed'] for r in s['results'] for p in r['profiles']))
        for k in ['simulator_runs','new_native_robot_trials','new_hardware_driver_executions']:self.assertEqual(s[k],0)
        for k in ['hardware_configuration_qualified','calibrated_SI_accuracy_qualified','conversion_freshness_admitted','acquisition_timestamp_mapping_admitted','raw_imu_feedback_admitted','full_m2_admitted']:self.assertFalse(s[k])
if __name__=='__main__':unittest.main()
