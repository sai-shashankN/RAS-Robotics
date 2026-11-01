import copy,random,struct,unittest
from phase import CounterPhaseModel
from adapter import ManufacturedBurstCounter
from model import BurstEpochModel
def data(n=1):return (b'\x8c'+struct.pack('<hhhhhh',0,0,0,0,0,16384))*n
def stamp(n):return b'\x44'+n.to_bytes(3,'little')
def ready_burst():
    b=BurstEpochModel();b.declare(1,{0x40:0xa9},0,0.,'manufactured');b.update(b'\x48\x00\x00\x00\x00'+data()+stamp(0),.001,.002,1);b.acknowledge({0x40:0xa9},.003,.004,.004,'manufactured');assert b.fault is None;return b
class Controls(unittest.TestCase):
    def test_original_unknown_phase_example_keeps_both_hypotheses(self):
        m=CounterPhaseModel();r=m.observe(1030,8,1,1);self.assertEqual(r['candidate_phases_ticks'],list(range(7,129)));self.assertIn(32,r['candidate_phases_ticks']);self.assertIn(96,r['candidate_phases_ticks']);self.assertEqual(r['conditional_last_output_elapsed_tick_bounds'],[903,1024]);self.assertEqual(r['conditional_last_output_age_tick_bounds'],[6,127])
    def test_additional_exact_constraints_can_narrow_without_acquisition_grant(self):
        m=CounterPhaseModel();m.observe(1030,8,1,1);a=m.observe(1055,8,2,1);b=m.observe(1056,9,3,1);self.assertEqual(a['candidate_phases_ticks'],list(range(32,129)));self.assertEqual(b['candidate_phases_ticks'],[32]);self.assertEqual(b['conditional_last_output_elapsed_tick_bounds'],[1056,1056]);self.assertIsNone(b['acquisition_s']);self.assertFalse(b['output_event_phase_physically_qualified']);self.assertFalse(b['usable_for_feedback'])
    def test_explicit_event_list_oracle_across_seeded_wrap_episodes(self):
        rng=random.Random(229128)
        for _ in range(100):
            p=rng.randrange(1,32);modulus=128;origin=rng.randrange(modulus);m=CounterPhaseModel(p,modulus,31,origin);true=rng.randrange(1,p+1);elapsed=0;constraints=[]
            for seq in range(1,26):
                elapsed+=rng.randrange(0,32);n=len(list(range(true,elapsed+1,p)));constraints.append((elapsed,n));result=m.observe((origin+elapsed)%modulus,n,seq,1);oracle=[phase for phase in range(1,p+1) if all(len(list(range(phase,t+1,p)))==count for t,count in constraints)];self.assertEqual(result['candidate_phases_ticks'],oracle);self.assertIn(true,oracle);self.assertIsNone(result['fault'])
    def test_wrap_and_declared_delta_interval_do_not_become_hardware_clock(self):
        m=CounterPhaseModel(8,256,64,240);r=m.observe(4,3,1,1,20,20);self.assertEqual(r['conditional_read_elapsed_ticks'],20);self.assertIn(3,r['candidate_phases_ticks']);self.assertFalse(r['host_device_clock_mapping_qualified'])
        bad=CounterPhaseModel(8,256,64,240).observe(4,3,1,1,21,30);self.assertIsNotNone(bad['fault'])
    def test_repeated_read_counter_and_unchanged_count_is_not_fresh_sample(self):
        m=CounterPhaseModel();m.observe(0,0,1,1);r=m.observe(0,0,2,1);self.assertIsNone(r['fault']);self.assertIsNone(r['conditional_last_output_elapsed_tick_bounds']);self.assertFalse(r['sample_sequence_proven']);self.assertFalse(r['conversion_freshness_proven'])
    def test_inconsistent_counts_epoch_or_sequence_latch_without_commit(self):
        for args in [(1030,100,2,1),(1031,7,2,1),(1031,8,3,1),(1031,8,2,2),(True,8,2,1)]:
            m=CounterPhaseModel();m.observe(1030,8,1,1);r=m.observe(*args);self.assertIsNotNone(r['fault']);self.assertEqual(r['last_accepted_elapsed_ticks'],1030);self.assertEqual(r['last_accepted_sequence'],1);self.assertIsNone(r['conditional_read_elapsed_ticks']);self.assertEqual(r['candidate_phases_ticks'],[]);self.assertEqual(m.observe(1055,8,2,1)['fault'],r['fault'])
    def test_bad_or_ambiguous_model_parameters_are_rejected(self):
        for kwargs in [dict(period_ticks=True),dict(period_ticks=0),dict(period_ticks=4097),dict(modulus_ticks=128),dict(max_gap_ticks=1<<24),dict(origin_raw_tick=-1),dict(epoch=0)]:
            with self.assertRaises(ValueError):CounterPhaseModel(**kwargs)
        self.assertIsNotNone(CounterPhaseModel().observe(1,0,1,1,provenance='hardware')['fault'])
    def test_timestamp_free_partial_drains_accumulate_until_read_anchor(self):
        b=ready_burst();a=ManufacturedBurstCounter(baseline_transaction_sequence=1);r1=a.update(b.update(data(5),.005,.006,2));r2=a.update(b.update(data(3)+stamp(1030),.007,.008,3));self.assertEqual(r1['status'],'awaiting_read_time_anchor');self.assertIsNone(r1['conditional_phase']);self.assertEqual(r2['cumulative_manufactured_combined_frames'],8);self.assertEqual(r2['conditional_phase']['candidate_phases_ticks'],list(range(7,129)))
    def test_adapter_rejects_faults_mixed_streams_bad_epoch_and_malformed_counts(self):
        b=ready_burst();good=b.update(data()+stamp(128),.005,.006,2)
        for change in ['fault','epoch','boolean_epoch','count','boolean_count','pending','hardware','time','sequence']:
            r=copy.deepcopy(good)
            if change=='fault':r['fault']='missing'
            elif change=='epoch':r['current_model_epoch']=2
            elif change=='boolean_epoch':r['current_model_epoch']=True
            elif change=='count':r['regular_frames']=2
            elif change=='boolean_count':r['regular_frames']=True
            elif change=='pending':r['pending_model_epoch']=2
            elif change=='hardware':r['hardware_driver_executed']=True
            elif change=='time':r['observed_terminal_read_time_ticks']=42
            else:r['transaction_sequence']=3
            a=ManufacturedBurstCounter(baseline_transaction_sequence=1);x=a.update(r);self.assertIsNotNone(x['fault']);self.assertEqual(x['cumulative_manufactured_combined_frames'],0);self.assertEqual(a.update(good)['fault'],x['fault'])
        b=ready_burst();r=b.update(b'\x84'+struct.pack('<hhh',1,2,3)+stamp(128),.005,.006,2);self.assertIsNotNone(ManufacturedBurstCounter(baseline_transaction_sequence=1).update(r)['fault'])
    def test_all_results_retain_unqualified_physical_and_acquisition_flags(self):
        m=CounterPhaseModel();r=m.observe(1030,8,1,1)
        for key in ['output_event_phase_physically_qualified','conversion_freshness_proven','sample_sequence_proven','host_device_clock_mapping_qualified','hardware_driver_executed','consumer_reset_authorized','usable_for_feedback']:self.assertFalse(r[key])
        self.assertIsNone(r['acquisition_s']);self.assertTrue(r['complete_no_loss_fixed_period_is_declared_assumption']);self.assertTrue(r['sub_wrap_gap_is_declared_assumption'])
if __name__=='__main__':unittest.main()
