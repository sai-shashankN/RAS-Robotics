import copy,struct,unittest
from model import BurstEpochModel
from frames import parse,FrameError
def data(v=(1,2,3),w=(4,5,6)):return b'\x8c'+struct.pack('<hhhhhh',*w,*v)
def time(t=1000):return b'\x44'+t.to_bytes(3,'little')
def config(flags=0,t=200):return b'\x48'+bytes([flags])+t.to_bytes(3,'little')
FINGERPRINT={0x40:0xa9,0x41:0,0x42:0xe9,0x43:3}
def ready():
    m=BurstEpochModel();m.declare(1,FINGERPRINT,0,0.,'manufactured');r=m.update(config()+data()+time(),.001,.002,1);assert r['fault'] is None;m.acknowledge(FINGERPRINT,.003,.004,.004,'manufactured');assert m.fault is None;return m
class Controls(unittest.TestCase):
    def test_owner_marker_and_later_readback_are_all_required(self):
        m=BurstEpochModel();r=m.update(data(),0.,.001,1);self.assertTrue(r['grammar_compatible']);self.assertEqual(r['status'],'pending_owner_evidence');self.assertFalse(r['model_owner_bytes_match'])
        m=ready();r=m.update(data()+time(),.005,.006,2);self.assertEqual(r['current_model_epoch'],1);self.assertTrue(r['model_owner_bytes_match']);self.assertTrue(all(e['model_epoch_id']==1 for e in r['events']))
    def test_missing_timestamp_is_valid_partial_drain_without_clock_grant(self):
        m=ready();r=m.update(data(),.005,.006,2);self.assertTrue(r['grammar_compatible']);self.assertEqual(r['regular_frames'],1);self.assertIsNone(r['observed_terminal_read_time_ticks']);self.assertEqual(r['drain_observation'],'no_empty_marker_observed');self.assertIsNone(r['acquisition_s'])
    def test_one_documented80_00_overread_after_data_or_timestamp(self):
        for burst in [b'\x80\x00',data()+b'\x80\x00',data()+time()+b'\x80\x00']:
            r=ready().update(burst,.005,.006,2);self.assertTrue(r['grammar_compatible']);self.assertEqual(r['events'][-1]['kind'],'overread')
        for burst in [b'\x80',b'\x80\x01',b'\x81\x00',b'\x80\x00\x80\x00',b'\x80\x00'+data()]:self.assertIsNotNone(ready().update(burst,.005,.006,2)['fault'])
    def test_timestamp_is_single_terminal_control_not_sample_time(self):
        for burst in [data()+time()+data(),data()+time()+time(2000),time(),time()+b'\x80\x00']:
            r=ready().update(burst,.005,.006,2);self.assertFalse(r['grammar_compatible']);self.assertEqual(r['events'],[])
        r=ready().update(data()+time(0xffffff)+b'\x80\x00',.005,.006,2);self.assertEqual(r['observed_terminal_read_time_ticks'],0xffffff);self.assertTrue(all(e['acquisition_s'] is None for e in r['events']))
    def test_skip_is_first_and_positive_loss_latches(self):
        r=ready().update(b'\x40\x00'+data()+time(),.005,.006,2);self.assertTrue(r['grammar_compatible'])
        for burst in [data()+b'\x40\x00',b'\x40\x00\x40\x00'+data(),b'\x40\x01'+data()]:
            m=ready();r=m.update(burst,.005,.006,2);self.assertIsNotNone(r['fault']);self.assertEqual(m.update(data(),.007,.008,3)['fault'],r['fault']);self.assertEqual(m.update(data(),.009,.010,4)['events'],[])
    def test_reconfiguration_stages_frame_epochs_and_never_auto_resets(self):
        m=ready();m.declare(2,{0x40:0xaa},1,.005,'manufactured');r=m.update(data()+config(1,123456)+data()+time(),.006,.007,2);self.assertEqual([e['model_epoch_id'] for e in r['events']],[1,2,2,2]);self.assertFalse(r['model_owner_bytes_match']);self.assertEqual(r['pending_model_epoch'],2);self.assertEqual(m.owner.pending['marker']['raw_next_frame_ticks'],123456);self.assertFalse(r['consumer_reset_authorized'])
        m.acknowledge({0x40:0xaa},.008,.009,.009,'manufactured');r=m.update(data(),.010,.011,3);self.assertTrue(r['model_owner_bytes_match']);self.assertEqual(r['current_model_epoch'],2);self.assertFalse(r['consumer_reset_authorized'])
    def test_readback_mismatch_before_marker_or_wrong_chronology_latches(self):
        m=BurstEpochModel();m.declare(1,FINGERPRINT,0,0.,'manufactured');m.acknowledge(FINGERPRINT,0.,.001,.001,'manufactured');self.assertIsNotNone(m.fault)
        for which in ['mismatch','early','future','wrong_provenance']:
            m=BurstEpochModel();m.declare(1,FINGERPRINT,0,0.,'manufactured');m.update(config()+data(),.001,.002,1);values=dict(FINGERPRINT);a,b,c=.003,.004,.004;provenance='manufactured'
            if which=='mismatch':values[0x40]^=1
            elif which=='early':a=.001
            elif which=='future':c=.003
            else:provenance='hardware'
            m.acknowledge(values,a,b,c,provenance);self.assertIsNotNone(m.fault);before=copy.deepcopy(m.owner.current);m.acknowledge(FINGERPRINT,.005,.006,.006,'manufactured');self.assertEqual(m.owner.current,before)
    def test_unexpected_multiple_or_mismatched_configuration_never_commits(self):
        for burst in [config(1)+data(),config()+config()+data(),config(0x10)+data()]:
            m=ready();before=copy.deepcopy(m.owner.current);r=m.update(burst,.005,.006,2);self.assertIsNotNone(r['fault']);self.assertEqual(m.owner.current,before);self.assertEqual(r['events'],[])
    def test_valid_planned_marker_followed_by_bad_tail_cannot_hide_error(self):
        for tail in [b'\x7f',b'\x84\x01',b'\x80\x00'+data()]:
            m=BurstEpochModel();m.declare(1,FINGERPRINT,0,0.,'manufactured');r=m.update(config()+data()+tail,.001,.002,1);self.assertIsNotNone(r['fault']);self.assertIsNone(m.owner.pending['marker']);self.assertEqual(m.sequence,0);self.assertEqual(r['events'],[])
    def test_read_sequence_short_read_and_clock_metadata_faults_latch(self):
        cases=[dict(transaction_sequence=3),dict(transaction_sequence=True),dict(read_start_s=.003),dict(read_end_s=.004),dict(read_end_s=float('nan')),dict(requested_bytes=999),dict(provenance='hardware')]
        for change in cases:
            m=ready();args=dict(read_start_s=.005,read_end_s=.006,transaction_sequence=2);args.update(change);r=m.update(data(),**args);self.assertIsNotNone(r['fault']);self.assertEqual(m.update(data(),.010,.011,3)['fault'],r['fault'])
    def test_declared_fingerprint_is_copied_and_bad_epochs_rejected(self):
        values=dict(FINGERPRINT);m=BurstEpochModel();m.declare(1,values,0,0.,'manufactured');values[0x40]=0;self.assertEqual(m.owner.pending['expected_register_bytes'],FINGERPRINT)
        for epoch,values,flags,stamp,origin in [(2,FINGERPRINT,0,0.,'manufactured'),(True,FINGERPRINT,0,0.,'manufactured'),(1,{True:0},0,0.,'manufactured'),(1,{},0,0.,'manufactured'),(1,FINGERPRINT,16,0.,'manufactured'),(1,FINGERPRINT,0,-1.,'manufactured'),(1,FINGERPRINT,0,0.,'hardware')]:
            m=BurstEpochModel();m.declare(epoch,values,flags,stamp,origin);self.assertIsNotNone(m.fault)
    def test_empty_reads_do_not_prove_dead_stream_or_fresh_samples(self):
        r=ready().update(b'',.005,.006,2);self.assertTrue(r['grammar_compatible']);self.assertEqual(r['regular_frames'],0);self.assertIsNone(r['observed_terminal_read_time_ticks']);self.assertFalse(r['conversion_freshness_proven'])
    def test_selected_signed_vectors_and_tag_bits_still_decode(self):
        for tag in range(4):
            r=ready().update(bytes([0x8c|tag])+data((-32768,32767,-1),(123,-456,0))[1:]+time(),.005,.006,2);e=r['events'][0];self.assertEqual(e['accel_counts'],[-32768,32767,-1]);self.assertEqual(e['gyro_counts'],[123,-456,0]);self.assertEqual(e['interrupt_tag'],tag)
    def test_no_hardware_timing_acquisition_scaling_or_feedback_grant(self):
        r=ready().update(data()+time(),.005,.006,2)
        for key in ['conversion_freshness_proven','sample_sequence_proven','host_device_clock_mapping_qualified','hardware_driver_executed','hardware_configuration_qualified','consumer_reset_authorized','usable_for_feedback']:self.assertFalse(r[key])
        self.assertIsNone(r['acquisition_s'])
        for e in r['events']:self.assertIsNone(e['acquisition_s']);self.assertFalse(e['usable_for_feedback']);self.assertFalse(e['hardware_configuration_qualified']);self.assertNotIn('specific_force_m_s2',e)
if __name__=='__main__':unittest.main()
