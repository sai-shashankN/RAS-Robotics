import random,struct,unittest
from decoder import HeaderBurstDecoder
from harness import VendorParser
CASES=[]
def xyz(v):return struct.pack('<hhh',*v)
class Controls(unittest.TestCase):
    def compare(self,data):
        own=HeaderBurstDecoder().decode(data);vendor=VendorParser().decode(data);acc=[e['accel_counts'] for e in own['events'] if e['accel_counts'] is not None];gyr=[e['gyro_counts'] for e in own['events'] if e['gyro_counts'] is not None];self.assertEqual(acc,vendor['accel']);self.assertEqual(gyr,vendor['gyro']);self.assertEqual(vendor['forbidden_io_calls'],0)
        times=[e['sensor_time_ticks'] for e in own['events'] if e['sensor_time_ticks'] is not None];skips=[e['skipped_frames'] for e in own['events'] if e['skipped_frames'] is not None];self.assertEqual(vendor['last_sensor_time_ticks'],times[-1] if times else 0);self.assertEqual(vendor['last_skipped_frame_count'],skips[-1] if skips else 0);CASES.append(dict(hex=data.hex(),candidate=own,vendor=vendor));return own
    def test_signed_endian_axis_order_and_interrupt_tags(self):
        acc=[-32768,32767,-1];gyro=[1234,-2345,0]
        for tag in range(4):
            result=self.compare(bytes([0x8c|tag])+xyz(gyro)+xyz(acc));self.assertTrue(result['syntax_compatible']);self.assertEqual(result['events'][0]['accel_counts'],acc);self.assertEqual(result['events'][0]['gyro_counts'],gyro);self.assertEqual(result['events'][0]['interrupt_tag'],tag)
    def test_separate_sensor_frames_are_not_invented_pairs(self):
        r=self.compare(b'\x84'+xyz([1,2,3])+b'\x88'+xyz([4,5,6]));self.assertEqual([e['kind'] for e in r['events']],['accel','gyro']);self.assertIsNone(r['events'][0]['gyro_counts']);self.assertIsNone(r['events'][1]['accel_counts']);self.assertTrue(all(e['acquisition_s'] is None for e in r['events']))
    def test_sensor_time_is_raw_control_metadata_even_at_wrap(self):
        data=b'\x8c'+xyz([1,2,3])+xyz([4,5,6])+b'\x44\xff\xff\xff'+b'\x44\x00\x00\x00';r=self.compare(data);self.assertEqual([e['sensor_time_ticks'] for e in r['events'] if e['kind']=='sensor_time'],[16777215,0]);self.assertTrue(all(e['acquisition_s'] is None and e['sample_sequence_proven'] is False for e in r['events']))
    def test_seeded_mixed_ordinary_bursts_match_manufacturer(self):
        rng=random.Random(270200)
        for _ in range(256):
            data=bytearray()
            for _ in range(rng.randrange(1,20)):
                kind=rng.choice([0x84,0x88,0x8c,0x44,0x40]);data.append(kind|rng.randrange(4))
                if kind in [0x84,0x88]:data.extend(xyz([rng.randrange(-32768,32768) for _ in range(3)]))
                elif kind==0x8c:data.extend(xyz([rng.randrange(-32768,32768) for _ in range(3)])+xyz([rng.randrange(-32768,32768) for _ in range(3)]))
                elif kind==0x44:data.extend(rng.randrange(1<<24).to_bytes(3,'little'))
                else:data.append(0)
            self.assertTrue(self.compare(bytes(data))['syntax_compatible'])
    def test_all_incomplete_data_and_control_frames_reject_and_latch(self):
        frames=[b'\x84'+xyz([1,2,3]),b'\x88'+xyz([1,2,3]),b'\x8c'+xyz([1,2,3])*2,b'\x44\x01\x02\x03',b'\x40\x01',b'\x48\x00\x00\x00\x00']
        for frame in frames:
            for cut in range(1,len(frame)):
                d=HeaderBurstDecoder();r=d.decode(frame[:cut]);self.assertFalse(r['syntax_compatible']);self.assertIn('truncated',r['fault']);again=d.decode(frames[0]);self.assertEqual(again['fault'],r['fault']);self.assertEqual(again['events'],[])
    def test_skip_and_configuration_changes_invalidate_without_reset(self):
        for control,reason in [(b'\x40\x03','unqualified_skipped_frames'),(b'\x48\x01\x02\x03\x04','unqualified_configuration_change')]:
            d=HeaderBurstDecoder();data=b'\x84'+xyz([1,2,3])+control+b'\x88'+xyz([4,5,6]);r=self.compare(data);self.assertEqual(r['fault'],reason);r=d.decode(data);self.assertEqual(r['fault'],reason);self.assertEqual(d.decode(b'\x84'+xyz([7,8,9]))['events'],[])
    def test_unknown_auxiliary_virtual_and_padding_modes_fail_closed(self):
        for data in [b'\x00',b'\x90'+b'\0'*8,b'\x9c'+b'\0'*20,b'\xc0'+b'\0'*15,b'\x80\x00',b'\x84'+xyz([1,2,3])+b'\x7f']:
            r=HeaderBurstDecoder().decode(data);self.assertFalse(r['syntax_compatible']);self.assertIsNotNone(r['fault'])
    def test_empty_burst_and_exact_end_marker_supply_no_new_sensor_data(self):
        for data in [b'',b'\x80',b'\x84'+xyz([1,2,3])+b'\x80']:
            r=self.compare(data);self.assertTrue(r['syntax_compatible']);self.assertFalse(r['conversion_freshness_proven']);self.assertFalse(r['sample_sequence_proven'])
    def test_no_hardware_acquisition_scale_or_feedback_grant(self):
        r=self.compare(b'\x8c'+xyz([123,456,789])+xyz([-1,-2,-3])+b'\x44\x01\x02\x03');self.assertIsNone(r['acquisition_s'])
        for key in ['conversion_freshness_proven','sample_sequence_proven','hardware_driver_executed','usable_for_feedback']:self.assertFalse(r[key])
        for event in r['events']:
            self.assertIsNone(event['acquisition_s']);self.assertFalse(event['sample_sequence_proven']);self.assertFalse(event['usable_for_feedback']);self.assertFalse(event['hardware_driver_executed']);self.assertNotIn('specific_force_m_s2',event)
if __name__=='__main__':unittest.main()
