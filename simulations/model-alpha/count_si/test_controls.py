import math,unittest
from fractions import Fraction
from scale import profile,RangeProfile,convert_pair,decode_axis,quantize_axis,G0,ACC_G,GYR_DPS

class Controls(unittest.TestCase):
    def test_all_range_codes_and_source_path_semantics(self):
        for a in range(4):
            for g in range(5):
                for ois in (0,8):
                    p=profile(1,a,g|ois,0x88)
                    self.assertEqual(p.description()['range_g'],ACC_G[a]);self.assertEqual(p.description()['range_dps'],GYR_DPS[g])
                    self.assertEqual(p.acc_scale,ACC_G[a]*G0/32768);self.assertEqual(p.gyr_scale,math.radians(GYR_DPS[g])/32768)
                    self.assertEqual(p.gyr_scale,profile(1,a,g,0x88).gyr_scale)

    def test_exhaustive_single_register_domain(self):
        for n in range(256):
            for which,valid in [('acc',n<4),('gyro',n<16 and (n&7)<=4),('downs',n&0x88==0x88)]:
                args=dict(acc_range_byte=0,gyro_range_byte=3,fifo_downs_byte=0x88)
                args[{'acc':'acc_range_byte','gyro':'gyro_range_byte','downs':'fifo_downs_byte'}[which]]=n
                if valid:profile(1,**args)
                else:
                    with self.assertRaises(ValueError):profile(1,**args)

    def test_downsampling_and_ois_do_not_invent_cadence_or_change_filtered_scale(self):
        p=profile(1,0,11,0xff);d=p.description()
        self.assertEqual(d['accel_downsample_factor'],128);self.assertEqual(d['gyro_downsample_factor'],128)
        self.assertEqual(d['ois_range_bit'],1);self.assertFalse(d['cadence_qualified'])
        self.assertEqual(p.gyr_scale,profile(1,0,3,0x88).gyr_scale)
        with self.assertRaises(ValueError):profile(1,0,3,0x80)

    def test_profile_rejects_missing_duplicate_extra_and_unsupported_fields(self):
        for regs in [((0x41,0),(0x43,3)),((0x41,0),(0x41,0),(0x43,3)),((0x41,0),(0x43,3),(0x45,0x88),(0x40,0xa9)),((0x43,3),(0x41,0),(0x45,0x88))]:
            with self.assertRaises(ValueError):RangeProfile(1,regs)
        with self.assertRaises(ValueError):profile(1,0,3,0x88,source_path='data_register')
        for n in [True,0,-1,1.0,'1',None]:
            with self.assertRaises(ValueError):profile(n,0,3,0x88)
        for n in [True,1.0,'0',None,256,-1]:
            with self.assertRaises(ValueError):profile(1,n,3,0x88)

    def test_gravity_and_signed_asymmetric_endpoints(self):
        p=profile(1,0,3,0x88)
        self.assertEqual(decode_axis(16384,p.acc_scale),G0)
        self.assertNotEqual(decode_axis(16384,p.acc_scale),9.81)
        self.assertEqual(decode_axis(-32768,p.acc_scale),-2*G0)
        self.assertLess(decode_axis(32767,p.acc_scale),2*G0)
        self.assertAlmostEqual(decode_axis(16384,p.gyr_scale),math.radians(125))

    def test_nearest_even_ties_and_clip_bounds(self):
        for x,c in [(.5,0),(1.5,2),(2.5,2),(-.5,0),(-1.5,-2),(32767.5,32767),(-32768.5,-32768),(1e100,32767),(-1e100,-32768)]:
            r=quantize_axis(x,1.0);self.assertEqual(r['count'],c);self.assertLessEqual(r['absolute_error_SI'],r['bound_SI'])

    def test_rail_occupancy_is_not_proof_of_saturation(self):
        for x in [32767.,32767.1]:
            r=quantize_axis(x,1);self.assertTrue(r['at_raw_rail']);self.assertFalse(r['physical_saturation_proven'])
            self.assertEqual(r['input_outside_representable_interval'],x>32767)

    def test_invalid_counts_finite_values_and_schema(self):
        for x in [True,1.,'1',32768,-32769,None]:
            with self.assertRaises(ValueError):decode_axis(x,1)
        for x in [True,'1',None,math.nan,math.inf,-math.inf,10**400]:
            with self.assertRaises(ValueError):quantize_axis(x,1)
        for s in [0,-1,math.nan,math.inf,1e308]:
            with self.assertRaises(ValueError):quantize_axis(1,s)
        p=profile(1,0,3,0x88)
        for vec in [[0,0], [0,0,True], [0,0,1.], '000']:
            with self.assertRaises(ValueError):convert_pair(p,vec,[0,0,0],epoch=1,sequence=1,read_clock_s=0)

    def test_epoch_mismatch_and_provenance_do_not_grant_timing(self):
        p=profile(1,0,3,0x88)
        kw=dict(epoch=1,sequence=1,read_clock_s=.005)
        r=convert_pair(p,[0,0,16384],[0,0,0],**kw)
        self.assertEqual(r['original_declared_read_clock_s'],.005);self.assertIsNone(r['acquisition_s'])
        for k in ['calibrated_SI_accuracy_qualified','factory_cross_axis_compensation_applied','mount_alignment_applied','hardware_driver_executed','configuration_physically_qualified','conversion_freshness_admitted','usable_for_feedback','consumer_reset_authorized']:self.assertFalse(r[k])
        for change in [dict(epoch=2),dict(epoch=True),dict(sequence=True),dict(read_clock_s=math.nan),dict(read_clock_s=-.1),dict(provenance='hardware')]:
            with self.assertRaises(ValueError):convert_pair(p,[0,0,0],[0,0,0],**(kw|change))
        self.assertEqual(r['raw_accel_counts'],[0,0,16384])

    def test_profile_identity_binds_epoch_and_all_selected_bytes(self):
        p=profile(1,0,3,0x88)
        for q in [profile(2,0,3,0x88),profile(1,1,3,0x88),profile(1,0,11,0x88),profile(1,0,3,0x89)]:self.assertNotEqual(p.fingerprint,q.fingerprint)

    def test_exact_rational_quantization_oracle(self):
        # Independent Fraction nearest-even oracle at exactly binary scales.
        for divisor in [1,2,128,32768]:
            s=Fraction(1,divisor)
            for numerator in range(-262150,262151,113):
                x=Fraction(numerator,8*divisor);ideal=x/s;q,r=divmod(ideal.numerator,ideal.denominator)
                expected=q+(2*r>ideal.denominator or (2*r==ideal.denominator and q%2!=0))
                expected=max(-32768,min(32767,expected));actual=quantize_axis(float(x),float(s))
                self.assertEqual(actual['count'],expected);self.assertLessEqual(actual['absolute_error_SI'],actual['bound_SI'])

if __name__=='__main__':unittest.main()
