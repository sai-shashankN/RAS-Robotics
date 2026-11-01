import ast,unittest
from context import ROOT,SOURCE,STUDY,read,sha
from derivation import reverse,reverse_world,reverse_native
class Preflight(unittest.TestCase):
    def test_frozen_plan_sources_and_original_limits(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['frozen_native_plan_sha256'],sha(STUDY/'frozen-native-plan.json'));self.assertEqual(p['references_sha256'],sha(STUDY/'references.json'))
        for n,h in p['source_indices'].items():self.assertEqual(sha(STUDY.parent/n/'checkpoint-index.json'),h)
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
        for k,v in read(STUDY.parent/'full_owned_pitch_v1/test-plan.json')['limits'].items():self.assertEqual(p['limits'][k],v)
    def test_full_runtime_reversible_and_only_declared_changes(self):
        es=read(STUDY/'manifest.json')['entries'];self.assertEqual(len(es),4);self.assertEqual({(e['axis'],e['physics_ms']) for e in es},{(a,h) for a in 'ab' for h in [.25,.125]})
        for e in es:
            p=ROOT/e['project'];p=p/'results/raw_full/sources' if(p/'results/raw_full/sources').exists() else p;old=ROOT/e['legacy_full_reference_output']/'sources';prefix=ROOT/e['owned200_prefix_output']/'sources'
            for f in old.rglob('*'):
                if not f.is_file() or '__pycache__' in f.parts:continue
                n=f.relative_to(old);name=n.as_posix()
                if name=='controllers/manip_controller/manip_controller.py':self.assertEqual(reverse((p/n).read_text()),f.read_text());self.assertNotIn('prefix_end_reached',(p/n).read_text());ast.parse((p/n).read_text())
                elif name==e['world'].replace('\\','/'):self.assertEqual(reverse_world((p/n).read_text()),f.read_text())
                elif name=='plugins/physics/robot_cfm_guard/robot_cfm_guard.c':self.assertEqual(reverse_native((p/n).read_text()),f.read_text());self.assertEqual(sha(p/n),sha(prefix/n))
                elif name=='plugins/physics/robot_cfm_guard/robot_cfm_guard.dll':self.assertEqual(sha(p/n),sha(prefix/n))
                elif name=='collision-fixtures.json':
                    actual=read(p/n);expected=read(f);self.assertEqual(actual.pop('world_sha256'),sha(p/e['world']));expected.pop('world_sha256');self.assertEqual(actual,expected)
                else:self.assertEqual(sha(p/n),sha(f),name)
            for n in ['full200.py','sidecar200.py','observed200.py','host_consumer.py','history200.py','consumer200.py','projection200.py','buffer.py']:self.assertEqual(sha(p/'controllers/manip_controller'/n),sha(SOURCE/n))
            self.assertEqual(sha(p/'plugins/physics/robot_cfm_guard/raw200_native.inc'),sha(prefix/'plugins/physics/robot_cfm_guard/raw200_native.inc'))
    def test_preserved_full_auditors_and_qualified200consumer_sources(self):
        old=STUDY.parent/'full_owned_pitch_v1/source_snapshot';new=STUDY.parent/'owned_200hz_v1/source_snapshot'
        for n in ['full_reference.py','full_wrench.py','mission_reference.py','raw_metrics.py','original_compare.py','measurement.py','window.py','attitude_shadow.py','timed_shadow.py','owner.py','history.py','consumer.py','projection.py']:self.assertEqual(sha(SOURCE/n),sha(old/n),n)
        self.assertEqual(sha(SOURCE/'legacy_full_reference.py'),sha(old/'evaluate.py'))
        for n in ['sidecar200.py','observed200.py','host_consumer.py','history200.py','consumer200.py','projection200.py','buffer.py']:self.assertEqual(sha(SOURCE/n),sha(new/n))
    def test_actual_original_endpoints_and_full_clock_counts(self):
        plan=read(STUDY/'frozen-native-plan.json')
        for e in read(STUDY/'manifest.json')['entries']:
            expected=plan['case_endpoints'][e['axis']];self.assertEqual(e['expected_control_ticks'],expected['ticks']);self.assertEqual(e['expected_endpoint_s'],expected['endpoint_s']);ms=round(e['expected_endpoint_s']*1000);self.assertEqual(ms//5,expected['producer_snapshots']);self.assertEqual((ms-1)//5+1,expected['native_interior_rows']);self.assertEqual((ms//5)*5,expected['last_producer_ms']);self.assertEqual(ms%5,expected['last_API_read_age_ms']);self.assertNotEqual(ms%5,0)
if __name__=='__main__':unittest.main()
