import unittest
from context import STUDY,read,sha
from evaluate import assess,compare
from test_preflight import Preflight
from test_clock_controls import ClockControls
class Evidence(unittest.TestCase):
    def test_all_complete_native_audits_and_fine_pairs_reproduce(self):
        s=read(STUDY/'summary.json')
        for r in s['results']:
            if 'observer200' in r:self.assertEqual(r,assess(r['entry'],STUDY/r['directory']))
        for c in s['comparisons']:self.assertEqual(c,compare([r for r in s['results'] if r['entry']['axis']==c['axis']]))
    def test_actual_counts_complete_task_credits_and_scope(self):
        s=read(STUDY/'summary.json');self.assertEqual(s['simulator_runs'],len(s['results']));self.assertEqual(s['new_complete_nominal_missions'],sum(r.get('new_complete_nominal_mission',False) for r in s['results']));self.assertEqual(s['explicit_old_native_trial_reuses'],0)
        if s['full200_nominal_observation_admitted']:self.assertEqual(s['simulator_runs'],4);self.assertEqual(s['new_complete_nominal_missions'],4);self.assertEqual(len(s['comparisons']),2);self.assertTrue(all(r['passed'] for r in s['results']+s['comparisons']))
        else:self.assertTrue((STUDY/'matrix-stop.json').exists())
        for k in ['hardware_driver_executed','conversion_freshness_admitted','raw_imu_feedback_admitted','realistic_sensing_admitted','full_m2_admitted']:self.assertFalse(s[k])
    def test_all_actual_launched_sources_bound(self):
        for r in read(STUDY/'summary.json')['results']:
            out=STUDY/r['directory'];v=read(out/'launch-inputs.json');self.assertEqual(v['plan_sha256'],sha(STUDY/'test-plan.json'));self.assertEqual(v['preflight_sha256'],sha(out/'preflight-before-launch.json'))
            for n,h in v['sha256'].items():self.assertEqual(sha(out/'sources'/n),h)
if __name__=='__main__':unittest.main()
