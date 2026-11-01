import hashlib,unittest
from context import SOURCE,STUDY,read,sha
class Preflight(unittest.TestCase):
    def test_frozen_sources_plan_and_prior_index(self):
        p=read(STUDY/'test-plan.json');self.assertEqual(p['prerequisite_index_sha256'],sha(STUDY.parent/'full_200hz_v1/checkpoint-index.json'));self.assertEqual(p['frozen_plan_sha256'],sha(STUDY/'frozen-plan.json'));self.assertEqual(p['new_native_trials_declared'],0);self.assertEqual(p['hardware_calls_declared'],0)
        for n,h in p['prelaunch_source_hashes'].items():self.assertEqual(sha(SOURCE/n),h);self.assertEqual(sha(STUDY/'prelaunch-source'/n),h)
    def test_manufacturer_sources_are_exact_original_git_blobs(self):
        p=read(STUDY/'test-plan.json')
        for r in read(SOURCE/'vendor/sources.json')['files']:
            data=(SOURCE/'vendor'/r['name']).read_bytes();self.assertEqual(hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest(),p['vendor_git_blobs'][r['name']]);self.assertEqual(len(data),r['bytes']);self.assertEqual(sha(SOURCE/'vendor'/r['name']),r['sha256'])
        self.assertIn('BSD-3-Clause',(SOURCE/'vendor/bmi2.c').read_text());self.assertTrue((SOURCE/'vendor/LICENSE').is_file())
    def test_bound_compiled_memory_parser_and_preserved_initial_failure(self):
        b=read(SOURCE/'build/build.json');self.assertEqual(b['returncode'],0);self.assertEqual(b['stderr'],'');self.assertEqual(b['dll_sha256'],sha(SOURCE/'build/fifo_probe.dll'))
        for n,h in b['inputs'].items():self.assertEqual(sha(SOURCE/n),h)
        failed=read(SOURCE/'build/build-initial-missing-runtime.json');self.assertEqual(failed['returncode'],3221225781);self.assertIsNone(failed['dll_sha256']);prepared=read(SOURCE/'preparation-controls.json');self.assertTrue(prepared['passed']);self.assertEqual(prepared['forbidden_io_calls'],0);self.assertEqual(prepared['new_native_trials'],0)
if __name__=='__main__':unittest.main()
