import io,unittest
from context import SOURCE,write
import test_controls
if __name__=='__main__':
    target=SOURCE/'preparation-controls.json';assert not target.exists();s=io.StringIO();r=unittest.TextTestRunner(stream=s).run(unittest.defaultTestLoader.loadTestsFromModule(test_controls));write(target,dict(tests=r.testsRun,passed=r.wasSuccessful(),vendor_memory_parser_calls=len(test_controls.CASES),forbidden_io_calls=sum(c['vendor']['forbidden_io_calls'] for c in test_controls.CASES),new_native_trials=0,hardware_driver_executed=False,log=s.getvalue()));print(s.getvalue());assert r.wasSuccessful()
