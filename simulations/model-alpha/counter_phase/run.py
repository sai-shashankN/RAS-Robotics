import io,unittest
from context import STUDY,write
import test_controls,test_preflight
from evaluate import main
if __name__=='__main__':
    log=io.StringIO();suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromModule(m) for m in [test_controls,test_preflight]);r=unittest.TextTestRunner(stream=log).run(suite);write(STUDY/'preflight.json',dict(tests=r.testsRun,passed=r.wasSuccessful(),log=log.getvalue()));assert r.wasSuccessful(),log.getvalue();main();print(log.getvalue())
