import sys,json
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence

if __name__=='__main__':
    c=ctx.read(ctx.STUDY/'refinement-results.json')
    report='''# Refined isolated floor impacts

Eight new five-second native Cube/floor observations complete the same four recorded pre-impact states at 0.0625 and 0.03125 ms. Eight explicitly reused normalized 0.25/0.125 ms observations supply earlier comparisons. The physical initial states, geometry, mass/inertia, friction, bounce, contact stiffness/damping and read-only observer are unchanged. Every new observation passes initial-state, complete native-step, contact-normal, finite-state and 1 mm penetration gates.

All twelve consecutive-step position/orientation comparisons fail the retained 0.5 mm / 0.5 degree common-absolute-time gate. At the finest pair, A differs by about 57.5–58.0 mm and roughly 172–173 degrees; B differs by about 2.964 mm and 1.76–1.82 degrees. Refinement has not established convergence. This finding does not prove chaotic dynamics or isolate a unique engine defect; it establishes that the current unmeasured sharp-cube floor-impact model cannot support a precise landing claim at these tested steps.

The normalized missing floor pair remains a consistency correction under investigation, not an adopted full-robot repair. Do not tune controller gains or choose a favorable step to hide this rejection. Robot recovery and dropped-object landing prediction remain distinct evidence, and the original full-pose fault matrix remains rejected.

Next: separately complete the two previously unrun fine navigation-stop faults using the unchanged qualified runtime. Preserve the release rejection and declare no overall fault-matrix or broader M2 admission. Then finish the sampled camera geometry review and plan a bounded contact-model characterization rather than another blind full-robot drop batch.

'''+'```json\n'+json.dumps(c,indent=2)+'\n```\n\n[Plan](test-plan.json), [actual observations](summary.json), [checks](checks/tests.json). Prepared 20 mm / 2 g software evidence only; no Beta/X or hardware completion.\n'
    publish(ctx,test_evidence,'Impact refinement remains unqualified','REFINEMENT_RESULTS.md',report,'Eight additional short native impact observations pass integrity gates; all twelve consecutive-step landing comparisons fail unchanged pose limits. The current floor-impact landing prediction remains unqualified.','Complete two previously unrun fine transit-stop cases in a separate unchanged-runtime study, preserving every drop rejection; finish the independent camera-geometry evidence review.',117,dict(isolated_native_integrity_admitted=True,impact_pose_convergence_admitted=False,full_m2_admitted=False))
