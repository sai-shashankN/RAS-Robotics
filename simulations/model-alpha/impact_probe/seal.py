import sys,json
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    c=ctx.read(ctx.STUDY/'comparison-results.json');report='''# Isolated cube-impact controls

Sixteen actual five-second native observations cover four recorded pre-impact states,default/normalized floor contacts,and0.25/0.125ms steps. Every pair shares the same declared initial pose/velocity. Actual Supervisor/native state,mass/inertia,all physics-step windows,active finite Cube,top-facing floor contacts and the original1mm depth bound pass. No robot or manipulation controller is simulated in these short controls.

Adding the explicit coarse-anchored floor pair reproduces all four default coarse trajectories byte-for-byte,including native motion/contact logs. The normalization therefore preserves the coarse model as intended. It improves some finer trajectories,but neither the default nor normalized four-pair set meets all original0.5mm/0.5degree limits. This is not an admitted full-mission repair.

Different recorded seed states also produce large landing changes at the same step:Alpha-A sensitivity exceeds50mm. Those are different-initial-state sensitivity observations,not numerical-convergence comparisons. The impact is highly sensitive even before considering hardware uncertainty. No controller gain search or acceptance relaxation is justified by these results.

'''+'```json\n'+json.dumps(c,indent=2)+'\n```\n\nNext:short normalized0.0625/0.03125ms isolated controls to measure refinement behavior before any expensive full-robot replay. Retain the rejected full-pose gate and separate robot-recovery evidence from unqualified debris-landing prediction. The normalized missing pair remains a consistency correction for testing,not a measured hardware material or a proven sole cause.\n\n[Plan](test-plan.json), [native observations](summary.json), [comparisons](comparison-results.json), [checks](checks/tests.json). No M2,Beta/X or hardware completion is claimed.\n'
    publish(ctx,test_evidence,'Isolated impact sensitivity measured','IMPACT_RESULTS.md',report,'Sixteen short native impact controls pass state/contact/depth integrity. The explicit pair preserves coarse behavior exactly,but normalized landing trajectories still fail original pose-convergence limits and are sensitive to tiny initial-state changes.','Run eight short normalized refined-step impacts at0.0625/0.03125ms from the same recorded states;use them to diagnose accuracy versus landing sensitivity before changing the full robot.',116,dict(isolated_native_integrity_admitted=True,impact_pose_convergence_admitted=False,full_m2_admitted=False))
