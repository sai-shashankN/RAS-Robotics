import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence

if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json');table='|Case|A physical/recovery|B physical/recovery|A full pose|B full pose|\n|---|---|---|---|---|\n'
    for case in ['no_grasp','blocked_open','drop_transit','transit_stop']:
        a,b=[next(r for r in s['results'] if r['axis']==axis and r['case']==case) for axis in 'ab'];yn=lambda v:'Pass' if v else 'Reject';table+='|'+case+'|'+yn(a['fine_physical_reference_packet_prefix_timing_passed'])+'|'+yn(b['fine_physical_reference_packet_prefix_timing_passed'])+'|'+yn(a['original_full_comparison_passed'])+'|'+yn(b['original_full_comparison_passed'])+'|\n'
    report='''# Actual centered fault coverage consolidated

All eight fine fault pairings now have actual observations, spread across three immutable studies. This review runs zero new simulations: it explicitly reuses eight fine and eight coarse native records. Blocked-opening uses the qualified one-guard clock repair pair; no-grasp and mechanical-release use the original fine study; navigation-stop uses its separate later study. The stopped original queue remains stopped and is not retroactively completed.

All eight fine records pass their retained physical/native/packet/prefix and measured five-second robot-recovery gates. All eight root-pose comparisons pass. Four full root-and-Cube pose comparisons pass:no-grasp and blocked-opening for A/B. Four fail:mechanical-release and navigation-stop for A/B. The dropped-Cube landing behavior remains the failed component;this review does not claim a new floor-impact localization for stop cases or replace the failed full-pose gate with root-only success.

'''+table+'''
The full fault matrix remains rejected. These are prepared centered20mm/2g missions with ideal sensing/actuation,not a robustness envelope or deployable robot. Nominal destination pickup/travel/place remains a separate four-trial success. Neither this summary nor the camera/transport prototypes completes broader Alpha,Beta/X or hardware.

Next: benchmark an RGB-only detection/localization baseline on the saved static images;preserve a separate contact-model characterization work package for the unresolved released-object landing model. Do not spend another expensive full-robot drop batch until a predeclared isolated-model experiment justifies it. [Plan](test-plan.json), [matrix](summary.json), [explicit reused records](reused-records.json), [checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Actual fault coverage consolidated','FAULT_COVERAGE_RESULTS.md',report,'Zero native launches;16explicit reused coarse/fine records. All8fine fault pairings are observed,8/8robot physical recovery and root-pose comparisons pass,but only4/8full root-and-Cube comparisons pass. Overall fault matrix remains rejected.','Benchmark RGB-only detection/localization on saved static images;retain a separately predeclared contact-model characterization before any new full-robot drop batch.',124,dict(fault_fine_matrix_admitted=False,full_m2_admitted=False,all_eight_fine_pairings_observed=True))
