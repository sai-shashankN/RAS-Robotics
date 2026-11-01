import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json')
    report='''# Complete nominal Alpha tasks with passive200Hzraw and causal observer

Four fresh A/Bcoarse/fine complete prepared20mm/2gnominal tasks declared. Actual attempts,complete-task credits and admission follow summary. All original pickup/turn/travel/placement/release/withdrawal/postverification/native/contact/geometry/frame/drive/packet/safety/scalar/full root+Cube0.5mm/0.5degree fine limits remain. All22complete physical bytes and all220old8msraw/reference/owner/history/consumer/shadow/pitch bytes must match exactly. First60snewraw/host/native/consumer bytes must match225continuous prefixes. Old cases are comparison references,not extra native attempts.

Dedicated5msAPIs,1msouter scheduling,exact225queue/history/core/consumer/projection and passive native5msledger run beside unchanged8mslegacy control. Each native observer output equals independent raw-only replay. Every accepted state is scored at its actual producer read clock against independent native-reconciled witnesses. Original0.5degree and0.001rad/slimits are retained. References never enter observer or command inputs.

Actual original endpoints remain339.384s(A)and339.368s(B). Last producer states are339.380/339.365s,with4/3msAPIread age. Native5msinterior ledger contains0andall5msproducer clocks strictly before task end. A separate actual audit-only endpoint witness matches the final old8msreference. No future callback,state retiming or acquisition timestamp is invented. Complete causal8mshost batches include the final partial cycle;all producer snapshots delivered once and no pending events. APIage excludes unknown filter/acquisition/hardware latency.

This is scoped software observation qualification only. No realFIFO/driver,conversion freshness,noise/filter/bias/latency calibration,gravity confidence,rawfeedback,broaderM2fault/impact envelope,Beta/Xorhardware completion follows. Legacy privileged control remains.

[Summary](summary.json),[frozen plan](frozen-native-plan.json),[checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Complete nominal Alpha200Hz passive observation','FULL200_RESULTS.md',report,'Four fresh complete nominal tasks declared with actual endpoints,all original physical/old8msbytes,225prefix bytes,complete5msnative/raw/causal observer and original fine gates retained. Summary determines actual qualification;no feedback/hardware or broaderM2fault credit.','Prepare measurement-noise/bias/filter/timing and gravity-confidence work using explicit recorded data and primary specifications,with native integration separately required before feedback. Preserve broaderM2fault/impact envelope,Beta/Xandhardware.',225,dict(full200_nominal_observation_admitted=s['full200_nominal_observation_admitted'],raw_imu_feedback_admitted=False,hardware_driver_executed=False,full_m2_admitted=False))
