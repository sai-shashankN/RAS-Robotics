import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json')
    report='''# Conditional output-event counter-phase family

Zero native robot trials and zero hardware executions. This is a manufactured fixed-period/no-loss counter model,not a reconstruction of real sensor acquisition time. Unknown first phase is retained as a set. Modulo differences are accumulated only under a declared less-than-one-wrap gap interval;actual hardware compliance with that bound is not established. Cumulative frame counts constrain phases,without automatically proving missing-sample freedom.

Six saved schedules include the original8frame/1030tick ambiguity,bounded wrap,repeated read counters,inconsistent counts and epoch changes.100fixed-seed small-period episodes supply2500observations checked against independent explicit event-list enumeration. Complete phase sets match;the earlier32and96tick hypotheses both survive the original observation. More exact manufactured observations may narrow the set to32,but all acquisition/freshness/physical-phase/clock/feedback grants stay false.

Exact228grammar/owner sources compose in one manufactured partial-drain example. Combined counts accumulate until a read-time anchor arrives. The initial empty FIFO/counter origin,exact combined-frame period,produce-before-drain convention,no loss,and bounded gaps are explicit external model assumptions,not consequences of owner-byte equality. Mixed sensor streams,pending/changed epochs,faults or inconsistent metadata are rejected. No host time is relabelled as device time and no filtered effective sample time is inferred.

Contradictions latch without committing staged counter/count state;later valid observations cannot reset a model or consumer. Repeated read-time counters with unchanged counts are allowed and do not prove a new sample. Even singleton conditional output-event bounds are not acquisition timestamps. Hardware calibration/driver evidence,native adoption,broaderM2faults,Beta/Xremain separate.

[Summary](summary.json),[schedule traces](manufactured-counter-traces.json),[composition](manufactured-burst-composition.json),[checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Conditional output-event counter phase family','COUNTER_PHASE_RESULTS.md',report,'Zero native/hardware trials;6manufactured schedules,2500independent event-list observations and one228model composition. Full conditional phase sets retained;singleton output-event bounds grant no acquisition,physical clock,freshness,reset or feedback.','Keep conditional timing separate from measured acquisition. Next prepare explicit count-to-SI/configuration semantics and calibrated measurement uncertainty models before any native sensor adapter or feedback proposal;no board selection or purchase implied. Preserve M2faults,Beta/Xandhardware scope.',228,dict(counter_phase_source_contract_prepared=s['counter_phase_source_contract_prepared'],raw_imu_feedback_admitted=False,hardware_driver_executed=False,full_m2_admitted=False))
