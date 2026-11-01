import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json')
    report='''# Selected FIFO burst grammar and manufactured owner epochs

Zero new native robot trials and zero hardware calls. All268existing227manufactured byte cases are classified separately from their old vendor byte extraction.266complete selected frame sequences retain the saved Bosch signed-vector/control fields;the2bare80end-marker fixtures are intentionally incomplete under the documented80/00overread frame. Grammar rejections are expected semantic/subset distinctions,not new robot failures or relaxed physical gates.

The selected layer checks first-only zero skip,single terminal read-time control,and one final80/00overread. Positive loss,unexpected/multiple configuration,malformed tails,short reads and declared host sequence/chronology faults latch. Complete partial drains without a timestamp and empty reads remain distinct from dead transport. AUX/virtual/headerless/otherpadding and timestamp-only drains remain unqualified.

Six explicit manufactured owner traces exercise pending configuration,frame epoch labels,post-marker readback equality,planned reconfiguration and faults. Owner evidence is accepted only with manufactured provenance. Exact supplied register-byte equality does not validate register meanings,actual settings or board readiness. A configuration marker's next-frame counter remains raw metadata and may refer to a dropped frame. Pending changes never reset a consumer. Transactions stage changes atomically;an early valid marker cannot hide a malformed tail or commit half an epoch.

No SI scaling,device-counter unwrap,per-sample timestamp,host clock map,conversion freshness,board configuration,driver or feedback grant follows. Native Alpha remains at its separately sealed prepared20mm/2gqualification;broaderM2faults,Beta/Xandhardware remain open.

[Bosch primary timing/frame reference](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf),[summary](summary.json),[owner traces](manufactured-owner-traces.json),[checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Selected FIFO burst grammar and manufactured owner epochs','FIFO_BURST_EPOCH_RESULTS.md',report,'Zero native/hardware trials;268existing byte cases classified,266frame-field reconciliations,2documented overread-subset rejections and6manufactured owner traces. Logical byte equality and epochs grant no hardware configuration,clock,SIscale,freshness or feedback.','Prepare explicit bounded/set-valued device-counter association with declared model assumptions;do not invent point acquisition times or promote caller-supplied owner evidence to hardware. Actual sensor/driver calibration and native adoption remain before feedback;M2faults,Beta/Xandhardware remain open.',227,dict(fifo_burst_epoch_source_contract_prepared=s['fifo_burst_epoch_source_contract_prepared'],new_hardware_driver_executions=0,raw_imu_feedback_admitted=False,hardware_driver_executed=False,full_m2_admitted=False))
