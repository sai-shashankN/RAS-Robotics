import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json')
    report='''# Selected raw-count conversion and ideal quantization

Zero new native trials and zero hardware executions. This source model decodes only declared ACC_RANGE, GYR_RANGE and FIFO_DOWNS bytes for filtered FIFO data. It rejects missing/extra fields, reserved range codes and pre-filtered sources. The independent OIS range bit is preserved but never used to scale filtered gyro counts. Downsampling factors are decoded without inventing cadence or effective configuration timing.

Signed16 counts become unmapped sensor-frame m/s² and rad/s under the Bosch example range/32768 convention, evaluated in binary64. Standard gravity is explicitly9.80665m/s², distinct from simulator9.81m/s². This is an ideal conversion convention, not calibrated sensitivity, factory cross-axis compensation, axis remapping or bias/filter/temperature correction. Raw registers and configuration epoch remain bound to each result. No hardware configuration has been selected or executed.

All65536 signed codes for each of4accel and5gyro scales round-trip exactly:589824code checks,9saved stream hashes.9000seeded continuous inputs satisfy distance-to-representable-interval plus half-LSB bounds with an explicit floating arithmetic allowance. An independent Fraction oracle checks nearest-even rounding and endpoint clipping. Endpoint rail occupancy remains diagnostic and does not prove physical saturation. These are software arithmetic controls, not robot trials or measured ADC behavior.

Three manufactured paired records preserve counts, profile identity and declared read clocks. Epoch mismatch/nonfinite/malformed inputs reject conversion; no record is made fresh by this stateless layer. All acquisition, calibration, physical configuration, consumer-reset, hardware and feedback grants remain false. Original physical gates, failures, broaderM2faults and Beta/X remain unchanged.

[Summary](summary.json), [exhaustive domains](exhaustive-counts.json), [bound cases](random-quantization-bounds.json), [checks](checks/tests.json).

Primary basis: [Bosch SensorAPI conversion example](https://github.com/boschsensortec/BMI270_SensorAPI/blob/master/bmi270_examples/accel_gyro_hw_int/accel_gyro_hw_int.c) and [Bosch datasheet revision1.6, printed pages103–104](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf). Exact BSD-licensed example/definition blobs and their hashes are saved with the source. No full manufacturer driver was executed in this phase.
'''
    publish(ctx,test_evidence,'Selected count-to-SI source model and ideal quantization','COUNT_SI_RESULTS.md',report,'Zero native/hardware trials;589824signed-code round trips and9000seeded bound cases. Selected filtered-FIFO range semantics,explicit standard gravity and configuration epochs;no calibration/acquisition/feedback grant.','Replay four explicitly existing226full5ms histories through declared ideal quantizers,retaining original simulation producer clocks and reporting range/rail/error tradeoffs separately. No new native or closed-loop credit;physical sensor errors and broaderM2/Beta/Xremain open.',229,dict(counts_to_SI_source_contract_prepared=s['counts_to_SI_source_contract_prepared'],calibrated_SI_accuracy_qualified=False,raw_imu_feedback_admitted=False,hardware_driver_executed=False,full_m2_admitted=False))
