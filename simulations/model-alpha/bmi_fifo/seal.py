import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json')
    report='''# Restricted manufacturer FIFO-header decoder source contract

Zero new robot/native trials and zero hardware driver executions.268manufactured byte bursts compare a restricted Python decoder against the actual unmodified Bosch SensorAPI Cparser executed in host memory. The bridge supplies blocked read/write/delay callbacks;every case records0I/O calls. Signed16little-endian axes,combined gyro-then-accel order,interrupt tags,separate streams and last raw24bit sensor-time/skip metadata match. Exact vendor Git blobs,headers and BSD-3-Clause license are archived;compiled DLL and inputs are bound.

Truncated complete-burst frames,unknown/AUX/virtual/headerless modes and unqualified padding reject and latch. Positive skipped frames or input-configuration changes make the entire byte burst syntax-incompatible;prefix events may remain for audit,but later valid bytes never reset the decoder. Strict whole-burst rejection is intentionally stronger than the vendor helper's partial-prefix extraction. No unknown frame length is guessed. The8192byte analysis cap is not a hardware FIFO capacity.

Sensor-time is a separate raw control event,not an invented per-sample timestamp. Adjacent accel/gyro frames are not silently paired. Raw counts are not SI-scaled without a separately qualified configuration. A syntax_compatible result is not a valid physical-burst grammar or configuration-epoch admission;manufactured multi/interleaved control records only exercise byte parsing. No sample-sequence proof,clock mapping,conversion freshness,acquisition time,filter/noise/bias/latency calibration,200Hzboard configuration,feedback or hardware integration follows.

The first direct Git blob archival request returned403before any vendor file was written. Raw-file fallback was accepted only after every Git blob SHA matched the connector's primary source listing. The initial compiler exit0xC0000135(missing runtime search path)and build record remain preserved;adding the existing bundled GCCsupport paths produced a clean-Werror build without changing vendor sources. Neither preparation failure was a robot trial.

[Official Bosch Cparser](https://github.com/boschsensortec/BMI270_SensorAPI/blob/master/bmi2.c),[header definitions](https://github.com/boschsensortec/BMI270_SensorAPI/blob/master/bmi2_defs.h),[source ledger](prelaunch-source/vendor/sources.json),[summary](summary.json),[checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Restricted BMI FIFO header decoder source contract','BMI_FIFO_RESULTS.md',report,'Zero new native trials;268manufactured memory-only differential cases against exact Bosch Cparser,zero I/O callbacks. Partial/unsupported/skip/configuration cases fail closed. No per-sample timestamp,SI calibration,board driver or feedback admission.','Prepare explicit FIFO read/configuration epochs,per-frame time association and host clock mapping with primary-source evidence;manufactured timing models are not hardware measurements. Continue actual motion/noise/filter/bias confidence work before feedback;M2faults,Beta/Xandhardware remain open.',226,dict(fifo_header_source_contract_prepared=s['fifo_header_source_contract_prepared'],new_hardware_driver_executions=0,raw_imu_feedback_admitted=False,hardware_driver_executed=False,full_m2_admitted=False))
