import sys,json
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json');report='''# Spawned latest-frame vision worker

Zero new native robot trials. One separate spawned process runs the unchanged frozen estimator on fourteen explicitly reused saved image bundles. Every selected pose/status and RGBpayload hash matches the saved static result. The producer never calls image inference. Two shared-memory slots retain at most one pending bundle,plus one in process;overload replaces old pending inputs rather than growing an unbounded queue.

Ten manufactured transport controls cover actual childPID,latest-only overload,try-lock contention,stale/replayed/malformed/foreign data,backward simulator clocks,estimator exception,worker death and single producer ownership. They are IPC/software tests,not robot trials. Explicit simulated capture/delivery times remain separate from measured host processing duration. The actual-image benchmark uses a declared80msminimum simulated delivery and100msmaximum age;those remain test policies,not physical sensor specifications. Controller usability staysFalse and calibrated uncertainty staysNone. Camera loss does not change grasp membership or actuators.

Publication/polling avoid waiting for IPC locks,but still perform memory copies and OS operations. No8msdeadline or whole-controller integration is qualified. A Webots controller must have a proper import-safe main guard before using Windows spawn;the current accepted script executes setup/loop at module top level. A separate derivative must wrap it without changing actuation logic and prove that child import cannot create another Supervisor/actuator loop.

Next:prepare a guarded sensor-only40spickup/hold prefix replay at both native resolutions. Add virtual cameras/tag visuals while preserving every physical bounding object,mass/inertia,motor/physics parameter and original command expression. Compare raw telemetry/native/contact/drive prefixes exactly;do not substitute an original full-mission success for the new observation test. [Plan](test-plan.json), [actual saved-image jobs](summary.json), [controls](checks/tests.json).
'''
    publish(ctx,test_evidence,'Spawned vision transport functional benchmark passed','VISION_WORKER_RESULTS.md',report,'Zero native trials. Separate-process estimator matches14reused image bundles exactly;latest-frame overload,timestamps,stale/error/death and lock controls pass. No control integration or deadline qualification.','Prepare import-safe guarded sensor-only prefix replays at0.25/0.125ms with exact raw physical noninterference checks before considering vision feedback.',137,dict(functional_worker_benchmark_admitted=True,runtime_deadline_admitted=False,controller_integrated=False,full_m2_admitted=False))
