import sys,json
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence
if __name__=='__main__':
    s=ctx.read(ctx.STUDY/'summary.json');timing=ctx.read(ctx.STUDY/'timing-profile.json');report='''# Raised-camera/preprocessing fresh snapshot benchmark

Fourteen new short static native captures complete:two explicit controls and twelve new task snapshots at close70%,lift50%,hold25%,transit25%,destination-lower75%and destination-open50%for A/B. These times differ from the development and first snapshot benchmark. They still come from the same two nominal traces and controlled tagged20mmCube,so this does not establish broad generalization or arbitrary-object perception.

The candidate freezes55mmbody-frame camera height(47mmabove the original lens origin),known hypothetical camera extrinsics,independently fitted virtual intrinsics,and oneGaussian3x3sigma.6prefilter before the unchangedAPRILTAG/joint-fit algorithm. Provisional opaque stalk/housing boxes remain static optical surrogates. No real robot mass,inertia,mechanical model or controller changes.

All12newtask snapshots return image-cost-selected poses under the primary pipeline,and every pose meets the predeclared1mm/1degree diagnostics. Clean-image maximum center error is0.242mm/orientation0.527degree. Across dimming,additional blur and seeded noise,the maximum center error is0.245mm. All12cases remain available in every variant. These48task-variant evaluations are derived from12native task captures,not48native trials. Positive and completely occluded controls pass. No oracle selects a hypothesis;outputs are saved before evaluating Cube truth.

Sixty additional software-only timing invocations reproduce the saved clean outputs exactly. Host estimator median is40.8ms,p95is62.3ms,and maximum63.2ms during this measurement. It excludes capture/transport/scheduling and is not a real-time bound. This computation exceeds the8msbalance-loop interval;future integration needs a nonblocking observation worker. No controller deadline,calibrated covariance or camera-feedback mission is qualified.

The static benchmark is complete in its declared scope. Mechanical clearance/strength/mass,wiring,continuous images,calibration uncertainty,frame timing,missing/reordered data and retained-grip recovery remain open. [Next integration package](DYNAMIC_VISION_NEXT.md), [plan](test-plan.json), [native results](summary.json), [timing](timing-profile.json), [checks](checks/tests.json). The full fault-pose matrix and floor-impact model remain rejected;Beta/X and hardware are not completed.

'''+'```json\n'+json.dumps(s['metrics'],indent=2)+'\n```\n'
    publish(ctx,test_evidence,'Raised-camera static pose benchmark passed','PREFILTER_HOLDOUT_RESULTS.md',report,'14new static native captures. Frozen55mmcamera/prefilter candidate returns12/12fresh task poses under all declared stresses,<=0.245mmcenter error. Host compute median40.8ms;no dynamic sensing,mechanical adoption or control admission.','Screen provisional camera mounts and design a nonblocking timestamped observer;qualify physical noninterference and missing-data/retained-grip behavior before any vision feedback. Preserve unresolved floor-impact and full fault-pose rejections.',135,dict(static_pose_benchmark_admitted=True,layout_adopted=False,runtime_deadline_admitted=False,camera_perception_admitted=False,full_m2_admitted=False))
