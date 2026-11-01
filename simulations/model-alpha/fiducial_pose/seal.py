import sys
import context as ctx
sys.path.insert(0,str(ctx.SOURCE.parent))
from phase_support import publish
import test_evidence

if __name__=='__main__':
    report='''# Fiducial/PnP isolated prototype

The already-installed OpenCV4.11.0 library supplies an AprilTag36h11 dictionary detector and IPPE_SQUARE pose solver. Its local extension binary hash is recorded;no new package is installed. This phase runs zero native simulations. Generated2Dtag images and manufactured projected corners exercise the code;none is a native robot observation or a task detection.

The RGB-only frontend decodes a known generated marker through four image rotations and returns explicit no detections on blank input. The pose API requires calibrated intrinsics,explicit tag edge in metres and canonical ordered corners. It retains every positive-depth planar hypothesis and reports reprojection residual,without inventing confidence or covariance. Known oblique pose and full camera/tag/object transform controls pass. Missing/reversed/nonfinite corners,invalid calibration and invalid tag size reject.

Controls demonstrate two important limits:a cyclic corner relabeling can preserve near-zero reprojection residual while changing orientation by about90degrees,and doubling assumed marker size doubles translation scale. Decoded identity/canonical corner order,correct measured tag size,calibration and tag-to-object center transform are essential. Surface/tag center is not automatically Cube center. Multiple planar hypotheses still require independent ambiguity handling;ground truth cannot choose the solution in deployable code.

Primary implementation references are the pinned [OpenCV4.11PnP documentation](https://docs.opencv.org/4.11.0/d5/d1f/calib3d_solvePnP.html) and [fiducial detection guide](https://docs.opencv.org/4.11.0/d5/dae/tutorial_aruco_detection.html). The broader method comparison remains in the preceding RGB study's plan. The narrow tagged-object visual assumption is a new research branch,not a replacement claim for arbitrary unmarked objects.

Next: create a separately declared static native tagged-Cube visual derivative using known face ID/size/transform registry,positive and occluded controls,and RGB-only detection before opening oracle evaluation data. Keep original plain/color images and all physical failures immutable. After that,held-out pose/light/occlusion/calibration/timing tests must precede any sensed robot branch. No Beta/X or hardware completion. [Plan](test-plan.json), [manufactured controls](summary.json), [checks](checks/tests.json).
'''
    publish(ctx,test_evidence,'Fiducial pose math prototype prepared','FIDUCIAL_POSE_RESULTS.md',report,'Zero native launches. Existing pinned OpenCV tag/PnP frontend passes manufactured image,pose,ambiguity and frame-chain controls;no actual tagged-object task or sensed controller is qualified.','Prepare static native tagged-Cube RGB controls with explicit face registry and held-out perturbations before any camera-feedback integration;retain unresolved released-object floor-impact modeling.',126,dict(fiducial_math_prepared=True,camera_perception_admitted=False,sensed_controller_admitted=False,full_m2_admitted=False))
