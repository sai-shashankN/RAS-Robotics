"""Capture static sandbox views. Recognition is ground truth, not perception."""
import json,os
from pathlib import Path
from controller import Supervisor
root=Path(os.environ['CAMERA_AUDIT_DIR']);r=Supervisor();spec=json.loads((root/'sources/scene.json').read_text());result=[]
for c in spec['cameras']:
    cam=r.getDevice(c['id']);cam.enable(8);cam.recognitionEnable(8);cam.enableRecognitionSegmentation()
for _ in range(3):
    if r.step(8)==-1:raise RuntimeError('Camera sandbox exited before capture')
for c in spec['cameras']:
    cam=r.getDevice(c['id']);rgb=cam.getImage();seg=cam.getRecognitionSegmentationImage();assert len(rgb)==len(seg)==640*480*4
    (root/(c['id']+'-rgb.bgra')).write_bytes(rgb);(root/(c['id']+'-mask.bgra')).write_bytes(seg)
    assert cam.saveImage(str(root/(c['id']+'-rgb.png')),100)==0
    assert cam.saveRecognitionSegmentationImage(str(root/(c['id']+'-mask.png')),100)==0
    result.append(dict(camera=c['id'],width=cam.getWidth(),height=cam.getHeight(),fov=cam.getFov(),near=cam.getNear(),ground_truth_objects=len(cam.getRecognitionObjects())))
(root/'capture.json').write_text(json.dumps(dict(static_render_sandbox=True,time_s=r.getTime(),views=result,robot_dynamic_trials=0,perception_admitted=False),indent=2)+'\n')
r.simulationQuit(0)
