from pathlib import Path
import json,os
from controller import Supervisor

ROOT=Path(__file__).resolve().parents[2];OUT=Path(os.environ['ROBOT_AUDIT_DIR']);C=json.loads((ROOT/'case.json').read_text());robot=Supervisor();names=['shoulder','elbow','wrist','finger_0','distal_0','finger_1','distal_1'];nodes={n:robot.getFromDef(n) for n in ['ROBOT_MODEL','ARM_BASE','FOREARM','PALM','FINGER_0','FINGER_1','TIP_0','TIP_1']};initial={n:dict(position=list(v.getPosition()),rotation=list(v.getOrientation())) for n,v in nodes.items()}
for n in names:
    m=robot.getDevice(n);m.setVelocity(.8 if n in ['shoulder','elbow','wrist'] else 1.5)
    if n.startswith(('finger','distal')):m.setAvailableTorque(.006)
    m.setPosition(C['initial_pose'][n.split('_')[0]])
for n in ['left_wheel','right_wheel']:
    m=robot.getDevice(n);m.setPosition(float('inf'));m.setVelocity(0.);m.setTorque(0.)
robot.step(8)
(OUT/'extraction.json').write_text(json.dumps(dict(initial_node_frames=initial,duration_s=robot.getTime(),physical_root=robot.getSelf().getField('physics').getSFNode() is not None,scope='Cold native mass/inertia/frame extraction only; not a motion/grasp/recovery trial.'),indent=2))
robot.simulationQuit(0)
