from pathlib import Path
import csv,json,os,traceback
from controller import Supervisor
ROOT=Path(__file__).resolve().parents[2];OUT=Path(os.environ['ROBOT_AUDIT_DIR']);robot=Supervisor();cube=robot.getFromDef('TEST_OBJECT')
def state():return dict(time_s=robot.getTime(),position=list(cube.getPosition()),rotation=list(cube.getOrientation()),velocity=list(cube.getVelocity()))
try:
    initial=state();rows=[]
    with (OUT/'telemetry.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s']+[k+str(i) for k,n in [('p',3),('r',9),('v',6)] for i in range(n)])
        while robot.step(8)!=-1:
            s=state();writer.writerow([s['time_s'],*s['position'],*s['rotation'],*s['velocity']]);rows.append(s)
            if robot.getTime()>=5:break
    (OUT/'report.json').write_text(json.dumps(dict(initial=initial,final=state(),cube_free=cube.getField('physics').getSFNode() is not None,observations=len(rows)),indent=2));robot.simulationQuit(0)
except Exception:
    (OUT/'controller-error.txt').write_text(traceback.format_exc());robot.simulationQuit(2)
