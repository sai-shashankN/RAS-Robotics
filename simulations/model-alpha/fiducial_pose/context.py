from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[3];SOURCE=Path(__file__).resolve().parent
STUDY=ROOT/'simulations/model-alpha/experiments/fiducial_pose_v1'
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n',encoding='utf-8')
