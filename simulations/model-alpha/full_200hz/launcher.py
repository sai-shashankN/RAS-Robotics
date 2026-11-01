from pathlib import Path
import os,shutil,subprocess,time,traceback
from context import ROOT,STUDY,read,write,sha
from evaluate import assess
def one(e):
    p=ROOT/e['project'];out=p/'results/raw_full';out.mkdir();hashes={}
    for f in p.rglob('*'):
        rel=f.relative_to(p)
        if not f.is_file() or 'results' in rel.parts or '__pycache__' in rel.parts:continue
        dest=out/'sources'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest);hashes[str(rel)]=sha(dest)
    shutil.copy2(STUDY/'preflight.json',out/'preflight-before-launch.json');write(out/'launch-inputs.json',dict(entry=e,case='nominal',sha256=hashes,plan_sha256=sha(STUDY/'test-plan.json'),preflight_sha256=sha(out/'preflight-before-launch.json')));install=ROOT/'.local-tools/webots';binary=install/'msys64/mingw64/bin';system=Path(os.environ['SystemRoot']);env=dict(os.environ,WEBOTS_HOME=str(install),M1_RUN='raw_full',M1_CASE='nominal',M1_CAPTURE='0',ROBOT_AUDIT_DIR=str(out),ROBOT_PHYSICS_MS=str(e['physics_ms']),ROBOT_CONTACT_AUDIT='1',ROBOT_FIXTURE_COUNT='8',ROBOT_FIXTURE_LIVE='1',LOAD_TRANSFER_MODE='both');env['PATH']=';'.join(map(str,[binary,binary/'cpp',install/'msys64/usr/bin',system/'System32',system,Path('C:/Python313')]));startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW;startup.wShowWindow=subprocess.SW_HIDE;started=time.perf_counter()
    try:
        with(out/'webots-stdout.log').open('w') as a,(out/'webots-stderr.log').open('w') as b:r=subprocess.run([str(binary/'webots.exe'),'--batch','--mode=fast','--no-rendering','--stdout','--stderr',str(p/e['world'])],cwd=p,env=env,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW,stdout=a,stderr=b,timeout=2400)
        write(out/'process.json',dict(returncode=r.returncode,wall_seconds=time.perf_counter()-started));assert r.returncode==0;result=assess(e,out)
    except Exception:result=dict(entry=e,directory=str(out.relative_to(STUDY)),passed=False,checks=dict(infrastructure_or_evaluation=False),error=traceback.format_exc());write(out/'infrastructure-error.json',result)
    print(dict(id=e['id'],passed=result['passed'],failed=[k for k,v in result['checks'].items() if not v]),flush=True);return result
