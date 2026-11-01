"""Materialize the fixed Y claw without changing the verified Alpha-A baseline."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
TARGET = MODEL.parent/'model-alpha-b'


def main():
    baseline = MODEL/'parameters.json'
    manifest = json.loads((MODEL/'variants/axis-comparison.json').read_text())
    sha = hashlib.sha256(baseline.read_bytes()).hexdigest()
    if sha!=manifest['baseline_sha256']:
        raise ValueError('Comparison baseline changed; review the manifest first')
    p = json.loads(baseline.read_text())
    b = dict(p,model='Alpha-B',finger_angles_rad=manifest['variants']['Alpha-B']['finger_angles_rad'],
             gripper_design='Two opposing jointed curling fingers on palm-local +/-Y; left/right pinch in reference grasp posture')
    changed = sorted(key for key in p if p[key]!=b[key])
    if changed!=['finger_angles_rad','gripper_design','model']:
        raise ValueError('Unexpected non-axis parameter change')
    TARGET.mkdir(exist_ok=True)
    target_parameters = TARGET/'parameters.json'
    if target_parameters.exists() and json.loads(target_parameters.read_text())!=b:
        raise ValueError('Existing Alpha-B parameters differ; preserve/review them before replacing')
    target_parameters.write_text(json.dumps(b,indent=2)+'\n',encoding='utf-8')
    shutil.copyfile(MODEL/'launch.ps1',TARGET/'launch.ps1')
    scripts = TARGET/'scripts'
    scripts.mkdir(exist_ok=True)
    (scripts/'generate_worlds.py').write_text('''from pathlib import Path
import runpy
import sys
root = Path(__file__).resolve().parents[1]
sys.argv = [sys.argv[0], str(root)]
runpy.run_path(str(root.parent / 'common' / 'generate_worlds.py'), run_name='__main__')
''',encoding='utf-8')
    comparison = {'baseline_parameters':str(baseline.relative_to(WORKSPACE)),
                  'baseline_sha256':sha,'variant_parameters':str(target_parameters.relative_to(WORKSPACE)),
                  'variant_sha256':hashlib.sha256(target_parameters.read_bytes()).hexdigest(),
                  'different_keys':changed,'only_physical_input_changed':'finger_angles_rad',
                  'finger_angles_A_rad':p['finger_angles_rad'],'finger_angles_B_rad':b['finger_angles_rad'],
                  'notes':'Same circular palm, body/link geometry, mass/COM, motor caps, gains and joint targets. No powered roll.'}
    (TARGET/'comparison-setup.json').write_text(json.dumps(comparison,indent=2)+'\n',encoding='utf-8')
    subprocess.run([sys.executable,str(scripts/'generate_worlds.py')],check=True)
    print(json.dumps(comparison),flush=True)


if __name__=='__main__':
    main()
