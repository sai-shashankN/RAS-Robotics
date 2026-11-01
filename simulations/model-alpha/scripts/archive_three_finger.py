"""Freeze the superseded three-finger project before its two-finger revision."""
from pathlib import Path
import hashlib
import json
import shutil

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
DESTINATION = MODEL/'archive/three-finger-2026-10-24'


def main():
    if DESTINATION.exists():
        raise FileExistsError('Checkpoint already exists; do not overwrite it')
    DESTINATION.mkdir(parents=True)
    shutil.copytree(MODEL.parent/'common',DESTINATION/'common',ignore=shutil.ignore_patterns('__pycache__'))
    frozen_model = DESTINATION/'model-alpha'
    frozen_model.mkdir()
    for name in ['scripts','controllers']:
        shutil.copytree(MODEL/name,frozen_model/name,ignore=shutil.ignore_patterns('__pycache__'))
    for path in MODEL.iterdir():
        if path.is_file():
            shutil.copy2(path,frozen_model/path.name)
    shutil.copytree(WORKSPACE/'project-notes',DESTINATION/'project-notes')
    shutil.copytree(MODEL.parent/'model-beta',DESTINATION/'model-beta',ignore=shutil.ignore_patterns('__pycache__'))
    # Check both absolute targets before moving complete evidence directories.
    for name in ['results','worlds','experiments']:
        source,target = (MODEL/name).resolve(),(frozen_model/name).resolve()
        if not source.is_relative_to(WORKSPACE.resolve()) or not target.is_relative_to(DESTINATION.resolve()):
            raise ValueError('Archive path leaves intended workspace/checkpoint')
        shutil.move(str(source),str(target))
        source.mkdir()
    hashes = {}
    for path in DESTINATION.rglob('*'):
        if path.is_file() and path.suffix in ['.py','.json','.wbt','.md','.ps1']:
            hashes[str(path.relative_to(DESTINATION))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (DESTINATION/'checkpoint-sha256.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')
    (DESTINATION/'README.md').write_text('''# Superseded three-finger checkpoint — 24 October 2026

The user replaced the three radial fingers with two opposing curling fingers.
Alpha has a fixed X-direction pinch; Beta/X additionally plan wrist/claw roll.
This checkpoint preserves all active sources, worlds, inputs, results and notebook
before that change. Prior archives remain at their existing sibling locations.

The return study completed five positive return/fold cycles, four expected
failure/refusal controls and four matched stationary controls. All 28 preceding
prepared/folded regression reports were revalidated. These show the old simulated
tasks only, not precise pickup or placement and not the new two-finger claw.

The user's contact concern was confirmed: closing alone displaced the reference
sphere about -5.88 mm in X and +17.44 mm in Z; opening began 17.55 mm above the
pole. Instrumentation matched sphere contact points to distal finger segments.
A calibrated supported-setdown trial still failed with 16 unintended TIP_0
contact samples and the ball ended in the tray. It is not a successful fix.
Its controller uses ideal simulator support information and a pose fitted to
the prior 2 g seating; it is explicitly diagnostic and not general sensing.

Results/contact_diagnostic contains the 8 ms contact traces and diagnostic summary.
Original reports retain original absolute world paths and workspace-relative
source keys. Resolve simulations/common under common here, and
simulations/model-alpha under model-alpha here to inspect their frozen inputs.
No previous grasp result transfers as validation of the replacement claw.
''',encoding='utf-8')
    print(str(DESTINATION))


if __name__=='__main__':
    main()
