"""Freeze verified A/B inputs and index original outputs without duplicating media."""
from pathlib import Path
import hashlib
import json
import shutil

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    reports = list((MODEL/'results/axis_contact_probe').glob('*/report.json'))
    reports += list((MODEL.parent/'model-alpha-b/results/axis_contact_probe').glob('*/report.json'))
    empty = json.loads((MODEL/'results/axis-comparison-summary.json').read_text())
    reports += [WORKSPACE/row['report_path'] for variant in empty['variants'].values() for row in variant['cases']]
    frozen = MODEL/'experiments/axis_contact_probe/source_snapshot'
    frozen.mkdir(parents=True,exist_ok=True)
    inputs,outputs = {},{}
    for path in reports:
        r = json.loads(path.read_text())
        outputs[str(path.relative_to(WORKSPACE))] = sha(path)
        for name,value in r['source_sha256'].items():
            source = WORKSPACE/name
            if sha(source)!=value:
                raise ValueError('Source changed before checkpoint: '+name)
            target = frozen/name
            target.parent.mkdir(parents=True,exist_ok=True)
            if target.exists() and sha(target)!=value:
                raise ValueError('Conflicting frozen source: '+name)
            shutil.copy2(source,target);inputs[name] = value
        for name in ['telemetry.csv','contact-trace.jsonl']:
            artifact = path.parent/name
            if artifact.exists():outputs[str(artifact.relative_to(WORKSPACE))] = sha(artifact)
        world = Path(r['world_path']) if r.get('world_path') else None
        if world:
            target = frozen/world.relative_to(WORKSPACE);target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(world,target);inputs[str(world.relative_to(WORKSPACE))] = sha(world)
    for name in ['prepare_alpha_b.py','run_empty_validation.py','audit_axis_variant.py','summarize_axis_comparison.py','run_axis_contact_probe.py','summarize_axis_contact_probe.py',Path(__file__).name]:
        source = MODEL/'scripts'/name;relative = str(source.relative_to(WORKSPACE))
        target = frozen/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target);inputs[relative] = sha(source)
    record = {'date':'2026-10-24','report_count':len(reports),'frozen_inputs':inputs,'original_output_hashes':outputs,
              'note':'Frozen workspace-relative input layout; original outputs remain in their A/B result folders. Original reports retain their paths and hashes. No success criterion is changed by this snapshot.'}
    (frozen.parent/'checkpoint-index.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    (frozen.parent/'README.md').write_text('''# A/B first contact checkpoint

The two fixed orientations each pass four geometry poses, thirteen empty
dynamics cases and six specified robot-clearance paths. The first 2 g sphere
probe retains/lifts with A but escapes B during closure. Both open controls
remain non-grasps. Neither positive probe demonstrates precise pole placement.

source_snapshot mirrors frozen workspace-relative sources/parameters/case
inputs and contact worlds. checkpoint-index.json records their hashes and
the original reports, CSV telemetry and contact traces in both model folders.
Failed B evidence is retained as failed. No hardware performance is claimed.
''',encoding='utf-8')
    print(json.dumps({'reports_indexed':len(reports),'frozen_input_files':len(inputs),'output_files_indexed':len(outputs)}))


if __name__=='__main__':
    main()
