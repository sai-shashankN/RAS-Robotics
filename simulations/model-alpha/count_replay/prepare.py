import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    assert not STUDY.exists();prior=STUDY.parent/'count_si_v1';native=STUDY.parent/'full_200hz_v1';assert(prior/'checkpoint-index.json').exists();STUDY.mkdir();entries=[]
    for r in read(native/'summary.json')['results']:
        key=r['entry']['id'];relative=str((native/r['directory']/'raw200-producer.jsonl').relative_to(ROOT));expected=67876 if key.startswith('a_') else 67873
        entries.append(dict(id=key,input=relative,expected_rows=expected,source_checkpoint=226,prior_physical_case_passed=r['passed']))
    assert len(entries)==4;write(STUDY/'manifest.json',dict(entries=entries));shutil.copy2(ROOT/'project-notes/COUNT_REPLAY_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,new_hardware_calls_declared=0,prerequisite_index_sha256=sha(prior/'checkpoint-index.json'),native_source_index_sha256=sha(native/'checkpoint-index.json'),frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),input_sha256={e['id']:sha(ROOT/e['input']) for e in entries},prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}))
if __name__=='__main__':main()
