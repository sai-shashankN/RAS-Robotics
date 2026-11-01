import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    prior=STUDY.parent/'counter_phase_v1';assert not STUDY.exists() and(prior/'checkpoint-index.json').exists();STUDY.mkdir()
    shutil.copy2(ROOT/'project-notes/COUNT_SI_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,new_hardware_calls_declared=0,prerequisite_index_sha256=sha(prior/'checkpoint-index.json'),frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},scope='Declared selected filtered FIFO range semantics and ideal range/32768 nearest-even/clip model only;no calibrated sensor or acquisition qualification.'))
if __name__=='__main__':main()
