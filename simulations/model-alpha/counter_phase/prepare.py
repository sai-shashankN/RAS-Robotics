import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    prior=STUDY.parent/'fifo_burst_epoch_v1';assert not STUDY.exists() and(prior/'checkpoint-index.json').exists() and read(prior/'summary.json')['fifo_burst_epoch_source_contract_prepared'];STUDY.mkdir()
    shutil.copy2(ROOT/'project-notes/COUNTER_PHASE_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'));write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,new_hardware_calls_declared=0,prerequisite_index_sha256=sha(prior/'checkpoint-index.json'),frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},scope='Manufactured fixed-period/no-loss/bounded-gap conditional output-event phase family only;no acquisition or physical clock qualification.'))
if __name__=='__main__':main()
