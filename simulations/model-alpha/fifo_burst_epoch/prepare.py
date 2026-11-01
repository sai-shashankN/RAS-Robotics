import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    prior=STUDY.parent/'bmi_fifo_contract_v1';assert not STUDY.exists() and(prior/'checkpoint-index.json').exists() and read(prior/'summary.json')['fifo_header_source_contract_prepared'];STUDY.mkdir()
    shutil.copy2(ROOT/'project-notes/FIFO_BURST_EPOCH_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,new_hardware_calls_declared=0,prerequisite_index_sha256=sha(prior/'checkpoint-index.json'),prior_manufactured_corpus_sha256=sha(prior/'manufactured-differential-cases.jsonl'),frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},primary_reference=dict(url='https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf',revision='1.6',printed_pages=[35,36,37]),scope='Restricted grammar and manufactured owner-byte equality only;no board configuration/readback,SIscale,per-sampleclockorfeedback qualification.'))
if __name__=='__main__':main()
