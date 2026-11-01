import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    prior=STUDY.parent/'full_200hz_v1';assert not STUDY.exists() and(prior/'checkpoint-index.json').exists() and read(prior/'summary.json')['full200_nominal_observation_admitted'];STUDY.mkdir()
    shutil.copy2(ROOT/'project-notes/BMI_FIFO_CONTRACT_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,hardware_calls_declared=0,prerequisite_index_sha256=sha(prior/'checkpoint-index.json'),frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},vendor_git_blobs={v['name']:v['git_blob_sha'] for v in read(SOURCE/'vendor/sources.json')['files']}))
if __name__=='__main__':main()
