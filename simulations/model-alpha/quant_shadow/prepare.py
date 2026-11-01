import shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    assert not STUDY.exists();prior=STUDY.parent/'count_replay_v1';native=STUDY.parent/'full_200hz_v1';assert(prior/'checkpoint-index.json').exists();STUDY.mkdir();entries=[]
    for r in read(prior/'summary.json')['results']:
        directory=str((ROOT/r['entry']['input']).parent.relative_to(ROOT));entries.append(dict(id=r['entry']['id'],native_directory=directory,counts_path=str((prior/r['saved_counts']).relative_to(ROOT)),expected_rows=r['producer_rows']))
    write(STUDY/'manifest.json',dict(entries=entries));shutil.copy2(ROOT/'project-notes/QUANT_SHADOW_PLAN.json',STUDY/'frozen-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    old_limits=read(native/'test-plan.json')['limits'];keys=['tilt_deg','relative_yaw_deg','roll_deg','pitch_deg','pitch_rate_rad_s'];limits={k:old_limits['observer200_'+k] for k in keys};inputs={}
    for e in entries:
        paths=[ROOT/e['native_directory']/n for n in ['raw200-producer.jsonl','raw200-reference.jsonl','raw200-host-observer.jsonl','full200-audit.json']]+[ROOT/e['counts_path']];inputs[e['id']]={str(p.relative_to(ROOT)):sha(p) for p in paths}
    write(STUDY/'test-plan.json',dict(new_native_trials_declared=0,new_hardware_calls_declared=0,source_indices={n:sha(STUDY.parent/n/'checkpoint-index.json') for n in ['full_200hz_v1','count_si_v1','count_replay_v1']},frozen_plan_sha256=sha(STUDY/'frozen-plan.json'),input_sha256=inputs,original_observer_limits=limits,prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}))
if __name__=='__main__':main()
