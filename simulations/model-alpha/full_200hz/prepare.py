import ast,shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
from derivation import controller,reverse,world,reverse_world,reverse_native
def main():
    prior=STUDY.parent/'full_owned_pitch_v1';prefix=STUDY.parent/'owned_200hz_v1';raw=STUDY.parent/'alpha_200hz_v1';assert not STUDY.exists()
    for p,k in [(prior,'full_owned_pitch_admitted'),(prefix,'owned200_prefix_admitted'),(raw,'alpha200_prefix_admitted')]:assert(p/'checkpoint-index.json').exists() and read(p/'summary.json')[k]
    STUDY.mkdir();entries=[];prefix_by={r['entry']['reference_id']:prefix/r['directory'] for r in read(prefix/'summary.json')['results'] if r['entry']['variant']=='continuous'}
    for r in read(prior/'summary.json')['results']:
        e=r['entry'];old=prior/r['directory'];p=STUDY/'projects'/e['id'];shutil.copytree(old/'sources',p,ignore=shutil.ignore_patterns('__pycache__'));(p/'results').mkdir();ctr=p/'controllers/manip_controller/manip_controller.py';t=ctr.read_text();new=controller(t);assert reverse(new)==t;ast.parse(new);ctr.write_text(new,encoding='utf-8')
        for n in ['full200.py','sidecar200.py','observed200.py','host_consumer.py','history200.py','consumer200.py','projection200.py','buffer.py']:shutil.copy2(SOURCE/n,ctr.parent/n)
        w=p/e['world'];t=w.read_text();new=world(t);assert reverse_world(new)==t;w.write_text(new,encoding='utf-8');profile=read(p/'collision-fixtures.json');profile['world_sha256']=sha(w);write(p/'collision-fixtures.json',profile)
        source_plugin=prefix_by[e['id']]/'sources/plugins/physics/robot_cfm_guard';plugin=p/'plugins/physics/robot_cfm_guard'
        assert reverse_native((source_plugin/'robot_cfm_guard.c').read_text())==(plugin/'robot_cfm_guard.c').read_text()
        for n in ['robot_cfm_guard.c','raw200_native.inc','robot_cfm_guard.dll']:shutil.copy2(source_plugin/n,plugin/n)
        entries.append(dict(e,project=str(p.relative_to(ROOT)),variant='continuous',legacy_full_reference_output=str(old.relative_to(ROOT)),owned200_prefix_output=str(prefix_by[e['id']].relative_to(ROOT))))
    shutil.copy2(prior/'references.json',STUDY/'references.json');write(STUDY/'manifest.json',dict(entries=entries));shutil.copy2(ROOT/'project-notes/FULL_200HZ_NATIVE_PLAN.json',STUDY/'frozen-native-plan.json');shutil.copytree(SOURCE,STUDY/'prelaunch-source',ignore=shutil.ignore_patterns('__pycache__'))
    plan=read(prior/'test-plan.json');plan.update(source_indices={p.name:sha(p/'checkpoint-index.json') for p in [prior,prefix,raw]},new_native_trials_declared=4,explicit_old_native_trial_reuses=0,frozen_native_plan_sha256=sha(STUDY/'frozen-native-plan.json'),references_sha256=sha(STUDY/'references.json'),prelaunch_source_hashes={str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts},scope='Four new complete original prepared20mm/2gnominal200Hzpassive observation tasks;legacy control unchanged,no feedback/hardware.')
    plan['limits'].update(gyro200_reference_error_rad_s=1e-12,observer200_tilt_deg=.5,observer200_relative_yaw_deg=.5,observer200_roll_deg=.5,observer200_pitch_deg=.5,observer200_pitch_rate_rad_s=.001)
    write(STUDY/'test-plan.json',plan)
if __name__=='__main__':main()
