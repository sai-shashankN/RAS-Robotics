import csv,math
from context import ROOT,SOURCE,STUDY,read,write,sha
def rows(p):
    with p.open() as f:return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
def compare(a,b):
    da,db=rows(a/'telemetry.csv'),rows(b/'telemetry.csv');assert len(da)==len(db)==625;assert all(abs(x['time_s']-y['time_s'])<1e-9 for x,y in zip(da,db));position=max(math.dist([x['p'+str(i)] for i in range(3)],[y['p'+str(i)] for i in range(3)]) for x,y in zip(da,db));rotation=max(math.acos(max(-1.,min(1.,(sum(x['r'+str(i)]*y['r'+str(i)] for i in range(9))-1)/2))) for x,y in zip(da,db));return dict(max_position_difference_m=position,max_rotation_difference_rad=rotation,passed=position<=.0005 and rotation<=math.radians(.5))
def main():
    assert not (STUDY/'checkpoint-index.json').exists();s=read(STUDY/'summary.json');results=s['results'];assert len(results)==16 and s['all_isolated_observations_valid'];out=lambda r:STUDY/r['directory'];by=lambda a,k,m,h:next(r for r in results if r['entry']['axis']==a and r['entry']['seed_kind']==k and r['entry']['model']==m and r['entry']['physics_ms']==h);paired=[];coarse=[];sensitivity=[]
    for axis in 'ab':
        for seed in ['coarse','fine']:
            for model in ['default','normalized']:
                a,b=by(axis,seed,model,.25),by(axis,seed,model,.125);v=compare(out(a),out(b));paired.append(dict(axis=axis,seed=seed,model=model,coarse_directory=a['directory'],fine_directory=b['directory'],**v));print(dict(type='step_pair',axis=axis,seed=seed,model=model,**v),flush=True)
            a,b=by(axis,seed,'default',.25),by(axis,seed,'normalized',.25);coarse.append(dict(axis=axis,seed=seed,telemetry_exact=sha(out(a)/'telemetry.csv')==sha(out(b)/'telemetry.csv'),native_motion_exact=sha(out(a)/'native-motion.csv')==sha(out(b)/'native-motion.csv'),native_contacts_exact=sha(out(a)/'native-contacts.csv')==sha(out(b)/'native-contacts.csv'),**compare(out(a),out(b))))
        for model in ['default','normalized']:
            for h in [.25,.125]:sensitivity.append(dict(axis=axis,model=model,physics_ms=h,comparison_kind='different_initial_states_sensitivity_not_convergence',**compare(out(by(axis,'coarse',model,h)),out(by(axis,'fine',model,h)))))
    write(STUDY/'comparison-results.json',dict(step_pairs=paired,coarse_anchor_checks=coarse,initial_state_sensitivity=sensitivity));s.update(comparison_results_sha256=sha(STUDY/'comparison-results.json'),coarse_anchor_exact=all(v['telemetry_exact'] and v['native_motion_exact'] and v['native_contacts_exact'] for v in coarse),normalized_common_state_pose_convergence=all(v['passed'] for v in paired if v['model']=='normalized'),default_common_state_pose_convergence=all(v['passed'] for v in paired if v['model']=='default'),full_m2_admitted=False);write(STUDY/'summary.json',s)
if __name__=='__main__':main()
