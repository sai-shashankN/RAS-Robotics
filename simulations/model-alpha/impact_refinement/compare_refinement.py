from context import ROOT,SOURCE,STUDY,read,write,sha
from analyze import compare
def main():
    s=read(STUDY/'summary.json');assert len(s['results'])==8 and s['all_isolated_observations_valid'];prior=STUDY.parent/'impact_probe_v1';old=read(prior/'summary.json')['results'];new=s['results'];paired=[];states=[]
    def record(axis,seed,h):
        if h in [.25,.125]:r=next(r for r in old if r['entry']['axis']==axis and r['entry']['seed_kind']==seed and r['entry']['model']=='normalized' and r['entry']['physics_ms']==h);return prior/r['directory']
        r=next(r for r in new if r['entry']['axis']==axis and r['entry']['seed_kind']==seed and r['entry']['physics_ms']==h);return STUDY/r['directory']
    for axis in 'ab':
        for seed in ['coarse','fine']:
            for a,b in [(.25,.125),(.125,.0625),(.0625,.03125)]:
                x,y=record(axis,seed,a),record(axis,seed,b);v=compare(x,y);paired.append(dict(axis=axis,seed=seed,coarse_ms=a,fine_ms=b,coarse_directory=str(x.relative_to(ROOT)),fine_directory=str(y.relative_to(ROOT)),**v));print(dict(axis=axis,seed=seed,steps=(a,b),**v),flush=True)
        for h in [.25,.125,.0625,.03125]:states.append(dict(axis=axis,physics_ms=h,comparison_kind='different_initial_states_sensitivity_not_convergence',**compare(record(axis,'coarse',h),record(axis,'fine',h))))
    write(STUDY/'refinement-results.json',dict(step_pairs=paired,initial_state_sensitivity=states));s.update(refinement_results_sha256=sha(STUDY/'refinement-results.json'),finest_pair_pose_convergence=all(r['passed'] for r in paired if r['fine_ms']==.03125),full_m2_admitted=False);write(STUDY/'summary.json',s)
if __name__=='__main__':main()
