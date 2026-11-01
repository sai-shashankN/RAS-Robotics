import shutil,re
from fractions import Fraction
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    prior=STUDY.parent/'impact_probe_v1';assert (prior/'checkpoint-index.json').exists() and read(prior/'summary.json')['all_isolated_observations_valid'];assert not STUDY.exists();STUDY.mkdir();helpers=['impact_probe.c','observe_controller.py','analyze.py'];
    for n in helpers:shutil.copy2(prior/'source_snapshot'/n,SOURCE/n)
    shutil.copytree(prior/'source_snapshot/build',SOURCE/'build');text=(prior/'source_snapshot/run.py').read_text();assert text.count('==16')==2;(SOURCE/'run.py').write_text(text.replace('==16','==8'))
    write(STUDY/'test-plan.json',dict(question='Does normalized isolated impact pose convergence improve at.0625/.03125ms,using the same four recorded states and unchanged physical contact model?',matrix='8newshort native5s isolated Cube/floor observations,4recordedstates at.0625/.03125ms. Explicitly compare to the8earlier normalized .25/.125ms observations;no old trial reruns.2hidden workers maximum. Complete the diagnostic matrix regardless of expected pose rejection;stop on initial-state/infrastructure/native-validity/depth failure.',scope='Diagnostic refinement only. No articulated robot,controller or M2 mission qualification. No runtime pose/velocity/force writes. Preserve all old numerical rejections and original.5mm/.5degree limits.',physical_model='Floor contact800000N/m,800Ns/m;global structural40000000N/m,80000Ns/m. Convert ERP/CFM exactly from those constants and each step. Friction1,bounce.5,bounceVelocity.01,slip0 and all Cube/floor geometry,mass,inertia,initial states and5s observation period unchanged.',prior_index_sha256=sha(prior/'checkpoint-index.json'),assembly_deferred=True));entries=[]
    for old_e in read(prior/'manifest.json')['entries']:
        if old_e['model']!='normalized' or old_e['physics_ms']!=.25:continue
        old=ROOT/old_e['project']
        for h in [.0625,.03125]:
            name=f'{old_e["axis"]}_{old_e["seed_kind"]}_normalized_{str(h).replace(".","p")}';p=STUDY/'projects'/name;shutil.copytree(old,p,ignore=shutil.ignore_patterns('results','worlds','__pycache__'));(p/'results').mkdir();(p/'worlds').mkdir();cfg=read(old/'normalization.json');step=Fraction(str(h))/1000;gerp=float(step*40000000/(step*40000000+80000));gcfm=float(1/(step*40000000+80000));erp=float(step*800000/(step*800000+800));cfm=float(1/(step*800000+800));s=(old/'worlds/impact_0.25.wbt').read_text();s,n=re.subn(r'basicTimeStep\s+[^ ]+',f'basicTimeStep {h}',s);assert n==1
            for key,value in [('ERP',gerp),('CFM',gcfm),('softERP',erp),('softCFM',cfm)]:s,n=re.subn(r'(?<!\w)'+key+r'\s+[^ ]+',key+' '+format(value,'.17g'),s);assert n==1,key
            (p/'worlds'/f'impact_{h}.wbt').write_text(s);cfg.update(physics_ms=h,world_erp=gerp,world_cfm=gcfm,contact_erp=erp,contact_cfm=cfm,normalization_scope='isolated_floor_contact');write(p/'normalization.json',cfg);entries.append(dict(old_e,id=name,physics_ms=h,project=str(p.relative_to(ROOT)),runtime_source_project=old_e['project']))
    write(STUDY/'manifest.json',dict(entries=entries,plan_sha256=sha(STUDY/'test-plan.json')))
if __name__=='__main__':main()
