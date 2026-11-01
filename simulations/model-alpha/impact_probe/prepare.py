import csv,math,shutil,sys
from context import ROOT,SOURCE,STUDY,read,write,sha
sys.path.insert(0,str(ROOT/'.local-tools/plotting-packages'))
import numpy as np
def axisangle(R):
    R=np.array(R).reshape(3,3);angle=math.acos(max(-1.,min(1.,(np.trace(R)-1)/2)))
    if angle<1e-10:return [0.,0.,1.,0.]
    values,vectors=np.linalg.eig(R);axis=np.real(vectors[:,int(np.argmin(abs(values-1)))]);axis/=np.linalg.norm(axis);skew=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
    if np.dot(axis,skew)<0:axis=-axis
    return [*map(float,axis),float(angle)]
def main():
    diagnosis=STUDY.parent/'m2_drop_diagnosis_v1';assert (diagnosis/'checkpoint-index.json').exists() and not STUDY.exists();STUDY.mkdir();material=read(diagnosis/'material-audit.json');seeds=[]
    for r in read(diagnosis/'summary.json')['results']:
        for kind in ['coarse','fine']:
            out=ROOT/r[kind+'_directory']
            with (out/'telemetry.csv').open() as f:row=next(v for v in csv.DictReader(f) if abs(float(v['time_s'])-157.648)<1e-9)
            R=[float(row['object_r'+str(i)]) for i in range(9)];s=dict(axis=r['axis'],kind=kind,source_directory=str(out.relative_to(ROOT)),source_time_s=157.648,position=[float(row['object_'+k+'_m']) for k in 'xyz'],rotation=R,axisangle=axisangle(R),velocity=[float(row['object_v'+k+'_m_s']) for k in 'xyz']+[float(row['object_w'+k+'_rad_s']) for k in 'xyz'],source_sha256=sha(out/'telemetry.csv'));seeds.append(s)
    write(STUDY/'test-plan.json',dict(question='Does the missing floor-pair normalization explain common-state isolated impact divergence,or is sensitivity to initial state/step still present?',matrix='16short native5s isolated free-Cube/floor observations:axisA/B recorded coarse/fine pre-impact states,each initialized identically at.25/.125ms,each with default unmatched contacts or coarse-anchored normalized normal contacts.2hidden workers maximum. Complete diagnostic matrix regardless of expected convergence rejection;stop only on infrastructure,initial-state,mass/native-validity/finite/depth failure.',scope='Isolated impact diagnosis only. Initial state is declared in world files from an unsupported recorded snapshot;no runtime pose/velocity/force writes. No robot/controller or full M2 task is simulated,qualified or repaired here.',intervention='Only an explicit floor/test_object normal-contact pair. Preserve default friction1,bounce.5,bounceVelocity.01,slip0,max10contacts. Normalized800000N/m,800Ns/m exactly preserves coarseERP.2/CFM.001 and changes fine toERP1/9,CFM1/900. Other global structural normalization retained. The same initial state is used across compared steps and model variants.',gates='Verify actual native and Supervisor initial pose/velocity/mass/inertia,complete32/64native steps per8ms,valid finite active Cube,actual floor contact,<=1mm depth and top-facing contact normals. Compare full cube poses at common absolute control times with original.5mm/.5degree limits;retain failed comparisons. Check default-vs-normalized coarse full-state identity. No parameter search.',diagnosis_index_sha256=sha(diagnosis/'checkpoint-index.json'),defaults_node_sha256=material['runtime_node_sha256'],assembly_deferred=True));write(STUDY/'seeds.json',seeds);entries=[]
    for seed in seeds:
        for model in ['default','normalized']:
            for h in [.25,.125]:
                name=f'{seed["axis"]}_{seed["kind"]}_{model}_{str(h).replace(".","p")}';p=STUDY/'projects'/name;(p/'worlds').mkdir(parents=True);(p/'results').mkdir();ctr=p/'controllers/observe_controller';ctr.mkdir(parents=True);shutil.copy2(SOURCE/'observe_controller.py',ctr/'observe_controller.py');(ctr/'runtime.ini').write_text('[python]\nCOMMAND = C:/Python313/python.exe\n');plugin=p/'plugins/physics/impact_probe';plugin.mkdir(parents=True);shutil.copy2(SOURCE/'impact_probe.c',plugin/'impact_probe.c');normal=read(ROOT/seed['source_directory']/'sources/normalization.json') if h==(.25 if seed['kind']=='coarse' else .125) else read(STUDY.parent/'m2_pilot_v1/projects'/f'{seed["axis"]}_center_{str(h).replace(".","p")}'/'normalization.json');pair=''
                if model=='normalized':
                    v=next(v for v in material['candidate_anchor']['values'] if v['physics_ms']==h);pair=f'contactProperties [ ContactProperties {{ material1 "floor" material2 "test_object" softERP {v["softERP"]:.17g} softCFM {v["softCFM"]:.17g} }} ]'
                values=lambda a:' '.join(format(v,'.17g') for v in a)
                world=f'''#VRML_SIM R2025a utf8
WorldInfo {{ physics "impact_probe" coordinateSystem "ENU" basicTimeStep {h} ERP {normal['world_erp']:.17g} CFM {normal['world_cfm']:.17g} physicsDisableTime 1000000 randomSeed 0 {pair} }}
Viewpoint {{ position .5 -.5 .4 }}
Background {{ skyColor [ .9 .9 .9 ] }}
DEF TEST_FLOOR Solid {{ name "floor" translation 0 0 -.01 contactMaterial "floor" boundingObject Box {{ size 4 4 .02 }} }}
DEF TEST_OBJECT Solid {{ translation {values(seed['position'])} rotation {values(seed['axisangle'])} linearVelocity {values(seed['velocity'][:3])} angularVelocity {values(seed['velocity'][3:])} contactMaterial "test_object" boundingObject Box {{ size .02 .02 .02 }} physics Physics {{ density -1 mass .002 }} }}
Robot {{ supervisor TRUE controller "observe_controller" }}
'''
                (p/'worlds'/f'impact_{h}.wbt').write_text(world);write(p/'seed.json',seed);write(p/'normalization.json',normal);entries.append(dict(id=name,axis=seed['axis'],seed_kind=seed['kind'],model=model,physics_ms=h,project=str(p.relative_to(ROOT)),seed_sha256=sha(p/'seed.json')))
    write(STUDY/'manifest.json',dict(entries=entries,plan_sha256=sha(STUDY/'test-plan.json')))
if __name__=='__main__':main()
