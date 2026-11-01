import csv,json
from context import ROOT,SOURCE,STUDY,read,write,sha
from fixture_sensor import decode,FORMAT,KEYS
EXACT=['contacts.jsonl','native-motion.csv','native-init.csv','native-properties.csv','native-drive.csv','native-actuator-mode.csv','native-contacts.csv','native-fixture-contacts.csv']
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def packet_checks(out,h):
    sent=rows(out/'native-fixture-sent.csv');faces=rows(out/'native-fixture-contacts.csv');native=rows(out/'native-contacts.csv');received=[json.loads(s) for s in (out/'fixture-received.jsonl').read_text().splitlines()];checks=dict(nonempty=bool(sent),equal_coverage=len(sent)==len(faces)==len(native)==len(received),exact_current_delivery=True,sent_native_reconciliation=True,packet_contract=True)
    maximum_age=0.
    for i,(s,f,a,r) in enumerate(zip(sent,faces,native,received),1):
        p={k:float(v) for k,v in s.items()}
        try:d=decode(FORMAT.pack(*(p[k] for k in KEYS)),h)
        except ValueError:checks['packet_contract']=False;continue
        checks['packet_contract'] &= d['sequence']==i and abs(d['end_s']-i*.008)<1e-9
        checks['exact_current_delivery'] &= {k:v for k,v in r.items() if k!='received_s'}==d and abs(r['received_s']-d['end_s'])<1e-9
        maximum_age=max(maximum_age,abs(r['received_s']-d['end_s']))
        fields=['native_steps','valid','grip_cube_points','cube_fixture_points','cube_floor_points','forbidden_points','unknown_points','overflow','min_grip_fingers','bilateral_native_steps','max_cube_static_depth_m','max_forbidden_depth_m']
        checks['sent_native_reconciliation'] &= all(p[k]==float(a[k]) for k in fields) and abs(p['end_s']-float(a['window_end_s']))<1e-9 and all(p[k]==float(f[k]) for k in ['native_steps','active_fixtures']+list(KEYS[18:]))
    return dict(checks=checks,windows=len(sent),maximum_delivery_age_s=maximum_age,passed=all(checks.values()))
def assess(e,out):
    gold=ROOT/e['golden_directory'];checks={};evidence=packet_checks(out,e['physics_ms']);checks.update(evidence['checks'])
    if e['kind_group']=='probe':
        prior=read(gold/'audit.json');assert prior['passed'];a=rows(out/'native-contacts.csv');f=rows(out/'native-fixture-contacts.csv');checks['one_native_window']=len(f)==len(a)==1
        checks.update({'exact_'+n:sha(out/n)==sha(gold/n) for n in ['extraction.json','native-contact-init.csv']+EXACT if (gold/n).exists()})
        checks['expected_target_positive']=int(f[0][f'fixture_{e["fixture"]}_points'])>0 if e['fixture'] is not None else int(a[0]['cube_floor_points' if e['kind']=='floor' else 'forbidden_points'])>0
        if e['fixture'] is not None:checks['expected_face']=int(f[0][f'fixture_{e["fixture"]}_'+('other' if e['kind']=='top' else 'top')+'_points'])==0
    else:
        checks.update({'exact_'+n:sha(out/n)==sha(gold/n) for n in EXACT+['telemetry.csv','wrench-received.jsonl']})
        report=read(out/'report.json');old=read(gold/'report.json');report.pop('wall_seconds');old.pop('wall_seconds');checks['physical_report_exact']=report==old and report['controller_success']
    checks['stderr_empty']=(out/'webots-stderr.log').stat().st_size==0
    launch=read(out/'launch-inputs.json');checks['source_snapshot_intact']=launch['entry']==e and launch['plan_sha256']==sha(STUDY/'test-plan.json') and all(sha(out/'sources'/n)==v for n,v in launch['sha256'].items())
    names=['native-fixture-sent.csv','native-fixture-contacts.csv','native-contacts.csv','fixture-received.jsonl','launch-inputs.json']
    result=dict(entry=e,directory=str(out.relative_to(STUDY)),passed=all(checks.values()),checks=checks,live=evidence,input_sha256={n:sha(out/n) for n in names})
    write(out/'audit.json',result);return result
