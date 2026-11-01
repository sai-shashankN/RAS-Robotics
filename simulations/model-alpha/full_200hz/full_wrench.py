import csv,json
from wrench_wire import FORMAT,KEYS,decode
def audit(out,physics_ms,control_ticks):
    with(out/'native-sent.csv').open() as f:sent=list(csv.DictReader(f))
    received=[json.loads(l) for l in(out/'wrench-received.jsonl').read_text().splitlines()];checks=dict(complete_deferred_wrench_pairs=len(sent)==len(received)==control_ticks-1,wire_fields_and_sequence=True,one_tick_measured_receive_age=True)
    for i,(s,r) in enumerate(zip(sent,received),1):
        p=decode(FORMAT.pack(*(float(s[k]) for k in KEYS)),round(8/physics_ms));checks['wire_fields_and_sequence'] &= p['sequence']==i and all(p[k]==r[k] for k in KEYS);checks['one_tick_measured_receive_age'] &= abs(p['end_s']-i*.008)<1e-9 and abs(r['received_s']-(i+1)*.008)<1e-9 and abs(r['age_s']-(r['received_s']-p['end_s']))<1e-9 and abs(r['age_s']-.008)<1e-9
    return dict(checks=checks,producer_rows=len(sent),receiver_rows=len(received),first_end_s=float(sent[0]['end_s']) if sent else None,last_end_s=float(sent[-1]['end_s']) if sent else None,last_received_s=received[-1]['received_s'] if received else None)
