import csv,json,hashlib,math
from wrench_wire import FORMAT,KEYS,decode
def reference(path,physics_ms,old_sha):
    name=path.name;assert name in ['native-sent.csv','wrench-received.jsonl'];whole=hashlib.sha256();prefix=hashlib.sha256();size=0;count=0;first=last=None;is_csv=name.endswith('.csv')
    with path.open('rb') as f:
        if is_csv:header=f.readline();whole.update(header);prefix.update(header);size+=len(header);fields=next(csv.reader([header.decode().rstrip('\r\n')]))
        for line in f:
            whole.update(line)
            if is_csv:values=next(csv.reader([line.decode().rstrip('\r\n')]));assert len(values)==len(fields);row=dict(zip(fields,values));t=float(row['end_s']);keep=t<60.-1e-9
            else:row=json.loads(line);t=row['received_s'];keep=t<=60.+1e-9
            if not keep:
                assert count==7499
                for chunk in iter(lambda:f.read(1024*1024),b''):whole.update(chunk)
                break
            assert count<7499 and abs(t-(count+(1 if is_csv else 2))*.008)<1e-9
            decoded=decode(FORMAT.pack(*(float(row[k]) for k in KEYS)),round(8/physics_ms));assert decoded['sequence']==count+1
            if not is_csv:assert abs(row['received_s']-row['end_s']-.008)<1e-9
            prefix.update(line);size+=len(line);count+=1;first=t if first is None else first;last=t
        assert count==7499 and whole.hexdigest()==old_sha
    return dict(sha256=prefix.hexdigest(),bytes=size,rows=count,first_s=first,last_s=last,source_full_sha256=whole.hexdigest(),contract='native callback emission strictly before60' if is_csv else 'controller receive clock through60')
def audit(out,physics_ms):
    with(out/'native-sent.csv').open() as f:sent=list(csv.DictReader(f))
    received=[json.loads(l) for l in(out/'wrench-received.jsonl').read_text().splitlines()];checks=dict(complete_deferred_wrench_pairs=len(sent)==len(received)==7499,wire_fields_and_sequence=True,one_tick_measured_receive_age=True)
    for i,(s,r) in enumerate(zip(sent,received),1):
        p=decode(FORMAT.pack(*(float(s[k]) for k in KEYS)),round(8/physics_ms));checks['wire_fields_and_sequence'] &= p['sequence']==i and all(p[k]==r[k] for k in KEYS);checks['one_tick_measured_receive_age'] &= abs(p['end_s']-i*.008)<1e-9 and abs(r['received_s']-(i+1)*.008)<1e-9 and abs(r['age_s']-(r['received_s']-p['end_s']))<1e-9 and abs(r['age_s']-.008)<1e-9
    return dict(checks=checks,producer_rows=len(sent),receiver_rows=len(received),first_end_s=float(sent[0]['end_s']) if sent else None,last_end_s=float(sent[-1]['end_s']) if sent else None,last_received_s=received[-1]['received_s'] if received else None)
