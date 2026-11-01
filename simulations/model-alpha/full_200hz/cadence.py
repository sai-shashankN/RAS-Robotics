"""Pinned logger schedules and compact exact-byte references,not a robot sensor."""
import csv,json,hashlib,math
STATIC={'native-contact-init.csv','native-contact-primitives.csv','native-init.csv','native-joint-properties.csv','native-properties.csv','observer-init.csv'}
CONTROL={'native-actuator-mode.csv':('slot',9),'native-drive.csv':('link_id',10),'native-motion.csv':(None,1)}
DENSE={'native-joints.csv':('slot',9),'native-links.csv':('link_id',10)}
END={'native-contacts.csv':'window_end_s','native-fixture-contacts.csv':'window_end_s','native-fixture-sent.csv':'end_s','native-sent.csv':'end_s','telemetry.csv':'time_s'}
JSON_END={'contacts.jsonl':'time_s','fixture-received.jsonl':'end_s','wrench-received.jsonl':'end_s','destination-reference.jsonl':'time_s'}
NAMES=STATIC|set(CONTROL)|set(DENSE)|set(END)|set(JSON_END)|{'native-wrench.csv','native-forbidden-contacts.csv'}
DENSE_US=[(0,200000),(9900000,10100000),(22300000,22900000),(45100000,45600000)]
WRENCH_US=[(18000000,23500000),(29500000,31000000),(44500000,46500000)]
def schedule(name,physics_ms):
    step=round(physics_ms*1000);assert step in [125,250]
    if name in DENSE:
        clocks=set(range(0,60000000,8000))
        for lo,hi in DENSE_US:clocks.update(range(lo,hi+1,step))
        slot,n=DENSE[name]
    elif name=='native-wrench.csv':clocks=[t for lo,hi in WRENCH_US for t in range(lo,hi+1,step)];slot,n=None,1
    elif name in CONTROL:clocks=range(0,60000000,8000);slot,n=CONTROL[name]
    elif name in END or name in JSON_END:
        clocks=[] if name=='destination-reference.jsonl' else range(8000,60000001,8000);slot,n=None,1
    else:assert name=='native-forbidden-contacts.csv';clocks=[];slot,n=None,1
    for t in sorted(clocks):
        for i in range(n):yield t,slot,i

def digest(path,physics_ms,expected_whole_sha=None):
    """Validate full selected cadence;bind full old file while saving only a digest.

    Old references may contain later records. Native outputs must end exactly at
    the declared prefix digest;the evaluator compares their complete byte hash.
    """
    name=path.name;assert name in NAMES;whole=hashlib.sha256();prefix=hashlib.sha256();size=0;rows=0;first=last=None
    if name in STATIC:
        data=path.read_bytes();whole.update(data);prefix.update(data);size=len(data);rows=max(0,len(data.splitlines())-1)
    else:
        sequence=iter(schedule(name,physics_ms));expected=next(sequence,None)
        with path.open('rb') as f:
            if path.suffix=='.csv':
                header=f.readline();whole.update(header);prefix.update(header);size+=len(header);fields=next(csv.reader([header.decode('utf-8-sig').rstrip('\r\n')]));assert len(fields)==len(set(fields))
            for line in f:
                whole.update(line)
                if path.suffix=='.csv':
                    values=next(csv.reader([line.decode('utf-8').rstrip('\r\n')]));assert len(values)==len(fields);row=dict(zip(fields,values));key=END.get(name,'time_s')
                else:row=json.loads(line);key=JSON_END[name]
                t=float(row[key]);assert math.isfinite(t)
                keep=t<=60.+1e-9 if name in END or name in JSON_END else t<60.-1e-9
                if not keep:
                    assert expected is None,(name,'missing declared records',expected)
                    for chunk in iter(lambda:f.read(1024*1024),b''):whole.update(chunk)
                    break
                assert expected is not None,(name,'unexpected record',t)
                us,slot,i=expected;assert abs(t-us*1e-6)<1e-9,(name,t,us)
                if slot is not None:assert int(row[slot])==i,(name,t,slot,row[slot],i)
                if name.startswith('native-'):assert all(math.isfinite(float(v)) for v in row.values()),name
                prefix.update(line);size+=len(line);rows+=1;first=t if first is None else first;last=t;expected=next(sequence,None)
            assert expected is None,(name,'incomplete cadence',expected)
    if expected_whole_sha is not None:assert whole.hexdigest()==expected_whole_sha,(path,'old source differs from sealed file')
    return dict(sha256=prefix.hexdigest(),bytes=size,rows=rows,first_s=first,last_s=last,source_full_sha256=whole.hexdigest())
