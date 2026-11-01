from context import ROOT,STUDY,read,write,sha
from legacy_full_reference import assess as original
from full_reference import compare
from metrics200 import audit as raw_audit
from observer_audit import audit as observer_audit
def prefix_equal(current,old):
    with current.open('rb') as a,old.open('rb') as b:
        for chunk in iter(lambda:b.read(1024*1024),b''):
            if a.read(len(chunk))!=chunk:return False
    return True
def assess(e,out):
    r=original(e,out);old=ROOT/e['legacy_full_reference_output'];prefix=ROOT/e['owned200_prefix_output']
    same={n:sha(out/n)==sha(old/n) for n in ['raw-imu.jsonl','raw-imu-reference.jsonl','imu-shadow.jsonl','imu-device-periods.jsonl','imu-device-actions.jsonl','imu-owner-init.json','imu-owner-history.jsonl','imu-owned-shadow.jsonl','imu-pitch-projection.jsonl']}
    first={n:prefix_equal(out/n,prefix/n) for n in ['raw200-producer.jsonl','raw200-reference.jsonl','raw200-host.jsonl','raw200-actions.jsonl','raw200-init.json','raw200-native.csv','raw200-host-observer.jsonl']}
    lim=read(STUDY/'test-plan.json')['limits'];raw=raw_audit(e,out,lim);observer=observer_audit(e,out,lim);r['checks'].update(all220legacy8msfull_sensor_bytes=all(same.values()),all225continuous_first60sbytes=all(first.values()));r['checks'].update(raw['checks']);r['checks'].update(observer['checks']);r.update(passed=all(r['checks'].values()),legacy220_full_byte_checks=same,explicit225_prefix_byte_checks=first,raw200=raw,observer200=observer)
    write(out/'full200-audit.json',r);return r
