"""Explicit ideal gripper-wrench receiver. No actuator or Supervisor access."""
import json,math,struct

KEYS=('version','sequence','start_s','end_s','samples','duration_s','valid','fx_N','fy_N','fz_N','mwx_Nm','mwy_Nm','mwz_Nm','mrx_Nm','mry_Nm','mrz_Nm','origin_x_m','origin_y_m','origin_z_m')
FORMAT=struct.Struct('<19d')

def decode(data,expected_samples=32):
    if len(data)!=FORMAT.size:raise ValueError('wrench packet length')
    p=dict(zip(KEYS,FORMAT.unpack(data)))
    if not all(math.isfinite(v) for v in p.values()):raise ValueError('nonfinite wrench packet')
    if p['version']!=1 or p['sequence']<1 or not p['sequence'].is_integer() or p['samples']!=expected_samples or p['valid'] not in (0,1):raise ValueError('wrench packet metadata')
    if abs(p['duration_s']-.008)>1e-9 or abs(p['end_s']-p['start_s']-.008)>1e-9 or abs(p['end_s']-p['sequence']*.008)>1e-9:raise ValueError('wrench packet interval')
    return p

class WrenchSensor:
    def __init__(self,receiver,path,physics_ms=.25):
        self.expected_samples=round(8/physics_ms)
        if physics_ms not in (.25,.125) or abs(self.expected_samples*physics_ms-8)>1e-9:raise ValueError('unsupported wrench physics step')
        self.receiver=receiver;receiver.enable(8);self.path=path;self.sequence=0;self.latest=None
    def observe(self,time_s):
        packets=[]
        while self.receiver.getQueueLength():
            p=decode(self.receiver.getBytes(),self.expected_samples);self.receiver.nextPacket()
            if p['sequence']!=self.sequence+1:raise ValueError('wrench packet gap/reordering')
            age=time_s-p['end_s']
            if age< -1e-9 or age>.016+1e-9:raise ValueError('wrench packet future/stale')
            if time_s>.024 and not p['valid']:raise ValueError('wrench observer invalid after startup')
            self.sequence=int(p['sequence']);p.update(received_s=time_s,age_s=max(0.,age));packets.append(p)
            if p['valid']:self.latest=p
        if packets:
            with self.path.open('a',encoding='utf-8') as f:
                for p in packets:f.write(json.dumps(p,allow_nan=False)+'\n')
        return packets
