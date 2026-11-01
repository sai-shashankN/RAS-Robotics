"""Ideal native contact window receiver. No file-based input or actuator access.

Every packet covers the immediately completed 8 ms interval. Counts are latched
over all native substeps; they are not instantaneous contact force estimates.
"""
import json,math,struct
HEADER=('version','sequence','start_s','end_s','native_steps','active_fixtures','valid','mapped_geoms','grip_cube_points','cube_fixture_points','cube_floor_points','forbidden_points','unknown_points','overflow','min_grip_fingers','bilateral_native_steps','max_cube_static_depth_m','max_forbidden_depth_m')
KEYS=HEADER+tuple(f'fixture_{i}_{kind}_points' if kind else f'fixture_{i}_points' for i in range(8) for kind in ('','top','other'))
FORMAT=struct.Struct('<42d')

def decode(data,physics_ms):
    if physics_ms not in (.25,.125) or len(data)!=FORMAT.size:raise ValueError('Fixture packet size or physics step')
    p=dict(zip(KEYS,FORMAT.unpack(data)))
    if not all(math.isfinite(v) for v in p.values()):raise ValueError('Nonfinite fixture packet')
    integer=[k for k in KEYS if k not in ('start_s','end_s','max_cube_static_depth_m','max_forbidden_depth_m')]
    if any(p[k]<0 or not p[k].is_integer() for k in integer):raise ValueError('Fixture count metadata')
    if p['version']!=1 or p['sequence']<1 or p['native_steps']!=round(8/physics_ms) or p['active_fixtures']!=8 or p['mapped_geoms']!=21 or p['valid']!=1 or p['unknown_points'] or p['overflow']:raise ValueError('Invalid fixture metadata')
    if p['min_grip_fingers']>2 or p['bilateral_native_steps']>p['native_steps']:raise ValueError('Invalid grip counts')
    if min(p['max_cube_static_depth_m'],p['max_forbidden_depth_m'])<0:raise ValueError('Negative contact depth')
    if abs(p['end_s']-p['start_s']-.008)>1e-9 or abs(p['end_s']-p['sequence']*.008)>1e-9:raise ValueError('Fixture interval')
    if any(p[f'fixture_{i}_points']!=p[f'fixture_{i}_top_points']+p[f'fixture_{i}_other_points'] for i in range(8)) or sum(p[f'fixture_{i}_points'] for i in range(8))!=p['cube_fixture_points']:raise ValueError('Fixture partition')
    for k in integer:p[k]=int(p[k])
    return p

class FixtureSensor:
    def __init__(self,receiver,path,physics_ms):
        self.receiver=receiver;receiver.enable(8);self.path=path;self.physics_ms=physics_ms;self.sequence=0;self.latest=None
    def observe(self,time_s):
        if not math.isfinite(time_s) or time_s<.008-1e-9:raise ValueError('Fixture control time')
        got=[]
        while self.receiver.getQueueLength():
            p=decode(self.receiver.getBytes(),self.physics_ms);self.receiver.nextPacket()
            if p['sequence']!=self.sequence+1:raise ValueError('Fixture packet gap/reordering')
            if abs(p['end_s']-time_s)>1e-9:raise ValueError('Fixture packet must end at current control time')
            self.sequence=p['sequence'];self.latest=p;got.append(p)
        if len(got)!=1:raise ValueError('Exactly one current fixture packet required')
        with self.path.open('a',encoding='utf-8') as f:f.write(json.dumps(dict(got[0],received_s=time_s),allow_nan=False)+'\n')
        return got[0]
