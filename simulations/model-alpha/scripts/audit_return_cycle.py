"""Specified empty return paths: conservative self and nominal fixture screens."""

from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys

from audit_clearance import audit_path,boxes,gap,ry
import numpy as np

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
sys.path.insert(0,str(MODEL.parent/'common'))
from grasp_planning import loaded_reference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--study',choices=['return_cycle_comparison','return_cycle_validated'],default='return_cycle_comparison')
    parser.add_argument('--case',default='reference_50mm')
    args = parser.parse_args()
    if not args.case or not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.case):
        raise ValueError('Invalid case')
    report_path = MODEL/'results'/args.study/args.case/'report.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    e,s = report['experiment'],report['setup']
    p = json.loads((MODEL/'parameters.json').read_text(encoding='utf-8'))
    closed = p['dynamic_arm_targets']['arm_reach']
    opened = dict(closed,**p['finger_targets']['claw_open'])
    folded = p['poses']['folded']
    self_paths = {'empty_close':audit_path(p,opened,closed),'fold':audit_path(p,closed,folded)}
    identity = np.eye(3)
    def box(name,center,size):
        return name,'fixture',np.array(center),identity,np.array(size)/2
    x,y,z = s['initial_object_position_m']
    bottom,top = s['tray_top_m'],s['stand_top_m']
    telemetry_path = report_path.parent/'telemetry.csv'
    with telemetry_path.open(newline='',encoding='utf-8') as stream:
        recorded = [r for r in csv.DictReader(stream) if float(r['time_s'])>=report['return_sequence']['started']]
    centers = [report['return_object_start_m'],report['final_object_position_m']]
    centers += [[float(r[f'object_{axis}_m']) for axis in 'xyz'] for r in recorded]
    low,high = np.min(centers,axis=0),np.max(centers,axis=0)
    sphere_center = (low+high)/2
    sphere_size = high-low+2*e['object_radius_m']+.004
    fixtures = [box('recorded rolling sphere envelope',sphere_center,sphere_size),
                box('stand envelope',[x,y,(top+bottom)/2],[2*e['stand_radius_m']]*2+[top-bottom]),
                box('tray floor',[x,y,.002],[.20,.16,.004])]
    for dx,dy,size in [(.102,0,[.004,.168,.020]),(-.102,0,[.004,.168,.020]),(0,.082,[.20,.004,.020]),(0,-.082,[.20,.004,.020])]:
        fixtures.append(box('tray wall',[x+dx,y+dy,.014],size))
    nominal = {}
    for name,start,end in [('retreat',opened,opened),('empty_close',opened,closed),('fold',closed,folded)]:
        minimum,floor,closest = float('inf'),float('inf'),None
        for i in range(501):
            u = i/500
            pose = {j:start[j]+(end[j]-start[j])*u for j in start}
            rotation = ry(loaded_reference(p,e,pose,0))
            base = np.array([e['alignment_target_offset_world_m'][0]*(1-u) if name=='retreat' else 0,0,p['wheel_radius']])
            for shape in boxes(p,pose):
                world = shape[0],shape[1],base+rotation@shape[2],rotation@shape[3],shape[4]
                if not shape[1].startswith('wheel'):
                    floor = min(floor,float(world[2][2]-np.abs(world[3][2])@world[4]))
                for fixture in fixtures:
                    distance = gap(world,fixture)
                    if distance<minimum:
                        minimum,closest = distance,[shape[0],fixture[0]]
        nominal[name] = {'minimum_sampled_gap_m':minimum,'minimum_sampled_nonwheel_floor_gap_m':floor,
                         'closest_pair':closest,'samples':501}
    sources = [Path(__file__),MODEL/'scripts/audit_clearance.py',report_path,telemetry_path,MODEL/'parameters.json',
               MODEL/'scripts/generate_grasp_worlds.py',MODEL.parent/'common/generate_worlds.py']
    sources += [MODEL.parent/'common'/name for name in ['return_planning.py','grasp_planning.py','control_math.py','mass_properties.py']]
    result = {'self_paths':self_paths,'nominal_fixture':nominal,
              'self_paths_clear':all(r['conservative_nonadjacent_clearance_passed'] for r in self_paths.values()),
              'nominal_samples_clear':all(r['minimum_sampled_gap_m']>0 and r['minimum_sampled_nonwheel_floor_gap_m']>0 for r in nominal.values()),
              'source_sha256':{str(p.relative_to(WORKSPACE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
              'limitations':'Self paths have a joint-motion bound with direct neighbors excluded. World screens sample nominal base/lean against an envelope of the recorded sphere path plus 2 mm padding; no bound on robot tracking or between-sample world clearance. Dynamic contacts/support and matched stationary controls remain required.'}
    (report_path.parent/'return-clearance.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'self_clear':result['self_paths_clear'],'nominal':nominal},indent=2))
    if not result['self_paths_clear'] or not result['nominal_samples_clear']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
