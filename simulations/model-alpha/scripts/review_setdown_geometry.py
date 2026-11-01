"""Static reach review from measured sphere seating; no simulation pose writes."""
from pathlib import Path
import csv
import hashlib
import json
import math
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]
STUDY = MODEL/'experiments/fixed_cadence_checks'
sys.path.insert(0,str(STUDY/'projects/common'))
from control_math import palm_position
from grasp_planning import loaded_reference

def rotate(x,z,a):return [math.cos(a)*x+math.sin(a)*z,-math.sin(a)*x+math.cos(a)*z]
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    rows,inputs = [],[Path(__file__)]+[STUDY/'projects/common'/name for name in ['control_math.py','grasp_planning.py','mass_properties.py']]
    for v in ['a','b']:
        case = f'{v}_reference_stops8_ctrl8_phys2';project = STUDY/'projects'/case;folder = project/'results/physics'/case
        report = read(folder/'report.json');p = read(project/'parameters.json')
        data = [json.loads(s) for s in (folder/'contact-trace.jsonl').open(encoding='utf-8')]
        sample = max((d for d in data if d['time_s']<report['schedule_actual_s']['release']),key=lambda d:d['time_s'])
        telemetry = list(csv.DictReader((folder/'telemetry.csv').open(encoding='utf-8')))
        body = max((d for d in telemetry if float(d['time_s'])<=sample['time_s']),key=lambda d:float(d['time_s']))
        joints = sample['joint_angles_rad'];theta = float(body['pitch_rad']);body_x = float(body['body_x_m'])
        pose = {name:joints[name] for name in ['shoulder','elbow','wrist']}
        pose.update({part:sum(joints[f'{part}_{i}'] for i in range(2))/2 for part in ['finger','distal']})
        local = sample['object_palm_m'];phi = sum(pose[n] for n in ['shoulder','elbow','wrist']);palm = palm_position(p,pose)
        offset = rotate(local[0],local[2],phi);object_body = [palm[0]+offset[0],palm[2]+offset[1]]
        object_rotated = rotate(*object_body,theta)
        root_z = sample['object_position_m'][2]-object_rotated[1]
        x_model_error = body_x+object_rotated[0]-sample['object_position_m'][0]
        target = report['setup']['initial_object_position_m'];object_target = rotate(target[0]-body_x,target[2]-root_z,-theta)
        palm_target = [object_target[i]-offset[i] for i in [0,1]]
        dx = palm_target[0]-p['arm_mount_offset'][0]-p['wrist_standoff']*math.sin(phi)
        dz = palm_target[1]-p['arm_mount_offset'][2]-p['wrist_standoff']*math.cos(phi)
        length1,length2 = p['upper_length'],p['forearm_length']
        cosine = (dx*dx+dz*dz-length1*length1-length2*length2)/(2*length1*length2)
        solutions = []
        if abs(cosine)<=1:
            for sign in [1,-1]:
                elbow = sign*math.acos(cosine)
                shoulder = math.atan2(dx,dz)-math.atan2(length2*math.sin(elbow),length1+length2*math.cos(elbow))
                wrist = phi-shoulder-elbow
                candidate = dict(pose,shoulder=shoulder,elbow=elbow,wrist=wrist)
                in_range = all(p['joint_ranges'][j][0]<=candidate[j]<=p['joint_ranges'][j][1] for j in ['shoulder','elbow','wrist'])
                nominal = loaded_reference(p,dict(report['experiment'],object_center_in_palm_m=local),candidate)
                solutions.append({'pose_rad':candidate,'within_joint_ranges':in_range,
                    'wrist_margin_rad':min(wrist-p['joint_ranges']['wrist'][0],p['joint_ranges']['wrist'][1]-wrist),
                    'nominal_loaded_lean_rad':nominal,'nominal_lean_within_guard':abs(nominal)<=p['dynamic_lean_limit_rad']})
        rows.append({'variant':v,'observed_preopening_gap_mm':1000*sample['stand_gap_m'],
            'object_center_correction_world_mm':[1000*(target[i]-sample['object_position_m'][i]) for i in range(3)],
            'current_wrist_rad':pose['wrist'],'current_body_pitch_rad':theta,'inferred_root_height_m':root_z,
            'planar_fk_x_consistency_error_mm':x_model_error*1000,'fixed_body_fixed_palm_orientation_ik_solutions':solutions})
        inputs += [project/'parameters.json',folder/'report.json',folder/'telemetry.csv',folder/'contact-trace.jsonl']
    out = MODEL/'results/placement_geometry_review';out.mkdir(exist_ok=True)
    result = {'date':'2026-10-24','cases':rows,'scope':'Static planar inverse kinematics at measured body pose and measured seated object offset. Preserve palm orientation; no wheel translation or arm-length change. Not a motion plan, collision certificate, balancing solution or successful placement.',
        'limitations':'Fixed body pitch may differ from the new nominal loaded lean. Object seat is treated as rigid for this calculation; finger slopes can move it during lowering/opening. Root height inferred from measured seat; Y mismatch is reported but not corrected. Cube geometry and stable support are the next benchmark.',
        'source_sha256':{str(path.relative_to(WORKSPACE)):sha(path) for path in inputs}}
    (out/'review.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    lines = ['# Placement geometry review','','Static diagnostic; no new physics run or object pose reset.','',
        '| Variant | Current pole gap (mm) | Object correction X/Z (mm) | In-range IK branch | Candidate wrist (rad) |',
        '| --- | ---: | --- | --- | ---: |']
    for r in rows:
        valid = [s for s in r['fixed_body_fixed_palm_orientation_ik_solutions'] if s['within_joint_ranges']]
        wrist = f"{valid[0]['pose_rad']['wrist']:.3f}" if valid else 'None'
        correction = r['object_center_correction_world_mm']
        lines.append(f"| {r['variant'].upper()} | {r['observed_preopening_gap_mm']:.2f} | {correction[0]:.2f} / {correction[2]:.2f} | {'Yes' if valid else 'No'} | {wrist} |")
    lines += ['',result['scope'],'',result['limitations'],'',
        'An in-range branch means the measured placement point is not excluded solely by these arm lengths or wrist angle limits in this planar pose calculation. It does not demonstrate a feasible balanced, collision-free path or support transfer. Inspect nominal lean and joint margins in review.json before choosing a candidate motion.','',
        'The 3 mm pole is a narrow diagnostic fixture. Begin the two-finger baseline with a flat-bottom cube and stable support, review the full sloping finger/joint contact path, and revise arm/wrist/base dimensions where task checks identify a limitation. Existing dimensions are not frozen.','']
    (out/'PLACEMENT_REVIEW.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(rows),flush=True)


if __name__=='__main__':main()
