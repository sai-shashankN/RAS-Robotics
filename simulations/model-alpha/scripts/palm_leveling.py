"""Palm attitude goals from body pitch and arm encoders, without object-pose feedback."""
import math


def wrist_goal(shoulder,elbow,body_pitch,frame):
    if frame not in ['world','body']:raise ValueError('Unknown palm reference frame')
    return math.pi-shoulder-elbow-(body_pitch if frame=='world' else 0)


def nominal_pose(p,e,pose,frame,fraction=1):
    from grasp_planning import loaded_reference
    result = dict(pose)
    for _ in range(60):
        pitch = loaded_reference(p,e,result,fraction)
        value = wrist_goal(result['shoulder'],result['elbow'],pitch,frame)
        if abs(value-result['wrist'])<1e-11:break
        result['wrist'] = value
    return result


def plan_level_load(p,e):
    from grasp_planning import loaded_reference
    base = p['dynamic_arm_targets']['arm_reach']
    poses = [dict(base,**p['finger_targets']['claw_open']),dict(base,**p['finger_targets']['claw_closed']),e['lift_pose']]
    peak,worst,samples = 0,None,0
    for start,end in zip(poses,poses[1:]):
        count = max(1,math.ceil(max(abs(start[j]-end[j]) for j in start)/.002))
        for i in range(count+1):
            pose = {j:start[j]+(end[j]-start[j])*i/count for j in start}
            for fraction in [0,1]:
                candidate = nominal_pose(p,e,pose,e['palm_reference_frame'],fraction)
                value = abs(loaded_reference(p,e,candidate,fraction));samples+=1
                if value>peak:peak,worst=value,candidate
    return {'within_lean_limit':peak<=p['dynamic_lean_limit_rad'],'peak_nominal_loaded_lean_rad':peak,
        'lean_limit_rad':p['dynamic_lean_limit_rad'],'worst_nominal_pose':worst,'samples':samples,
        'assumption':'Nominal attitude-compensated poses with empty/full known load; sampled lean guard, not dynamic tracking/clearance certification.'}


def angle_error(value):return math.atan2(math.sin(value-math.pi),math.cos(value-math.pi))


if __name__=='__main__':
    q,e,pitch=-.25,1.5,-.26
    assert abs(pitch+q+e+wrist_goal(q,e,pitch,'world')-math.pi)<1e-12
    assert abs(q+e+wrist_goal(q,e,pitch,'body')-math.pi)<1e-12
