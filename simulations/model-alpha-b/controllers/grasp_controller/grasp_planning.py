"""Nominal known-load COM and lean preflight. No simulator state or object pose."""

import math
from control_math import palm_position
from mass_properties import mass_properties


def loaded_reference(parameters, experiment, angles, fraction=1.0):
    model = mass_properties(parameters, joint_angles=angles)
    com = model['com_body_m']
    palm = palm_position(parameters, angles)
    angle = sum(angles[j] for j in ['shoulder', 'elbow', 'wrist'])
    local = experiment['object_center_in_palm_m']
    load = [palm[0]+local[0]*math.cos(angle)+local[2]*math.sin(angle), 0,
            palm[2]-local[0]*math.sin(angle)+local[2]*math.cos(angle)]
    mass = fraction*experiment['object_mass_kg']
    x = (com[0]*model['mass_kg']+load[0]*mass)/(model['mass_kg']+mass)
    z = (com[2]*model['mass_kg']+load[2]*mass)/(model['mass_kg']+mass)
    return -math.atan2(x, z)


def plan_load(parameters, experiment):
    raised = parameters['dynamic_arm_targets']['arm_reach']
    opened = dict(raised, **parameters['finger_targets']['claw_open'])
    closed = dict(raised, **parameters['finger_targets']['claw_closed'])
    poses = [opened, closed, experiment['lift_pose']]
    peak, worst, samples = 0.0, None, 0
    for start, end in zip(poses, poses[1:]):
        count = max(1, math.ceil(max(abs(end[j]-start[j]) for j in start)/.002))
        for i in range(count+1):
            angles = {j: start[j]+(end[j]-start[j])*i/count for j in start}
            lean = abs(loaded_reference(parameters, experiment, angles))
            samples += 1
            if lean > peak:
                peak, worst = lean, angles
    return {'within_lean_limit': peak <= parameters['dynamic_lean_limit_rad'],
            'peak_nominal_loaded_lean_rad': peak, 'lean_limit_rad': parameters['dynamic_lean_limit_rad'],
            'worst_nominal_pose': worst, 'samples': samples,
            'assumption': 'Full known load at nominal palm location across sampled closure and lift; commanded finger angles, not contact-constrained angles. Quasi-static guard, not a dynamics or clearance certificate.'}
