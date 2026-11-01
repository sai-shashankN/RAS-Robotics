"""Folded-to-raised staging before a straight, supplied-target base approach."""

from grasp_planning import loaded_reference


def plan_approach(parameters, experiment):
    enabled = experiment.get('start_folded', False)
    folded = parameters['poses']['folded']
    closed = parameters['dynamic_arm_targets']['arm_reach']
    opened = dict(closed, **parameters['finger_targets']['claw_open'])
    deploy_start = 3.0
    deploy_time = max(abs(closed[j]-folded[j])/parameters['dynamic_arm_rate_rad_s'] for j in ['shoulder', 'elbow', 'wrist'])
    open_start = deploy_start+deploy_time+3.0
    open_time = max(abs(opened[j]-closed[j])/parameters['finger_target_rate_rad_s'] for j in ['finger', 'distal'])
    ready = open_start+open_time+3.0 if enabled else 0.0
    peak = 0.0
    for start, end in [(folded, closed), (closed, opened)]:
        for i in range(501):
            angles = {j: start[j]+(end[j]-start[j])*i/500 for j in start}
            peak = max(peak, abs(loaded_reference(parameters, experiment, angles, 0.0)))
    return {'enabled': enabled, 'deploy_start_s': deploy_start, 'open_start_s': open_start,
            'ready_s': ready, 'folded_pose': folded, 'raised_closed_pose': closed, 'open_pose': opened,
            'peak_sampled_empty_lean_rad': peak, 'within_lean_limit': peak<=parameters['dynamic_lean_limit_rad'],
            'alignment_deadline_s': ready+experiment.get('alignment_deadline_after_staging_s', 40.0) if enabled else 26.0,
            'limitations': 'Sampled nominal empty-load lean only; geometry/fixture audits and physical contact checks are separate.'}


def staging_target(plan, time_s):
    if not plan['enabled'] or time_s>=plan['ready_s']:
        return None
    if time_s<plan['deploy_start_s']:
        return 'folded', plan['folded_pose']
    if time_s<plan['open_start_s']:
        return 'deploying', plan['raised_closed_pose']
    return 'opening', plan['open_pose']
