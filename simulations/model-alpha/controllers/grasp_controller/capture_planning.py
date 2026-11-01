"""Short longitudinal base alignment using a supplied target, not perception."""


def plan_capture(experiment):
    enabled = 'alignment_target_offset_world_m' in experiment
    offset = experiment.get('alignment_target_offset_world_m', [0.0, 0.0, 0.0])
    folded = experiment.get('start_folded', False)
    max_translation = .075 if folded else .015
    reason = None
    if abs(offset[1]) > .001:
        reason = 'Lateral target requires base heading/repositioning; no sideways arm swivel'
    elif abs(offset[2]) > .001:
        reason = 'Target height requires a separately validated arm path'
    elif abs(offset[0]) > max_translation:
        reason = 'Translation exceeds this local alignment experiment limit'
    return {'enabled': enabled, 'admitted': reason is None, 'reason': reason,
            'target_offset_world_m': offset, 'translation_m': offset[0] if reason is None else 0.0,
            'profile_duration_s': 6.0 if folded else 2.5,
            'target_source': 'Supplied world offset from reference capture frame; no live object position or camera input',
            'assumptions': 'Initial heading is world X. Wheel odometry and gyro feedback hold a short longitudinal alignment; no base yaw plan or approach from folded pose.',
            'admission_limits': {'longitudinal_m': max_translation, 'lateral_m': .001, 'height_m': .001},
            'limits_are_tested_tolerances': False}


def translation_reference(plan, time_s):
    if not plan['enabled'] or not plan['admitted']:
        return 0.0, 0.0
    start, duration = .5, plan['profile_duration_s']
    u = max(0.0, min(1.0, (time_s-start)/duration))
    distance = plan['translation_m']
    return distance*(3*u*u-2*u*u*u), distance*6*u*(1-u)/duration
