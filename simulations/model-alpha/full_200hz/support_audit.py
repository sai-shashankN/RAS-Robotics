import math
from m1_audit import observe as frozen_observe

def observe(data,path,setup):
    actual=dict(setup,source_object_xyz_m=[*setup['platform_center_xy_m'],setup['source_object_xyz_m'][2]])
    consistent=frozen_observe(data,path,actual)
    for r in data:r['initial_error_m']=math.hypot(r['object_x_m']-setup['source_object_xyz_m'][0],r['object_y_m']-setup['source_object_xyz_m'][1])
    return consistent
