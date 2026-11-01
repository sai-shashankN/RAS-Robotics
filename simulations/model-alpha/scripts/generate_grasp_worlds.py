"""Prepared-pose grasp fixture: free sphere, narrow stand, and catch tray.

Builds on the current generator without changing the existing validation worlds.
All object placement is initial scene setup; the controller never relocates it.
"""

from pathlib import Path
import argparse
import importlib.util
import json
import math
import shutil
import sys

MODEL = Path(__file__).resolve().parents[1]
COMMON = MODEL.parent/'common'
sys.path.insert(0, str(COMMON))
from control_math import finger_angles, palm_position
from mass_properties import mass_properties


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment', type=Path, default=MODEL/'grasp_parameters.json')
    parser.add_argument('--case')
    parser.add_argument('--study', default='grasp_sweep')
    args = parser.parse_args()
    if args.case and not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.case):
        raise ValueError('Case name must use lowercase letters, digits, or underscores')
    if not args.study or not all(c in 'abcdefghijklmnopqrstuvwxyz0123456789_' for c in args.study):
        raise ValueError('Study name must use lowercase letters, digits, or underscores')
    experiment_path = args.experiment.resolve()
    setup_path = (experiment_path.parent/'setup.json') if args.case else MODEL/'grasp_setup.json'
    sys.argv = [str(COMMON/'generate_worlds.py'), str(MODEL)]
    spec = importlib.util.spec_from_file_location('generator', COMMON/'generate_worlds.py')
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    p = g.P
    experiment = json.loads(experiment_path.read_text())
    start = dict(p['dynamic_arm_targets']['arm_reach'], **p['finger_targets']['claw_open'])
    lean = mass_properties(p, joint_angles=start)['lean_to_align_com_rad']
    palm = palm_position(p, start)
    local = experiment['object_center_in_palm_m']
    angle = sum(start[j] for j in ['shoulder', 'elbow', 'wrist'])
    bx = palm[0]+local[0]*math.cos(angle)+local[2]*math.sin(angle)
    bz = palm[2]-local[0]*math.sin(angle)+local[2]*math.cos(angle)
    object_position = [bx*math.cos(lean)+bz*math.sin(lean), local[1],
                       p['wheel_radius']-bx*math.sin(lean)+bz*math.cos(lean)]
    placement = experiment.get('placement_offset_world_m', [0, 0, 0])
    object_position = [a+b for a, b in zip(object_position, placement)]
    if experiment.get('start_folded', False):
        start = dict(p['poses']['folded'])
        lean = mass_properties(p, joint_angles=start)['lean_to_align_com_rad']
    radius = experiment['object_radius_m']
    stand_top = object_position[2]-radius
    tray_top = .004
    height = stand_top-tray_top
    x = object_position[0]
    y = object_position[1]
    fixture = f'''
DEF CATCH_TRAY Solid {{ name "catch tray" translation {x} {y} 0.002 contactMaterial "fixture"
  children [ {g.box('0.20 0.16 0.004', '0.30 0.41 0.48')} ]
  boundingObject Box {{ size 0.20 0.16 0.004 }}
}}
DEF TRAY_WALLS Solid {{ name "catch tray walls" translation {x} {y} 0.014 contactMaterial "fixture"
  children [ {g.box('0.004 0.168 0.020', '0.30 0.41 0.48', '0.102 0 0')}
             {g.box('0.004 0.168 0.020', '0.30 0.41 0.48', '-0.102 0 0')}
             {g.box('0.20 0.004 0.020', '0.30 0.41 0.48', '0 0.082 0')}
             {g.box('0.20 0.004 0.020', '0.30 0.41 0.48', '0 -0.082 0')} ]
  boundingObject Group {{ children [
    Pose {{ translation 0.102 0 0 children [ Box {{ size 0.004 0.168 0.020 }} ] }}
    Pose {{ translation -0.102 0 0 children [ Box {{ size 0.004 0.168 0.020 }} ] }}
    Pose {{ translation 0 0.082 0 children [ Box {{ size 0.20 0.004 0.020 }} ] }}
    Pose {{ translation 0 -0.082 0 children [ Box {{ size 0.20 0.004 0.020 }} ] }}
  ] }}
}}
DEF OBJECT_STAND Solid {{ name "object stand" translation {x} {y} {tray_top+height/2} contactMaterial "fixture"
  children [ {g.shape(f'Cylinder {{ radius {experiment["stand_radius_m"]} height {height} }}', '0.45 0.48 0.52')} ]
  boundingObject Cylinder {{ radius {experiment['stand_radius_m']} height {height} }}
}}
DEF TEST_OBJECT Solid {{ name "test sphere" translation {' '.join(map(str, object_position))} contactMaterial "test_object"
  children [ {g.shape(f'Sphere {{ radius {radius} subdivision 3 }}', '1 0.65 0.02')} ]
  boundingObject Sphere {{ radius {radius} }}
  {g.physics(experiment['object_mass_kg'])}
}}
'''
    # Cylinder is Z-axis aligned in Webots R2025a; stand is vertical.
    p['poses']['folded'] = start
    validation_world = None
    world_names = []
    for validate in ([True] if args.case else [False, True]):
        content = g.world('dynamics', validate, 'grasp')
        content = content.replace('controller "robot_controller"', 'controller "grasp_controller"')
        if args.case:
            extra = ' '.join('"'+value+'"' for value in [f'--experiment={experiment_path.as_posix()}', f'--setup={setup_path.as_posix()}', f'--case={args.case}', f'--study={args.study}'])
            content = content.replace('"--scenario=grasp"', '"--scenario=grasp" '+extra)
        content = content.replace('rotation 0 1 0 0.0\n  supervisor', f'rotation 0 1 0 {lean}\n  supervisor')
        content = content.replace('coulombFriction [ 0.8 ] } ]', f'coulombFriction [ 0.8 ] }} ContactProperties {{ material1 "gripper" material2 "test_object" coulombFriction [ {experiment["finger_object_friction"]} ] }} ContactProperties {{ material1 "fixture" material2 "test_object" coulombFriction [ 0.5 ] }} ]')
        for part in ['finger', 'tip']:
            for i in range(len(finger_angles(p))):
                content = content.replace(f'name "{part}_{i}" translation', f'name "{part}_{i}" contactMaterial "gripper" translation')
        content += fixture
        filename = ('alpha_'+args.study+'_'+args.case if args.case else 'alpha_grasp')+('_validation' if validate else '')+'.wbt'
        (MODEL/'worlds'/filename).write_text(content, encoding='utf-8')
        world_names.append(Path(filename).stem)
        if validate:
            validation_world = (MODEL/'worlds'/filename).resolve()
            if not args.case:
                negative = content.replace('"--scenario=grasp"', '"--scenario=grasp" "--negative-control"')
                (MODEL/'worlds/alpha_grasp_negative_validation.wbt').write_text(negative, encoding='utf-8')
                world_names.append('alpha_grasp_negative_validation')
    folder = MODEL/'controllers/grasp_controller'
    folder.mkdir(exist_ok=True)
    for name in ['grasp_controller.py', 'grasp_planning.py', 'capture_planning.py', 'approach_planning.py', 'return_planning.py', 'control_math.py', 'mass_properties.py']:
        shutil.copyfile(COMMON/name, folder/name)
    for name in world_names:
        perspective = MODEL/'worlds'/('.'+name+'.wbproj')
        text = 'Webots Project File version R2025a\nrenderingDevicePerspectives: Model Alpha:front_camera;0;1;0;0\n'
        # Existing .wbproj files can be hidden on Windows; avoid O_CREAT there.
        with perspective.open('r+' if perspective.exists() else 'w', encoding='utf-8') as stream:
            stream.seek(0)
            stream.write(text)
            stream.truncate()
    (folder/'runtime.ini').write_text('[python]\nCOMMAND = C:/Python313/python.exe\n', encoding='utf-8')
    setup = {'start_pose': start, 'start_body_pitch_rad': lean, 'initial_object_position_m': object_position,
             'stand_top_m': stand_top, 'tray_top_m': tray_top, 'validation_world': str(validation_world)}
    setup_path.write_text(json.dumps(setup, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(setup, indent=2))


if __name__ == '__main__':
    main()
