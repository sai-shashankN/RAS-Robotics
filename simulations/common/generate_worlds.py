"""Generate self-contained Webots worlds from provisional Alpha parameters."""

from pathlib import Path
import json
import math
import re
import shutil
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from control_math import finger_angles

COMMON = Path(__file__).resolve().parent
ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else COMMON.parent / "model-alpha"
P = json.loads((ROOT / "parameters.json").read_text(encoding="utf-8"))
LABEL = P["model"]
BASE = P["arm_base_joint"]
RED, GREY, BLACK, BLUE = "0.82 0.065 0.09", "0.58 0.63 0.69", "0.035 0.045 0.06", "0.03 0.36 0.85"


def shape(geometry, color, position="0 0 0", rotation="0 0 1 0"):
    return f"Pose {{ translation {position} rotation {rotation} children [ Shape {{ appearance PBRAppearance {{ baseColor {color} roughness 0.65 metalness 0.12 }} geometry {geometry} }} ] }}"


def box(size, color, position="0 0 0"):
    return shape(f"Box {{ size {size} }}", color, position)


def physics(mass, com="0 0 0"):
    return f"physics Physics {{ density -1 mass {mass} centerOfMass [ {com} ] }}"


def joint_devices(name, torque, speed=1.5, limits=None):
    limit_fields = "" if limits is None else f"minPosition {limits[0]} maxPosition {limits[1]}"
    return f'device [ RotationalMotor {{ name "{name}" maxTorque {torque} maxVelocity {speed} {limit_fields} }} PositionSensor {{ name "{name}_encoder" }} ]'


def ring_mesh():
    points, faces = [], []
    count = 40
    for i in range(count):
        a = 2 * math.pi * i / count
        for radius, y in [(P["sleeve_outer_radius"], -P["sleeve_width"] / 2),
                          (P["sleeve_outer_radius"], P["sleeve_width"] / 2),
                          (P["sleeve_inner_radius"], P["sleeve_width"] / 2),
                          (P["sleeve_inner_radius"], -P["sleeve_width"] / 2)]:
            points.append(f"{radius * math.cos(a):.7f} {y:.7f} {radius * math.sin(a):.7f}")
    for i in range(count):
        for side in range(4):
            faces.append(f"{4*i+side} {4*((i+1)%count)+side} {4*((i+1)%count)+(side+1)%4} {4*i+(side+1)%4} -1")
    return f"IndexedFaceSet {{ solid FALSE coord Coordinate {{ point [ {', '.join(points)} ] }} coordIndex [ {' '.join(faces)} ] }}"


def finger(index, initial):
    length, distal_length = P["finger_proximal_length"], P["finger_distal_length"]
    phi = finger_angles(P)[index]
    q, d = initial["finger"], initial["distal"]
    return f"""Pose {{ rotation 0 0 1 {phi} children [
      HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 -1 0 anchor 0.018 0 0.009 position {q} dampingConstant 0.002 }}
        {joint_devices(f'finger_{index}', P['joint_torque_limits']['finger'], 1.5, [-0.65, 0.35])}
        endPoint DEF FINGER_{index} Solid {{ name "finger_{index}" translation 0.018 0 0.009 rotation 0 -1 0 {q}
          children [
            {box(f'0.010 0.010 {length}', GREY, f'0 0 {length/2}')}
            {box(f'0.0025 0.010 {length*0.88}', BLUE, f'-0.006 0 {length/2}')}
            HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 -1 0 anchor 0 0 {length} position {d} dampingConstant 0.002 }}
              {joint_devices(f'distal_{index}', P['joint_torque_limits']['finger'], 1.5, [0, 1.1])}
              endPoint DEF TIP_{index} Solid {{ name "tip_{index}" translation 0 0 {length} rotation 0 -1 0 {d}
                children [
                  {box(f'0.010 0.009 {distal_length}', GREY, f'0 0 {distal_length/2}')}
                  {box(f'0.0025 0.009 {distal_length*0.9}', BLUE, f'-0.006 0 {distal_length/2}')}
                ]
                boundingObject Pose {{ translation 0 0 {distal_length/2} children [ Box {{ size 0.013 0.010 {distal_length} }} ] }}
                {physics(P['finger_segment_mass'], f'0 0 {distal_length/2}')}
              }}
            }}
          ]
          boundingObject Pose {{ translation 0 0 {length/2} children [ Box {{ size 0.013 0.011 {length} }} ] }}
          {physics(P['finger_segment_mass'], f'0 0 {length/2}')}
        }}
      }}
    ] }}"""


def arm(initial):
    r = P["sleeve_outer_radius"] if P["has_sleeve"] else 0.0
    upper, forearm = P["upper_length"], P["forearm_length"]
    anchor = ' '.join(map(str, P["arm_mount_offset"]))
    q, e, w = initial[BASE], initial["elbow"], initial["wrist"]
    neck = P.get('wrist_standoff', 0.0)
    neck_visual = box(f'0.012 0.012 {neck}', GREY, f'0 0 {-neck/2}') if neck else ''
    palm_bounds = f'Group {{ children [ Cylinder {{ radius {P["palm_radius"]} height 0.016 }} Pose {{ translation 0 0 {-neck/2} children [ Box {{ size 0.012 0.012 {neck} }} ] }} ] }}' if neck else f'Cylinder {{ radius {P["palm_radius"]} height 0.016 }}'
    sleeve_visual = shape(ring_mesh(), GREY) if P["has_sleeve"] else shape('Cylinder { radius 0.018 height 0.034 }', GREY, rotation='1 0 0 1.57079632679')
    base_collision_radius = r if P["has_sleeve"] else 0.018
    base_collision_width = P["sleeve_width"] if P["has_sleeve"] else 0.034
    return f"""HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor {anchor} position {q} dampingConstant 0.01 }}
      {joint_devices(BASE, P['joint_torque_limits'][BASE], 0.8, P['joint_ranges'][BASE])}
      endPoint DEF ARM_BASE Solid {{ name "{BASE} and upper arm" translation {anchor} rotation 0 1 0 {q}
        children [
          {sleeve_visual}
          {box(f'0.025 0.024 {upper}', GREY, f'0 0 {r+upper/2}')}
          HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0 {r+upper} position {e} dampingConstant 0.01 }}
            {joint_devices('elbow', P['joint_torque_limits']['elbow'], 0.8, P['joint_ranges']['elbow'])}
            endPoint DEF FOREARM Solid {{ name "forearm" translation 0 0 {r+upper} rotation 0 1 0 {e}
              children [
                {box(f'0.020 0.023 {forearm}', GREY, f'0 0 {forearm/2}')}
                HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0 {forearm} position {w} dampingConstant 0.005 }}
                  {joint_devices('wrist', P['joint_torque_limits']['wrist'], 0.9, P['joint_ranges']['wrist'])}
                  endPoint DEF PALM Solid {{ name "{len(finger_angles(P))} finger palm" translation {neck*math.sin(w)} 0 {forearm+neck*math.cos(w)} rotation 0 1 0 {w}
                    children [
                      {neck_visual}
                      {shape(f"Cylinder {{ radius {P['palm_radius']} height 0.016 }}", GREY)}
                      {''.join(finger(i, initial) for i in range(len(finger_angles(P))))}
                    ]
                    boundingObject {palm_bounds}
                    {physics(P['palm_mass'])}
                  }}
                }}
              ]
              boundingObject Pose {{ translation 0 0 {forearm/2} children [ Box {{ size 0.020 0.023 {forearm} }} ] }}
              {physics(P['forearm_mass'], f'0 0 {forearm/2}')}
            }}
          }}
        ]
        boundingObject Group {{ children [
          Pose {{ rotation 1 0 0 1.57079632679 children [ Cylinder {{ radius {base_collision_radius} height {base_collision_width} }} ] }}
          Pose {{ translation 0 0 {r+upper/2} children [ Box {{ size 0.025 0.024 {upper} }} ] }}
        ] }}
        {physics(P['sleeve_upper_mass'], f'0 0 {r+upper*0.35}')}
      }}
    }}"""


def wheel(side, y):
    return f"""HingeJoint {{ jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {y} 0 dampingConstant 0.0005 }}
      {joint_devices(side+'_wheel', P['wheel_torque_limit'], P['wheel_speed_limit'])}
      endPoint Solid {{ name "{side} wheel" translation 0 {y} 0 contactMaterial "rubber"
        children [
          {shape(f"Cylinder {{ radius {P['wheel_radius']} height {P['wheel_width']} subdivision 40 }}", BLACK, rotation='1 0 0 1.57079632679')}
          {shape(f"Cylinder {{ radius {P['wheel_radius']*0.55} height {P['wheel_width']+0.001} }}", GREY, rotation='1 0 0 1.57079632679')}
          {box(f'0.007 {P["wheel_width"]+0.002} 0.09', BLUE)}
        ]
        boundingObject Pose {{ rotation 1 0 0 1.57079632679 children [ Cylinder {{ radius {P['wheel_radius']} height {P['wheel_width']} subdivision 40 }} ] }}
        {physics(P['wheel_mass'])}
      }}
    }}"""


def viewpoint_rotation(position, target):
    direction = [b - a for a, b in zip(position, target)]
    yaw = math.atan2(direction[1], direction[0])
    pitch = math.atan2(-direction[2], math.hypot(*direction[:2]))
    sz, cz, sy, cy = math.sin(yaw/2), math.cos(yaw/2), math.sin(pitch/2), math.cos(pitch/2)
    v, qw = [-sz*sy, cz*sy, sz*cy], cz*cy
    n = math.sqrt(sum(x*x for x in v))
    return ' '.join(str(x/n) for x in v) + f' {2*math.atan2(n,qw)}'


def world(mode, validate=False, scenario="balance"):
    fixture = mode == "geometry"
    initial = P["poses"]["folded"]
    camera_position = [0.72, -0.94, 0.58]
    flags = f'"{mode}"' + (' "--validate"' if validate else '')
    if validate and mode == 'dynamics':
        flags += f' "--scenario={scenario}"'
    initial_pitch = {'tilt_positive': math.radians(3), 'tilt_negative': -math.radians(3)}.get(scenario, 0.0)
    root_physics = "" if fixture else physics(P["body_mass"], f"{P.get('body_com_x',0)} 0 {P['body_com_z']}")
    floor_lines = ''.join(box('4 0.002 0.001', '0.78 0.82 0.87', f'0 {i*0.1} 0.0005') + box('0.002 4 0.001', '0.78 0.82 0.87', f'{i*0.1} 0 0.0005') for i in range(-15,16))
    content = f'''#VRML_SIM R2025a utf8
# Generated by scripts/generate_worlds.py. Edit parameters.json and regenerate.
WorldInfo {{ title "Model {LABEL} - {mode}" basicTimeStep 8 coordinateSystem "ENU"
  contactProperties [ ContactProperties {{ material1 "rubber" material2 "floor" coulombFriction [ 0.8 ] }} ]
}}
Viewpoint {{ position {' '.join(map(str,camera_position))} orientation {viewpoint_rotation(camera_position,[0.06,0,0.13])} fieldOfView 0.65 }}
Background {{ skyColor [ 0.94 0.96 0.985 ] }}
DirectionalLight {{ direction -0.5 0.3 -1 intensity 1 ambientIntensity 0.6 castShadows TRUE }}
Solid {{ name "floor" translation 0 0 -0.01 contactMaterial "floor" children [
  {box('4 4 0.02','0.93 0.94 0.96')}
  {floor_lines}
] boundingObject Box {{ size 4 4 0.02 }} }}
DEF ROBOT_MODEL Robot {{ name "Model {LABEL}" translation 0 0 {P['wheel_radius']+0.002}
  rotation 0 1 0 {initial_pitch}
  supervisor TRUE controller "robot_controller" controllerArgs [ {flags} ] selfCollision {"TRUE" if P.get("self_collision", False) and not fixture else "FALSE"}
  children [
    {shape(f"Cylinder {{ radius {P['body_radius']} height {P['body_length']} subdivision 40 }}",RED,rotation='1 0 0 1.57079632679')}
    {box('0.07 0.15 0.025',RED,'0 0 -0.024')}
    {wheel('left',P['wheel_track']/2)}
    {wheel('right',-P['wheel_track']/2)}
    InertialUnit {{ name "body_imu" }}
    Gyro {{ name "body_gyro" }}
    Camera {{ name "front_camera" translation 0.042 0.085 0.008 width 320 height 240 fieldOfView 1.05 }}
    {box('0.012 0.025 0.022',BLACK,'0.040 0.085 0.008')}
    {box('0.012 0.025 0.022',BLACK,'0.040 -0.085 0.008')}
    {arm(initial)}
  ]
  boundingObject Pose {{ rotation 1 0 0 1.57079632679 children [ Cylinder {{ radius {P['body_radius']} height {P['body_length']} subdivision 32 }} ] }}
  {root_physics}
}}
'''
    # Geometry mode is deliberately kinematic throughout: no contact/torque claims.
    return re.sub(r'physics Physics \{[^{}]*\}', '', content) if fixture else content


def main():
    folder = ROOT / "worlds"
    folder.mkdir(exist_ok=True)
    controller_folder = ROOT / "controllers" / "robot_controller"
    controller_folder.mkdir(parents=True, exist_ok=True)
    for filename in ["robot_controller.py", "control_math.py", "mass_properties.py"]:
        shutil.copyfile(COMMON / filename, controller_folder / filename)
    (controller_folder / "runtime.ini").write_text('[python]\nCOMMAND = C:/Python313/python.exe\n', encoding="utf-8")
    for mode in ["geometry", "dynamics"]:
        for validate, scenario in [(False, 'balance'), (True, 'balance')] + ([(True, s) for s in ['tilt_positive', 'tilt_negative', 'push', 'steering', 'drive'] + (['pose_guard', 'claw_cycle_folded', 'claw_cycle_raised'] if not P['has_sleeve'] else []) + list(P.get('dynamic_arm_targets', {}))] if mode == 'dynamics' else []):
            suffix = '_' + scenario if scenario != 'balance' else ''
            name = f"{LABEL.lower()}_{mode}" + suffix + ("_validation" if validate else "") + ".wbt"
            (folder / name).write_text(world(mode, validate, scenario), encoding="utf-8")
            perspective = folder / ("." + name.replace('.wbt', '.wbproj'))
            perspective_text = perspective.read_text(encoding='utf-8') if perspective.exists() else 'Webots Project File version R2025a\n'
            camera_line = f'renderingDevicePerspectives: Model {LABEL}:front_camera;0;1;0;0'
            if 'renderingDevicePerspectives:' in perspective_text:
                perspective_text = re.sub(r'renderingDevicePerspectives:[^\n]*', camera_line, perspective_text)
            else:
                perspective_text += camera_line + '\n'
            # Windows rejects O_CREAT/O_TRUNC on existing hidden .wbproj files.
            with perspective.open('r+' if perspective.exists() else 'w', encoding='utf-8') as stream:
                stream.seek(0)
                stream.write(perspective_text)
                stream.truncate()
    print(f"Generated Model {LABEL} worlds and validation scenarios. Sleeve present: {P['has_sleeve']}. Self-collision enabled: {P.get('self_collision', False)}.")


if __name__ == "__main__":
    main()
