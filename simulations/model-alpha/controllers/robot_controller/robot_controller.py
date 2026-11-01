"""Alpha geometry fixture and an experimental free-body balance controller.

The geometry fixture is kinematic, with no Physics nodes. It is not evidence of balance or driving.
The free-body world uses bounded motor commands; no body pose is overwritten.
"""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

from controller import Supervisor, Keyboard
from control_math import finger_angles, finger_tip_position, palm_position, saturate_pair, wheel_speeds
from mass_properties import mass_properties

ROOT = Path(__file__).resolve().parents[2]
P = json.loads((ROOT / "parameters.json").read_text(encoding="utf-8"))
WORKSPACE = ROOT.parents[1]
source_paths = [ROOT.parent / 'common' / name for name in ['robot_controller.py', 'generate_worlds.py', 'control_math.py', 'mass_properties.py']] + [ROOT / 'parameters.json']
SOURCE_HASHES = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}
FINGER_COUNT = len(finger_angles(P))
LABEL, BASE = P["model"], P["arm_base_joint"]
MODE = sys.argv[1] if len(sys.argv) > 1 else "geometry"
VALIDATE = "--validate" in sys.argv
SCENARIO = next((arg.split('=', 1)[1] for arg in sys.argv if arg.startswith('--scenario=')), 'balance')
OUT = ROOT / "results" / MODE
if MODE == 'dynamics' and SCENARIO != 'balance':
    OUT = OUT / SCENARIO
OUT.mkdir(parents=True, exist_ok=True)
robot = Supervisor()
step = int(robot.getBasicTimeStep())
dt = step / 1000
body = robot.getSelf()
imu, gyro = robot.getDevice("body_imu"), robot.getDevice("body_gyro")
imu.enable(step)
gyro.enable(step)
camera = robot.getDevice("front_camera")
keyboard = robot.getKeyboard()
keyboard.enable(step)
motors, encoders = {}, {}
for name in ["left_wheel", "right_wheel", BASE, "elbow", "wrist"] + [f"{part}_{i}" for part in ["finger", "distal"] for i in range(FINGER_COUNT)]:
    motors[name] = robot.getDevice(name)
    encoders[name] = robot.getDevice(name + "_encoder")
    encoders[name].enable(step)
for side in ["left_wheel", "right_wheel"]:
    motors[side].setPosition(float("inf"))
    motors[side].setVelocity(0)


commanded = dict(P["poses"]["folded"])
desired = dict(commanded)
arm_scenario = MODE == 'dynamics' and SCENARIO.startswith('arm_')
finger_scenario = MODE == 'dynamics' and SCENARIO.startswith('claw_cycle_')
rejected_pose_commands = []


def command_pose(name):
    target = P.get('dynamic_arm_targets', {}).get(name, P['poses'].get(name))
    if name in P.get('finger_targets', {}):
        target = dict(desired, **P['finger_targets'][name])
    if MODE == 'dynamics' and not P['has_sleeve']:
        # Check the commanded joint-space path before accepting a geometry pose.
        # This is a quasi-static lean guard, not a collision/feasibility planner.
        for i in range(101):
            sample = {j: commanded[j] + (target[j] - commanded[j]) * i / 100 for j in commanded}
            lean = mass_properties(P, joint_angles=sample)['lean_to_align_com_rad']
            if abs(lean) > P.get('dynamic_lean_limit_rad', 0.30):
                rejected_pose_commands.append(name)
                print(f'Rejected {name}: modeled path exceeds dynamic lean limit. Use geometry mode to explore it.', flush=True)
                return False
    desired.update(target)
    return True


def advance_pose():
    # Slew targets so bounded hinge moves never take an unintended wrap-around path.
    # Dynamic poses share one interpolation fraction, preserving the checked
    # joint-space path even when joint excursions have different lengths.
    rates = {j: P.get('dynamic_arm_rate_rad_s', 0.08) if j in [BASE, 'elbow', 'wrist'] else P.get('finger_target_rate_rad_s', 1.0) for j in commanded}
    duration = max(abs(desired[j]-commanded[j])/rates[j] for j in commanded)
    fraction = min(1.0, dt/duration) if duration else 1.0
    for joint in [BASE, "elbow", "wrist", "finger", "distal"]:
        rate = (P.get('dynamic_arm_rate_rad_s', 0.08) if MODE == 'dynamics' else 0.65) if joint in [BASE, "elbow", "wrist"] else 1.0
        difference = desired[joint] - commanded[joint]
        commanded[joint] += difference*fraction if MODE == 'dynamics' else max(-rate * dt, min(rate * dt, difference))
    for joint in [BASE, "elbow", "wrist"]:
        motors[joint].setPosition(commanded[joint])
    for index in range(FINGER_COUNT):
        motors[f"finger_{index}"].setPosition(commanded["finger"])
        motors[f"distal_{index}"].setPosition(commanded["distal"])
    if MODE == 'geometry':
        # This massless fixture displays prescribed geometry, not motor tracking.
        # Direct positioning is NEVER used in free-body dynamics or grasp tests.
        for key,node_name in [(BASE,'ARM_BASE'),('elbow','FOREARM'),('wrist','PALM')]:
            robot.getFromDef(node_name).getParentNode().setJointPosition(commanded[key])
        for i in range(FINGER_COUNT):
            robot.getFromDef(f'FINGER_{i}').getParentNode().setJointPosition(commanded['finger'])
            robot.getFromDef(f'TIP_{i}').getParentNode().setJointPosition(commanded['distal'])


pose_name = "folded"
command_pose(pose_name)
robot.setLabel(0, f"MODEL {LABEL.upper()} | {MODE.upper()}", 0.02, 0.03, 0.065, 0x172B4D)
robot.setLabel(1, "Kinematic geometry fixture: body fixed" if MODE == "geometry" else "Free body: experimental balance control", 0.02, 0.11, 0.045, 0x172B4D)
third_pose = "back" if P["has_sleeve"] else "lower"
robot.setLabel(2, f"1 fold | 2 front | 3 {third_pose} | 4 open | arrows drive/turn (free body)", 0.02, 0.18, 0.036, 0x172B4D)
robot.setLabel(3, "No arm-assisted mobility. Provisional masses and actuators.", 0.02, 0.94, 0.037, 0x8B2430)
print(f"{LABEL} {MODE}: no arm-base swivel, no arm-assisted mobility. Sleeve: {P['has_sleeve']}", flush=True)

pose_names = list(P["poses"])
checks = []
captured = set()
max_pitch, max_tracking_error, max_fk_error = 0.0, 0.0, 0.0
fallen = False
initial_position = None
initial_com = None
last_wheel_position = 0.0
last_left_angle = last_right_angle = 0.0
odometry_offset = None
position_reference = 0.0
max_wheel_speed = 0.0
max_applied_wheel_torque = 0.0
max_yaw = 0.0
max_displacement = 0.0
lean_reference = last_lean_reference = 0.0
max_pitch_error = 0.0
min_arm_clearance = float('inf')
max_com_model_error = 0.0
max_requested_lean = 0.0
guard_attempts = set()
finger_checkpoints = [(12, 'claw_open'), (20, 'claw_closed'), (28, 'claw_open'), (38, 'claw_closed'), (46, 'folded')]
if SCENARIO == 'claw_cycle_raised':
    finger_checkpoints = [(18, 'arm_reach'), (26, 'claw_open'), (34, 'claw_closed'), (42, 'claw_open'), (50, 'claw_closed'), (70, 'folded')]
arm_nodes = {name: robot.getFromDef(name) for name in ['ARM_BASE', 'FOREARM', 'PALM'] + [f'{part}_{i}' for part in ['FINGER', 'TIP'] for i in range(FINGER_COUNT)]}
arm_contacts = {}
max_arm_contact_count = 0
max_finger_fk_error = 0.0


def arm_floor_clearance():
    # Lowest point of each generated arm collision primitive above z=0 floor.
    def box_bottom(name, size, center_z):
        node = robot.getFromDef(name)
        pos, rot = node.getPosition(), node.getOrientation()
        return pos[2] + rot[8] * center_z - sum(abs(rot[6+i]) * size[i] / 2 for i in range(3))
    base = robot.getFromDef('ARM_BASE')
    pos, rot = base.getPosition(), base.getOrientation()
    radius = P['sleeve_outer_radius'] if P['has_sleeve'] else 0.018
    width = P['sleeve_width'] if P['has_sleeve'] else 0.034
    bottoms = [pos[2] - radius * math.hypot(rot[6], rot[8]) - width/2 * abs(rot[7]),
               box_bottom('ARM_BASE', [0.025, 0.024, P['upper_length']], radius + P['upper_length']/2 if P['has_sleeve'] else P['upper_length']/2),
               box_bottom('FOREARM', [0.020, 0.023, P['forearm_length']], P['forearm_length']/2)]
    palm = robot.getFromDef('PALM')
    pos, rot = palm.getPosition(), palm.getOrientation()
    bottoms.append(pos[2] - P['palm_radius']*math.hypot(rot[6], rot[7]) - 0.008*abs(rot[8]))
    for i in range(FINGER_COUNT):
        bottoms.extend([box_bottom(f'FINGER_{i}', [0.013, 0.011, P['finger_proximal_length']], P['finger_proximal_length']/2),
                        box_bottom(f'TIP_{i}', [0.013, 0.010, P['finger_distal_length']], P['finger_distal_length']/2)])
    return min(bottoms)


logfile = (OUT / "telemetry.csv").open("w", newline="", encoding="utf-8")
writer = csv.writer(logfile)
writer.writerow(["time_s", "pose", "pitch_rad", "gyro_y_rad_s", "left_rad", "right_rad", "x_m", "y_m", "z_m", "yaw_rad", "left_command", "right_command", 'lean_reference_rad', 'pitch_error_rad', BASE+'_rad', 'elbow_rad', 'wrist_rad', 'arm_floor_clearance_m'] + [f'{part}_{i}_rad' for part in ['finger', 'distal'] for i in range(FINGER_COUNT)] + ['arm_contact_count'])

while robot.step(step) != -1:
    now = robot.getTime()
    rpy = imu.getRollPitchYaw()
    pitch, yaw, pitch_rate = rpy[1], rpy[2], gyro.getValues()[1]
    position = body.getPosition()
    if initial_position is None:
        initial_position = position
        initial_com = body.getCenterOfMass()
    max_pitch = max(max_pitch, abs(pitch))
    max_yaw = max(max_yaw, abs(yaw))
    max_displacement = max(max_displacement, math.hypot(position[0] - initial_position[0], position[1] - initial_position[1]))
    forward, yaw_rate = 0.0, 0.0
    left = right = 0.0
    contact_count = 0
    if MODE == 'dynamics' and P.get('self_collision', False):
        for name, node in arm_nodes.items():
            contacts = node.getContactPoints()
            contact_count += len(contacts)
            if contacts:
                record = arm_contacts.setdefault(name, {'first_time_s': now, 'contact_samples': 0, 'first_point_m': list(contacts[0].point)})
                record['contact_samples'] += len(contacts)
        max_arm_contact_count = max(max_arm_contact_count, contact_count)
    key = keyboard.getKey()
    while key != -1:
        if ord("1") <= key <= ord("4"):
            selected = pose_names[key - ord("1")]
            if command_pose(selected):
                pose_name = selected
        elif key == Keyboard.UP:
            forward = 0.04
        elif key == Keyboard.DOWN:
            forward = -0.04
        elif key == Keyboard.LEFT:
            yaw_rate = 0.5
        elif key == Keyboard.RIGHT:
            yaw_rate = -0.5
        key = keyboard.getKey()

    if MODE == "geometry":
        if VALIDATE:
            stage = min(int(now / 8), len(pose_names) - 1)
            desired_name = pose_names[stage]
            if desired_name != pose_name:
                pose_name = desired_name
                command_pose(pose_name)
            if now % 8 > 7.5 and pose_name not in captured:
                pose = P["poses"][pose_name]
                errors = {joint: abs(encoders[joint].getValue() - pose[joint]) for joint in [BASE, "elbow", "wrist"]}
                measured = robot.getFromDef("PALM").getPosition()
                expected = palm_position(P, {j: encoders[j].getValue() for j in [BASE, "elbow", 'wrist']})
                expected_world = [position[i] + expected[i] for i in range(3)]
                fk_error = math.dist(measured, expected_world)
                finger_errors, finger_fk_errors = {}, []
                for i in range(FINGER_COUNT):
                    for part in ['finger','distal']:
                        finger_errors[f'{part}_{i}'] = abs(encoders[f'{part}_{i}'].getValue()-pose[part])
                    expected_tip = finger_tip_position(P, {j:encoders[j].getValue() for j in [BASE,'elbow','wrist']}, i,
                                                       encoders[f'finger_{i}'].getValue(),encoders[f'distal_{i}'].getValue())
                    tip = arm_nodes[f'TIP_{i}']
                    origin,rotation = tip.getPosition(),tip.getOrientation()
                    measured_tip = [origin[k]+rotation[3*k+2]*P['finger_distal_length'] for k in range(3)]
                    finger_fk_errors.append(math.dist(measured_tip,[position[k]+expected_tip[k] for k in range(3)]))
                errors.update(finger_errors)
                fk_error = max(fk_error,*finger_fk_errors)
                max_fk_error = max(max_fk_error, fk_error)
                max_tracking_error = max(max_tracking_error, *errors.values())
                checks.append({"pose": pose_name, "joint_error_rad": errors, "maximum_fk_error_m": fk_error,
                               "finger_tip_fk_error_m":finger_fk_errors, "palm_position_m": measured, "body_position_m": list(position)})
                robot.exportImage(str(OUT / f"{pose_name}.png"), 95)
                captured.add(pose_name)
                print(f"Geometry checkpoint {pose_name}: tracking={max(errors.values()):.4f} rad; FK error={fk_error:.6f} m", flush=True)
        for side in ["left_wheel", "right_wheel"]:
            motors[side].setVelocity(0)
    else:
        if VALIDATE and finger_scenario:
            sequence = [(0, 'folded'), (6, 'claw_open'), (14, 'claw_closed'), (22, 'claw_open'), (30, 'claw_closed')]
            if SCENARIO == 'claw_cycle_raised':
                sequence = [(0, 'folded'), (6, 'arm_reach'), (20, 'claw_open'), (28, 'claw_closed'), (36, 'claw_open'), (44, 'claw_closed'), (52, 'folded')]
            selected = next(name for when, name in reversed(sequence) if now >= when)
            if selected != pose_name and command_pose(selected):
                pose_name = selected
            for when, name in finger_checkpoints:
                if now >= when and when not in captured:
                    target = dict(P['dynamic_arm_targets']['arm_reach'] if SCENARIO == 'claw_cycle_raised' and name != 'folded' else P['poses']['folded'])
                    target.update(P.get('finger_targets', {}).get(name, {}))
                    errors = {j: abs(encoders[j].getValue()-target[j]) for j in [BASE, 'elbow', 'wrist']}
                    errors.update({f'{part}_{i}': abs(encoders[f'{part}_{i}'].getValue()-target[part]) for part in ['finger', 'distal'] for i in range(FINGER_COUNT)})
                    actual_angles = {j: encoders[j].getValue() for j in [BASE, 'elbow', 'wrist']}
                    finger_fk_errors = []
                    body_rot = body.getOrientation()
                    for i in range(FINGER_COUNT):
                        expected = finger_tip_position(P, actual_angles, i, encoders[f'finger_{i}'].getValue(), encoders[f'distal_{i}'].getValue())
                        expected_world = [position[k]+sum(body_rot[3*k+j]*expected[j] for j in range(3)) for k in range(3)]
                        node = arm_nodes[f'TIP_{i}']
                        node_pos, node_rot = node.getPosition(), node.getOrientation()
                        measured_tip = [node_pos[k]+node_rot[3*k+2]*P['finger_distal_length'] for k in range(3)]
                        finger_fk_errors.append(math.dist(expected_world, measured_tip))
                    max_finger_fk_error = max(max_finger_fk_error, *finger_fk_errors)
                    checks.append({'time_s': now, 'pose': name, 'joint_error_rad': errors,
                                   'finger_tip_fk_error_m': finger_fk_errors,
                                   'joint_angles_rad': {j: encoders[j].getValue() for j in errors},
                                   'palm_position_m': robot.getFromDef('PALM').getPosition()})
                    robot.exportImage(str(OUT / f'{when}_{name}.png'), 95)
                    captured.add(when)
        if VALIDATE and SCENARIO == 'pose_guard':
            for when, name in [(6, 'reach_forward'), (8, 'reach_lower')]:
                if now >= when and name not in guard_attempts:
                    command_pose(name)
                    guard_attempts.add(name)
        if VALIDATE and arm_scenario:
            selected = SCENARIO if 6 <= now < 26 else 'folded'
            if selected != pose_name and command_pose(selected):
                pose_name = selected
            for checkpoint, expected_name in [(22, SCENARIO), (46, 'folded')]:
                if now >= checkpoint and checkpoint not in captured:
                    target = P['dynamic_arm_targets'][SCENARIO] if expected_name == SCENARIO else P['poses']['folded']
                    errors = {j: abs(encoders[j].getValue() - target[j]) for j in [BASE, 'elbow', 'wrist']}
                    checks.append({'time_s': now, 'pose': expected_name, 'joint_error_rad': errors,
                                   'palm_position_m': robot.getFromDef('PALM').getPosition()})
                    robot.exportImage(str(OUT / f'{expected_name}.png'), 95)
                    captured.add(checkpoint)
        # This is an initial control experiment, not a commissioned controller.
        left_angle, right_angle = encoders["left_wheel"].getValue(), encoders["right_wheel"].getValue()
        joint_position = (left_angle + right_angle) / 2
        left_speed, right_speed = (left_angle - last_left_angle) / dt, (right_angle - last_right_angle) / dt
        last_left_angle, last_right_angle = left_angle, right_angle
        joint_speed = (left_speed + right_speed) / 2
        max_wheel_speed = max(max_wheel_speed, abs(left_speed), abs(right_speed))
        # Wheel encoders are relative to the pitching chassis. Include body pitch
        # when estimating axle travel under the no-slip rolling assumption.
        wheel_position = (joint_position + pitch) * P["wheel_radius"]
        if odometry_offset is None:
            odometry_offset = wheel_position
        wheel_position -= odometry_offset
        velocity = (wheel_position - last_wheel_position) / dt
        last_wheel_position = wheel_position
        if VALIDATE and SCENARIO == 'steering':
            yaw_rate = 0.4 if 6 <= now < 10 else (-0.4 if 14 <= now < 18 else 0.0)
        if VALIDATE and SCENARIO == 'drive':
            forward = 0.04 if 6 <= now < 12 else (-0.04 if 18 <= now < 24 else 0.0)
        if VALIDATE and SCENARIO == 'push' and 6 <= now < 6.15:
            body.addForceWithOffset([1.0, 0, 0], [0, 0, 0.04], True)
        position_reference += forward * dt
        gains = P["balance_gains"]
        # Use measured joint angles, not future targets, to align total COM over
        # the axle. Reference rate compensates the deliberately slow arm motion.
        if not P['has_sleeve']:
            angles = {j: encoders[j].getValue() for j in [BASE, 'elbow', 'wrist']}
            angles.update({part: sum(encoders[f'{part}_{i}'].getValue() for i in range(FINGER_COUNT))/FINGER_COUNT for part in ['finger', 'distal']})
            modeled = mass_properties(P, joint_angles=angles)
            max_requested_lean = max(max_requested_lean, abs(modeled['lean_to_align_com_rad']))
            lean_reference = max(-P.get('dynamic_lean_limit_rad', 0.30), min(P.get('dynamic_lean_limit_rad', 0.30), modeled['lean_to_align_com_rad']))
            if int(now / dt) % 8 == 0:
                rot = body.getOrientation()
                com_world = [position[i] + sum(rot[3*i+j]*modeled['com_body_m'][j] for j in range(3)) for i in range(3)]
                max_com_model_error = max(max_com_model_error, math.dist(com_world, body.getCenterOfMass()))
        lean_rate = (lean_reference - last_lean_reference) / dt
        last_lean_reference = lean_reference
        max_pitch_error = max(max_pitch_error, abs(pitch - lean_reference))
        clearance = arm_floor_clearance()
        min_arm_clearance = min(min_arm_clearance, clearance)
        correction = (gains["pitch"] * (pitch - lean_reference) + gains["pitch_rate"] * (pitch_rate - lean_rate)
                      + gains["position"] * (wheel_position - position_reference)
                      + gains["velocity"] * (velocity - forward))
        torque_control = P.get("wheel_control") == "torque"
        if torque_control:
            limit = P["wheel_torque_limit"]
            common = max(-limit, min(limit, correction / 2))
            remaining = limit - abs(common)
            turn = gains.get("yaw_rate", 0.03) * (yaw_rate - gyro.getValues()[2])
            turn = max(-remaining, min(remaining, turn))
            left, right = common - turn, common + turn
            if abs(left_speed) >= P["wheel_speed_limit"] and left * left_speed > 0:
                left = 0
            if abs(right_speed) >= P["wheel_speed_limit"] and right * right_speed > 0:
                right = 0
            max_applied_wheel_torque = max(max_applied_wheel_torque, abs(left), abs(right))
        else:
            left, right = wheel_speeds(forward, yaw_rate, P["wheel_radius"], P["wheel_track"])
            left, right = saturate_pair(correction + left, correction + right, P["wheel_speed_limit"])
        if abs(pitch) > 0.85 or abs(rpy[0]) > 0.85:
            fallen = True
            left = right = 0
        if torque_control:
            motors["left_wheel"].setTorque(left)
            motors["right_wheel"].setTorque(right)
        else:
            motors["left_wheel"].setVelocity(left)
            motors["right_wheel"].setVelocity(right)

    advance_pose()

    if int(now / dt) % 8 == 0:
        writer.writerow([now, pose_name, pitch, pitch_rate, encoders["left_wheel"].getValue(), encoders["right_wheel"].getValue(), *position, yaw, left, right, lean_reference, pitch-lean_reference, *[encoders[j].getValue() for j in [BASE, 'elbow', 'wrist']], arm_floor_clearance() if MODE == 'dynamics' else '', *[encoders[f'{part}_{i}'].getValue() for part in ['finger', 'distal'] for i in range(FINGER_COUNT)], contact_count])
    robot.setLabel(4, f"Pose: {pose_name} | t={now:.1f}s | pitch={math.degrees(pitch):.1f} deg", 0.02, 0.25, 0.04, 0x172B4D)

    duration = 32 if MODE == "geometry" else (72 if SCENARIO == 'claw_cycle_raised' else 48) if finger_scenario else P.get('dynamic_arm_validation_duration_s', 48) if arm_scenario else P.get("validation_duration_s", 30)
    if VALIDATE and (now >= duration or fallen):
        orientation_snapshot = body.getOrientation()
        velocity_snapshot = body.getVelocity()
        gyro_snapshot = list(gyro.getValues())
        camera.enable(step)
        robot.step(step)
        camera.saveImage(str(OUT / "camera-view.png"), 95)
        camera.disable()
        robot.step(step)
        robot.exportImage(str(OUT / "final.png"), 95)
        geometry_pass = len(checks) == 4 and max_tracking_error < 0.12 and max_fk_error < 0.002 and max_displacement<1e-9
        final_drift = math.hypot(position[0] - initial_position[0], position[1] - initial_position[1])
        balance_checks = {'completed_duration': now >= duration, 'did_not_fall': not fallen,
                          'peak_pitch_under_20_deg': max_pitch < math.radians(20),
                          'final_pitch_under_1_deg': abs(pitch) < math.radians(1),
                          'final_drift_under_5_cm': final_drift < 0.05,
                          'wheel_speed_within_limit': max_wheel_speed <= P['wheel_speed_limit'] + 0.5}
        if SCENARIO == 'steering':
            balance_checks.update({'heading_changed_over_1_rad': max_yaw > 1.0,
                                   'returned_heading_under_10_deg': abs(yaw) < math.radians(10)})
        if SCENARIO == 'drive':
            balance_checks['travelled_over_5_cm'] = max_displacement > 0.05
        if P.get('self_collision', False):
            balance_checks['no_arm_contacts_detected'] = max_arm_contact_count == 0
        if SCENARIO == 'pose_guard':
            balance_checks.update({'both_large_lean_poses_rejected': rejected_pose_commands == ['reach_forward', 'reach_lower'],
                                   'arm_remained_folded': all(abs(encoders[j].getValue()-P['poses']['folded'][j]) < 0.03 for j in [BASE, 'elbow', 'wrist'])})
        if arm_scenario:
            balance_checks.update({'both_arm_checkpoints_reached': len(checks) == 2 and all(max(c['joint_error_rad'].values()) < 0.03 for c in checks),
                                   'arm_stayed_over_1_cm_above_floor': min_arm_clearance > 0.01,
                                   'peak_pitch_error_under_5_deg': max_pitch_error < math.radians(5),
                                   'com_model_matches_simulator_under_1_mm': max_com_model_error < 0.001,
                                   'modeled_lean_within_guard': max_requested_lean <= P.get('dynamic_lean_limit_rad', 0.30),
                                   'no_pose_commands_rejected': not rejected_pose_commands})
        if finger_scenario:
            balance_checks.update({'all_claw_checkpoints_reached': len(checks) == len(finger_checkpoints) and all(max(c['joint_error_rad'].values()) < 0.03 for c in checks),
                                   'finger_tip_fk_under_2_mm': max_finger_fk_error < 0.002,
                                   'arm_stayed_over_1_cm_above_floor': min_arm_clearance > 0.01,
                                   'peak_pitch_error_under_5_deg': max_pitch_error < math.radians(5),
                                   'com_model_matches_simulator_under_1_mm': max_com_model_error < 0.001,
                                   'modeled_lean_within_guard': max_requested_lean <= P['dynamic_lean_limit_rad'],
                                   'no_pose_commands_rejected': not rejected_pose_commands})
        dynamics_pass = all(balance_checks.values())
        report = {"finger_count": FINGER_COUNT, "finger_angles_rad": list(finger_angles(P)), "wrist_roll_present": False, "model": LABEL, "sleeve_present": P["has_sleeve"], "mode": MODE, "scenario": SCENARIO, "root_fixed": MODE == "geometry", "time_s": now,
                  "max_abs_pitch_rad": max_pitch, "fallen": fallen, "final_yaw_rad": yaw,
                  "final_position_m": position, "checkpoints": checks,
                  "final_body_orientation": orientation_snapshot,
                  "final_body_velocity_world": velocity_snapshot,
                  "final_imu_rpy": list(rpy), "final_gyro": gyro_snapshot,
                  "initial_center_of_mass_world_m": initial_com,
                  "mass_model": mass_properties(P), "balance_gains": P["balance_gains"],
                  "wheel_control": P.get("wheel_control", "velocity"), "max_wheel_joint_speed_rad_s": max_wheel_speed,
                  "max_applied_wheel_torque_nm": max_applied_wheel_torque,
                  "max_abs_yaw_rad": max_yaw, "max_displacement_m": max_displacement,
                  'max_pitch_error_rad': max_pitch_error, 'min_arm_floor_clearance_m': min_arm_clearance if MODE == 'dynamics' else None,
                  'max_com_model_error_m': max_com_model_error, 'rejected_pose_commands': rejected_pose_commands,
                  'max_modeled_lean_rad': max_requested_lean, 'source_sha256': SOURCE_HASHES,
                  'self_collision_enabled': P.get('self_collision', False) and MODE!='geometry', 'arm_contact_events': arm_contacts,
                  'geometry_direct_joint_positioning':MODE=='geometry',
                  'max_arm_contact_count': max_arm_contact_count,
                  'max_finger_tip_fk_error_m': max_finger_fk_error,
                  'dynamic_arm_target': P.get('dynamic_arm_targets', {}).get(SCENARIO),
                  "final_drift_m": final_drift,
                  "dynamics_checks": balance_checks if MODE == 'dynamics' else None,
                  "dynamics_validation_passed": dynamics_pass if MODE == 'dynamics' else None,
                  "geometry_joint_and_fk_check_passed": geometry_pass if MODE == "geometry" else None,
                  "limitations": ["Geometry fixture is kinematic and does not validate torque, balance or driving.", "Self-collision excludes directly joined solids; mounting/joint pockets still need CAD checks. Only recorded trajectories are checked.", "Motor limits, masses and control gains are provisional.", "Camera pose/field of view is not validated by a saved frame alone."]}
        (OUT / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        logfile.close()
        success = geometry_pass if MODE == "geometry" else dynamics_pass
        print(json.dumps(report), flush=True)
        robot.simulationQuit(0 if success else 1)
        break

logfile.close()
