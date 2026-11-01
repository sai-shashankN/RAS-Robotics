"""Prepared-pose, contact-only Alpha grasp experiment. No object pose writes."""

from pathlib import Path
import csv
import hashlib
import json
import math
import sys

from controller import Supervisor
from control_math import finger_angles
from grasp_planning import loaded_reference, plan_load
from capture_planning import plan_capture, translation_reference
from approach_planning import plan_approach, staging_target
from return_planning import ReturnSequence

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = ROOT.parents[1]
P = json.loads((ROOT/'parameters.json').read_text())
E_PATH = Path(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--experiment=')), str(ROOT/'grasp_parameters.json'))).resolve()
SETUP_PATH = Path(next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--setup=')), str(ROOT/'grasp_setup.json'))).resolve()
CASE = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--case=')), None)
STUDY = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--study=')), 'grasp_sweep')
FINGER_COUNT = len(finger_angles(P))
E = json.loads(E_PATH.read_text())
SETUP = json.loads(SETUP_PATH.read_text())
NEGATIVE = '--negative-control' in sys.argv or E.get('open_claw_control', False)
OUT = ROOT/'results'/STUDY/CASE if CASE else ROOT/'results'/('grasp_negative' if NEGATIVE else 'grasp')
OUT.mkdir(parents=True, exist_ok=True)
sources = [ROOT.parent/'common'/name for name in ['grasp_controller.py', 'grasp_planning.py', 'capture_planning.py', 'approach_planning.py', 'return_planning.py', 'control_math.py', 'mass_properties.py', 'generate_worlds.py']]
sources += [ROOT/'scripts/generate_grasp_worlds.py', ROOT/'parameters.json', E_PATH, SETUP_PATH]
WORLD_PATH = Path(SETUP['validation_world']) if 'validation_world' in SETUP else None
if WORLD_PATH and NEGATIVE and not CASE:
    WORLD_PATH = WORLD_PATH.with_name('alpha_grasp_negative_validation.wbt')
HASHES = {str(path.relative_to(WORKSPACE)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
VALIDATE = '--validate' in sys.argv
robot = Supervisor()
step = int(robot.getBasicTimeStep())
dt = step/1000
body = robot.getSelf()
imu, gyro = robot.getDevice('body_imu'), robot.getDevice('body_gyro')
imu.enable(step)
gyro.enable(step)
names = ['shoulder', 'elbow', 'wrist']+[f'{part}_{i}' for part in ['finger', 'distal'] for i in range(FINGER_COUNT)]
motors, sensors = {}, {}
for name in names+['left_wheel', 'right_wheel']:
    motors[name] = robot.getDevice(name)
    sensors[name] = robot.getDevice(name+'_encoder')
    sensors[name].enable(step)
for side in ['left_wheel', 'right_wheel']:
    motors[side].setPosition(float('inf'))
    motors[side].setVelocity(0)
    motors[side].setTorque(0)
commanded = dict(SETUP['start_pose'])
for name in names:
    key = name.split('_')[0]
    motors[name].setPosition(commanded[key])
nodes = {name: robot.getFromDef(name) for name in ['ARM_BASE', 'FOREARM', 'PALM']+[f'{part}_{i}' for part in ['FINGER', 'TIP'] for i in range(FINGER_COUNT)]}
ball = robot.getFromDef('TEST_OBJECT')
stand, tray = robot.getFromDef('OBJECT_STAND'), robot.getFromDef('CATCH_TRAY')
S = dict(E['schedule_s'])
approach = plan_approach(P, E)
returning = ReturnSequence(P,E,approach)
return_object_start = None
return_object_displacement = 0.0
return_blocked_pose_errors = []
return_contact_samples = return_unsupported_samples = 0
return_body_start = None
return_max_body_motion = 0.0
if approach['enabled']:
    S = {key: value+approach['ready_s'] for key, value in S.items()}
    nodes['BODY'] = body
    children = body.getField('children')
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node.getTypeName() == 'HingeJoint':
            endpoint = node.getField('endPoint').getSFNode()
            name = endpoint.getField('name').getSFString()
            if name.endswith('wheel'):
                nodes['WHEEL_'+name] = endpoint
staging_samples = {}
preclose_max_contacts = 0
preclose_max_displacement = 0.0
approach_rejected = approach['enabled'] and not approach['within_lean_limit']
raised = dict(P['dynamic_arm_targets']['arm_reach'])
opened = dict(raised, **P['finger_targets']['claw_open'])
closed = dict(raised, **P['finger_targets']['claw_closed'])
lifted = E['lift_pose']
load_plan = plan_load(P, E)
load_rejected = E.get('enforce_payload_lean_guard', True) and not load_plan['within_lean_limit']
capture_plan = plan_capture(E)
alignment_rejected = capture_plan['enabled'] and not capture_plan['admitted']
alignment_failed = alignment_evaluated = False
alignment_samples = []
alignment_max_error = alignment_max_speed = None
alignment_wait = 0.0
last_angles = [0, 0]
last_position = offset = None
initial_position = None
last_reference = SETUP['start_body_pitch_rad']
max_pitch = max_error = max_speed = max_torque = max_drift = 0.0
unexpected_contacts = 0
unexpected_by_node = {}
unexpected_examples = []
hold_samples = []
release_samples = []
preclose_supported = False
fell = False
snapshots = []
captured = set()
payload_fraction = 0.0
prelift_object_z = None
max_contacting_fingers = 0
max_object_displacement = 0.0
logfile = (OUT/'telemetry.csv').open('w', newline='', encoding='utf-8')
tracefile = (OUT/'contact-trace.jsonl').open('w',encoding='utf-8') if E.get('trace_contacts',False) else None
trace_frames = set()
writer = csv.writer(logfile)
writer.writerow(['time_s', 'stage', 'pitch_rad', 'reference_rad', 'body_x_m', 'body_y_m',
                 'object_x_m', 'object_y_m', 'object_z_m', 'object_vz_m_s', 'stand_gap_m',
                 'object_supported', 'finger_contact_count', 'contacting_fingers', 'unexpected_arm_contacts',
                 'object_palm_x_m', 'object_palm_y_m', 'object_palm_z_m', 'left_torque_nm', 'right_torque_nm']+names+['payload_fraction', 'wheel_odometry_m', 'wheel_velocity_m_s', 'raw_reference_rad', 'position_reference_m', 'alignment_error_m'])


def points(node):
    return [list(c.point) for c in node.getContactPoints()]


def matches(point, candidates):
    return any(math.dist(point, other)<0.0001 for other in candidates)


while robot.step(step) != -1:
    now = robot.getTime()
    if now < S['close']:
        stage, target = 'open', opened
    elif now < S['lift']:
        stage, target = 'close', closed
    elif now < S['lower']:
        stage, target = 'lift_hold', lifted
    elif now < S['release']:
        stage, target = 'lower', closed
    else:
        stage, target = 'release', opened
    if NEGATIVE:
        target = dict(target, **P['finger_targets']['claw_open'])
    staged = staging_target(approach, now)
    if staged:
        stage, target = staged
    if load_rejected:
        stage, target = 'rejected_load', opened
    if alignment_rejected or alignment_failed:
        stage, target = 'rejected_alignment', opened
    if approach['enabled'] and (load_rejected or alignment_rejected or approach_rejected):
        stage, target = 'rejected_plan', approach['folded_pose']
    rpy = imu.getRollPitchYaw()
    pitch, rates = rpy[1], gyro.getValues()
    pos, ball_pos = body.getPosition(), ball.getPosition()
    max_object_displacement = max(max_object_displacement, math.dist(ball_pos, SETUP['initial_object_position_m']))
    if initial_position is None:
        initial_position = list(pos)
    drift = math.hypot(pos[0]-initial_position[0], pos[1]-initial_position[1])
    max_drift = max(max_drift, drift)
    ball_points = points(ball)
    # Static fixtures do not provide a symmetric contact list here. Use actual
    # sphere contact points on their known upper faces as support witnesses.
    supported = any((abs(point[2]-SETUP['stand_top_m'])<.0001 and math.hypot(point[0]-SETUP['initial_object_position_m'][0], point[1]-SETUP['initial_object_position_m'][1])<=E['stand_radius_m']+.0001)
                    or (abs(point[2]-SETUP['tray_top_m'])<.0001 and abs(point[0]-SETUP['initial_object_position_m'][0])<=.10 and abs(point[1]-SETUP['initial_object_position_m'][1])<=.08)
                    for point in ball_points)
    ball_speed = math.sqrt(sum(v*v for v in ball.getVelocity()[:3]))
    # Settled/sleeping bodies can have empty contact lists. A sphere at rest
    # within 0.1 mm of a known support face is a geometric support witness.
    sphere_bottom = ball_pos[2]-E['object_radius_m']
    stand_rest = abs(sphere_bottom-SETUP['stand_top_m'])<.0001 and math.hypot(ball_pos[0]-SETUP['initial_object_position_m'][0], ball_pos[1]-SETUP['initial_object_position_m'][1])<E['stand_radius_m'] and ball_speed<.005
    tray_rest = abs(sphere_bottom-SETUP['tray_top_m'])<.0001 and abs(ball_pos[0]-SETUP['initial_object_position_m'][0])<.10 and abs(ball_pos[1]-SETUP['initial_object_position_m'][1])<.08 and ball_speed<.005
    supported = supported or stand_rest or tray_rest
    contacting = set()
    finger_contacts = 0
    bad_contacts = 0
    matched_contacts = []
    for name, node in nodes.items():
        for point in points(node):
            # Ground contacts may lie below z=0 because of contact penetration.
            # Fixture projections are excluded from this floor classification.
            inside_fixture = (abs(point[0]-SETUP['initial_object_position_m'][0])<.105
                              and abs(point[1]-SETUP['initial_object_position_m'][1])<.085)
            if name.startswith('WHEEL_') and point[2]<=.0001 and not inside_fixture:
                continue
            if name.startswith(('FINGER_', 'TIP_')) and matches(point, ball_points):
                finger_contacts += 1
                contacting.add(int(name[-1]))
                if tracefile:
                    origin,rotation = node.getPosition(),node.getOrientation()
                    local_point = [sum(rotation[3*j+i]*(point[j]-origin[j]) for j in range(3)) for i in range(3)]
                    matched_contacts.append({'segment':name,'point_world_m':point,'point_segment_local_m':local_point})
            else:
                bad_contacts += 1
                unexpected_by_node[name] = unexpected_by_node.get(name, 0)+1
                if len(unexpected_examples)<5:
                    unexpected_examples.append({'time_s': now, 'node': name, 'point_m': point})
    unexpected_contacts += bad_contacts
    max_contacting_fingers = max(max_contacting_fingers, len(contacting))
    if S['lift']-.5 <= now < S['lift']:
        prelift_object_z = ball_pos[2]
    palm = nodes['PALM']
    palm_pos, palm_rot = palm.getPosition(), palm.getOrientation()
    relative = [sum(palm_rot[3*j+i]*(ball_pos[j]-palm_pos[j]) for j in range(3)) for i in range(3)]
    gap = ball_pos[2]-E['object_radius_m']-SETUP['stand_top_m']
    if tracefile and now>=S['close']-.5:
        on_stand = stand_rest or any(abs(point[2]-SETUP['stand_top_m'])<.0001 and math.hypot(point[0]-SETUP['initial_object_position_m'][0],point[1]-SETUP['initial_object_position_m'][1])<=E['stand_radius_m']+.0001 for point in ball_points)
        on_tray = tray_rest or any(abs(point[2]-SETUP['tray_top_m'])<.0001 and abs(point[0]-SETUP['initial_object_position_m'][0])<=.10 and abs(point[1]-SETUP['initial_object_position_m'][1])<=.08 for point in ball_points)
        tracefile.write(json.dumps({'time_s':now,'stage':stage,'scheduled_close_s':S['close'],'scheduled_lift_s':S['lift'],
                                    'scheduled_release_s':S['release'],'object_position_m':list(ball_pos),
                                    'object_orientation_matrix':ball.getOrientation(),'object_velocity':list(ball.getVelocity()),
                                    'object_palm_m':relative,'stand_gap_m':gap,'on_stand':bool(on_stand),'on_tray':bool(on_tray),
                                    'contacting_fingers':sorted(contacting),'finger_contacts':matched_contacts,
                                    'unintended_arm_contact_points':bad_contacts,
                                    'joint_angles_rad':{name:sensors[name].getValue() for name in names}})+'\n')
        for tag,ready in [('first_contact',finger_contacts>0),('before_lift',now>=S['lift']-.05),('release_contact',now>=S['release']+.2)]:
            if ready and tag not in trace_frames:
                robot.exportImage(str(OUT/(tag+'.png')),95)
                trace_frames.add(tag)
    if 4 <= now <= 5.5 and supported:
        preclose_supported = True
    if S['hold_start'] <= now <= S['hold_end']:
        hold_samples.append({'time_s': now, 'stand_gap_m': gap, 'supported': supported,
                             'contacting_fingers': len(contacting), 'relative_m': relative, 'object_position_m': list(ball_pos)})
    if now >= S['finish']-3:
        release_samples.append({'time_s': now, 'contacts': finger_contacts, 'supported': supported,
                                'speed_m_s': math.sqrt(sum(v*v for v in ball.getVelocity()[:3])), 'object_z_m': ball_pos[2]})

    angles = {j: sensors[j].getValue() for j in ['shoulder', 'elbow', 'wrist']}
    angles.update({part: sum(sensors[f'{part}_{i}'].getValue() for i in range(FINGER_COUNT))/FINGER_COUNT for part in ['finger', 'distal']})
    # Optional known-payload feedforward uses the specified mass/location only.
    # Object world pose is telemetry, never balance feedback or a pose command.
    requested_load = 1.0 if len(contacting)>=2 and not supported and now<S['release'] and not (load_rejected or alignment_rejected or alignment_failed) else 0.0
    payload_fraction += max(-dt/E.get('payload_ramp_s', 1.0), min(dt/E.get('payload_ramp_s', 1.0), requested_load-payload_fraction))
    raw_reference = loaded_reference(P, E, angles, payload_fraction if E['payload_compensation'] else 0.0)
    reference = max(-P['dynamic_lean_limit_rad'], min(P['dynamic_lean_limit_rad'], raw_reference))
    reference_rate = (reference-last_reference)/dt
    last_reference = reference
    wheel_angles = [sensors[j].getValue() for j in ['left_wheel', 'right_wheel']]
    speeds = [(a-b)/dt for a, b in zip(wheel_angles, last_angles)]
    last_angles = wheel_angles
    wheel_position = P['wheel_radius']*(sum(wheel_angles)/2+pitch)
    if offset is None:
        offset = wheel_position
    wheel_position -= offset
    velocity = 0.0 if last_position is None else (wheel_position-last_position)/dt
    last_position = wheel_position
    position_reference, velocity_reference = translation_reference(capture_plan, now-approach['ready_s'])
    if load_rejected or approach_rejected:
        position_reference = velocity_reference = 0.0
    alignment_error = wheel_position-position_reference
    if capture_plan['enabled'] and not alignment_evaluated:
        alignment_samples.append((now, abs(alignment_error), abs(velocity)))
        alignment_samples = [row for row in alignment_samples if row[0]>=now-.5]
    if capture_plan['enabled'] and now >= S['close'] and not alignment_evaluated:
        alignment_max_error = max((r[1] for r in alignment_samples), default=None)
        alignment_max_speed = max((r[2] for r in alignment_samples), default=None)
        settled = (alignment_samples and alignment_samples[-1][0]-alignment_samples[0][0]>=.45
                   and alignment_max_error<=.0005 and alignment_max_speed<=.003)
        if alignment_rejected or load_rejected or settled:
            alignment_evaluated = True
        elif now < approach['alignment_deadline_s']:
            alignment_wait += dt
            S = {key: value+dt for key, value in S.items()}
            stage, target = 'aligning', opened
        else:
            alignment_evaluated = alignment_failed = True
            stage, target = 'rejected_alignment', opened
    if approach['enabled']:
        if now<S['close']:
            preclose_max_contacts = max(preclose_max_contacts, finger_contacts)
            preclose_max_displacement = max(preclose_max_displacement, math.dist(ball_pos, SETUP['initial_object_position_m']))
        for tag, when, expected in [('folded', 2.0, approach['folded_pose']),
                                    ('raised_closed', approach['open_start_s']-1, approach['raised_closed_pose']),
                                    ('raised_open', approach['ready_s']-1, approach['open_pose'])]:
            if when-.5<=now<=when:
                error = max(abs(sensors[name].getValue()-expected[name.split('_')[0]]) for name in names)
                staging_samples.setdefault(tag, []).append(error)
    def pose_error(expected):
        return max(abs(sensors[name].getValue()-expected[name.split('_')[0]]) for name in names)
    recovery = returning.update(now,S['finish'],wheel_position,velocity,
                                finger_contacts==0 and supported and ball_speed<.02,
                                pose_error,pitch-reference,load_rejected or alignment_rejected or approach_rejected)
    if recovery:
        stage,target,position_reference,velocity_reference = recovery
        alignment_error = wheel_position-position_reference
        if return_object_start is None:
            return_object_start = list(ball_pos)
            return_body_start = list(pos)
        return_object_displacement = max(return_object_displacement,math.dist(ball_pos,return_object_start))
        return_contact_samples += finger_contacts
        return_unsupported_samples += int(not supported)
        return_max_body_motion = max(return_max_body_motion,math.dist(pos,return_body_start))
        if returning.state in ['blocked_release','blocked_retreat','hold_control']:
            return_blocked_pose_errors.append(pose_error(opened))
    gains = dict(P['balance_gains'], **E.get('grasp_balance_gains', {}))
    correction = (gains['pitch']*(pitch-reference)+gains['pitch_rate']*(rates[1]-reference_rate)
                  +gains['position']*alignment_error+gains['velocity']*(velocity-velocity_reference))
    limit = P['wheel_torque_limit']
    common = max(-limit, min(limit, correction/2))
    turn = max(-limit+abs(common), min(limit-abs(common), -gains['yaw_rate']*rates[2]))
    commands = [common-turn, common+turn]
    max_speed = max(max_speed, *map(abs, speeds))
    max_pitch = max(max_pitch, abs(pitch))
    max_error = max(max_error, abs(pitch-reference))
    if abs(pitch)>.85 or abs(rpy[0])>.85:
        fell = True
    for i, name in enumerate(['left_wheel', 'right_wheel']):
        if fell or (abs(speeds[i])>=P['wheel_speed_limit'] and commands[i]*speeds[i]>0):
            commands[i] = 0
        motors[name].setTorque(commands[i])
    max_torque = max(max_torque, *map(abs, commands))
    duration = max(abs(target[j]-commanded[j])/(P['dynamic_arm_rate_rad_s'] if j in ['shoulder', 'elbow', 'wrist'] else P['finger_target_rate_rad_s']) for j in commanded)
    fraction = min(1, dt/duration) if duration else 1
    for j in commanded:
        commanded[j] += (target[j]-commanded[j])*fraction
    for name in names:
        motors[name].setPosition(commanded[name.split('_')[0]])
    if int(now/dt)%8 == 0:
        writer.writerow([now, stage, pitch, reference, pos[0], pos[1], *ball_pos, ball.getVelocity()[2], gap, int(supported), finger_contacts, len(contacting), bad_contacts, *relative, *commands]+[sensors[name].getValue() for name in names]+[payload_fraction, wheel_position, velocity, raw_reference, position_reference, alignment_error])
    robot.setLabel(0, 'MODEL ALPHA | CONTACT GRASP TEST', .02, .03, .055, 0x172B4D)
    robot.setLabel(1, f'{CASE or "reference"} | {stage} | t={now:.1f}s | sphere: {E["object_mass_kg"]*1000:g} g | pitch={math.degrees(pitch):.1f} deg', .02, .12, .032, 0x172B4D)
    robot.setLabel(2, f'Finger contacts: {len(contacting)} | support contact: {supported} | stand gap: {gap*1000:.1f} mm', .02, .19, .03, 0x172B4D)
    robot.setLabel(3, 'Prepared pose. Free sphere; no attachment or pose reset. Provisional physics.', .02, .94, .03, 0x8B2430)
    for when in [5, 10, 24, 34, 44]:
        if now-alignment_wait-approach['ready_s']>=when and when not in captured:
            snapshots.append({'time_s': now, 'stage': stage, 'object_position_m': list(ball_pos),
                              'object_contact_points_m': ball_points,
                              'stand_gap_m': gap, 'contacting_fingers': sorted(contacting), 'supported': supported})
            robot.exportImage(str(OUT/f'{when}_{stage}.png'), 95)
            captured.add(when)
    mission_complete = returning.done if returning.enabled else now>=S['finish']
    if mission_complete or fell:
        hold_time = hold_samples[-1]['time_s']-hold_samples[0]['time_s'] if hold_samples else 0
        slip = max((math.dist(row['relative_m'], hold_samples[0]['relative_m']) for row in hold_samples), default=None)
        endpoint_goal = 0.0 if returning.enabled or load_rejected or alignment_rejected or approach_rejected else capture_plan['translation_m']
        endpoint_error = math.hypot(pos[0]-initial_position[0]-endpoint_goal, pos[1]-initial_position[1])
        position_check = 'final_body_position_error_under_5_mm' if approach['enabled'] else 'final_body_drift_under_5_cm'
        checks = {'capture_alignment_admitted_and_settled': not (alignment_rejected or alignment_failed),
                  'payload_plan_within_lean_limit': load_plan['within_lean_limit'],
                  'completed_duration': now>=S['finish'], 'did_not_fall': not fell,
                  'peak_pitch_under_20_deg': max_pitch<math.radians(20), 'pitch_error_under_5_deg': max_error<math.radians(5),
                  'final_pitch_error_under_1_deg': abs(pitch-reference)<math.radians(1), position_check: endpoint_error<.005 if approach['enabled'] else drift<.05,
                  'wheel_speed_within_limit': max_speed<=P['wheel_speed_limit']+.5,
                  'object_initially_supported': preclose_supported, 'no_unintended_arm_contacts': unexpected_contacts==0,
                  'held_for_at_least_5_s': hold_time>=E['minimum_hold_s'],
                  'object_lifted_above_stand': bool(hold_samples) and min(row['stand_gap_m'] for row in hold_samples)>=E['minimum_lift_m'],
                  'arm_motion_added_at_least_6_mm_lift': prelift_object_z is not None and bool(hold_samples) and min(row['object_position_m'][2] for row in hold_samples)-prelift_object_z>=E['minimum_lift_m'],
                  'object_unsupported_through_hold': bool(hold_samples) and not any(row['supported'] for row in hold_samples),
                  'at_least_two_fingers_through_hold': bool(hold_samples) and min(row['contacting_fingers'] for row in hold_samples)>=2,
                  'object_slip_under_3_mm': slip is not None and slip<.003,
                  'released_and_settled': bool(release_samples) and all(row['contacts']==0 and row['supported'] and row['speed_m_s']<.02 for row in release_samples)}
        if approach['enabled']:
            checks.update({'started_folded_and_deployed': len(staging_samples)==3 and all(rows and max(rows)<.03 for rows in staging_samples.values()),
                           'deployment_plan_within_lean_limit': not approach_rejected,
                           'object_undisturbed_before_closing': preclose_max_contacts==0 and preclose_max_displacement<.002,
                           'no_unintended_robot_contacts': unexpected_contacts==0})
        if returning.enabled:
            checks.update({'returned_home_and_folded':returning.successful,
                           'no_postrelease_finger_contact':return_contact_samples==0,
                           'object_supported_during_return':return_unsupported_samples==0,
                           'final_folded_joint_error_under_0p03_rad':pose_error(approach['folded_pose'])<.03})
        report = {'finger_count':FINGER_COUNT, 'design_validation_scope':'Grasp scripts are exploratory after the two-finger revision; precise pickup/placement is not certified.', 'passed': all(checks.values()), 'checks': checks, 'time_s': now, 'source_sha256': HASHES, 'case': CASE, 'study': STUDY,
                  'grasp_balance_gains': gains,
                  'load_plan': load_plan, 'load_rejected': load_rejected,
                  'capture_plan': capture_plan, 'alignment_rejected': alignment_rejected, 'alignment_failed_to_settle': alignment_failed,
                  'preclose_alignment_max_error_m': alignment_max_error, 'preclose_alignment_max_speed_m_s': alignment_max_speed,
                  'alignment_wait_s': alignment_wait, 'schedule_actual_s': S,
                  'approach_plan': approach, 'deployment_tracking_error_rad': {key: max(rows) for key, rows in staging_samples.items()},
                  'preclose_max_contacts': preclose_max_contacts, 'preclose_max_object_displacement_m': preclose_max_displacement,
                  'contact_monitor_nodes': list(nodes),
                  'return_sequence':returning.report(), 'return_object_start_m':return_object_start,
                  'return_object_displacement_m':return_object_displacement,
                  'return_finger_contact_samples':return_contact_samples,'return_unsupported_samples':return_unsupported_samples,
                  'return_max_body_motion_m':return_max_body_motion,
                  'return_blocked_max_open_pose_error_rad':max(return_blocked_pose_errors,default=None),
                  'final_folded_joint_error_rad':pose_error(approach['folded_pose']),
                  'world_path': str(WORLD_PATH) if WORLD_PATH else None,
                  'world_sha256': hashlib.sha256(WORLD_PATH.read_bytes()).hexdigest() if WORLD_PATH else None,
                  'negative_control': NEGATIVE,
                  'experiment': E, 'setup': SETUP, 'max_pitch_deg': math.degrees(max_pitch), 'max_pitch_error_deg': math.degrees(max_error),
                  'max_body_drift_m': max_drift, 'final_body_drift_m': drift, 'max_wheel_speed_rad_s': max_speed,
                  'final_body_position_error_m': endpoint_error,
                  'max_wheel_torque_nm': max_torque, 'hold_time_s': hold_time, 'max_hold_relative_slip_m': slip,
                  'min_hold_stand_gap_m': min((row['stand_gap_m'] for row in hold_samples), default=None),
                  'min_hold_contacting_fingers': min((row['contacting_fingers'] for row in hold_samples), default=0),
                  'max_contacting_fingers': max_contacting_fingers, 'prelift_object_z_m': prelift_object_z,
                  'max_object_displacement_m': max_object_displacement,
                  'arm_added_lift_m': min((row['object_position_m'][2] for row in hold_samples), default=0)-(prelift_object_z or 0),
                  'unexpected_contact_samples': unexpected_contacts, 'unexpected_contact_examples': unexpected_examples,
                  'unexpected_contact_counts_by_node': unexpected_by_node,
                  'snapshots': snapshots, 'final_object_position_m': list(ball_pos),
                  'limitations': ['Prepared-pose scripted experiment; no perception/navigation/ground pickup.', 'Provisional rigid contact/friction, mass and actuators; no real hardware validation.', 'All object motion after initial placement comes from gravity and contacts; no artificial attachment.']}
        report['negative_control_passed'] = (NEGATIVE and max_contacting_fingers==0 and max_object_displacement<.002 and not checks['object_lifted_above_stand']
                                             and all(checks[k] for k in ['completed_duration', 'did_not_fall', 'object_initially_supported', 'no_unintended_arm_contacts']))
        report['load_guard_passed'] = (load_rejected and max_contacting_fingers==0 and max_object_displacement<.002
                                       and all(checks[k] for k in ['completed_duration', 'did_not_fall', 'peak_pitch_under_20_deg',
                                           'pitch_error_under_5_deg', 'final_pitch_error_under_1_deg', position_check,
                                           'wheel_speed_within_limit', 'object_initially_supported', 'no_unintended_arm_contacts', 'released_and_settled']))
        report['alignment_guard_passed'] = ((alignment_rejected or alignment_failed) and max_contacting_fingers==0 and max_object_displacement<.002
                                            and all(checks[k] for k in ['completed_duration', 'did_not_fall', 'peak_pitch_under_20_deg',
                                                'pitch_error_under_5_deg', 'final_pitch_error_under_1_deg', position_check,
                                                'wheel_speed_within_limit', 'object_initially_supported', 'no_unintended_arm_contacts', 'released_and_settled']))
        report['validation_passed'] = (report['load_guard_passed'] if load_rejected else
                                      report['alignment_guard_passed'] if alignment_rejected or alignment_failed else
                                      report['negative_control_passed'] if NEGATIVE else report['passed'])
        report['return_safety_control_passed'] = (returning.enabled and returning.failure in ['release_unconfirmed','retreat_unsettled']
                                                and mission_complete and not fell and unexpected_contacts==0
                                                and return_blocked_pose_errors and max(return_blocked_pose_errors)<.03
                                                and all(checks[key] for key in ['peak_pitch_under_20_deg','pitch_error_under_5_deg',
                                                    'final_pitch_error_under_1_deg','wheel_speed_within_limit','released_and_settled'])
                                                and return_contact_samples==0 and return_unsupported_samples==0)
        if returning.enabled and returning.failure:
            report['validation_passed'] = bool(report['return_safety_control_passed'])
        report['return_hold_control_passed'] = (returning.enabled and E.get('test_hold_after_release',False) and mission_complete
                                              and not fell and unexpected_contacts==0 and return_contact_samples==0
                                              and return_unsupported_samples==0 and return_max_body_motion<.002
                                              and return_blocked_pose_errors and max(return_blocked_pose_errors)<.03
                                              and all(checks[k] for k in ['peak_pitch_under_20_deg','pitch_error_under_5_deg',
                                                  'final_pitch_error_under_1_deg','wheel_speed_within_limit','released_and_settled']))
        if E.get('test_hold_after_release',False):
            report['validation_passed'] = bool(report['return_hold_control_passed'])
        if tracefile:
            tracefile.close()
            report['contact_trace_sha256'] = hashlib.sha256((OUT/'contact-trace.jsonl').read_bytes()).hexdigest()
            report['contact_trace_scope'] = 'Observational 8 ms simulator state/contact trace; no extra object feedback or pose/force writes.'
        (OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        logfile.close()
        robot.exportImage(str(OUT/'final.png'), 95)
        print(json.dumps(report), flush=True)
        if VALIDATE:
            robot.simulationQuit(0 if report['validation_passed'] else 1)
            break
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        break

logfile.close()
