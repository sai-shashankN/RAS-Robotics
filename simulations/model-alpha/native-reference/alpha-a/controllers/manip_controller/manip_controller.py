"""Free cube M1 manipulation. Internal motor actuation only; no pose writes."""
from pathlib import Path
import sys
import csv,json,math,os,time,traceback
from controller import Supervisor
from support_reference import SupportReference
from grasp_policy import GraspClose
from wrench_sensor import WrenchSensor
from cube_geometry import corners,bottom,point_on_top,resting_on_top
from bench_math import Dwell
from solid_math import measure
from route_math import Odometry
from turn_math import wrap,heading_request,recovery_reference
from fixture_sensor import FixtureSensor
from tracking_math import PI
from control_math import palm_position
from path_math import world_ik
from manip_math import MAIN,DT,robot_mass,equilibrium,Governor,smooth
ROOT=Path(__file__).resolve().parents[2];P=json.loads((ROOT/'parameters.json').read_text());C=json.loads((ROOT/'case.json').read_text());S=json.loads((ROOT/'setup.json').read_text());OUT=ROOT/'results'/os.environ['M1_RUN'];CASE=os.environ.get('M1_CASE','nominal');CAPTURE=os.environ.get('M1_CAPTURE','0')=='1';K=json.loads((ROOT/'prepared-model.json').read_text())['K']
sys.path.insert(0,json.loads((ROOT/'runtime-dependencies.json').read_text())['numpy_path'])
import numpy as np
from dynamics_core import Model,ry
from transit_math import rotation as body_rotation,to_world,to_local,euler_rates,known_load_gravity
MODEL=Model(P,C,np.loadtxt(ROOT/'model-properties.csv',delimiter=',',skiprows=1));TK=np.array(json.loads((ROOT/'torque-design.json').read_text())['K']);TLIMIT=np.array([.81,.9,.585,.27])
from mission_bridge import MissionTask,observation,PHASES

LOAD_MODE=os.environ['LOAD_TRANSFER_MODE'];ADMISSION_HEIGHT=0. if LOAD_MODE in ['early','both'] else .001
robot=Supervisor();body=robot.getSelf();cube=robot.getFromDef('TEST_OBJECT');imu=robot.getDevice('body_imu');gyro=robot.getDevice('body_gyro');imu.enable(8);gyro.enable(8)
from measurement import PassiveCapture
from timed_shadow import TimedShadowAttitude
from audit_reference import capture as capture_raw_reference
from owner import AccelOwner
from history import ApiHistory
from consumer import HistoryConsumer
from projection import project as project_pitch
raw_accel=robot.getDevice("body_accel");accel_owner=AccelOwner(raw_accel,gyro,robot.getTime());raw_capture=PassiveCapture(raw_accel,gyro);shadow_attitude=TimedShadowAttitude();api_history=ApiHistory(robot.getTime());owned_consumer=HistoryConsumer()
OWNED_ACCEL_LOSS=False
from full200 import FullObservedAlpha200
full200=FullObservedAlpha200(robot,body,OUT,False)
fixture_sensor=FixtureSensor(robot.getDevice("fixture_contacts"),OUT/"fixture-received.jsonl",float(os.environ["ROBOT_PHYSICS_MS"]))
wrench_sensor=WrenchSensor(robot.getDevice('gripper_wrench'),OUT/'wrench-received.jsonl',robot.getBasicTimeStep())
names=MAIN+[f'{kind}_{i}' for kind in ['finger','distal'] for i in range(2)];wheel_names=['left_wheel','right_wheel'];motors={n:robot.getDevice(n) for n in names+wheel_names};sensors={n:robot.getDevice(n+'_encoder') for n in names+wheel_names}
for s in sensors.values():s.enable(8)
for n in names:
    motors[n].enableTorqueFeedback(8);motors[n].setVelocity(.8 if n in MAIN else 1.5)
    if n not in MAIN:motors[n].setAvailableTorque(.006)
for n in wheel_names:motors[n].setPosition(float('inf'));motors[n].setVelocity(0.);motors[n].setTorque(0.)
command={n:C['initial_pose'][n.split('_')[0]] for n in names}
for n in names:motors[n].setPosition(command[n])
nodes={n:robot.getFromDef(n) for n in ['ARM_BASE','FOREARM','PALM','FINGER_0','FINGER_1','TIP_0','TIP_1']};wheel_nodes=[]
for i in range(body.getField('children').getCount()):
    n=body.getField('children').getMFNode(i)
    if n.getTypeName()=='HingeJoint':
        ep=n.getField('endPoint').getSFNode()
        if ep and ep.getField('name').getSFString() in ['left wheel','right wheel']:wheel_nodes.append(ep)
assert len(wheel_nodes)==2
COLLIDERS=json.loads((ROOT/"collision-fixtures.json").read_text())


def loop():
    x,z=S['source_palm_xz_m'];target=[x,z+.015];phase_start=list(target);stage='stabilize';entered=0.;events=[dict(time_s=0.,stage=stage)];failure=None;success=False;payload=0.;prelift=None;hold_origin=None;min_lift=1.;hold_slip=0.;contact_z=None;loss_detected=False;release_triggered=False;fault_trigger=None;guess=x-C['initial_base_x_m'];governor=Governor(C['initial_base_x_m']);odom=Odometry(C["initial_base_x_m"],0.,0.);last_angles=None;capture=set();rows=[];started=time.perf_counter()
    support_dwell=Dwell();hold_dwell=Dwell();clear_dwell=Dwell();loss_dwell=Dwell();grip=GraspClose();lower_goal=None;opened_from=None;jam_pose=None;torque_active=False;recovery_pose=None;recovery_x=None;carry_pose=None;carry_base=None;carry_seat=None;carry_origin=None;carry_slip=0.;carry_displacement=0.;previous_reference=np.array([C['initial_base_x_m'],0.,command['shoulder'],command['elbow'],command['wrist']]);turn_task=None;transit_started=None;carry_local_seat=None;yaw_pi=PI();recovery_yaw=None;support_reference=SupportReference(1.);placement=None;placement_row=None;carry_palm_R=None;mission_observation=None
    with (OUT/'telemetry.csv').open('w',newline='',encoding='utf-8') as stream,(OUT/'contacts.jsonl').open('w',encoding='utf-8') as contact_stream,(OUT/'destination-reference.jsonl').open('w',encoding='utf-8') as destination_stream,(OUT/'raw-imu.jsonl').open('w',encoding='utf-8') as raw_imu_stream,(OUT/'raw-imu-reference.jsonl').open('w',encoding='utf-8') as raw_reference_stream,(OUT/'imu-shadow.jsonl').open('w',encoding='utf-8') as shadow_stream,(OUT/'imu-device-periods.jsonl').open('w',encoding='utf-8') as imu_period_stream,(OUT/'imu-device-actions.jsonl').open('w',encoding='utf-8') as imu_action_stream,(OUT/'imu-owner-history.jsonl').open('w',encoding='utf-8') as owner_history_stream,(OUT/'imu-owned-shadow.jsonl').open('w',encoding='utf-8') as owned_shadow_stream,(OUT/'imu-pitch-projection.jsonl').open('w',encoding='utf-8') as pitch_projection_stream:
        writer=None
        raw_reference_stream.write(json.dumps(capture_raw_reference(body,robot.getTime()),allow_nan=False)+'\n')
        (OUT/'imu-owner-init.json').write_text(json.dumps(accel_owner.initial_action,allow_nan=False),encoding='utf-8')
        while robot.step(1)!=-1:
            full200.tick(robot.getTime())
            if round(robot.getTime()*1000)%8:continue
            wrench_sensor.observe(robot.getTime())
            fixture_packet=fixture_sensor.observe(robot.getTime())
            t=robot.getTime();u=t-entered;pitch=imu.getRollPitchYaw()[1];roll=imu.getRollPitchYaw()[0];yaw=imu.getRollPitchYaw()[2];rates=gyro.getValues();a={n:sensors[n].getValue() for n in names+wheel_names};speeds={n:0. if last_angles is None else (a[n]-last_angles[n])/DT for n in a};last_angles=dict(a)
            raw_observation=raw_capture.read(t)
            raw_imu_stream.write(json.dumps(raw_observation,allow_nan=False)+'\n')
            shadow_stream.write(json.dumps(shadow_attitude.update(raw_observation),allow_nan=False)+'\n')
            raw_reference_stream.write(json.dumps(capture_raw_reference(body,t),allow_nan=False)+'\n')
            owner_period=accel_owner.periods(t)
            owner_actions=accel_owner.after_read(t,OWNED_ACCEL_LOSS)
            owner_metadata=api_history.update(raw_observation,owner_period,owner_actions)
            imu_period_stream.write(json.dumps(owner_period,allow_nan=False)+'\n')
            for owner_event in owner_actions:imu_action_stream.write(json.dumps(owner_event,allow_nan=False)+'\n')
            owner_history_stream.write(json.dumps(owner_metadata,allow_nan=False)+'\n')
            owned_output=owned_consumer.update(raw_observation,owner_metadata)
            owned_shadow_stream.write(json.dumps(owned_output,allow_nan=False)+'\n')
            pitch_projection_stream.write(json.dumps(project_pitch(owned_output,raw_observation),allow_nan=False)+'\n')
            odom_x,odom_y,odom_yaw,path_s,vx=odom.update(P['wheel_radius'],a['left_wheel'],a['right_wheel'],pitch,yaw);control_x=C['initial_base_x_m']+path_s;estimated_root=np.array([odom_x,odom_y,P['wheel_radius']]);body_est_R=body_rotation([roll,pitch,yaw]);pitch_rate,heading_rate=euler_rates(rates,roll,pitch)[1:]
            pose={j:a[j] for j in MAIN};pose.update(finger=(a['finger_0']+a['finger_1'])/2,distal=(a['distal_0']+a['distal_1'])/2);lp=palm_position(P,pose);palm_est=to_world(estimated_root,body_est_R,lp).tolist();palm_est_R=body_est_R@ry(sum(a[n] for n in MAIN))
            xyz=list(cube.getPosition());r=list(cube.getOrientation());ov=list(cube.getVelocity());obj_speed=math.sqrt(sum(v*v for v in ov[:3]));points=corners(xyz,r,S['object_size_m']);obj_bottom=bottom(points);cube_contacts=[list(pt.point) for pt in cube.getContactPoints()];finger=set();bad=[];contact_rows=[]
            for name,node in nodes.items():
                for point in node.getContactPoints():
                    pos=list(point.point);matched=name.startswith(('FINGER','TIP')) and any(math.dist(pos,other)<.0001 for other in cube_contacts)
                    if matched:finger.add(int(name[-1]))
                    else:bad.append(dict(node=name,point_m=pos))
                    contact_rows.append(dict(node=name,point_m=pos,with_cube=matched))
            body_contacts=body.getContactPoints();bad_wheels=[pt for node in wheel_nodes for pt in node.getContactPoints() if pt.point[2]>.001]
            center=S['source_object_xyz_m'][:2];support_center=S['platform_center_xy_m'];half=[v/2 for v in S['platform_size_m']];top=S['platform_top_m'];stand_contact=any(point_on_top(pt,support_center,half,top) for pt in cube_contacts);supported=stand_contact or resting_on_top(points,support_center,half,top,obj_speed);center_error=math.hypot(xyz[0]-center[0],xyz[1]-center[1]);tilt=math.acos(max(-1.,min(1.,r[8])));stable=supported and abs(obj_bottom-top)<=.0003 and center_error<=.003 and tilt<=math.radians(3) and obj_speed<.005
            solid_state=measure(xyz,r,S['object_size_m'],COLLIDERS);seat_local_est=to_local(palm_est,palm_est_R,xyz)
            seat=[xyz[i]-palm_est[i] for i in range(3)];palm=nodes['PALM'].getPosition();pr=nodes['PALM'].getOrientation();real_seat=[xyz[i]-palm[i] for i in range(3)];next_stage=None;hard=False
            carrying=stage in ['carry_back','carry_pause','carry_return','carry_settle','transit']
            carry_displacement=-.02*smooth(u/10) if stage=='carry_back' else -.02 if stage=='carry_pause' else -.02*(1-smooth(u/10)) if stage=='carry_return' else 0.
            motion_error=math.hypot(xyz[0]-(carry_origin[0]+carry_displacement),xyz[1]-carry_origin[1]) if carrying else center_error
            sample=None;planned_palm=np.array([target[0],0.,target[1]]);planned_object=np.array(S['source_object_xyz_m']);planned_yaw=turn_task.heading if turn_task is not None else 0.
            if stage=='transit':
                sample,planned_palm,planned_object,planned_yaw=turn_task.sample(u);carry_displacement=sample['path_m'];motion_error=math.dist(xyz,planned_object);carry_slip=max(carry_slip,float(np.linalg.norm(seat_local_est-carry_local_seat)))
            elif carrying:carry_slip=max(carry_slip,math.dist(real_seat,carry_seat))
            elif stage=='recovery' and turn_task is not None:
                _,planned_palm,planned_object,planned_yaw=turn_task.sample(t-transit_started)
            if bad or body_contacts or bad_wheels:failure=failure or 'forbidden_contact';hard=True
            if abs(pitch)>.85 or abs(roll)>.85:failure=failure or 'fell';hard=True
            if solid_state['violated']:failure=failure or 'penetration';hard=True
            if not failure and stage=='close' and center_error>.00025:failure='closure_displacement'
            if placement is None and not failure and (motion_error>.003 or tilt>math.radians(3)):failure='object_displacement' if motion_error>.003 else 'object_tilt'
            if not failure and carrying and carry_slip>.003:failure='carry_slip'
            # Model load is an observation-conditioned estimate, never a physical connection.
            if payload and placement is None:
                if stand_contact and stage in ['lift','hold','carry_back','carry_pause','carry_return','carry_settle','transit']:
                    events.append(dict(time_s=t,event='load_removed',reason='support',mass_kg=payload));payload=0.
                    if stage in ['lift','hold','carry_back','carry_pause','carry_return','carry_settle','transit']:
                        loss_detected=True;failure=failure or 'grasp_lost';next_stage='recovery'
                elif stand_contact and stage=='open' and len(finger)==0:
                    events.append(dict(time_s=t,event='load_removed',reason='supported_release',mass_kg=payload));payload=0.;loss_dwell.update(t,False)
                elif loss_dwell.update(t,len(finger)<2 and not(stage=='open' and stand_contact))>=.064:
                    events.append(dict(time_s=t,event='load_removed',reason='grasp_loss',mass_kg=payload));payload=0.;loss_detected=True;failure=failure or 'grasp_lost';next_stage='recovery'
            else:loss_dwell.update(t,False)
            if not payload and not failure and stage in ['lift','hold','lower'] and len(finger)==2 and not supported and prelift is not None and xyz[2]-prelift>ADMISSION_HEIGHT:
                payload=.002;events.append(dict(time_s=t,event='load_admitted',mass_kg=payload,seat_estimate_m=seat,admission_height_m=ADMISSION_HEIGHT,observed_rise_m=xyz[2]-prelift));loss_dwell.update(t,False)
            f=.12
            if not failure:
                if stage=='stabilize':
                    target=[x,z+.015];f=-.3
                    if u>=5:next_stage='approach'
                elif stage=='approach':
                    q=smooth(u/10);target=[x,phase_start[1]+(z-phase_start[1])*q];f=-.3
                    if u>=11:next_stage='close'
                elif stage=='close':
                    target=[x,z]
                    if CASE!='no_grasp':
                        ready,emitted=grip.update(t,finger,a);events.extend(emitted)
                        if ready and u>=7.5:prelift=xyz[2];next_stage='lift'
                    if u>=8 and next_stage!='lift':failure='grasp_not_established';next_stage='recovery'
                elif stage=='lift':
                    target=[x,phase_start[1]+(z+.01-phase_start[1])*smooth(u/6)]
                    if u>=8:next_stage='hold';hold_origin=None
                elif stage=='hold':
                    target=[x,z+.01];valid=len(finger)==2 and not supported and prelift is not None and xyz[2]-prelift>=.008
                    if hold_origin is None:hold_origin=real_seat
                    hold_slip=max(hold_slip,math.dist(real_seat,hold_origin));min_lift=min(min_lift,xyz[2]-prelift)
                    if hold_slip>.003:failure='hold_slip';next_stage='recovery'
                    elif hold_dwell.update(t,valid)>=6:next_stage='transit'
                    elif u>=10:failure='unsupported_hold_not_established';next_stage='recovery'
                elif stage=='transit':
                    target=[float(planned_palm[0]),float(planned_palm[2])]
                    if u>=turn_task.reference.duration:
                        if abs(wrap(planned_yaw-odom_yaw))>.02:failure='transit_final_alignment';next_stage='recovery'
                        else:
                            try:
                                mission_observation=observation(t,estimated_root,[roll,pitch,yaw],vx,euler_rates(rates,roll,pitch),control_x,palm_est,palm_est_R,xyz,r,ov[:3],len(finger),{n:a[n] for n in names},fixture_packet,bool(bad or body_contacts or bad_wheels),solid_state['violated'])
                                placement=turn_task.arrive(u,mission_observation,command,grip.goals,payload);next_stage='dest_lower';events.extend(placement.events)
                            except ValueError as exc:failure='destination_arrival_rejected';next_stage='recovery';events.append(dict(time_s=t,event='arrival_rejection',detail=str(exc)))
                elif carrying:
                    target=[x+carry_displacement,z+.01]
                    if stage=='carry_back' and u>=10:next_stage='carry_pause'
                    elif stage=='carry_pause' and u>=3:next_stage='carry_return'
                    elif stage=='carry_return' and u>=10:next_stage='carry_settle'
                    elif stage=='carry_settle' and u>=3:next_stage='lower'
                elif stage=='lower':
                    if lower_goal is None:
                        lower_goal=max(z-.001,min(z,phase_start[1]-max(0.,obj_bottom-top)-.0002));events.append(dict(time_s=t,event='seating_derived_lower',target_z_m=lower_goal,gap_m=obj_bottom-top))
                    target=[x,phase_start[1]+(lower_goal-phase_start[1])*smooth(u/10)]
                    if stand_contact:contact_z=target[1];next_stage='settle'
                    elif u>=12:failure='support_not_found';next_stage='recovery'
                elif stage=='settle':
                    target=[x,contact_z]
                    if support_dwell.update(t,stable)>=.6:next_stage='open';opened_from=list(grip.goals)
                    elif u>=8:failure='support_not_stable';next_stage='recovery'
                elif stage=='open':
                    target=list(phase_start);grip.goals=[g+(-.3-g)*smooth(u/5) for g in opened_from]
                    if u>=6:
                        opened=all(abs(a[f'finger_{i}']+.3)<=.02 and abs(a[f'distal_{i}']-.3)<=.02 for i in range(2))
                        if finger or not opened:failure='opening_blocked';next_stage='recovery'
                        else:next_stage='withdraw'
                elif stage=='withdraw':
                    target=[x,phase_start[1]+(z+.015-phase_start[1])*smooth(u/10)];f=-.3
                    if u>=11:next_stage='verify'
                elif stage=='verify':
                    target=[x,z+.015];f=-.3
                    if clear_dwell.update(t,stable and len(finger)==0)>=3:success=True
                    elif u>=6:failure='final_placement_unstable';next_stage='recovery'
            if CASE=='no_grasp':f=-.3
            trigger=(CASE=='drop_hold' and stage=='hold' and u>=2) or (CASE=='premature_release' and stage=='lift' and u>=2) or (CASE=='drop_transit' and stage=='transit' and u>=120)
            if trigger and not release_triggered:release_triggered=True;fault_trigger=t;events.append(dict(time_s=t,event='fault_triggered',case=CASE))
            if release_triggered:f=-.3
            if CASE=='blocked_open' and stage in ['open','dest_open'] and fault_trigger is None:
                fault_trigger=t;jam_pose={n:a[n] for n in ['finger_0','distal_0']};events.append(dict(time_s=t,event='fault_triggered',case=CASE,actual_grasp_angles_rad=jam_pose))
            if CASE=='transit_stop' and stage=='transit' and u>=51 and failure is None:
                failure='injected_transit_stop';fault_trigger=t;events.append(dict(time_s=t,event='fault_triggered',case=CASE,phase='backout_settle'))
            placement_row=None
            if placement is not None and stage!='transit':
                mission_observation=observation(t,estimated_root,[roll,pitch,yaw],vx,euler_rates(rates,roll,pitch),control_x,palm_est,palm_est_R,xyz,r,ov[:3],len(finger),{n:a[n] for n in names},fixture_packet,bool(bad or body_contacts or bad_wheels),solid_state['violated'])
                if failure:placement.abort(failure,mission_observation)
                before=len(placement.events);placement_row=placement.update(mission_observation);events.extend(placement.events[before:]);payload=placement_row['observed_membership_kg'];success=placement_row['success'];failure=failure or placement_row['failure'];planned_palm=np.array(placement_row['target_world_xyz']);planned_object=placement.planner.object_goal;target=[float(planned_palm[0]),float(planned_palm[2])]
                if placement_row['next_stage']:next_stage='dest_'+placement_row['next_stage']
                if failure:next_stage='recovery'
                destination_stream.write(json.dumps(placement_row,allow_nan=False)+'\n')
            reference_was_started=support_reference.started is not None
            model_payload,reference_guard=support_reference.update(t,stage,payload,stand_contact,stable,failure is not None)
            if support_reference.started is not None and not reference_was_started:events.append(dict(time_s=t,event='support_reference_started',mass_kg=support_reference.mass,duration_s=support_reference.duration))
            if placement_row is not None:model_payload=placement_row['model_reference_kg'];reference_guard=bool(placement.failure=='support_reference_lost')
            if reference_guard:failure=failure or 'support_reference_lost'
            if failure:model_payload=0.
            if failure and stage!='recovery' and not hard:next_stage='recovery'
            if stage=='recovery':f=-.3;target=list(phase_start)
            if failure and payload:
                events.append(dict(time_s=t,event='load_removed',reason='task_failure',mass_kg=payload));payload=0.
            # Plan shoulder/elbow against the slow governed base reference.
            # Keep their fast IMU/odometry compensation out of the balance loop.
            if placement_row is not None:
                base_goal=placement_row['base_goal_path_m'];ref_x=placement_row['base_reference_path_m'];ref_v=placement_row['base_reference_speed_m_s'];residual=0.;desired=placement_row['main_target']
            elif failure:
                base_goal,ref_x,ref_v,residual,desired=recovery_reference(a,control_x,MAIN)
            elif stage=='transit' and not failure:
                base_goal=carry_base+sample['path_m'];ref_x=base_goal;ref_v=sample['speed_m_s'];residual=0.;desired=dict(carry_pose)
            else:
                try:base_goal,guess,residual=equilibrium(P,target,a,model_payload,seat[0] if model_payload else 0.,guess)
                except ValueError:failure=failure or 'equilibrium_unavailable';hard=True;base_goal=governor.x;residual=0.
                if carrying and not failure:base_goal=carry_base+carry_displacement
                ref_x,ref_v=governor.update(base_goal);desired=world_ik(P,target[0]-ref_x,target[1],0.,0.,0.,1)
                if carrying and not failure:desired=dict(carry_pose)
            if desired is None:failure=failure or 'unreachable_ik';hard=True;desired={j:command[j] for j in MAIN}
            previous=dict(command)
            for n in names:
                if placement_row is not None:goal=placement_row['joint_commands'][n]
                elif n in MAIN:goal=desired[n]
                elif n.startswith('finger'):
                    goal=grip.goals[int(n[-1])] if stage in ['close','lift','hold','carry_back','carry_pause','carry_return','carry_settle','transit','lower','settle','open'] and not failure and not release_triggered and CASE!='no_grasp' else -.3
                else:
                    level=grip.preload_time is not None and stage in ['close','lift','hold','carry_back','carry_pause','carry_return','carry_settle','transit','lower','settle'] and not failure and not release_triggered
                    goal=-a['finger_'+n[-1]] if level else -command['finger_'+n[-1]]
                if jam_pose is not None and stage in ['open','dest_open','recovery'] and n in jam_pose:goal=jam_pose[n]
                limit=P['joint_ranges'].get(n,[-.65,.35] if n.startswith('finger') else P['distal_joint_range_rad']);goal=max(limit[0],min(limit[1],goal));step=(.08 if n in MAIN else .3)*DT;command[n]+=max(-step,min(step,goal-command[n]));
                if n not in MAIN or not torque_active:motors[n].setPosition(command[n])
            rm=robot_mass(P,a);robot_com=rm['com_body_m'];combined=list(robot_com)
            if payload:
                obj_body=to_local(estimated_root,body_est_R,xyz);combined=[(rm['mass_kg']*robot_com[i]+payload*obj_body[i])/(rm['mass_kg']+payload) for i in range(3)]
            mass_lean=-math.atan2(combined[0],combined[2]);pitch_reference=mass_lean if LOAD_MODE in ['lean','both'] else 0.;main_torques=np.zeros(3)
            if not torque_active and t>=5 and not failure:
                torque_active=True;events.append(dict(time_s=t,event='coupled_torque_enter'))
            if torque_active and failure:
                torque_active=False;recovery_pose={n:a[n] for n in MAIN};recovery_x=control_x;events.append(dict(time_s=t,event='position_recovery',pose_rad=recovery_pose))
                for n in MAIN:motors[n].setPosition(recovery_pose[n])
            if recovery_pose is not None:
                for n in MAIN:motors[n].setPosition(recovery_pose[n]);command[n]=recovery_pose[n]
            if torque_active:
                qref=MODEL.q0.copy();qref[0]=ref_x;qref[6:9]=[command[n] for n in MAIN];qref[9:13]=[a[n] for n in ['finger_0','distal_0','finger_1','distal_1']];qref[5]=planned_yaw if turn_task is not None else 0.;reference=np.array([ref_x,0.,*qref[6:9]]);reference_velocity=(reference-previous_reference)/DT;previous_reference=reference.copy();M,G=MODEL.mass_gravity(qref)
                if model_payload:G=G+known_load_gravity(MODEL,qref,model_payload,seat_local_est)
                u=np.array([0.,*G[6:9]])-TK@np.r_[np.array([control_x,pitch,*[a[n] for n in MAIN]])-reference,np.array([vx,pitch_rate,*[speeds[n] for n in MAIN]])-reference_velocity];u=np.clip(u,-TLIMIT,TLIMIT);total=float(u[0]);main_torques=u[1:];pitch_reference=0.
                for i,n in enumerate(MAIN):
                    if abs(speeds[n])>=.8 and main_torques[i]*speeds[n]>0:main_torques[i]=0.
                    motors[n].setTorque(float(main_torques[i]))
            else:
                total=-sum(k*v for k,v in zip(K,[control_x-(recovery_x if recovery_x is not None else ref_x),vx-(0. if recovery_x is not None else ref_v),pitch-pitch_reference,pitch_rate]))
            cap=.405;common=max(-cap,min(cap,total/2));remaining=cap-abs(common);yaw_request=0.;yaw_error=wrap(planned_yaw-odom_yaw);yaw_rate_error=0.
            if turn_task is not None:
                if failure:
                    if recovery_yaw is None:recovery_yaw=odom_yaw;yaw_pi.integral=0.
                    yaw_error=wrap(recovery_yaw-odom_yaw)
                planned_rate=sample['heading_rate_rad_s'] if stage=='transit' and not failure else 0.;yaw_request=heading_request(planned_rate,yaw_error);turn,yaw_rate_error,_,_=yaw_pi.update(yaw_request,heading_rate,min(.08,remaining),motion_reference=planned_rate)
            else:turn=max(-remaining,min(remaining,-P['balance_gains']['yaw_rate']*rates[2]))
            tau=[common-turn,common+turn]
            for i,n in enumerate(wheel_names):
                if abs(speeds[n])>=P['wheel_speed_limit'] and tau[i]*speeds[n]>0:tau[i]=0.
                motors[n].setTorque(tau[i])
            # Robot world pose/velocity is an audit witness, not a command input.
            position=body.getPosition();rotation=body.getOrientation();velocity=body.getVelocity();predicted=[position[i]+sum(rotation[3*i+j]*robot_com[j] for j in range(3)) for i in range(3)];ranges=dict(P['joint_ranges'],**{f'{kind}_{i}':lim for kind,lim in [('finger',[-.65,.35]),('distal',P['distal_joint_range_rad'])] for i in range(2)});excess=max(max(lo-a[n],a[n]-hi,0.) for n,(lo,hi) in ranges.items())
            row=dict(time_s=t,stage=stage,expected_fixture_id=(4 if placement is not None else 0),fixture_packet_end_s=fixture_packet['end_s'],destination_top_points=fixture_packet['fixture_4_top_points'],destination_other_points=fixture_packet['fixture_4_other_points'],destination_mode=int(placement_row is not None),heading_feedback_gain=3.,nominal_transit_elapsed_s=(t-transit_started if transit_started is not None else 0.),recovery_heading_reference_rad=(recovery_yaw if recovery_yaw is not None else planned_yaw),transit_phase=(PHASES.index(sample['stage'])+1 if sample is not None else 0),transit_elapsed_s=(t-entered if sample is not None else 0.),planned_yaw_rad=planned_yaw,yaw_error_rad=yaw_error,yaw_request_rad_s=yaw_request,yaw_rate_error_rad_s=yaw_rate_error,yaw_turn_torque_nm=turn,**{'planned_palm_'+k+'_m':float(planned_palm[i]) for i,k in enumerate('xyz')},**{'planned_object_'+k+'_m':float(planned_object[i]) for i,k in enumerate('xyz')},carry_displacement_reference_m=carry_displacement,carry_motion_error_m=motion_error,carry_relative_slip_m=carry_slip,torque_mode=int(torque_active),shoulder_torque_command_nm=float(main_torques[0]),elbow_torque_command_nm=float(main_torques[1]),wrist_torque_command_nm=float(main_torques[2]),stand_contact=int(stand_contact),pitch_reference_rad=pitch_reference,admission_height_m=ADMISSION_HEIGHT,gyro_x_rad_s=rates[0],gyro_y_rad_s=rates[1],gyro_z_rad_s=rates[2],object_wx_rad_s=ov[3],object_wy_rad_s=ov[4],object_wz_rad_s=ov[5],preload_active=int(grip.preload_time is not None),aligned=int(all(abs(a[f'finger_{i}']+a[f'distal_{i}'])<=.001 for i in range(2))),object_displacement_m=center_error,target_x_m=target[0],target_z_m=target[1],object_x_m=xyz[0],object_y_m=xyz[1],object_z_m=xyz[2],object_bottom_m=obj_bottom,object_vx_m_s=ov[0],object_vy_m_s=ov[1],object_vz_m_s=ov[2],speed_m_s=obj_speed,tilt_rad=tilt,source_support=int(supported),source_gap_m=obj_bottom-top,source_error_m=center_error,finger_count=len(finger),finger_mask=sum(1<<i for i in finger),bad_contact_count=len(bad)+len(body_contacts)+len(bad_wheels),penetration=int(solid_state['violated']),solid_gap_m=solid_state['minimum_fixture_gap_m'],floor_bottom_m=solid_state['floor_bottom_m'],legacy_plane_penetration=int(obj_bottom<top-.001 and center_error<.009),odom_x_m=odom_x,odom_y_m=odom_y,odom_yaw_rad=odom_yaw,path_s_m=path_s,control_pitch_rate_rad_s=pitch_rate,control_heading_rate_rad_s=heading_rate,**{'palm_est_'+k+'_m':palm_est[i] for i,k in enumerate('xyz')},**{'seat_local_est_'+k+'_m':float(seat_local_est[i]) for i,k in enumerate('xyz')},**{'palm_r'+str(i):float(pr[i]) for i in range(9)},model_payload_reference_kg=model_payload,load_reference_transition_s=1.,support_reference_guard=int(reference_guard),load_reference_transfer_elapsed_s=(t-support_reference.started if support_reference.started is not None else -1.),payload_estimate_kg=payload,loss_detected=int(loss_detected),fault_triggered=int(fault_trigger is not None),failure_latched=int(failure is not None),palm_x_m=palm[0],palm_y_m=palm[1],palm_z_m=palm[2],palm_axis_z=pr[8],seat_est_x_m=seat[0],seat_est_y_m=seat[1],seat_est_z_m=seat[2],control_x_m=control_x,body_x_m=position[0],body_y_m=position[1],body_z_m=position[2],body_speed_m_s=math.hypot(*velocity[:2]),pitch_rad=pitch,roll_rad=roll,yaw_rad=yaw,mass_lean_rad=mass_lean,base_goal_m=base_goal,base_reference_m=ref_x,base_velocity_reference_m_s=ref_v,equilibrium_residual_m=residual,left_encoder_rad=a['left_wheel'],right_encoder_rad=a['right_wheel'],left_speed_rad_s=speeds['left_wheel'],right_speed_rad_s=speeds['right_wheel'],left_torque_nm=tau[0],right_torque_nm=tau[1],range_excess_rad=excess,robot_com_model_error_m=math.dist(predicted,body.getCenterOfMass()),main_command_rate_rad_s=max(abs(command[n]-previous[n])/DT for n in MAIN),finger_command_rate_rad_s=max(abs(command[n]-previous[n])/DT for n in names if n not in MAIN),**{'object_r'+str(i):v for i,v in enumerate(r)},**{n:a[n] for n in names},**{n+'_command_rad':command[n] for n in names},**{n+'_effort_nm':motors[n].getTorqueFeedback() for n in names})
            if writer is None:writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
            writer.writerow(row);rows.append(row);contact_stream.write(json.dumps(dict(time_s=t,stage=stage,object_points_m=cube_contacts,arm=contact_rows,forbidden=bad))+'\n')
            if abs(mass_lean)>.3:failure=failure or 'mass_lean_guard';hard=True
            if excess>.03:failure=failure or 'joint_range';hard=True
            if CAPTURE and stage not in capture and stage in ['close','hold','open','verify','recovery'] and (u>=1 or hard):
                robot.setLabel(0,f'Alpha-{C["variant"].upper()} | M1 free cube | {CASE} | {stage}',.02,.03,.04,0x25364D,0,'Arial');robot.exportImage(str(OUT/(stage+'.png')),100);capture.add(stage)
            if hard or success or (stage=='recovery' and u>=5) or t>=400:break
            if next_stage:
                if next_stage=='transit':
                    transit_started=t;carry_palm_R=palm_est_R.copy();turn_task=MissionTask(P,COLLIDERS,S,estimated_root,odom_yaw,palm_est,xyz,ref_x,seat_local_est,carry_palm_R);carry_local_seat=np.array(seat_local_est);carry_pose={n:command[n] for n in MAIN};carry_base=ref_x;carry_seat=list(real_seat);carry_origin=list(xyz);events.append(dict(time_s=t,event='transit_reference_snapshot',main_reference_rad=carry_pose,base_reference_m=carry_base,root_estimate_m=estimated_root.tolist(),heading_rad=odom_yaw,palm_estimate_m=palm_est,object_origin_m=carry_origin,seat_local_estimate_m=carry_local_seat.tolist(),duration_s=turn_task.reference.duration))
                if next_stage=='carry_back':carry_pose={n:command[n] for n in MAIN};carry_base=ref_x;carry_seat=list(real_seat);carry_origin=list(xyz);events.append(dict(time_s=t,event='carry_reference_snapshot',main_reference_rad=carry_pose,base_reference_m=carry_base,object_origin_m=carry_origin,relative_seat_m=carry_seat))
                if next_stage=='recovery':events.append(dict(time_s=t,event='task_failure',reason=failure))
                stage=next_stage;entered=t;phase_start=list(target);events.append(dict(time_s=t,stage=stage))
    if not success and failure is None:failure='timeout'
    full200.close(robot.getTime())
    report=dict(case=CASE,model_mass_semantics='Observed grasp membership retained until actual supported release or failure/loss. Separate support-gated1s smooth feedforward reference;not measured carried weight. Failure clears both immediately.',load_transfer_mode=LOAD_MODE,controller_success=success,failure=failure,duration_s=robot.getTime(),events=events,physical_root=body.getField('physics').getSFNode() is not None,cube_free=cube.getField('physics').getSFNode() is not None,min_hold_lift_m=None if min_lift==1 else min_lift,hold_slip_m=hold_slip,loss_detected=loss_detected,fault_trigger_time_s=fault_trigger,wall_seconds=time.perf_counter()-started,scope='Unqualified prepared20mm/2g two-station M2 mission pilot. Fixed destination4,272s signed-path travel,retained heading,privileged current native contact windows and sealed placement core. Independent full-task/native/fine gates required;no real sensing,Beta/X or hardware claim.')
    (OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')


try:loop();robot.simulationQuit(0)
except Exception:
    error=traceback.format_exc();(OUT/'controller-error.txt').write_text(error);print(error,flush=True);robot.simulationQuit(2)
