"""Destination placement reference controller. No Webots or actuator interface.

Body/palm inputs are IMU/encoder/FK estimates; cube/contact inputs retain the
current ideal-observation assumption. Independent native evaluation is required.
"""
import math
import numpy as np
from cube_geometry import corners,bottom
from manip_math import smooth
from support_reference import SupportReference
from observations import MAIN,FINGERS,JOINTS
from planning import arrival_checks,wrap
DT=.008

class PlacementController:
    def __init__(self,planner,task,path_start,transit_elapsed,arrival,initial_commands,grip_goals,observed_payload_kg):
        self.planner=planner;self.destination=planner.destination;self.size=planner.size;arrival.validate()
        checks=arrival_checks(task,path_start,transit_elapsed,arrival,self.destination,self.size)
        if not all(checks.values()):raise ValueError('Arrival rejected: '+','.join(k for k,v in checks.items() if not v))
        if observed_payload_kg!=.002:raise ValueError('Explicit previously admitted2g grasp membership required')
        if set(initial_commands)!=set(JOINTS) or not all(math.isfinite(v) for v in initial_commands.values()) or len(grip_goals)!=2 or not all(math.isfinite(v) for v in grip_goals):raise ValueError('Finite initial commands and two clamp goals required')
        self.limits=dict(planner.params['joint_ranges'])
        for i in range(2):self.limits['finger_'+str(i)]=[-.65,.35];self.limits['distal_'+str(i)]=planner.params['distal_joint_range_rad']
        if any(not self.limits[n][0]<=v<=self.limits[n][1] for n,v in initial_commands.items()) or any(not -.65<=v<=.35 for v in grip_goals):raise ValueError('Initial references must respect actual joint limits')
        if abs(wrap(planner.frame.heading-task.heading_goal))>1e-12:raise ValueError('Arrival heading must remain the destination heading')
        self.state='lower';self.entered=arrival.time_s;self.initial_time=arrival.time_s;self.last_time=None;self.failure=None;self.failure_time=None;self.hold_pose=None;self.hold_path=None;self.hold_heading=None
        self.commands=dict(initial_commands);self.grip_goals=tuple(grip_goals);self.opened_from=None;self.target=np.array(arrival.palm_world,dtype=float);self.phase_start=self.target.copy();self.contact_target=None
        self.support_since=None;self.clear_since=None;self.loss_since=None;self.membership=float(observed_payload_kg);self.reference=SupportReference(1.);self.success=False
        self.events=[dict(time_s=arrival.time_s,event='destination_arrival_accepted',fixture_id=4,planned_anchor=planner.frame.origin,planned_path=planner.frame.path,heading=planner.frame.heading)]
    @staticmethod
    def dwell(t,condition,since):
        if not condition:return None,0.
        if since is None:since=t
        return since,t-since
    def features(self,obs):
        low=bottom(corners(obs.cube_world,obs.cube_rotation,self.size));gap=float(low-self.destination.top_m);center=math.dist(obs.cube_world[:2],self.destination.center_xy);speed=math.sqrt(sum(v*v for v in obs.cube_velocity));tilt=math.acos(max(-1.,min(1.,obs.cube_rotation[8])));support=bool(obs.contacts.intended_top(4))
        return dict(gap_m=gap,center_error_m=center,cube_speed_m_s=speed,cube_tilt_rad=tilt,intended_support=support,stable=bool(support and abs(gap)<=.0003 and center<=.003 and speed<.005 and tilt<=math.radians(3)))
    def abort(self,reason,obs):
        if self.failure is None:
            self.failure=reason;self.failure_time=obs.time_s;self.hold_pose={n:float(obs.joints[n]) for n in MAIN};self.hold_path=float(obs.path_m);self.hold_heading=float(obs.root_rpy[2]);self.events.append(dict(time_s=obs.time_s,event='measured_state_recovery',reason=reason,pose=dict(self.hold_pose),path_m=self.hold_path,heading=self.hold_heading))
        self.membership=0.;self.state='recovery';self.success=False
    def ranges_ok(self,obs):
        limits=dict(self.planner.params['joint_ranges'])
        for i in range(2):limits['finger_'+str(i)]=[-.65,.35];limits['distal_'+str(i)]=self.planner.params['distal_joint_range_rad']
        return all(lo-.03<=obs.joints[n]<=hi+.03 for n,(lo,hi) in limits.items())
    def update(self,obs):
        obs.validate();t=obs.time_s
        if t<self.initial_time-1e-9:raise ValueError('Control time precedes accepted arrival')
        if self.last_time is not None and abs(t-self.last_time-DT)>1e-9:self.abort('control_cadence',obs)
        if self.last_time is None and t-self.initial_time>DT+1e-9:self.abort('control_cadence',obs)
        self.last_time=t;f=self.features(obs);stage=self.state;elapsed=t-self.entered;next_stage=None
        if self.failure is None:
            reason=None
            if not obs.contacts.valid:reason='contact_observation_invalid'
            elif obs.forbidden_contact:reason='forbidden_contact'
            elif obs.penetration:reason='penetration'
            elif not self.ranges_ok(obs):reason='joint_range'
            elif max(abs(v) for v in obs.root_rpy[:2])>=math.radians(20):reason='body_tilt'
            elif obs.contacts.wrong_contact(4):reason='wrong_support_contact'
            elif f['center_error_m']>.003:reason='object_displacement'
            elif f['cube_tilt_rad']>math.radians(3):reason='object_tilt'
            elif math.acos(max(-1.,min(1.,-obs.palm_rotation[8])))>math.radians(2):reason='palm_level'
            elif abs(wrap(obs.root_rpy[2]-self.planner.frame.heading))>.05:reason='destination_heading'
            elif math.dist(self.planner.frame.world(obs.path_m,self.planner.params['wheel_radius']),obs.root_world)>.003:reason='station_odometry_inconsistent'
            elif math.dist(obs.palm_world,self.target)>.003:reason='hand_tracking'
            elif self.membership and np.linalg.norm(np.array(obs.palm_rotation).reshape(3,3).T@(np.array(obs.cube_world)-obs.palm_world)-self.planner.local_seat)>.003:reason='grasp_slip'
            if reason:self.abort(reason,obs)
        if self.failure is None:
            if self.membership:
                if stage=='open' and f['intended_support'] and obs.finger_count==0:
                    self.membership=0.;self.loss_since=None;self.events.append(dict(time_s=t,event='observed_supported_release',fixture_id=4))
                else:
                    self.loss_since,loss=self.dwell(t,obs.finger_count<2 and not(stage=='open' and f['intended_support']),self.loss_since)
                    if loss>=.064:self.abort('grasp_lost',obs)
            if self.failure is None:
                if stage=='lower':
                    self.target=self.planner.rest_palm.copy();self.target[2]=self.phase_start[2]+(self.planner.rest_palm[2]-.0002-self.phase_start[2])*smooth(elapsed/10)
                    if f['intended_support']:self.contact_target=self.target.copy();next_stage='settle'
                    elif elapsed>=12:self.abort('support_not_found',obs)
                elif stage=='settle':
                    self.target=self.contact_target.copy();self.support_since,duration=self.dwell(t,f['stable'],self.support_since)
                    if duration>=.6:next_stage='open';self.opened_from=tuple(self.grip_goals)
                    elif elapsed>=8:self.abort('support_not_stable',obs)
                elif stage=='open':
                    self.target=self.phase_start.copy()
                    if elapsed>=6:
                        opened=all(abs(obs.joints[f'finger_{i}']+.3)<=.02 and abs(obs.joints[f'distal_{i}']-.3)<=.02 for i in range(2))
                        if obs.finger_count or not opened:self.abort('opening_blocked',obs)
                        else:next_stage='withdraw'
                elif stage=='withdraw':
                    self.target=self.planner.rest_palm.copy();self.target[2]=self.phase_start[2]+(self.planner.rest_palm[2]+.015-self.phase_start[2])*smooth(elapsed/10)
                    if elapsed>=11:next_stage='verify'
                elif stage=='verify':
                    self.target=self.planner.rest_palm.copy();self.target[2]+=.015
                    opened=all(abs(obs.joints[f'finger_{i}']+.3)<=.02 and abs(obs.joints[f'distal_{i}']-.3)<=.02 for i in range(2));withdrawn=obs.palm_world[2]>=self.planner.rest_palm[2]+.012
                    self.clear_since,duration=self.dwell(t,f['stable'] and obs.finger_count==0 and opened and withdrawn and self.membership==0,self.clear_since)
                    if duration>=3:self.success=True
                    elif elapsed>=6:self.abort('final_placement_unstable',obs)
        reference,reference_guard=self.reference.update(t,stage,self.membership,f['intended_support'],f['stable'],self.failure is not None)
        if reference_guard:self.abort('support_reference_lost',obs);reference=0.
        plan=None
        if self.failure is None:
            try:plan=self.planner.step(self.target,obs.joints,reference)
            except ValueError:self.abort('planning_unavailable',obs);reference=0.
        if self.failure is not None:
            stage='recovery';reference=0.;next_stage=None
            for n in MAIN:self.commands[n]=self.hold_pose[n]
            plan=dict(base_goal_path_m=self.hold_path,base_reference_path_m=self.hold_path,base_reference_speed_m_s=0.,heading_reference_rad=self.hold_heading,main_target=dict(self.hold_pose),target_world_xyz=[float(v) for v in self.target],projected_target_world_xyz=[float(v) for v in self.target],scalar_target_path_m=None,lateral_residual_m=None)
        else:
            for n in MAIN:self.commands[n]+=max(-.08*DT,min(.08*DT,plan['main_target'][n]-self.commands[n]))
        for i in range(2):
            n=f'finger_{i}';goal=-.3 if self.failure is not None or stage in ['withdraw','verify'] else self.opened_from[i]+(-.3-self.opened_from[i])*smooth(elapsed/5) if stage=='open' else self.grip_goals[i]
            lo,hi=self.limits[n];goal=max(lo,min(hi,goal));self.commands[n]+=max(-.3*DT,min(.3*DT,goal-self.commands[n]))
        for i in range(2):
            n=f'distal_{i}';goal=-obs.joints[f'finger_{i}'] if self.failure is None and stage in ['lower','settle'] else -self.commands[f'finger_{i}'];lo,hi=self.limits[n];goal=max(lo,min(hi,goal));self.commands[n]+=max(-.3*DT,min(.3*DT,goal-self.commands[n]))
        recovery_elapsed=0. if self.failure is None else t-self.failure_time
        row=dict(time_s=t,stage=stage,next_stage=next_stage,failure=self.failure,success=self.success,joint_commands={n:float(self.commands[n]) for n in JOINTS},main_mode='position_hold' if self.failure else 'coupled_torque_reference',observed_membership_kg=self.membership,model_reference_kg=float(reference),expected_fixture_id=4,recovery_elapsed_s=recovery_elapsed,recovery_window_complete=bool(self.failure and recovery_elapsed>=5.),recovery_observation_settled=bool(self.failure and max(abs(v) for v in obs.root_rpy[:2])<math.radians(1) and math.hypot(*obs.root_velocity[:2])<.01),**f,**plan)
        if next_stage and not self.failure:self.state=next_stage;self.entered=t;self.phase_start=self.target.copy();self.events.append(dict(time_s=t,event='placement_stage',stage=next_stage,fixture_id=4))
        return row
