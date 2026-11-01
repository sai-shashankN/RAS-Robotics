"""Balanced planar IK waypoints and contact-gated small vertical cube cycle.

Only commanded joints are returned. Cube pose is never an IK target input.
"""
import math


def rotate(x,z,angle):
    return [x*math.cos(angle)+z*math.sin(angle),-x*math.sin(angle)+z*math.cos(angle)]


def ik_at_pitch(p,point,pitch,finger,distal):
    x,z=rotate(point[0],point[1]+p['wrist_standoff']-p['wheel_radius'],-pitch)
    dx,dz=x-p['arm_mount_offset'][0],z-p['arm_mount_offset'][2]
    a,b=p['upper_length'],p['forearm_length'];cosine=(dx*dx+dz*dz-a*a-b*b)/(2*a*b)
    if not -1<=cosine<=1:return None
    elbow=math.acos(cosine);shoulder=math.atan2(dx,dz)-math.atan2(b*math.sin(elbow),a+b*math.cos(elbow))
    wrist=math.pi-pitch-shoulder-elbow
    pose=dict(shoulder=shoulder,elbow=elbow,wrist=wrist,finger=finger,distal=distal)
    if any(not p['joint_ranges'][j][0]<=pose[j]<=p['joint_ranges'][j][1] for j in ['shoulder','elbow','wrist']):return None
    return pose


def balanced_pose(p,e,point,finger,distal,fraction,previous_pitch):
    from grasp_planning import loaded_reference
    def residual(pitch):
        pose=ik_at_pitch(p,point,pitch,finger,distal)
        return None if pose is None else (loaded_reference(p,e,pose,fraction)-pitch,pose)
    roots=[];previous=None;limit=p['dynamic_lean_limit_rad']
    for index in range(181):
        pitch=-limit+2*limit*index/180;result=residual(pitch)
        if result and previous and result[0]*previous[1][0]<=0:
            lo,hi=previous[0],pitch
            for _ in range(35):
                mid=(lo+hi)/2;left,value=residual(lo),residual(mid)
                if left[0]*value[0]<=0:hi=mid
                else:lo=mid
            solved=(lo+hi)/2;roots.append((solved,residual(solved)[1]))
        previous=(pitch,result) if result else None
    if not roots:raise ValueError('No balanced in-range IK at '+str(point))
    pitch,pose=min(roots,key=lambda row:abs(row[0]-previous_pitch))
    return dict(pose=pose,pitch_rad=pitch)


def make_plan(p,e,lift=.008):
    from control_math import palm_position
    from grasp_planning import loaded_reference
    start=dict(p['dynamic_arm_targets']['arm_reach'],**p['finger_targets']['claw_open'])
    pitch=loaded_reference(p,e,start,0);palm=palm_position(p,start);anchor=rotate(palm[0],palm[2],pitch);anchor[1]+=p['wheel_radius']
    heights=[round(-.002+i*.0002,7) for i in range(round((lift+.002)/.0002)+1)]
    families={}
    for name,finger,distal,fraction in [('open',start['finger'],start['distal'],0),('closed_empty',0,.25,0),('closed_full',0,.25,1)]:
        previous=pitch;rows=[]
        for height in heights:
            row=balanced_pose(p,e,[anchor[0],anchor[1]+height],finger,distal,fraction,previous)
            previous=row['pitch_rad'];rows.append(dict(height_m=height,**row))
        families[name]=rows
    peak=max(abs(row['pitch_rad']) for rows in families.values() for row in rows)
    return {'anchor_world_xz_m':anchor,'lift_m':lift,'minimum_seek_m':-.002,'height_step_m':.0002,'families':families,
        'load_plan':{'within_lean_limit':peak<=p['dynamic_lean_limit_rad'],'peak_nominal_loaded_lean_rad':peak,
            'lean_limit_rad':p['dynamic_lean_limit_rad'],'samples':sum(map(len,families.values())),
            'assumption':'Balanced nominal planar IK families, empty/full known load, fixed wheel-height root and no base translation. Runtime joint/attitude tracking and physical contacts require validation.'}}


def at_height(plan,family,height):
    rows=plan['families'][family];height=max(rows[0]['height_m'],min(rows[-1]['height_m'],height))
    index=min(len(rows)-2,max(0,int((height-rows[0]['height_m'])/plan['height_step_m'])))
    a,b=rows[index:index+2];t=(height-a['height_m'])/(b['height_m']-a['height_m'])
    return {j:a['pose'][j]+t*(b['pose'][j]-a['pose'][j]) for j in a['pose']}


def blend(a,b,t):return {j:a[j]+t*(b[j]-a[j]) for j in a}
def smooth(t):t=max(0,min(1,t));return t*t*(3-2*t)


class VerticalCycle:
    def __init__(self,plan,negative,dt):
        self.plan,self.negative,self.dt=plan,negative,dt
        self.height=0.;self.support_since=None;self.released_at=None;self.blocked=False;self.seek_stopped=False
    def update(self,now,payload,on_stand,speed,schedule):
        if now<12:self.height=0.
        elif now<28:self.height=self.plan['lift_m']*smooth((now-12)/8)
        elif now<35:self.height=self.plan['lift_m']*(1-smooth((now-28)/7))
        elif self.released_at is None and not self.negative and not self.blocked:
            if not on_stand:
                self.height=max(self.plan['minimum_seek_m'],self.height-.0005*self.dt);self.support_since=None
            elif speed<.005:
                if self.support_since is None:self.support_since=now
            else:self.support_since=None
        if self.released_at is None and now>=schedule['release']:
            if self.negative or (self.support_since is not None and now-self.support_since>=.6):
                self.released_at=now;schedule['release']=now;schedule['finish']=max(schedule['finish'],now+12)
            elif now>=41:
                self.blocked=True;schedule['release']=1e6;schedule['finish']=53.
            else:
                schedule['release']=now+self.dt;schedule['finish']=schedule['release']+12
        closure=0 if self.negative else smooth((now-6)/2)
        if self.released_at is not None:closure=0 if self.negative else 1-smooth((now-self.released_at)/2)
        if self.released_at is not None and now>self.released_at+2:
            # Withdraw vertically only after the wider opening is commanded.
            self.height=min(self.plan['lift_m'],self.height+.002*self.dt)
        empty=blend(at_height(self.plan,'open',self.height),at_height(self.plan,'closed_empty',self.height),closure)
        loaded=at_height(self.plan,'closed_full',self.height)
        pose={j:empty[j]+payload*closure*(loaded[j]-at_height(self.plan,'closed_empty',self.height)[j]) for j in empty}
        stage='blocked_support' if self.blocked else 'cartesian_release' if self.released_at is not None else 'support_seek' if now>=35 else None
        return pose,stage
    def report(self):
        return {'released_at_s':self.released_at,'support_blocked':self.blocked,'support_since_s':self.support_since,
            'final_height_m':self.height,'support_gate':'Ideal simulated pedestal-support witness and speed below 5 mm/s for 0.6 s; no object pose target or force/pose writes.'}
