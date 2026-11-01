/* Only dParamCFM is set, once at initialization. All motion logging is read-only.
 * Native collisions, actuation, body poses/velocities and random seeds are untouched. */
#include <plugins/physics.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static dBodyID bodies[8];
static const char *defs[8]={"ROBOT_MODEL","ARM_BASE","FOREARM","PALM","FINGER_0","FINGER_1","TIP_0","TIP_1"};
static const char *names[9]={"left_wheel","right_wheel","shoulder","elbow","wrist","finger_0","distal_0","finger_1","distal_1"};
static const int parents[9]={0,0,0,1,2,3,4,3,5},children[9]={-1,-1,1,2,3,4,6,5,7};
static dJointID joints[9];
static dGeomID root_geom=NULL;
static int valid=1,geom_count=0,unknown=0;
static FILE *audit=NULL,*motion=NULL,*wrench=NULL;
static dBodyID cube_body=NULL;
static dJointID wrist_damper=NULL;
static dJointFeedback damper_feedback;
static double previous_direct_gripper_torque[3];
static const double motor_cfm=.00001,local_angle=1.57079632679;
static void find_geom(dGeomID g){
  if(!g)return;
  if(dGeomIsSpace(g)){for(int i=0;i<dSpaceGetNumGeoms((dSpaceID)g);i++)find_geom(dSpaceGetGeom((dSpaceID)g,i));return;}
  if(dGeomGetClass(g)==dCylinderClass && dGeomGetBody(g)==bodies[0]){
    dReal r,l;dGeomCylinderGetParams(g,&r,&l);
    if(fabs(r-.04)<1e-10 && fabs(l-.245)<1e-10){root_geom=g;geom_count++;}
  }
}
static void root_rotation(double R[9]){
  const dReal *g=dGeomGetRotation(root_geom);double c=cos(local_angle),s=sin(local_angle);
  for(int i=0;i<3;i++){R[3*i]=g[4*i];R[3*i+1]=g[4*i+1]*c-g[4*i+2]*s;R[3*i+2]=g[4*i+1]*s+g[4*i+2]*c;}
}
static int pair(dJointID j,dBodyID a,dBodyID b){dBodyID x=dJointGetBody(j,0),y=dJointGetBody(j,1);return (x==a&&y==b)||(x==b&&y==a);}
static void inspect_joint(dJointID j){
  int type=dJointGetType(j);if(type==dJointTypeSlider){valid=0;return;}if(type!=dJointTypeHinge)return;
  for(int i=0;i<9;i++)if(joints[i]==j)return;
  int slot=-1;
  for(int i=2;i<9;i++)if(pair(j,bodies[parents[i]],bodies[children[i]]))slot=i;
  if(slot<0 && (dJointGetBody(j,0)==bodies[0]||dJointGetBody(j,1)==bodies[0])){
    dVector3 a;dJointGetHingeAnchor(j,a);const dReal *p=dGeomGetPosition(root_geom);double R[9],v[3]={0,0,0};root_rotation(R);
    for(int i=0;i<3;i++)for(int k=0;k<3;k++)v[i]+=R[3*k+i]*(a[k]-p[k]);
    if(fabs(v[0])<1e-6&&fabs(v[2])<1e-6){if(fabs(v[1]-.135)<1e-6)slot=0;else if(fabs(v[1]+.135)<1e-6)slot=1;}
  }
  if(slot<0||joints[slot]){unknown++;valid=0;return;}joints[slot]=j;
}
static void cross(const double a[3],const double b[3],double out[3]){
  out[0]=a[1]*b[2]-a[2]*b[1];out[1]=a[2]*b[0]-a[0]*b[2];out[2]=a[0]*b[1]-a[1]*b[0];
}
static void momentum(dBodyID body,const dReal gravity[4],double P[3],double L[3],double G[3],double *mass){
  dMass m;dBodyGetMass(body,&m);const dReal *r=dBodyGetPosition(body),*v=dBodyGetLinearVel(body),*w=dBodyGetAngularVel(body),*R=dBodyGetRotation(body);
  double wb[3]={0,0,0},ibw[3]={0,0,0},spin[3]={0,0,0},p[3],mg[3],orbital[3],grav[3];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)wb[i]+=R[4*j+i]*w[j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)ibw[i]+=m.I[4*i+j]*wb[j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)spin[i]+=R[4*i+j]*ibw[j];
  for(int i=0;i<3;i++){p[i]=m.mass*v[i];mg[i]=m.mass*gravity[i];}
  cross(r,p,orbital);cross(r,mg,grav);
  for(int i=0;i<3;i++){P[i]+=p[i];L[i]+=spin[i]+orbital[i];G[i]+=grav[i];}
  *mass+=m.mass;
}
static void log_wrench(double ms){
  if(!wrench||!cube_body||!wrist_damper||!((ms>=18000&&ms<=23500)||(ms>=29500&&ms<=31000)||(ms>=44500&&ms<=46500)))return;
  dJointFeedback *fb=dJointGetFeedback(joints[4]),*df=dJointGetFeedback(wrist_damper);dVector3 gravity;dWorldGetGravity(dBodyGetWorld(bodies[0]),gravity);
  double gp[3]={0,0,0},gl[3]={0,0,0},gg[3]={0,0,0},gm=0.,cp[3]={0,0,0},cl[3]={0,0,0},cg[3]={0,0,0},cm=0.;
  for(int j=3;j<8;j++)momentum(bodies[j],gravity,gp,gl,gg,&gm);
  momentum(cube_body,gravity,cp,cl,cg,&cm);
  const dReal *force=fb?(dJointGetBody(joints[4],0)==bodies[3]?fb->f1:fb->f2):NULL,*torque=fb?(dJointGetBody(joints[4],0)==bodies[3]?fb->t1:fb->t2):NULL;
  const dReal *dfc=df?(dJointGetBody(wrist_damper,0)==bodies[3]?df->f1:df->f2):NULL,*dtq=df?(dJointGetBody(wrist_damper,0)==bodies[3]?df->t1:df->t2):NULL;
  fprintf(wrench,"%.12g",ms*.001);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",(force?force[k]:0.)+(dfc?dfc[k]:0.));
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",(torque?torque[k]:0.)+(dtq?dtq[k]:0.)+previous_direct_gripper_torque[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",gp[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",cp[k]);
  fprintf(wrench,",%.17g,%.17g",gm,cm);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",gravity[k]);
  fprintf(wrench,",%d",fb!=NULL&&df!=NULL);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",gl[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",cl[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",gg[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",cg[k]);
  const dReal *pc=dBodyGetPosition(bodies[3]),*root=dGeomGetPosition(root_geom);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",pc[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",root[k]);
  for(int k=0;k<3;k++)fprintf(wrench,",%.17g",dtq?dtq[k]:0.);
  fprintf(wrench,"\n");
}
static FILE *sent=NULL;
static double expected_native_dt=.00025;
static double prev_t=-1.,prev_P[3],prev_L[3],prev_G[3],prev_pc[3],impulse[9],duration=0.;
static int samples=0,packet_valid=1,sequence=0;
static void send_wrench(double ms){
  if(!sent||!wrist_damper||!root_geom)return;
  dJointFeedback *fb=dJointGetFeedback(joints[4]),*df=dJointGetFeedback(wrist_damper);dVector3 gravity;dWorldGetGravity(dBodyGetWorld(bodies[0]),gravity);
  double P[3]={0,0,0},L[3]={0,0,0},G[3]={0,0,0},mass=0.;
  for(int j=3;j<8;j++)momentum(bodies[j],gravity,P,L,G,&mass);
  const dReal *pc=dBodyGetPosition(bodies[3]),*root=dGeomGetPosition(root_geom);
  double now=ms*.001;
  if(prev_t>=0){
    double h=now-prev_t,F[3]={0,0,0},T[3]={0,0,0},EF[3],EM[3],arm[3],shift[3];
    if(fb&&df){
      const dReal *f=dJointGetBody(joints[4],0)==bodies[3]?fb->f1:fb->f2,*q=dJointGetBody(joints[4],0)==bodies[3]?fb->t1:fb->t2;
      const dReal *d=dJointGetBody(wrist_damper,0)==bodies[3]?df->f1:df->f2,*e=dJointGetBody(wrist_damper,0)==bodies[3]?df->t1:df->t2;
      for(int i=0;i<3;i++){F[i]=f[i]+d[i];T[i]=q[i]+e[i];}
    }else packet_valid=0;
    packet_valid &= valid&&fabs(h-expected_native_dt)<1e-9;
    for(int i=0;i<3;i++)T[i]+=previous_direct_gripper_torque[i];
    cross(prev_pc,F,arm);
    for(int i=0;i<3;i++){EF[i]=(P[i]-prev_P[i])/h-F[i]-mass*gravity[i];EM[i]=(L[i]-prev_L[i])/h-T[i]-arm[i]-prev_G[i];}
    cross(root,EF,shift);
    for(int i=0;i<3;i++){impulse[i]+=EF[i]*h;impulse[3+i]+=EM[i]*h;impulse[6+i]+=(EM[i]-shift[i])*h;}
    duration+=h;samples++;
    if(fabs(ms/8.-round(ms/8.))<1e-7){
      double packet[19]={1.,++sequence,now-duration,now,samples,duration,packet_valid};
      for(int i=0;i<9;i++)packet[7+i]=impulse[i]/duration;
      for(int i=0;i<3;i++)packet[16+i]=root[i];
      dWebotsSend(17,packet,sizeof(packet));
      for(int i=0;i<19;i++)fprintf(sent,i?",%.17g":"%.17g",packet[i]);
      fprintf(sent,"\n");
      for(int i=0;i<9;i++)impulse[i]=0.;
      duration=0.;samples=0;packet_valid=1;
    }
  }
  prev_t=now;
  for(int i=0;i<3;i++){prev_P[i]=P[i];prev_L[i]=L[i];prev_G[i]=G[i];prev_pc[i]=pc[i];}
}
static dBodyID all_bodies[10];
static const int dyn_child[9]={8,9,1,2,3,4,6,5,7};
static dJointID passive[9];
static dJointFeedback hinge_storage[9],passive_storage[9];
static FILE *states=NULL,*joint_states=NULL;
static void init_dynamics(const char *dir){
  char path[4096];
  for(int i=0;i<8;i++)all_bodies[i]=bodies[i];
  for(int i=0;i<2;i++)all_bodies[8+i]=dJointGetBody(joints[i],0)==bodies[0]?dJointGetBody(joints[i],1):dJointGetBody(joints[i],0);
  for(int i=0;i<10;i++){if(!all_bodies[i]){valid=0;return;}for(int k=0;k<i;k++)if(all_bodies[i]==all_bodies[k]){valid=0;return;}}
  snprintf(path,sizeof(path),"%s/native-properties.csv",dir);FILE *props=fopen(path,"w");
  if(!props){valid=0;return;}
  fprintf(props,"link_id,mass_kg,px,py,pz,r0,r1,r2,r3,r4,r5,r6,r7,r8,i0,i1,i2,i3,i4,i5,i6,i7,i8,cx,cy,cz\n");
  for(int i=0;i<10;i++){
    dMass m;dBodyGetMass(all_bodies[i],&m);const dReal *p=dBodyGetPosition(all_bodies[i]),*R=dBodyGetRotation(all_bodies[i]);
    fprintf(props,"%d,%.17g",i,m.mass);
    for(int k=0;k<3;k++)fprintf(props,",%.17g",p[k]);
    for(int a=0;a<3;a++)for(int b=0;b<3;b++)fprintf(props,",%.17g",R[4*a+b]);
    for(int a=0;a<3;a++)for(int b=0;b<3;b++)fprintf(props,",%.17g",m.I[4*a+b]);
    for(int k=0;k<3;k++)fprintf(props,",%.17g",m.c[k]);
    fprintf(props,"\n");
  }
  fclose(props);
  snprintf(path,sizeof(path),"%s/native-joint-properties.csv",dir);props=fopen(path,"w");
  if(!props){valid=0;return;}
  fprintf(props,"slot,initial_ode_angle,passive_count,passive_axes,passive_cfm,passive_erp\n");
  for(int i=0;i<9;i++){
    int count=0;dBodyID child=all_bodies[dyn_child[i]],parent=bodies[parents[i]];
    for(int j=0;j<dBodyGetNumJoints(child);j++){dJointID q=dBodyGetJoint(child,j);if(dJointGetType(q)==dJointTypeAMotor&&pair(q,parent,child)){passive[i]=q;count++;}}
    if(count!=1){valid=0;fclose(props);return;}
    if(!dJointGetFeedback(joints[i]))dJointSetFeedback(joints[i],&hinge_storage[i]);
    if(!dJointGetFeedback(passive[i]))dJointSetFeedback(passive[i],&passive_storage[i]);
    fprintf(props,"%d,%.17g,%d,%d,%.17g,%.17g\n",i,dJointGetHingeAngle(joints[i]),count,dJointGetAMotorNumAxes(passive[i]),dJointGetAMotorParam(passive[i],dParamStopCFM),dJointGetAMotorParam(passive[i],dParamStopERP));
  }
  fclose(props);
  snprintf(path,sizeof(path),"%s/native-links.csv",dir);states=fopen(path,"w");snprintf(path,sizeof(path),"%s/native-joints.csv",dir);joint_states=fopen(path,"w");
  if(!states||!joint_states){valid=0;return;}
  fprintf(states,"time_s,link_id,px,py,pz,vx,vy,vz,wx,wy,wz,r0,r1,r2,r3,r4,r5,r6,r7,r8,kinetic_J\n");
  fprintf(joint_states,"time_s,slot,ode_angle,ode_rate,motor_torque_Nm,passive_torque_Nm,feedback_valid\n");
}
static void log_dynamics(double ms){
  if(!states||!joint_states)return;
  int coarse=fabs(ms/8.-round(ms/8.))<1e-7;
  int dense=(ms<=200)||(ms>=9900&&ms<=10100)||(ms>=22300&&ms<=22900)||(ms>=45100&&ms<=45600);
  if(!coarse&&!dense)return;
  for(int i=0;i<10;i++){
    dMass m;dBodyGetMass(all_bodies[i],&m);const dReal *p=dBodyGetPosition(all_bodies[i]),*v=dBodyGetLinearVel(all_bodies[i]),*w=dBodyGetAngularVel(all_bodies[i]),*R=dBodyGetRotation(all_bodies[i]);double wb[3]={0,0,0},kinetic=0.;
    for(int a=0;a<3;a++){kinetic+=.5*m.mass*v[a]*v[a];for(int b=0;b<3;b++)wb[a]+=R[4*b+a]*w[b];}
    for(int a=0;a<3;a++)for(int b=0;b<3;b++)kinetic+=.5*wb[a]*m.I[4*a+b]*wb[b];
    fprintf(states,"%.12g,%d",ms*.001,i);
    for(int k=0;k<3;k++)fprintf(states,",%.17g",p[k]);
    for(int k=0;k<3;k++)fprintf(states,",%.17g",v[k]);
    for(int k=0;k<3;k++)fprintf(states,",%.17g",w[k]);
    for(int a=0;a<3;a++)for(int b=0;b<3;b++)fprintf(states,",%.17g",R[4*a+b]);
    fprintf(states,",%.17g\n",kinetic);
  }
  for(int i=0;i<9;i++){
    dJointFeedback *h=dJointGetFeedback(joints[i]),*d=dJointGetFeedback(passive[i]);dBodyID child=all_bodies[dyn_child[i]];dVector3 anchor,axis;
    if(dJointGetBody(joints[i],0)==child)dJointGetHingeAnchor(joints[i],anchor);else dJointGetHingeAnchor2(joints[i],anchor);
    dJointGetHingeAxis(joints[i],axis);const dReal *p=dBodyGetPosition(child);double motor=0.,damping=0.;
    if(h&&d){
      const dReal *f=dJointGetBody(joints[i],0)==child?h->f1:h->f2,*q=dJointGetBody(joints[i],0)==child?h->t1:h->t2,*df=dJointGetBody(passive[i],0)==child?d->f1:d->f2,*dq=dJointGetBody(passive[i],0)==child?d->t1:d->t2;double arm[3],fm[3],fd[3];
      for(int k=0;k<3;k++)arm[k]=anchor[k]-p[k];
      cross(f,arm,fm);cross(df,arm,fd);
      for(int k=0;k<3;k++){motor+=axis[k]*(q[k]+fm[k]);damping+=axis[k]*(dq[k]+fd[k]);}
    }
    fprintf(joint_states,"%.12g,%d,%.17g,%.17g,%.17g,%.17g,%d\n",ms*.001,i,dJointGetHingeAngle(joints[i]),dJointGetHingeAngleRate(joints[i]),motor,damping,h!=NULL&&d!=NULL);
  }
}
static FILE *drive=NULL,*drive_mode=NULL;
static void init_drive(const char *dir){
  char path[4096];snprintf(path,sizeof(path),"%s/native-drive.csv",dir);drive=fopen(path,"w");snprintf(path,sizeof(path),"%s/native-actuator-mode.csv",dir);drive_mode=fopen(path,"w");
  if(!drive||!drive_mode){valid=0;return;}
  fprintf(drive,"time_s,link_id,tx_Nm,ty_Nm,tz_Nm\n");fprintf(drive_mode,"time_s,slot,velocity_motor_fmax_Nm\n");
}
static void log_drive(double ms){
  if(!drive||!drive_mode)return;
  if(fabs(ms/8.-round(ms/8.))<1e-7){
    for(int i=0;i<10;i++){const dReal *t=dBodyGetTorque(all_bodies[i]);fprintf(drive,"%.12g,%d,%.17g,%.17g,%.17g\n",ms*.001,i,t[0],t[1],t[2]);}
    for(int i=0;i<9;i++)fprintf(drive_mode,"%.12g,%d,%.17g\n",ms*.001,i,dJointGetHingeParam(joints[i],dParamFMax));
  }
  for(int k=0;k<3;k++){previous_direct_gripper_torque[k]=0.;for(int i=3;i<8;i++)previous_direct_gripper_torque[k]+=dBodyGetTorque(all_bodies[i])[k];}
}
#include "contact_observer.inc"
#include "raw200_native.inc"
void webots_physics_init(void){
  const char *dir=getenv("ROBOT_AUDIT_DIR");char path[4096];if(!dir){valid=0;return;}
  snprintf(path,sizeof(path),"%s/native-init.csv",dir);audit=fopen(path,"w");snprintf(path,sizeof(path),"%s/native-motion.csv",dir);motion=fopen(path,"w");
  if(!audit||!motion){valid=0;return;}
  snprintf(path,sizeof(path),"%s/native-wrench.csv",dir);wrench=fopen(path,"w");cube_body=dWebotsGetBodyFromDEF("TEST_OBJECT");
  if(!wrench||!cube_body){valid=0;return;}
  fprintf(wrench,"time_s,fx_N,fy_N,fz_N,tx_Nm,ty_Nm,tz_Nm,px_kgm_s,py_kgm_s,pz_kgm_s,cpx_kgm_s,cpy_kgm_s,cpz_kgm_s,gripper_mass_kg,cube_mass_kg,gx_m_s2,gy_m_s2,gz_m_s2,feedback_valid,gripper_Lx,gripper_Ly,gripper_Lz,cube_Lx,cube_Ly,cube_Lz,gripper_Gx,gripper_Gy,gripper_Gz,cube_Gx,cube_Gy,cube_Gz,palm_x,palm_y,palm_z,root_x,root_y,root_z,damper_tx,damper_ty,damper_tz\n");
  for(int i=0;i<8;i++){bodies[i]=dWebotsGetBodyFromDEF(defs[i]);if(!bodies[i])valid=0;}
  if(!valid)return;
  find_geom(dWebotsGetGeomFromDEF("ROBOT_MODEL"));
  if(geom_count!=1){valid=0;return;}
  for(int i=0;i<8;i++)for(int k=0;k<dBodyGetNumJoints(bodies[i]);k++)inspect_joint(dBodyGetJoint(bodies[i],k));
  int damper_count=0;
  for(int k=0;k<dBodyGetNumJoints(bodies[3]);k++){
    dJointID j=dBodyGetJoint(bodies[3],k);
    if(dJointGetType(j)==dJointTypeAMotor&&pair(j,bodies[2],bodies[3])){wrist_damper=j;damper_count++;}
  }
  if(damper_count!=1){valid=0;return;}
  int preexisting=dJointGetFeedback(wrist_damper)!=NULL;
  if(!preexisting)dJointSetFeedback(wrist_damper,&damper_feedback);
  snprintf(path,sizeof(path),"%s/observer-init.csv",dir);FILE *obs=fopen(path,"w");
  if(!obs){valid=0;return;}
  fprintf(obs,"damper_count,num_axes,stop_cfm,stop_erp,preexisting_feedback,feedback_present\n%d,%d,%.17g,%.17g,%d,%d\n",damper_count,dJointGetAMotorNumAxes(wrist_damper),dJointGetAMotorParam(wrist_damper,dParamStopCFM),dJointGetAMotorParam(wrist_damper,dParamStopERP),preexisting,dJointGetFeedback(wrist_damper)!=NULL);fclose(obs);
  fprintf(audit,"motor,parent,child,cfm_before,cfm_after,world_erp,world_cfm,stop_erp,stop_cfm,susp_erp,susp_cfm,geom_count,unknown,dreal_bytes\n");dWorldID w=dBodyGetWorld(bodies[0]);
  for(int i=0;i<9;i++){
    if(!joints[i]){valid=0;continue;}double before=dJointGetHingeParam(joints[i],dParamCFM);dJointSetHingeParam(joints[i],dParamCFM,motor_cfm);
    fprintf(audit,"%s,%s,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%zu\n",names[i],defs[parents[i]],i<2?names[i]:defs[children[i]],before,dJointGetHingeParam(joints[i],dParamCFM),dWorldGetERP(w),dWorldGetCFM(w),dJointGetHingeParam(joints[i],dParamStopERP),dJointGetHingeParam(joints[i],dParamStopCFM),dJointGetHingeParam(joints[i],dParamSuspensionERP),dJointGetHingeParam(joints[i],dParamSuspensionCFM),geom_count,unknown,sizeof(dReal));
  }
  snprintf(path,sizeof(path),"%s/native-sent.csv",dir);sent=fopen(path,"w");
  if(!sent){valid=0;return;}
  fprintf(sent,"version,sequence,start_s,end_s,samples,duration_s,valid,fx_N,fy_N,fz_N,mwx_Nm,mwy_Nm,mwz_Nm,mrx_Nm,mry_Nm,mrz_Nm,origin_x_m,origin_y_m,origin_z_m\n");
  init_dynamics(dir);init_drive(dir);
  const char *step=getenv("ROBOT_PHYSICS_MS");
  if(step)expected_native_dt=atof(step)*.001;
  if(fabs(expected_native_dt-.00025)>1e-12&&fabs(expected_native_dt-.000125)>1e-12)valid=0;
  valid &= sizeof(dReal)==8;
  ct_init(dir);
  raw200_init(dir);
  fprintf(motion,"time_s,x_m,y_m,z_m,com_x_m,com_y_m,com_z_m,vx_m_s,vy_m_s,vz_m_s,origin_vx_m_s,origin_vy_m_s,origin_vz_m_s,wx_rad_s,wy_rad_s,wz_rad_s,r0,r1,r2,r3,r4,r5,r6,r7,r8,guard_valid\n");fflush(audit);
}
void webots_physics_step(void){
  if(!motion||!root_geom)return;
  for(int i=0;i<9;i++)if(!joints[i]||fabs(dJointGetHingeParam(joints[i],dParamCFM)-motor_cfm)>1e-14)valid=0;
  double ms=dWebotsGetTime();raw200_log(ms);ct_begin(ms);log_wrench(ms);send_wrench(ms);log_dynamics(ms);log_drive(ms);if(fabs(ms/8.-round(ms/8.))>1e-7)return;
  const dReal *p=dGeomGetPosition(root_geom),*cm=dBodyGetPosition(bodies[0]),*v=dBodyGetLinearVel(bodies[0]),*w=dBodyGetAngularVel(bodies[0]);dVector3 ov;double R[9];root_rotation(R);dBodyGetPointVel(bodies[0],p[0],p[1],p[2],ov);
  fprintf(motion,"%.12g",ms*.001);
  for(int i=0;i<3;i++)fprintf(motion,",%.17g",p[i]);
  for(int i=0;i<3;i++)fprintf(motion,",%.17g",cm[i]);
  for(int i=0;i<3;i++)fprintf(motion,",%.17g",v[i]);
  for(int i=0;i<3;i++)fprintf(motion,",%.17g",ov[i]);
  for(int i=0;i<3;i++)fprintf(motion,",%.17g",w[i]);
  for(int i=0;i<9;i++)fprintf(motion,",%.17g",R[i]);
  fprintf(motion,",%d\n",valid);
}
int webots_physics_collide(dGeomID a,dGeomID b){return ct_collide(a,b);}
void webots_physics_step_end(void){ct_end();}
void webots_physics_cleanup(void){raw200_cleanup();ct_cleanup();if(audit)fclose(audit);if(motion)fclose(motion);if(wrench)fclose(wrench);if(sent)fclose(sent);if(states)fclose(states);if(joint_states)fclose(joint_states);if(drive)fclose(drive);if(drive_mode)fclose(drive_mode);audit=NULL;motion=NULL;}
