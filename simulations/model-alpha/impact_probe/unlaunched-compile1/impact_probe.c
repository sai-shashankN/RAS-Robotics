/* Read-only isolated Cube/floor observer. Return0keeps default handling. */
#include <plugins/physics.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
static dBodyID cube=NULL;static dGeomID cg=NULL,fg=NULL;static FILE *motion=NULL,*contacts=NULL;static double dt=0.,now=0.,previous=-1.,maxdepth=0.,minnormal=1.;static unsigned steps=0,points=0;static int valid=1;static pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
static void floor_geom(dGeomID g){
  if(!g)return;
  if(dGeomIsSpace(g)){for(int i=0;i<dSpaceGetNumGeoms((dSpaceID)g);i++)floor_geom(dSpaceGetGeom((dSpaceID)g,i));return;}
  if(!dGeomGetBody(g)&&dGeomGetClass(g)==dBoxClass){dVector3 s;dGeomBoxGetLengths(g,s);if(fabs(s[0]-4.)<1e-12&&fabs(s[1]-4.)<1e-12&&fabs(s[2]-.02)<1e-12){if(fg)valid=0;fg=g;}}
}
static void state(double t){
  if(!motion||!cube)return;
  const dReal *p=dBodyGetPosition(cube),*v=dBodyGetLinearVel(cube),*w=dBodyGetAngularVel(cube),*R=dBodyGetRotation(cube);dMass m;dBodyGetMass(cube,&m);dVector3 g;dWorldGetGravity(dBodyGetWorld(cube),g);double wb[3]={0,0,0},kinetic=0.,potential=0.;
  for(int i=0;i<3;i++){kinetic+=.5*m.mass*v[i]*v[i];potential-=m.mass*g[i]*p[i];for(int j=0;j<3;j++)wb[i]+=R[4*j+i]*w[j];}
  for(int i=0;i<3;i++)for(int j=0;j<3;j++)kinetic+=.5*wb[i]*m.I[4*i+j]*wb[j];
  fprintf(motion,"%.12g",t);for(int i=0;i<3;i++)fprintf(motion,",%.17g",p[i]);for(int i=0;i<3;i++)fprintf(motion,",%.17g",v[i]);for(int i=0;i<3;i++)fprintf(motion,",%.17g",w[i]);for(int i=0;i<3;i++)for(int j=0;j<3;j++)fprintf(motion,",%.17g",R[4*i+j]);fprintf(motion,",%.17g,%.17g,%d\n",kinetic,potential,valid&&dBodyIsEnabled(cube));
}
void webots_physics_init(void){
  const char *dir=getenv("ROBOT_AUDIT_DIR"),*h=getenv("ROBOT_PHYSICS_MS");if(!dir||!h){valid=0;return;}dt=atof(h)*.001;cube=dWebotsGetBodyFromDEF("TEST_OBJECT");if(!cube){valid=0;return;}cg=dBodyGetFirstGeom(cube);floor_geom(dWebotsGetGeomFromDEF("TEST_FLOOR"));if(!cg||!fg||dGeomGetClass(cg)!=dBoxClass||dBodyGetNextGeom(cg))valid=0;
  char path[4096];snprintf(path,sizeof(path),"%s/native-motion.csv",dir);motion=fopen(path,"w");snprintf(path,sizeof(path),"%s/native-contacts.csv",dir);contacts=fopen(path,"w");if(!motion||!contacts){valid=0;return;}
  fprintf(motion,"time_s,px,py,pz,vx,vy,vz,wx,wy,wz,r0,r1,r2,r3,r4,r5,r6,r7,r8,kinetic_J,potential_J,valid\n");fprintf(contacts,"window_end_s,native_steps,floor_points,max_depth_m,min_cube_normal_z,valid\n");snprintf(path,sizeof(path),"%s/native-init.csv",dir);FILE *init=fopen(path,"w");if(init){dMass m;dBodyGetMass(cube,&m);fprintf(init,"mass_kg,i00,i11,i22,world_erp,world_cfm,valid\n%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",m.mass,m.I[0],m.I[5],m.I[10],dWorldGetERP(dBodyGetWorld(cube)),dWorldGetCFM(dBodyGetWorld(cube)),valid);fclose(init);}else valid=0;state(0.);
}
void webots_physics_step(void){pthread_mutex_lock(&mutex);now=dWebotsGetTime()*.001;if(previous>=0&&fabs(now-previous-dt)>1e-8)valid=0;previous=now;pthread_mutex_unlock(&mutex);}
int webots_physics_collide(dGeomID a,dGeomID b){
  if(dGeomIsSpace(a)||dGeomIsSpace(b))return 0;pthread_mutex_lock(&mutex);
  if((a==cg&&b==fg)||(b==cg&&a==fg)){dContactGeom c[32];int n=dCollide(a,b,32,c,sizeof(dContactGeom));if(n==32)valid=0;points+=(unsigned)n;for(int i=0;i<n;i++){if(c[i].depth>maxdepth)maxdepth=c[i].depth;double z=(a==cg?1.:-1.)*c[i].normal[2];if(z<minnormal)minnormal=z;}}
  else valid=0;
  pthread_mutex_unlock(&mutex);return 0;
}
void webots_physics_step_end(void){
  pthread_mutex_lock(&mutex);steps++;double end=now+dt;int boundary=fabs(end/.008-round(end/.008))<1e-7;if(end<=.032+1e-10||boundary)state(end);if(boundary&&contacts){fprintf(contacts,"%.12g,%u,%u,%.17g,%.17g,%d\n",end,steps,points,maxdepth,minnormal,valid);steps=points=0;maxdepth=0.;minnormal=1.;}pthread_mutex_unlock(&mutex);
}
void webots_physics_cleanup(void){if(motion)fclose(motion);if(contacts)fclose(contacts);motion=contacts=NULL;pthread_mutex_destroy(&mutex);}
