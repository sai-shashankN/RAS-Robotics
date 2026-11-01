/* These counters share ct_mutex, add no physics setters, and are offline only. */
static FILE *fx_log=NULL;
static int fx_nfixtures=4,fx_valid=1;
static unsigned fx_points[8],fx_top[8],fx_other[8];
#include "live_packet.inc"
static void fx_config(void){
  const char *n=getenv("ROBOT_FIXTURE_COUNT");
  if(n){int count=atoi(n);if(count==4||count==8)fx_nfixtures=count;else fx_valid=0;}
}
static void fx_init(const char *dir){
  char path[4096];snprintf(path,sizeof(path),"%s/native-fixture-contacts.csv",dir);fx_log=fopen(path,"w");
  if(!fx_log){fx_valid=0;return;}
  fprintf(fx_log,"window_end_s,native_steps,active_fixtures,valid");
  for(int i=0;i<8;i++)fprintf(fx_log,",fixture_%d_points,fixture_%d_top_points,fixture_%d_other_points",i,i,i);
  fprintf(fx_log,"\n");fx_live_init(dir);
}
static void fx_observe(const dContactGeom *p){
  if(!p->g1||!p->g2){fx_valid=0;return;}
  int a=ct_owner(p->g1),b=ct_owner(p->g2),first=a==10;
  if(a!=10&&b!=10){fx_valid=0;return;}
  int owner=first?b:a,index=-100-owner;dGeomID fixture=first?p->g2:p->g1;
  if(index<0||index>=fx_nfixtures||dGeomGetClass(fixture)!=dBoxClass){fx_valid=0;return;}
  fx_points[index]++;dReal bounds[6];dGeomGetAABB(fixture,bounds);
  if(index==0||index==4){
    const dReal *R=dGeomGetRotation(fixture);
    for(int i=0;i<3;i++)for(int j=0;j<3;j++)if(fabs(R[4*i+j]-(i==j?1.:0.))>1e-10)fx_valid=0;
  }
  if(fx_top_face(index,bounds,p,first))fx_top[index]++;else fx_other[index]++;
}
static void fx_window(void){
  if(!fx_log)return;
  unsigned sum=0;
  for(int i=0;i<8;i++){sum+=fx_points[i];if(fx_points[i]!=fx_top[i]+fx_other[i])fx_valid=0;}
  if(sum!=ct_counts[CT_CUBE_FIXTURE])fx_valid=0;
  fx_send_window();
  fprintf(fx_log,"%.12g,%u,%d,%d",ct_now+expected_native_dt,ct_steps,fx_nfixtures,fx_valid);
  for(int i=0;i<8;i++){fprintf(fx_log,",%u,%u,%u",fx_points[i],fx_top[i],fx_other[i]);fx_points[i]=fx_top[i]=fx_other[i]=0;}
  fprintf(fx_log,"\n");
}
static void fx_cleanup(void){fx_live_cleanup();if(fx_log)fclose(fx_log);fx_log=NULL;}
