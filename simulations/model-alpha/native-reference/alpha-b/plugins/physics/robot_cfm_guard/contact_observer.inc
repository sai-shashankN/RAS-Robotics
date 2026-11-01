/* Additional read-only collision queries and latched counters.
 * Return0preserves Webots default handling. No joints/forces/poses are written.
 * Callback precedes Webots filtering;skip spaces,rays,static-static,same-body
 * and directly connected bodies. Current model has no same-anchor hinge chain.
 */
#include "fixture_face_math.inc"
static pthread_mutex_t ct_mutex=PTHREAD_MUTEX_INITIALIZER;
static FILE *ct_log=NULL,*ct_events=NULL;
static int ct_enabled=0,ct_valid=1,ct_map_count=0,ct_expected_steps=0;
static dGeomID ct_geoms[512];
static int ct_owners[512];
static unsigned long long ct_ticks=0,ct_event_count=0;
static double ct_now=0.,ct_previous=-1.,ct_first=0.;
static unsigned ct_steps=0,ct_pairs=0,ct_hits=0,ct_points=0;
static unsigned ct_counts[6],ct_connected=0,ct_spaces=0,ct_rays=0,ct_overflow=0;
static unsigned ct_grip_mask=0,ct_min_grip=2,ct_bilateral_steps=0;
static double ct_cube_depth=0.,ct_forbidden_depth=0.,ct_first_floor=-1.,ct_last_floor=-1.;
enum {CT_WHEEL_FLOOR,CT_GRIP_CUBE,CT_CUBE_FIXTURE,CT_CUBE_FLOOR,CT_FORBIDDEN,CT_UNKNOWN};

static void ct_map_geom(dGeomID g,int owner){
  if(!g){ct_valid=0;return;}
  if(dGeomIsSpace(g)){
    for(int i=0;i<dSpaceGetNumGeoms((dSpaceID)g);i++)ct_map_geom(dSpaceGetGeom((dSpaceID)g,i),owner);
    return;
  }
  for(int i=0;i<ct_map_count;i++)if(ct_geoms[i]==g){if(ct_owners[i]!=owner)ct_valid=0;return;}
  if(ct_map_count==512){ct_valid=0;return;}
  ct_geoms[ct_map_count]=g;ct_owners[ct_map_count++]=owner;
}

static int ct_owner(dGeomID g){
  dBodyID b=dGeomGetBody(g);
  for(int i=0;i<10;i++)if(b && b==all_bodies[i])return i;
  if(b && b==cube_body)return 10;
  for(int i=0;i<ct_map_count;i++)if(ct_geoms[i]==g)return ct_owners[i];
  if(!b){
    dReal a[6];dGeomGetAABB(g,a);
    if(fabs(a[1]-a[0]-4.)<1e-8 && fabs(a[3]-a[2]-4.)<1e-8 && fabs(a[4]+.02)<1e-8 && fabs(a[5])<1e-8)return -1;
  }
  return -999;
}

static int ct_kind(int a,int b){
  if(a==-999||b==-999)return CT_UNKNOWN;
  if(a==10||b==10){
    int other=a==10?b:a;
    if(other>=4&&other<=7)return CT_GRIP_CUBE;
    if(other==-1)return CT_CUBE_FLOOR;
    if(other<=-100&&other>=-107)return CT_CUBE_FIXTURE;
    return CT_FORBIDDEN;
  }
  if((a==8||a==9)&&b==-1)return CT_WHEEL_FLOOR;
  if((b==8||b==9)&&a==-1)return CT_WHEEL_FLOOR;
  return CT_FORBIDDEN;
}

#include "fixture_logger.inc"

static void ct_init(const char *dir){
  const char *flag=getenv("ROBOT_CONTACT_AUDIT");if(!flag||atoi(flag)!=1)return;
  char path[4096];
  snprintf(path,sizeof(path),"%s/native-contacts.csv",dir);ct_log=fopen(path,"w");
  snprintf(path,sizeof(path),"%s/native-forbidden-contacts.csv",dir);ct_events=fopen(path,"w");
  if(!ct_log||!ct_events)return;
  fx_config();fx_init(dir);
  ct_enabled=1;ct_expected_steps=(int)round(.008/expected_native_dt);
  for(int i=0;i<10;i++)for(dGeomID g=dBodyGetFirstGeom(all_bodies[i]);g;g=dBodyGetNextGeom(g))ct_map_geom(g,i);
  for(dGeomID g=dBodyGetFirstGeom(cube_body);g;g=dBodyGetNextGeom(g))ct_map_geom(g,10);
  for(int i=0;i<fx_nfixtures;i++){char def[64];snprintf(def,sizeof(def),"M1_FIXTURE_%d",i);ct_map_geom(dWebotsGetGeomFromDEF(def),-100-i);}
  snprintf(path,sizeof(path),"%s/native-contact-primitives.csv",dir);FILE *primitive=fopen(path,"w");
  dGeomID cg=dBodyGetFirstGeom(cube_body);
  if(!primitive||!cg){ct_valid=0;}else{
    dContactGeom initial[100];int n=dCollide(root_geom,cg,100,initial,sizeof(dContactGeom));
    const dReal *rp=dGeomGetPosition(root_geom),*cp=dGeomGetPosition(cg),*rr=dGeomGetRotation(root_geom);
    dReal ra[6],ca[6];dGeomGetAABB(root_geom,ra);dGeomGetAABB(cg,ca);dMass mass;dBodyGetMass(cube_body,&mass);
    fprintf(primitive,"direct_root_cube_contacts,root_class,cube_class,root_enabled,cube_enabled,connected,root_category,root_collide,cube_category,cube_collide,cube_mass_kg,root_x,root_y,root_z,cube_x,cube_y,cube_z,root_axis_x,root_axis_y,root_axis_z,root_aabb_min_x,root_aabb_max_x,root_aabb_min_y,root_aabb_max_y,root_aabb_min_z,root_aabb_max_z,cube_aabb_min_x,cube_aabb_max_x,cube_aabb_min_y,cube_aabb_max_y,cube_aabb_min_z,cube_aabb_max_z\n");
    fprintf(primitive,"%d,%d,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%.17g",n,dGeomGetClass(root_geom),dGeomGetClass(cg),dBodyIsEnabled(bodies[0]),dBodyIsEnabled(cube_body),dAreConnectedExcluding(bodies[0],cube_body,dJointTypeContact),dGeomGetCategoryBits(root_geom),dGeomGetCollideBits(root_geom),dGeomGetCategoryBits(cg),dGeomGetCollideBits(cg),(double)mass.mass);
    for(int i=0;i<3;i++)fprintf(primitive,",%.17g",rp[i]);
    for(int i=0;i<3;i++)fprintf(primitive,",%.17g",cp[i]);
    for(int i=0;i<3;i++)fprintf(primitive,",%.17g",rr[4*i+2]);
    for(int i=0;i<6;i++)fprintf(primitive,",%.17g",ra[i]);
    for(int i=0;i<6;i++)fprintf(primitive,",%.17g",ca[i]);
    fprintf(primitive,"\n");
  }
  if(primitive)fclose(primitive);
  snprintf(path,sizeof(path),"%s/native-contact-init.csv",dir);FILE *f=fopen(path,"w");
  if(!f){ct_valid=0;}else{
    fprintf(f,"mapped_geoms,expected_native_dt_s,steps_per_window,valid\n%d,%.17g,%d,%d\n",ct_map_count,expected_native_dt,ct_expected_steps,ct_valid);fclose(f);
  }
  fprintf(ct_log,"window_end_s,native_steps,first_native_s,last_native_s,eligible_pairs,hit_pairs,point_count,wheel_floor_points,grip_cube_points,cube_fixture_points,cube_floor_points,forbidden_points,unknown_points,skipped_connected,skipped_space,skipped_ray,overflow,max_cube_static_depth_m,max_forbidden_depth_m,first_cube_floor_s,last_cube_floor_s,min_grip_fingers,bilateral_native_steps,valid\n");
  fprintf(ct_events,"time_s,owner_a,owner_b,category,point_count,max_depth_m,px_m,py_m,pz_m,nx,ny,nz\n");
}

static void ct_begin(double ms){
  if(!ct_enabled)return;
  pthread_mutex_lock(&ct_mutex);
  ct_now=ms*.001;
  if(ct_previous>=0. && fabs(ct_now-ct_previous-expected_native_dt)>1e-8)ct_valid=0;
  if(ct_previous<0. && fabs(ct_now)>1e-10)ct_valid=0;
  if(ct_steps==0)ct_first=ct_now;
  ct_previous=ct_now;ct_grip_mask=0;
  pthread_mutex_unlock(&ct_mutex);
}

static int ct_collide(dGeomID a,dGeomID b){
  if(!ct_enabled)return 0;
  pthread_mutex_lock(&ct_mutex);
  if(dGeomIsSpace(a)||dGeomIsSpace(b)){ct_spaces++;pthread_mutex_unlock(&ct_mutex);return 0;}
  if(dGeomGetClass(a)==dRayClass||dGeomGetClass(b)==dRayClass){ct_rays++;pthread_mutex_unlock(&ct_mutex);return 0;}
  dBodyID ba=dGeomGetBody(a),bb=dGeomGetBody(b);
  if((!ba&&!bb)||(ba&&ba==bb)||(ba&&bb&&dAreConnectedExcluding(ba,bb,dJointTypeContact))){ct_connected++;pthread_mutex_unlock(&ct_mutex);return 0;}
  ct_pairs++;dContactGeom contacts[100];int n=dCollide(a,b,100,contacts,sizeof(dContactGeom));
  if(n==100){ct_overflow++;ct_valid=0;}
  if(n>0){
    int ia=ct_owner(a),ib=ct_owner(b),kind=ct_kind(ia,ib);ct_hits++;ct_points+=(unsigned)n;ct_counts[kind]+=(unsigned)n;
    if(kind==CT_UNKNOWN)ct_valid=0;
    if(kind==CT_GRIP_CUBE){int finger=ia==10?ib:ia;ct_grip_mask|=(finger==4||finger==6)?1u:2u;}
    if(kind==CT_CUBE_FLOOR){if(ct_first_floor<0.)ct_first_floor=ct_now;ct_last_floor=ct_now;}
    int deepest=0;
    for(int i=0;i<n;i++){
      if(kind==CT_CUBE_FIXTURE)fx_observe(&contacts[i]);
      if(contacts[i].depth>contacts[deepest].depth)deepest=i;
      if((kind==CT_CUBE_FLOOR||kind==CT_CUBE_FIXTURE)&&contacts[i].depth>ct_cube_depth)ct_cube_depth=contacts[i].depth;
      if((kind==CT_FORBIDDEN||kind==CT_UNKNOWN)&&contacts[i].depth>ct_forbidden_depth)ct_forbidden_depth=contacts[i].depth;
    }
    if(kind==CT_FORBIDDEN||kind==CT_UNKNOWN){
      if(ct_event_count++<100000){dContactGeom *p=&contacts[deepest];fprintf(ct_events,"%.12g,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",ct_now,ia,ib,kind,n,p->depth,p->pos[0],p->pos[1],p->pos[2],p->normal[0],p->normal[1],p->normal[2]);}
      else {ct_overflow++;ct_valid=0;}
    }
  }
  pthread_mutex_unlock(&ct_mutex);return 0;
}

static void ct_end(void){
  if(!ct_enabled)return;
  pthread_mutex_lock(&ct_mutex);ct_steps++;ct_ticks++;
  unsigned fingers=(ct_grip_mask&1u)+((ct_grip_mask>>1)&1u);if(fingers<ct_min_grip)ct_min_grip=fingers;if(fingers==2)ct_bilateral_steps++;
  if(ct_steps==(unsigned)ct_expected_steps){
    fx_window();
    fprintf(ct_log,"%.12g,%u,%.12g,%.12g,%u,%u,%u",ct_now+expected_native_dt,ct_steps,ct_first,ct_now,ct_pairs,ct_hits,ct_points);
    for(int i=0;i<6;i++)fprintf(ct_log,",%u",ct_counts[i]);
    fprintf(ct_log,",%u,%u,%u,%u,%.17g,%.17g,%.12g,%.12g,%u,%u,%d\n",ct_connected,ct_spaces,ct_rays,ct_overflow,ct_cube_depth,ct_forbidden_depth,ct_first_floor,ct_last_floor,ct_min_grip,ct_bilateral_steps,ct_valid);
    ct_steps=ct_pairs=ct_hits=ct_points=ct_connected=ct_spaces=ct_rays=ct_overflow=0;
    for(int i=0;i<6;i++)ct_counts[i]=0;
    ct_cube_depth=ct_forbidden_depth=0.;ct_first_floor=ct_last_floor=-1.;ct_min_grip=2;ct_bilateral_steps=0;
  }
  pthread_mutex_unlock(&ct_mutex);
}

static void ct_cleanup(void){
  fx_cleanup();
  if(ct_log)fclose(ct_log);
  if(ct_events)fclose(ct_events);
  if(ct_enabled)pthread_mutex_destroy(&ct_mutex);
  ct_log=ct_events=NULL;ct_enabled=0;
}
