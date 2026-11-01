/* Passive step-begin ledger only. Old window state and callback cadence untouched. */
static FILE *raw200_file=NULL;
static void raw200_init(const char *dir){
  char path[4096];snprintf(path,sizeof(path),"%s/raw200-native.csv",dir);
  raw200_file=fopen(path,"w");
  if(raw200_file)fprintf(raw200_file,"time_s,x_m,y_m,z_m,com_x_m,com_y_m,com_z_m,vx_m_s,vy_m_s,vz_m_s,origin_vx_m_s,origin_vy_m_s,origin_vz_m_s,wx_rad_s,wy_rad_s,wz_rad_s,r0,r1,r2,r3,r4,r5,r6,r7,r8,guard_valid\n");
}
static void raw200_log(double ms){
  if(!raw200_file||!root_geom||fabs(ms/5.-round(ms/5.))>1e-7)return;
  const dReal *p=dGeomGetPosition(root_geom),*cm=dBodyGetPosition(bodies[0]),*v=dBodyGetLinearVel(bodies[0]),*w=dBodyGetAngularVel(bodies[0]);
  dVector3 ov;double R[9];root_rotation(R);dBodyGetPointVel(bodies[0],p[0],p[1],p[2],ov);
  fprintf(raw200_file,"%.12g",ms*.001);
  for(int i=0;i<3;i++)fprintf(raw200_file,",%.17g",p[i]);
  for(int i=0;i<3;i++)fprintf(raw200_file,",%.17g",cm[i]);
  for(int i=0;i<3;i++)fprintf(raw200_file,",%.17g",v[i]);
  for(int i=0;i<3;i++)fprintf(raw200_file,",%.17g",ov[i]);
  for(int i=0;i<3;i++)fprintf(raw200_file,",%.17g",w[i]);
  for(int i=0;i<9;i++)fprintf(raw200_file,",%.17g",R[i]);
  fprintf(raw200_file,",%d\n",valid);
}
static void raw200_cleanup(void){if(raw200_file)fclose(raw200_file);raw200_file=NULL;}
