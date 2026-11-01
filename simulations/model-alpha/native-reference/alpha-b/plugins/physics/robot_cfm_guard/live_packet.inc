/* Called under ct_mutex before any window counter reset. No physics writes. */
static FILE *fx_sent=NULL;
static unsigned fx_sequence=0;
static int fx_live=0;
static void fx_live_init(const char *dir){
  const char *flag=getenv("ROBOT_FIXTURE_LIVE");fx_live=flag&&atoi(flag)==1;
  if(!fx_live)return;
  char path[4096];snprintf(path,sizeof(path),"%s/native-fixture-sent.csv",dir);fx_sent=fopen(path,"w");
  if(!fx_sent){fx_valid=0;return;}
  fprintf(fx_sent,"version,sequence,start_s,end_s,native_steps,active_fixtures,valid,mapped_geoms,grip_cube_points,cube_fixture_points,cube_floor_points,forbidden_points,unknown_points,overflow,min_grip_fingers,bilateral_native_steps,max_cube_static_depth_m,max_forbidden_depth_m");
  for(int i=0;i<8;i++)fprintf(fx_sent,",fixture_%d_points,fixture_%d_top_points,fixture_%d_other_points",i,i,i);
  fprintf(fx_sent,"\n");
}
static void fx_send_window(void){
  if(!fx_live||!fx_sent)return;
  double end=ct_now+expected_native_dt;
  double packet[42]={1.,++fx_sequence,end-.008,end,ct_steps,fx_nfixtures,ct_valid&&fx_valid,ct_map_count,ct_counts[CT_GRIP_CUBE],ct_counts[CT_CUBE_FIXTURE],ct_counts[CT_CUBE_FLOOR],ct_counts[CT_FORBIDDEN],ct_counts[CT_UNKNOWN],ct_overflow,ct_min_grip,ct_bilateral_steps,ct_cube_depth,ct_forbidden_depth};
  for(int i=0;i<8;i++){packet[18+3*i]=fx_points[i];packet[19+3*i]=fx_top[i];packet[20+3*i]=fx_other[i];}
  dWebotsSend(18,packet,sizeof(packet));
  for(int i=0;i<42;i++)fprintf(fx_sent,i?",%.17g":"%.17g",packet[i]);
  fprintf(fx_sent,"\n");
}
static void fx_live_cleanup(void){if(fx_sent)fclose(fx_sent);fx_sent=NULL;}
