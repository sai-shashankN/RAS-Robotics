/* ODE contact.normal acts on g1. Flip it when the cube is g2.
 * Classification only: penetration limits remain independent and unchanged. */
static int fx_top_face(int fixture,const dReal bounds[6],const dContactGeom *p,int cube_is_g1){
  if(fixture!=0 && fixture!=4)return 0;
  const double nz=(cube_is_g1?1.:-1.)*p->normal[2];
  const double tolerance=1e-6+fmax(0.,p->depth);
  if(!isfinite(nz)||!isfinite(p->depth)||p->depth<0.)return 0;
  for(int i=0;i<3;i++)if(!isfinite(p->pos[i]))return 0;
  return nz>=.95 && fabs(p->pos[2]-bounds[5])<=tolerance &&
    p->pos[0]>=bounds[0]-1e-6 && p->pos[0]<=bounds[1]+1e-6 &&
    p->pos[1]>=bounds[2]-1e-6 && p->pos[1]<=bounds[3]+1e-6;
}
