"""Floating-base mass, gravity and bias model from frozen native link properties."""
import numpy as np

Q_NAMES=['x','y','z','roll','pitch','yaw','shoulder','elbow','wrist','finger_0','distal_0','finger_1','distal_1','left_wheel','right_wheel']
SLOTS=[13,14,6,7,8,9,10,11,12]
ANCESTORS=[[],[6],[6,7],[6,7,8],[6,7,8,9],[6,7,8,11],[6,7,8,9,10],[6,7,8,11,12],[13],[14]]

def rx(a):
    c,s=np.cos(a),np.sin(a);return np.array([[1.,0,0],[0,c,-s],[0,s,c]])
def ry(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1.,0],[-s,0,c]])
def rz(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def euler(R):return np.array([np.arctan2(R[2,1],R[2,2]),np.arcsin(np.clip(-R[2,0],-1,1)),np.arctan2(R[1,0],R[0,0])])
def angular_axes(q):return np.column_stack((rz(q[5])@ry(q[4])@np.array([1.,0,0]),rz(q[5])@np.array([0.,1,0]),[0.,0,1.]))

def initial_q(p,case):
    a=case['initial_pose'];return np.array([case['initial_base_x_m'],0,p['wheel_radius'],0,case['initial_pitch_rad'],0,a['shoulder'],a['elbow'],a['wrist'],a['finger'],a['distal'],a['finger'],a['distal'],0,0.])

class Model:
    def __init__(self,p,case,properties):
        self.p=p;self.q0=initial_q(p,case);origins,rot,_,_=self.frames(self.q0);self.mass=properties[:,1].copy();native_pos=properties[:,2:5];native_R=properties[:,5:14].reshape(10,3,3);native_I=properties[:,14:23].reshape(10,3,3);self.local_com=np.einsum('nji,nj->ni',rot,native_pos-origins);self.align=np.einsum('nji,njk->nik',rot,native_R);self.local_I=self.align@native_I@self.align.transpose(0,2,1)
        assert abs(sum(self.mass)-1.207)<1e-12
        assert np.max(abs(properties[:,23:26]))<1e-12
        assert np.all(np.linalg.eigvalsh(self.local_I)>0)

    def frames(self,q):
        p=self.p;origin=np.zeros((10,3));rot=np.zeros((10,3,3));anchor=np.zeros((15,3));axis=np.zeros((15,3));origin[0]=q[:3];rot[0]=rz(q[5])@ry(q[4])@rx(q[3]);Y=rot[0][:,1]
        origin[1]=origin[0]+rot[0]@np.array(p['arm_mount_offset']);rot[1]=rot[0]@ry(q[6]);anchor[6]=origin[1];axis[6]=Y
        origin[2]=origin[1]+rot[1]@np.array([0.,0,p['upper_length']]);rot[2]=rot[1]@ry(q[7]);anchor[7]=origin[2];axis[7]=Y
        anchor[8]=origin[2]+rot[2]@np.array([0.,0,p['forearm_length']]);rot[3]=rot[2]@ry(q[8]);origin[3]=anchor[8]+rot[3]@np.array([-p.get('wrist_forward_offset_m',0.),0,p['wrist_standoff']]);axis[8]=Y
        for f,tip,col,az in [(4,6,9,p['finger_angles_rad'][0]),(5,7,11,p['finger_angles_rad'][1])]:
            radial=rot[3]@rz(az);origin[f]=origin[3]+radial@np.array([.018,0,.009]);anchor[col]=origin[f];axis[col]=-radial[:,1];rot[f]=radial@ry(-q[col]);origin[tip]=origin[f]+rot[f]@np.array([0.,0,p['finger_proximal_length']]);anchor[col+1]=origin[tip];axis[col+1]=-rot[f][:,1];rot[tip]=rot[f]@ry(-q[col+1])
        for link,col,sign in [(8,13,1),(9,14,-1)]:
            origin[link]=origin[0]+rot[0]@np.array([0.,sign*p['wheel_track']/2,0]);rot[link]=rot[0]@ry(q[col]);anchor[col]=origin[link];axis[col]=Y
        return origin,rot,anchor,axis

    def geometry(self,q):
        origin,R,anchor,axis=self.frames(q);com=origin+np.einsum('nij,nj->ni',R,self.local_com);I=R@self.local_I@R.transpose(0,2,1);jv=np.zeros((10,3,15));jw=np.zeros_like(jv);jv[:,:,:3]=np.eye(3);E=angular_axes(q)
        for i in range(10):
            jw[i,:,3:6]=E;jv[i,:,3:6]=np.cross(E.T,com[i]-q[:3]).T
            for col in ANCESTORS[i]:jw[i,:,col]=axis[col];jv[i,:,col]=np.cross(axis[col],com[i]-anchor[col])
        return com,R,I,jv,jw

    def mass_gravity(self,q):
        com,R,I,Jv,Jw=self.geometry(q);M=np.einsum('n,nai,naj->ij',self.mass,Jv,Jv)+np.einsum('nai,nab,nbj->ij',Jw,I,Jw);G=np.einsum('n,ni->i',self.mass,Jv[:,2,:])*9.81
        return M,G

    def inverse(self,q,qd,qdd):
        com,R,I,Jv,Jw=self.geometry(q);epsilon=1e-5
        plus=self.geometry(q+epsilon*qd);minus=self.geometry(q-epsilon*qd);Jvd=(plus[3]-minus[3])/(2*epsilon);Jwd=(plus[4]-minus[4])/(2*epsilon);v=np.einsum('nai,i->na',Jv,qd);w=np.einsum('nai,i->na',Jw,qd);a=np.einsum('nai,i->na',Jv,qdd)+np.einsum('nai,i->na',Jvd,qd);alpha=np.einsum('nai,i->na',Jw,qdd)+np.einsum('nai,i->na',Jwd,qd);force=self.mass[:,None]*(a-np.array([0.,0,-9.81]));Iw=np.einsum('nab,nb->na',I,w);torque=np.einsum('nab,nb->na',I,alpha)+np.cross(w,Iw)
        return np.einsum('nai,na->i',Jv,force)+np.einsum('nai,na->i',Jw,torque)

    def palm_jacobian(self,q):
        origin,R,anchor,axis=self.frames(q);Jv=np.zeros((3,15));Jw=np.zeros_like(Jv);Jv[:,:3]=np.eye(3);E=angular_axes(q);Jw[:,3:6]=E;Jv[:,3:6]=np.cross(E.T,origin[3]-q[:3]).T
        for col in [6,7,8]:Jw[:,col]=axis[col];Jv[:,col]=np.cross(axis[col],origin[3]-anchor[col])
        return origin[3],Jv,Jw

    def external(self,q,F,Mworld):
        palm,Jv,Jw=self.palm_jacobian(q);return Jv.T@F+Jw.T@(Mworld-np.cross(palm,F))

    def coordinates(self,links,joints,initial_angles):
        # Geometry alignment is fixed from initialization, never fitted per frame.
        Rnative=links[0,11:20].reshape(3,3);R=Rnative@self.align[0].T;q=self.q0.copy();q[3:6]=euler(R);q[:3]=links[0,2:5]-R@self.local_com[0];q[SLOTS]=self.q0[SLOTS]-(joints[:,2]-initial_angles);qd=np.zeros(15);w=links[0,8:11];qd[:3]=links[0,5:8]-np.cross(w,R@self.local_com[0]);qd[3:6]=np.linalg.solve(angular_axes(q),w);qd[SLOTS]=-joints[:,3]
        return q,qd

    def reduction(self):
        S=np.zeros((15,5));S[0,0]=1;S[4,1]=1;S[6:9,2:5]=np.eye(3);S[13:15,0]=1/self.p['wheel_radius'];S[13:15,1]=-1
        B=np.zeros((5,4));B[0,0]=1/self.p['wheel_radius'];B[1,0]=-1;B[2:,1:]=np.eye(3)
        return S,B
