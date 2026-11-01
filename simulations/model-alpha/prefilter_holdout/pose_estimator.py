"""RGB-only multi-view known-object fit. No ground-truth input or actuator API."""
import cv2,numpy as np,math
from pose_core import square_points,hypotheses,compose_body_object
from pinhole import camera_rotation
cv2.ocl.setUseOpenCL(False)
cv2.setNumThreads(1)

def project(R,p,obs,K):
    points=obs['object_points']@R.T+p;camera=(points-obs['camera_position'])@obs['camera_rotation']
    if not np.isfinite(camera).all() or camera[:,2].min()<=.001:return None
    return cv2.projectPoints(camera,np.zeros(3),np.zeros(3),K,np.zeros(5))[0].reshape(-1,2)
def residual(R,p,observations,K):
    pieces=[]
    for o in observations:
        q=project(R,p,o,K)
        if q is None:return None
        pieces.append(q-o['corners'])
    return np.concatenate(pieces)
def objective(r):
    norms=np.linalg.norm(r,axis=1);return float(np.sum(np.where(norms<=2,.5*norms*norms,2*norms-2)))
def update(R,p,step):return cv2.Rodrigues(step[:3])[0]@R,p+step[3:]
def optimize(R,p,observations,K):
    R,p=np.array(R),np.array(p);lam=.001;r=residual(R,p,observations,K)
    if r is None:return None
    cost=objective(r)
    for iteration in range(40):
        jac=[]
        for i in range(6):
            eps=1e-6 if i<3 else 1e-7;step=np.zeros(6);step[i]=eps;a=residual(*update(R,p,step),observations,K);b=residual(*update(R,p,-step),observations,K)
            if a is None or b is None:return None
            jac.append(((a-b)/(2*eps)).reshape(-1))
        J=np.column_stack(jac);norms=np.linalg.norm(r,axis=1);weights=np.repeat(np.minimum(1,2/np.maximum(norms,1e-12)),2);H=J.T@(weights[:,None]*J);gradient=J.T@(weights*r.reshape(-1))
        try:step=-np.linalg.solve(H+lam*np.diag(np.maximum(np.diag(H),1e-9)),gradient)
        except np.linalg.LinAlgError:return None
        scale=max(1,np.linalg.norm(step[:3])/.25,np.linalg.norm(step[3:])/.01);step/=scale;newR,newp=update(R,p,step);rr=residual(newR,newp,observations,K);newcost=objective(rr) if rr is not None else float('inf')
        if newcost<cost:
            improvement=cost-newcost;R,p,r,cost=newR,newp,rr,newcost;lam=max(1e-9,lam*.3)
            if improvement<1e-10:break
        else:
            lam*=10
            if lam>1e12:break
    return dict(rotation=R.tolist(),position_m=p.tolist(),robust_cost=cost,mean_robust_cost=cost/len(r),rms_px=float(np.sqrt(np.mean(np.sum(r*r,axis=1)))),outlier_fraction=float(np.mean(np.linalg.norm(r,axis=1)>2)),iterations=iteration+1)
def angle(a,b):return math.acos(float(np.clip((np.trace(np.array(a).T@np.array(b))-1)/2,-1,1)))
def fit_observations(observations,seeds,K):
    if len({o['camera_id'] for o in observations})<2:return dict(status='insufficient_camera_support',chosen=None,solutions=[])
    centers=[o['camera_position'] for o in observations]
    if max(np.linalg.norm(a-b) for a in centers for b in centers)<=.001:return dict(status='insufficient_baseline',chosen=None,solutions=[])
    solutions=[]
    for R,p in seeds:
        fit=optimize(R,p,observations,K)
        if fit is not None:solutions.append(fit)
    solutions.sort(key=lambda s:s['robust_cost'])
    if not solutions:return dict(status='fit_failed',chosen=None,solutions=[])
    best=solutions[0];far=[s for s in solutions[1:] if np.linalg.norm(np.array(s['position_m'])-best['position_m'])>.001 or angle(s['rotation'],best['rotation'])>math.radians(1)];ambiguous=any(s['mean_robust_cost']-best['mean_robust_cost']<.25 for s in far)
    status='ambiguous' if ambiguous else 'residual_rejected' if best['rms_px']>1 or best['outlier_fraction']>.25 else 'image_pose_candidate'
    return dict(status=status,chosen=best if status=='image_pose_candidate' else None,solutions=solutions)
def estimate(frames,calibration,mode='subpix'):
    methods={'subpix':cv2.aruco.CORNER_REFINE_SUBPIX,'apriltag':cv2.aruco.CORNER_REFINE_APRILTAG};params=cv2.aruco.DetectorParameters();params.cornerRefinementMethod=methods[mode];detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),params);observations=[];seeds=[];detections=[];K=np.array(calibration['intrinsics'])
    for frame in frames:
        c=next(v for v in calibration['cameras'] if v['id']==frame['camera_id']);a=np.frombuffer(frame['rgb'],np.uint8).reshape(frame['height'],frame['width'],3);a=cv2.GaussianBlur(a,(3,3),.6);gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY);corners,ids,_=detector.detectMarkers(gray);decoded=[]
        for q,tagid in zip(corners,[] if ids is None else ids):
            tagid=int(tagid[0]);face=next((v for v in calibration['registry'] if v['id']==tagid),None)
            if face is None:return dict(status='unknown_tag_id',chosen=None,solutions=[],detections=detections)
            q=q.reshape(4,2).astype(float);BC=camera_rotation(c['direction']);cp=np.array(c['position']);points=square_points(face['tag_size_m'])@np.array(face['cube_tag_rotation']).T+np.array(face['cube_tag_translation_m']);observations.append(dict(camera_id=c['id'],object_points=points,corners=q,camera_position=cp,camera_rotation=BC));decoded.append(tagid)
            for h in hypotheses(q,K,face['tag_size_m']):seeds.append(compose_body_object(BC,cp,h,face['tag_object_rotation'],face['tag_object_translation_m']))
        detections.append(dict(camera=c['id'],tag_ids=decoded))
    result=fit_observations(observations,seeds,K);result.update(detections=detections,object_corners_observed=len(observations)*4,calibrated_uncertainty=None,controller_usable=False);return result
