"""Isolated coded-tag/PnP prototype. Returns hypotheses, not motor commands."""
import cv2
import numpy as np
FAMILY=cv2.aruco.DICT_APRILTAG_36h11

def detect(rgb,width,height):
    if type(rgb)!=bytes or len(rgb)!=width*height*3:raise ValueError('RGB24 required')
    image=np.frombuffer(rgb,dtype=np.uint8).reshape(height,width,3);gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY);dictionary=cv2.aruco.getPredefinedDictionary(FAMILY);detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters());corners,ids,rejected=detector.detectMarkers(gray)
    return dict(family='DICT_APRILTAG_36h11',detections=[] if ids is None else [dict(id=int(i[0]),corners_px=c.reshape(4,2).astype(float).tolist()) for c,i in zip(corners,ids)],rejected_quads=len(rejected))

def square_points(size_m):
    if not np.isfinite(size_m) or size_m<=0:raise ValueError('Positive finite measured marker edge required')
    s=size_m/2;return np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],dtype=np.float64)

def hypotheses(corners_px,K,size_m,distortion=None):
    corners=np.ascontiguousarray(corners_px,dtype=np.float64);K=np.ascontiguousarray(K,dtype=np.float64);distortion=np.zeros(5) if distortion is None else np.ascontiguousarray(distortion,dtype=np.float64)
    if corners.shape!=(4,2) or K.shape!=(3,3) or not np.isfinite(corners).all() or not np.isfinite(K).all() or not np.isfinite(distortion).all() or min(K[0,0],K[1,1])<=0:raise ValueError('Finite calibrated four-corner observation required')
    if not np.allclose(K[2],[0,0,1],atol=1e-12,rtol=0) or abs(K[0,1])+abs(K[1,0])>1e-12 or distortion.ndim!=1 or len(distortion) not in {4,5,8,12,14}:raise ValueError('Conventional pinhole calibration and supported distortion vector required')
    edges=np.roll(corners,-1,axis=0)-corners;cross=edges[:,0]*np.roll(edges,-1,axis=0)[:,1]-edges[:,1]*np.roll(edges,-1,axis=0)[:,0]
    if not (cross>1e-8).all():raise ValueError('Canonical convex clockwise image corner order required')
    points=square_points(size_m);count,rotations,translations,_=cv2.solvePnPGeneric(points,corners,K,distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE);result=[]
    for rv,tv in zip(rotations,translations):
        R=cv2.Rodrigues(rv)[0];position=tv.reshape(3);camera_points=points@R.T+position
        if not np.isfinite(camera_points).all() or camera_points[:,2].min()<=0:continue
        projected=cv2.projectPoints(points,rv,tv,K,distortion)[0].reshape(4,2);error=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))));result.append(dict(rotation_camera_tag=R.tolist(),translation_camera_tag_m=position.tolist(),reprojection_rms_px=error))
    return sorted(result,key=lambda r:r['reprojection_rms_px'])

def compose_body_object(body_camera_R,body_camera_p,candidate,tag_object_R,tag_object_p):
    # T_body_object = T_body_cv_camera * T_cv_camera_tag * T_tag_object.
    BC=np.array(body_camera_R);CT=np.array(candidate['rotation_camera_tag']);TO=np.array(tag_object_R);p=BC@(CT@np.array(tag_object_p)+np.array(candidate['translation_camera_tag_m']))+np.array(body_camera_p)
    return (BC@CT@TO).tolist(),p.tolist()
