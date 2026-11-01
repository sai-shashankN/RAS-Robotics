import shutil,sys,math
from context import ROOT,SOURCE,STUDY,read,write,sha
import cv2,numpy as np

def registry(size):
    faces=[]
    for i,n in enumerate([[-1.,0,0],[1.,0,0],[0,-1.,0],[0,1.,0],[0,0,-1.],[0,0,1.]]):
        n=np.array(n);u=np.cross([0,0,1.],n) if abs(n[2])<.5 else np.array([1.,0,0]);u/=np.linalg.norm(u);v=np.cross(n,u);R=np.column_stack([u,v,n]);p=n*(size/2);faces.append(dict(id=i,tag_size_m=size*.8,cube_tag_rotation=R.tolist(),cube_tag_translation_m=p.tolist(),tag_object_rotation=R.T.tolist(),tag_object_translation_m=(-R.T@p).tolist()))
    return faces

def main():
    assert not STUDY.exists();STUDY.mkdir();prior=STUDY.parent/'camera_render_flu_v1';pose=STUDY.parent/'fiducial_pose_v1';assert all((p/'checkpoint-index.json').exists() for p in [prior,pose])
    for old,new in [('run.py','native_capture.py'),('capture_controller.py','capture_controller.py'),('scene_builder.py','scene_builder.py'),('pinhole.py','pinhole.py')]:shutil.copy2(prior/'source_snapshot'/old,SOURCE/new)
    shutil.copy2(pose/'source_snapshot/core.py',SOURCE/'pose_core.py');import scene_builder as builder
    original_box=builder.box
    def tagged_box(name,c,R,size,color,recognition=False):
        text=original_box(name,c,R,size,color,recognition)
        if name!='TARGET_CUBE':return text
        initial_shape='Shape { appearance PBRAppearance { baseColor '+builder.nums(color)+' roughness 1 metalness 0 } geometry Box { size '+builder.nums(size)+' } }'
        assert text.count(initial_shape)==1;text=text.replace(initial_shape,'')
        shapes=[]
        for face in registry(float(size[0])):
            half=size[0]/2;s=builder.nums;points=[[-half,-half,0],[half,-half,0],[half,half,0],[-half,half,0]]
            shapes.append('Pose { translation '+s(face['cube_tag_translation_m'])+' rotation '+s(builder.axis_angle(np.array(face['cube_tag_rotation'])))+' children [ Shape { appearance PBRAppearance { baseColor 1 1 1 roughness 1 metalness 0 baseColorMap ImageTexture { url [ "../textures/tag'+str(face['id'])+'.png" ] repeatS FALSE repeatT FALSE } } geometry IndexedFaceSet { coord Coordinate { point [ '+','.join(s(p) for p in points)+' ] } coordIndex [ 0 1 2 3 -1 ] texCoord TextureCoordinate { point [ 0 0,1 0,1 1,0 1 ] } } } ] }')
        token=' ]\n    boundingObject';assert text.count(token)==1;return text.replace(token,' '+'\n'.join(shapes)+token)
    builder.box=tagged_box;entries=[]
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for e in read(prior/'manifest.json')['entries']:
        old=ROOT/e['project'];p=STUDY/'projects'/old.name;shutil.copytree(old,p,ignore=shutil.ignore_patterns('results','__pycache__'));scene=read(p/'scene.json');assert len(set(scene['cube']['size']))==1;size=scene['cube']['size'][0];textures=p/'textures';textures.mkdir()
        for i in range(6):
            marker=cv2.aruco.generateImageMarker(dictionary,i,256);image=np.full((320,320),255,dtype=np.uint8);image[32:288,32:288]=marker;assert cv2.imwrite(str(textures/f'tag{i}.png'),image)
        write(p/'calibration.json',dict(cameras=scene['cameras'],intrinsics=[[640/(2*math.tan(math.radians(35))),0,320],[0,640/(2*math.tan(math.radians(35))),240],[0,0,1]],distortion=[0]*5,family='DICT_APRILTAG_36h11',registry=registry(size)))
        (p/'worlds/scene.wbt').write_text(builder.scene_world(scene),encoding='utf-8');entries.append(dict(e,project=str(p.relative_to(ROOT)),world_sha256=sha(p/'worlds/scene.wbt'),original_project=e['project']))
    write(STUDY/'manifest.json',dict(entries=entries));write(STUDY/'test-plan.json',dict(question='Can the isolated RGB tag/PnP frontend operate on actual static rendered tagged-Cube views,after positive/occluded controls?',matrix='10short native static image launches:2manufactured optical controls first,then8saved A/B task poses with4camera views each. Zero dynamic robot/manipulation trials. At most2hidden workers.',derivative='Exact static scenes,camera poses,optics and original Cube bounding boxes;add only six visual coded face textures on planes0.2micrometres beyond each face. Tags have white margins and black border;encoded outer edge is80%ofCube side(16mm on taskCube,160mm on scaled200mmcontrol). Plain/color images remain immutable.',estimator='RGB-only detect(),fixed camera calibration and ID/size/tag-to-object registry. Retain every positive-depth pose hypothesis;do not use native masks/Cube pose to choose a solution. Save all hypotheses before the independent oracle evaluator opens scene data. Do not claim covariance/confidence or output an automatically chosen object pose.',gates='Retain original camera capture/mask/projection gates. Visible control must decode expected front-faceID0;occluded control must decode no marker. Every decoded ID must belong to the registry. Task detection coverage and oracle minimum candidate errors are diagnostic reports,not admission;partial tag occlusion may yield no observation. Stop on positive/negative-control or infrastructure failure.',scope='Ideal static tagged-object visual derivative,not the original unmarked target or a camera-feedback controller. No physical mass/strength/material or M2 result is extended.',source_indices={p.name:sha(p/'checkpoint-index.json') for p in [prior,pose]},assembly_deferred=True))
if __name__=='__main__':main()
