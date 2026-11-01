import csv,math,shutil,sys
from context import ROOT,SOURCE,STUDY,read,write,sha
sys.path.insert(0,str(ROOT/'.local-tools/plotting-packages'))
import numpy as np

def axis_angle(R):
    # Stable quaternion conversion also handles exactly pi orientations.
    a=np.array([[R[0,0]-R[1,1]-R[2,2],R[1,0]+R[0,1],R[2,0]+R[0,2],R[2,1]-R[1,2]],
                [R[1,0]+R[0,1],R[1,1]-R[0,0]-R[2,2],R[2,1]+R[1,2],R[0,2]-R[2,0]],
                [R[2,0]+R[0,2],R[2,1]+R[1,2],R[2,2]-R[0,0]-R[1,1],R[1,0]-R[0,1]],
                [R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1],R.trace()]])/3
    _,v=np.linalg.eigh(a);q=v[:,-1]
    if q[3]<0:q=-q
    n=np.linalg.norm(q[:3]);return [0.,0.,1.,0.] if n<1e-12 else [*map(float,q[:3]/n),float(2*math.atan2(n,q[3]))]
def nums(x):return ' '.join(format(float(v),'.17g') for v in x)
def box(name,c,R,size,color,recognition=False):
    return f'''Solid {{ translation {nums(c)} rotation {nums(axis_angle(np.array(R)))} name "{name}"
    children [ Shape {{ appearance PBRAppearance {{ baseColor {nums(color)} roughness 1 metalness 0 }} geometry Box {{ size {nums(size)} }} }} ]
    boundingObject Box {{ size {nums(size)} }} {'recognitionColors [ 1 1 1 ]' if recognition else ''} }}'''
def scene_world(scene):
    text=['#VRML_SIM R2025a utf8','WorldInfo { basicTimeStep 8 coordinateSystem "ENU" randomSeed 0 }','Viewpoint { position 1 1 1 }','Background { skyColor [ .15 .18 .22 ] }','DirectionalLight { direction -1 -1 -2 intensity 2 }']
    for o in scene['shapes']:text.append(box(o['name'],o['center'],o['rotation'],o['size'],o['color']))
    o=scene['cube'];text.append(box('TARGET_CUBE',o['center'],o['rotation'],o['size'],[1,.65,.05],True));cams=[]
    from pinhole import camera_rotation
    for c in scene['cameras']:
        # Pinned Webots node uses FLU; mathematical pinhole uses right/down/forward.
        P=camera_rotation(c['direction']);R=np.column_stack([P[:,2],-P[:,0],-P[:,1]]);cams.append(f'Camera {{ translation {nums(c["position"])} rotation {nums(axis_angle(R))} name "{c["id"]}" width 640 height 480 fieldOfView {math.radians(70):.17g} near .001 far 5 recognition Recognition {{ segmentation TRUE occlusion 2 frameThickness 0 }} }}')
    text.append('Robot { name "STATIC_IMAGE_OBSERVER" supervisor TRUE controller "capture_controller" children [ '+'\n'.join(cams)+' ] }');return '\n'.join(text)+'\n'

def main():
    prior=STUDY.parent/'camera_geometry_v1';assert (prior/'checkpoint-index.json').exists();assert not STUDY.exists();STUDY.mkdir()
    for n in ['pinhole.py','envelope_math.py']:shutil.copy2(prior/'source_snapshot'/n,SOURCE/n)
    import envelope_math as env
    plan=read(prior/'test-plan.json');cameras=[c for c in plan['cameras'] if c['group']!='center_reference'];scenes=[]
    for record in read(prior/'source-records.json'):
        if record['physics_ms']!=.25:continue
        out=ROOT/record['source_directory'];p=read(out/'sources/parameters.json');setup=read(out/'sources/setup.json');profile=read(out/'sources/collision-fixtures.json');samples=[r for r in read(prior/'summary.json')['results'] if r['axis']==record['axis'] and r['physics_ms']==.25]
        weakest=min(samples,key=lambda r:min(v['visible_fraction'] for v in r['views'] if v['group']=='shaft_ends_aimed'))
        selected=[('weakest_sample',weakest)]+[(stage,max([r for r in samples if r['stage']==stage],key=lambda r:r['time_s'])) for stage in ['hold','transit','dest_open']]
        with (out/'telemetry.csv').open() as f:data={round(float(r['time_s']),6):r for r in csv.DictReader(f)}
        for kind,selection in selected:
            r={k:(v if k=='stage' else float(v)) for k,v in data[round(selection['time_s'],6)].items()};cr,sr=math.cos(r['roll_rad']),math.sin(r['roll_rad']);bodyR=env.rz(r['yaw_rad'])@env.ry(r['pitch_rad'])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]);origin=np.array([r['body_'+k+'_m'] for k in 'xyz']);cube=bodyR.T@(np.array([r['object_'+k+'_m'] for k in 'xyz'])-origin);cubeR=bodyR.T@np.array([r['object_r'+str(i)] for i in range(9)]).reshape(3,3);pose={n:r[n] for n in ['shoulder','elbow','wrist','finger_0','finger_1','distal_0','distal_1']};pose.update(finger=0.,distal=0.);shapes=[]
            for name,owner,c,R,h in env.boxes(p,pose):shapes.append(dict(name=name,center=c.tolist(),rotation=R.tolist(),size=(h*2).tolist(),color=[.7,.08,.06] if owner=='body' else [.12,.2,.6] if 'finger' in owner else [.35,.4,.45]))
            for v in profile['fixtures']:shapes.append(dict(name=v['name'],center=(bodyR.T@(np.array(v['center_m'])-origin)).tolist(),rotation=(bodyR.T@np.array(v['rotation'])).tolist(),size=(np.array(v['half_m'])*2).tolist(),color=[.35,.45,.5]))
            scenes.append(dict(id=record['axis']+'_'+kind,kind='recorded_static_pose',axis=record['axis'],source_time_s=r['time_s'],source_stage=r['stage'],source_directory=record['source_directory'],source_telemetry_sha256=record['input_sha256']['telemetry.csv'],cameras=cameras,cube=dict(center=cube.tolist(),rotation=cubeR.tolist(),size=setup['object_size_m']),shapes=shapes))
    for occluded in [False,True]:scenes.append(dict(id='control_'+('occluded' if occluded else 'visible'),kind='manufactured_optical_control',cameras=[dict(id='control',position=[0,0,0],direction=[1,0,0])],cube=dict(center=[1,0,0],rotation=np.eye(3).tolist(),size=[.2]*3),shapes=[dict(name='screen',center=[.5,0,0],rotation=np.eye(3).tolist(),size=[.2,.6,.6],color=[.4]*3)] if occluded else []))
    entries=[]
    for scene in scenes:
        p=STUDY/'projects'/scene['id'];(p/'worlds').mkdir(parents=True);c=p/'controllers/capture_controller';c.mkdir(parents=True);shutil.copy2(SOURCE/'capture_controller.py',c/'capture_controller.py');(c/'runtime.ini').write_text('[python]\nCOMMAND = C:/Python313/python.exe\n');write(p/'scene.json',scene);(p/'worlds/scene.wbt').write_text(scene_world(scene));entries.append(dict(id=scene['id'],project=str(p.relative_to(ROOT)),scene_sha256=sha(p/'scene.json'),world_sha256=sha(p/'worlds/scene.wbt')))
    write(STUDY/'manifest.json',dict(entries=entries));write(STUDY/'test-plan.json',dict(question='Does actual pinned OpenGL image capture agree with the sampled conservative optical envelope screen?',matrix='10 short static rendering launches:8saved coarse A/B poses,4cameras each,and2manufactured visible/occluded controls. Zero dynamic robot or manipulation trials. At most2native workers;do not overlap with an active robot queue.',geometry='Actual saved body/joint/Cube/fixture poses,with conservative opaque robot boxes from the prior screen. All bodies are deliberately static; no Physics, no joint motors and no runtime pose writes. Not an exact visual replica of the articulated robot.',gates='640x480 RGB and segmentation-ground-truth images, expected camera optics,finite scene transforms,actual pixel mask inside projected Cube bounds,and nonzero aimed-camera target masks. Visible and fully occluded manufactured controls must distinguish visibility. Stop on capture/infrastructure/gate failure. Recognition is privileged ground truth for auditing only;not an implemented detector or sensed controller.',scope='Ideal static image capture only. No dynamic sensing,realistic optics,noise,latency,object localization,stereo correspondence,Beta/X or hardware admission.',prior_index_sha256=sha(prior/'checkpoint-index.json'),assembly_deferred=True))
if __name__=='__main__':main()
