"""Explicit, independently checked fixed two-station box declarations."""
import math,re
import numpy as np

def rotation(aa):
    axis=np.array(aa[:3],dtype=float);angle=aa[3]
    assert np.all(np.isfinite(aa)) and abs(np.linalg.norm(axis)-1)<1e-10
    x,y,z=axis;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)

def parse(world,count):
    lines=[v for v in world.splitlines() if v.startswith('DEF M1_FIXTURE_')]
    assert len(lines)==count
    out=[]
    for i,line in enumerate(lines):
        assert line.startswith(f'DEF M1_FIXTURE_{i} Solid {{ ') and 'physics ' not in line
        assert 'contactMaterial "fixture"' in line
        name=re.search(r'name "([^"]+)"',line)[1]
        center=list(map(float,re.search(r'translation ([^ ]+) ([^ ]+) ([^ ]+)',line).groups()))
        aa=list(map(float,re.search(r'rotation ([^ ]+) ([^ ]+) ([^ ]+) ([^ ]+)',line).groups()))
        visible=list(map(float,re.search(r'geometry Box \{ size ([^ ]+) ([^ ]+) ([^ ]+)',line).groups()))
        physical=list(map(float,re.search(r'boundingObject Box \{ size ([^ ]+) ([^ ]+) ([^ ]+)',line).groups()))
        assert visible==physical and all(v>0 and math.isfinite(v) for v in physical)
        assert all(math.isfinite(v) for v in center)
        out.append(dict(id=i,name=name,center_m=center,rotation=rotation(aa).tolist(),half_m=[v/2 for v in physical]))
    return out

def expected(source,distance=.6):
    """Mirror support geometry about its top, matching a heading-pi approach.

    Square top/post/foot rotations are canonicalized to identity. The sloped
    brace keeps the full Rz(pi)Rx(a) rotation. No cube-derived fixture target.
    """
    assert len(source)==4 and distance>0
    Q=np.diag([-1.,-1.,1.]);anchor=np.array(source[0]['center_m'])
    dest_anchor=anchor+[-distance,0,0];out=[]
    for i,v in enumerate(source):
        R=Q@np.array(v['rotation']) if i==1 else np.eye(3)
        if i!=1:assert abs(v['half_m'][0]-v['half_m'][1])<1e-12
        out.append(dict(id=i+4,name=v['name'].replace('source','destination'),center_m=(dest_anchor+Q@(np.array(v['center_m'])-anchor)).tolist(),rotation=R.tolist(),half_m=v['half_m']))
    return out

def append_destination(world,distance=.6):
    source=parse(world,4);dest=expected(source,distance);old=[v for v in world.splitlines() if v.startswith('DEF M1_FIXTURE_')];blocks=[]
    for i,(line,v) in enumerate(zip(old,dest)):
        line=line.replace(f'DEF M1_FIXTURE_{i} ',f'DEF M1_FIXTURE_{i+4} ').replace('name "source ','name "destination ')
        xyz=' '.join(format(x,'.17g') for x in v['center_m'])
        line,n=re.subn(r'(Solid \{ name "[^"]+" translation )[^ ]+ [^ ]+ [^ ]+',lambda m:m[1]+xyz,line);assert n==1
        if i==1:
            a=math.atan2(source[i]['rotation'][2][1],source[i]['rotation'][1][1])
            aa=[0.,math.sin(a/2),math.cos(a/2),math.pi]
        else:aa=[1.,0.,0.,0.]
        aa_text=' '.join(format(x,'.17g') for x in aa)
        line,n=re.subn(r'( translation '+re.escape(xyz)+r' rotation )[^ ]+ [^ ]+ [^ ]+ [^ ]+',lambda m:m[1]+aa_text,line);assert n==1
        blocks.append(line)
    return world.rstrip()+'\n\n'+'\n\n'.join(blocks)+'\n'

def validate(world,source,distance=.6):
    actual=parse(world,8);want=source+expected(source,distance);errors=[]
    for a,b in zip(actual,want):
        assert a['id']==b['id'] and a['name']==b['name']
        for name in ['center_m','rotation','half_m']:errors.append(float(np.max(abs(np.array(a[name])-np.array(b[name])))))
    return dict(fixtures=8,max_declaration_error=max(errors),passed=max(errors)<1e-12)
