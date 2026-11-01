import os,subprocess,shutil
from context import ROOT,SOURCE,STUDY,read,write,sha
def main():
    folder=SOURCE/'build';folder.mkdir(exist_ok=True);install=ROOT/'.local-tools/webots';binary=install/'msys64/mingw64/bin';obj=install/'resources/projects/plugins/physics/physics.o';ode=binary/'ode.dll';dll=folder/'impact_probe.dll';env=dict(os.environ);env['PATH']=';'.join(map(str,[binary,binary/'cpp',install/'msys64/usr/bin']))+';'+env['PATH'];args=[str(binary/'gcc.exe'),'-O2','-Wall','-Wextra','-Werror','-shared','-I'+str(install/'include'),'-I'+str(install/'include/ode'),str(SOURCE/'impact_probe.c'),str(obj),str(ode),'-lpthread','-o',str(dll)];r=subprocess.run(args,env=env,capture_output=True,text=True);write(folder/'build.json',dict(command=args,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr,source_sha256=sha(SOURCE/'impact_probe.c'),support_sha256=sha(obj),ode_sha256=sha(ode),dll_sha256=sha(dll) if dll.exists() else None));assert r.returncode==0,r.stderr
    for e in read(STUDY/'manifest.json')['entries']:
        shutil.copy2(dll,ROOT/e['project']/'plugins/physics/impact_probe/impact_probe.dll');shutil.copy2(SOURCE/'impact_probe.c',ROOT/e['project']/'plugins/physics/impact_probe/impact_probe.c')
if __name__=='__main__':main()
