import os,subprocess
from context import ROOT,SOURCE,write,sha
def main():
    folder=SOURCE/'build';folder.mkdir(exist_ok=True);assert not(folder/'fifo_probe.dll').exists()
    if(folder/'build.json').exists():
        previous=folder/'build-initial-missing-runtime.json';assert not previous.exists();(folder/'build.json').rename(previous)
    install=ROOT/'.local-tools/webots';binary=install/'msys64/mingw64/bin';env=dict(os.environ);env['PATH']=';'.join(map(str,[binary,binary/'cpp',install/'msys64/usr/bin']))+';'+env['PATH'];dll=folder/'fifo_probe.dll';args=[str(binary/'gcc.exe'),'-O2','-Wall','-Wextra','-Werror','-shared','-I'+str(SOURCE/'vendor'),str(SOURCE/'vendor/bmi2.c'),str(SOURCE/'vendor_bridge.c'),'-o',str(dll)]
    inputs={str(p.relative_to(SOURCE)):sha(p) for p in [SOURCE/'vendor_bridge.c',SOURCE/'vendor/bmi2.c',SOURCE/'vendor/bmi2.h',SOURCE/'vendor/bmi2_defs.h',SOURCE/'vendor/LICENSE']};r=subprocess.run(args,env=env,capture_output=True,text=True);write(folder/'build.json',dict(command=args,returncode=r.returncode,stdout=r.stdout,stderr=r.stderr,inputs=inputs,dll_sha256=sha(dll) if dll.exists() else None,scope='Host memory-only manufacturer parser bridge;not a board driver or robot/native simulation.'));assert r.returncode==0,r.stderr;print('Built exact manufacturer parser with blocked I/O callbacks;no hardware executed.')
if __name__=='__main__':main()
