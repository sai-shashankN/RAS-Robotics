"""Rebuild the 19 prepared grasp/alignment/load regressions and their summaries."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys

MODEL = Path(__file__).resolve().parents[1]
WORKSPACE = MODEL.parents[1]


def python_script(name, *args):
    subprocess.run([sys.executable, '-X', 'utf8', str(MODEL/'scripts'/name), *args], cwd=WORKSPACE, check=True)


def reference(mode):
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(MODEL/'launch.ps1'),
                    '-Mode', mode, '-Validate'], cwd=WORKSPACE, startupinfo=startup,
                   creationflags=subprocess.CREATE_NO_WINDOW, check=True)
    print(mode+' control complete', flush=True)


def main():
    python_script('run_loaded_clearance.py')
    python_script('run_capture_alignment.py')
    python_script('run_capture_alignment.py', '--phase', 'controls')
    python_script('generate_grasp_worlds.py')
    python_script('audit_grasp_clearance.py')
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(reference, ['grasp','grasp_negative']))
    python_script('summarize_loaded_clearance.py')
    python_script('summarize_grasp_validation.py')
    python_script('summarize_capture_alignment.py')
    print('All 19 prepared regression reports match final sources and expected outcomes.', flush=True)


if __name__=='__main__':
    main()
