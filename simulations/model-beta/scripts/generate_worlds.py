from pathlib import Path
import runpy
import sys
root = Path(__file__).resolve().parents[1]
sys.argv = [sys.argv[0], str(root)]
runpy.run_path(str(root.parent / 'common' / 'generate_worlds.py'), run_name='__main__')
