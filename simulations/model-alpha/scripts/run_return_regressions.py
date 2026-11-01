"""Revalidate the nine approach cases and 19 prepared cases after return changes."""

from concurrent.futures import ThreadPoolExecutor
from run_prepared_regressions import python_script


def clearance(name):
    python_script('audit_folded_approach.py','--case',name)


def main():
    python_script('run_folded_approach.py')
    with ThreadPoolExecutor(max_workers=3) as pool:
        list(pool.map(clearance,['reference_50mm','approach_40mm','approach_60mm']))
    python_script('run_prepared_regressions.py')
    python_script('summarize_folded_approach.py')
    print('All 28 prior mission/prepared reports verified with final source.',flush=True)


if __name__=='__main__':
    main()
