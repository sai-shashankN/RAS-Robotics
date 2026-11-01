# RAS Robotics

A software-first robotics research project exploring a two-wheel balancing robot with an articulated arm and a two-finger gripper. Python and Webots are used to investigate manipulation, contact dynamics, sensing and numerical consistency before hardware assembly.

## Model roadmap

| Model | Intended features | Current boundary |
| --- | --- | --- |
| Alpha-A / Alpha-B | Fixed front arm, wrist pitch, two opposing curling fingers; different fixed pinch axes | Active simulation research |
| Beta | Rotating sleeve around the body and powered claw roll | Planned; not validated |
| X | Additional arm orientation and arm-assisted support/lifting or elevated-area mobility | Planned; claw/load-path redesign remains open |

## Current results

The latest sealed local checkpoint is **231**: **692 native simulation attempts and 2,510 final evidence checks**. Attempts include earlier rejected studies;these totals are not a completion percentage.

Four complete nominal Alpha-A/B missions at two physics resolutions pass the declared **prepared20mm / 2g** passive-observation scope. Recent source-contract work validates selected Bosch FIFO parsing/ordering,conditional counter-phase reasoning,range/count-to-SI semantics,and ideal quantization of four explicitly existing full-mission histories. The latest two phases add24software checks and zero robot trials.

Broader M2 dropped-object/fault pose convergence,realistic sensing and closed-loop sensor feedback remain unresolved. The hardware has not been built or qualified. Simulated nominal success does not establish useful payload capacity,physical calibration,acquisition timing or hardware readiness.

## Repository contents

- `simulations/common`: shared planning,control and world-generation code.
- `simulations/model-alpha` and `model-alpha-b`: model assets,controllers and research modules. The `full_200hz` package contains the latest completed native-observation study source;`count_si` and `count_replay` contain the latest completed conversion/replay source.
- `docs`: current design,boundaries,reproduction notes and compact checkpoint outcomes.

This is a curated source snapshot. The approximately121GB local experiment/conversation workspace is **not** uploaded. Raw trajectories,image captures,redundant project copies,compiled runtimes,private conversation exports and local orchestration logs remain local. Source checkpoint hashes in `docs/results` identify the local sealed evidence;the public snapshot is not a full archival backup.

## Quick software checks

Python3.13 was used for the recorded local work. No robot hardware is needed for these arithmetic and model controls:

```powershell
python -m unittest discover -s simulations/model-alpha/count_si -p test_controls.py
python -m unittest discover -s simulations/model-alpha/counter_phase -p test_controls.py
python -m unittest discover -s simulations/model-alpha/fifo_burst_epoch -p test_controls.py
```

Run each command in a separate process:the research packages intentionally use local module imports. Full evidence/preflight/seal scripts also depend on the excluded local study archive. See [reproduction notes](docs/REPRODUCIBILITY.md) before launching simulations.

Third-party Bosch SensorAPI sources retain their BSD-3-Clause license files. No project-wide license is selected for original code in this snapshot.
