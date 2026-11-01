# Reproduction and evidence boundaries

The recorded software environment used Python3.13 and WebotsR2025a. Some dynamics/image work also requires NumPy,SciPy,Matplotlib or OpenCV;install those only for the modules being used. Bundled runtimes and compiled physics plugins are not distributed here.

The README's three unit-control commands run without the native experiment archive. World assets and controller/source modules are included for inspection and further development. The earliest geometry/dynamics model assets and later full200Hzstudy sources represent different research stages;do not assume opening an early world reproduces the latest complete mission.

Complete native studies were built from frozen predecessor projects and audited against raw trajectories,source snapshots,model-property extraction and independent native contact/frame/reference data. Those inputs comprise most of the local121GBarchive and are excluded from Git. Consequently,a study's `prepare.py`,`test_preflight.py`,`test_evidence.py`,`run.py`or`seal.py` may require unpublished local evidence and notebook paths. The compact JSONoutcomes in `docs/results` are summaries,not substitutes for those inputs. Never rerun or overwrite a sealed local study.

Generate/compile/run new experiments in new directories. Keep original physical and numerical gates,known failures,and explicit source/profile/clock assumptions. At most two native workers were used. Do not speed up research by coarsening the physics step or pretending an offline replay is a new native test.

IMU count conversion uses an ideal manufacturer-example scale convention. Filtered and pre-filtered FIFO sources can have different gyro range semantics;the selected conversion model rejects pre-filtered inputs. Simulation producer read clocks are retained as read clocks. Calibrated sensitivity,real acquisition timestamps,physical FIFO driver behavior and closed-loop robustness are separate evidence requirements.
