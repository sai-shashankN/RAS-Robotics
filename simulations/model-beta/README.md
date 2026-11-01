# Model Beta — sleeve and rotating-claw stage

Current design: two opposing curling fingers, a rotating body sleeve and powered wrist/claw roll. The roll joint changes pinch direction about the claw approach axis. It is distinct from wrist pitch, the sleeve and X's extra arm-base swivel. No arm-assisted mobility in Beta.

The geometry preview has been regenerated with two fingers. **Powered claw roll is planned and not implemented in this preview.** The old debug results do not validate the revised Beta. Develop Alpha first; roll range, bearings, motor, wiring and combined clearance remain open. Unlimited rotation is not assumed.

From the workspace root:

```powershell
py -3.13 -X utf8 simulations/model-beta/scripts/generate_worlds.py
powershell -ExecutionPolicy Bypass -File simulations/model-beta/launch.ps1 -Mode geometry
```

Geometry is a prescribed massless joint fixture with self-collision disabled. It does not demonstrate dynamics, strength, contact or motor tracking. Shared source in simulations/common is authoritative. Existing Beta dynamics remains exploratory and unvalidated. The previous prototype/configuration is preserved in Alpha's three-finger checkpoint as well as Beta's existing archives.

## Latest roadmap clarification — 24 October 2026

Alpha-A is the existing front/back pinch configuration; Alpha-B is the otherwise matching fixed left/right comparison, specified but not generated/run. Beta targets powered full 360-degree claw orientation, with useful task testing and separate movement-limit exploration. Its roll joint is not implemented yet. A/B results inform Beta without validating intermediate angles or added-joint dynamics.

The circular palm remains a prototype proxy; a compact bridge/rectangular palm is a proposed comparison. Alpha/Beta claws are for manipulation. Reopen the claw design for robot-weight support/lifting at X time. See [the comparison plan](../../project-notes/GRIPPER_COMPARISON.md).
