# Model Alpha-B — fixed left/right pinch

Alpha-B uses the same two curling fingers as Alpha-A, mounted on palm-local +/-Y (pi/2, 3pi/2). Reference pinch is left/right, versus A's front/back. No powered claw roll, sleeve, extra arm-base swivel or arm mobility. The circular palm, dimensions, masses/COM, gains, motor limits and targets match A; only finger orientation and descriptive labels differ.

## Verified empty behavior

Four prescribed geometry poses, thirteen empty dynamics cases and six specified conservative robot-clearance paths pass with matching sources/worlds. Zero arm contacts are detected in the empty dynamics tests. A was verified from its unchanged source/model evidence. See [paired results](../model-alpha/results/AXIS_COMPARISON_RESULTS.md).

Geometry is a stationary massless fixture with prescribed joint positioning and self-collision disabled. It demonstrates shapes/FK, not motor/contact performance. Dynamics uses Physics, bounded motor commands and nonadjacent self-collision, with no body/joint pose writes. Direct joint pockets, arbitrary workspace and hardware remain open.

## Initial object-contact result

The matched 2 g/20 mm sphere probe fails B retention: both fingers touch at 7.360 s, but contact is lost after 7.656 s while closing; arm lift is not scheduled until 12 s. Last contacts reach the side edges of the distal collision faces. The sphere falls into the tray. A retains the same sphere through the six-second unsupported hold, but its release also finishes in the tray. Both open controls leave the sphere on its pole. These are one-point diagnostics, not overall A/B rankings or precise placement.

See [contact traces, plot and failed checks](../model-alpha/results/axis_contact_probe/CONTACT_PROBE_RESULTS.md). Contact geometry/closing motion, directional objects and supported release are next. No motor/friction/geometry changes were made to conceal B's failure.

## Reproduce from the workspace root

```powershell
py -3.13 -X utf8 simulations/model-alpha/scripts/prepare_alpha_b.py
py -3.13 -X utf8 simulations/model-alpha/scripts/audit_axis_variant.py
py -3.13 -X utf8 simulations/model-alpha/scripts/run_empty_validation.py --model simulations/model-alpha-b
py -3.13 -X utf8 simulations/model-alpha/scripts/summarize_axis_comparison.py
```

Interactive geometry:

```powershell
powershell -ExecutionPolicy Bypass -File simulations/model-alpha-b/launch.ps1 -Mode geometry
```

The first contact study is protected against overwrite. Its frozen source/inputs are in model-alpha/experiments/axis_contact_probe/source_snapshot; raw reports, traces, telemetry and actual renders remain in each model's results/axis_contact_probe folders. Preserve it before another study; the probe runner does not silently replace it.

Beta later adds the sleeve and full 360-degree powered claw orientation. X support/lifting claw design remains deferred. No hardware purchases or performance claims.

## Controlled distal-width result — 24 October 2026

The isolated 10/16/24 mm flat-face comparison retains with A at every width and loses B during closing at every width. Added width delays B's last contact by only 32/56 ms; both wide-face open controls remain on-pole non-grasps. No width is promoted. Original A/B files and evidence are unchanged; the next hypothesis is a retaining/contact profile or separate closure-motion change. Precise supported placement remains open.

[Verified width study and plot](../model-alpha/experiments/contact_width_study/WIDTH_STUDY_RESULTS.md).


## Shallow contact-profile result — 24 October 2026

Isolated 10° and 15° V profiles across the original 10 mm distal-face width retain the 2 g/20 mm sphere with both A and B through a six-second unsupported hold. Fresh flat A/B controls exactly reproduce the earlier position traces; flat B still escapes during closure. Five of six positive trials pass the inherited cycle, and both open-claw controls pass their non-grasp checks. All eight runs record zero unintended arm contacts; no simulator warnings or infrastructure errors were recorded.

Closing still moves retained spheres about 6–7 mm horizontally and 18–19 mm upward; release ends in the catch tray. This improves retention in the prepared scene without establishing precise supported placement. The two convex half-prisms preserve the actual V recess in collision geometry. Mass/COM and all control inputs remain fixed; derived inertia changes with shape. Main A/B models are unchanged and no profile is adopted yet.

Next: use 10°, the smallest tested nonzero angle, as an isolated robustness candidate for matched offset/size/friction trials. Then test directional objects and supported lowering/release, and run appropriate candidate empty-motion checks before adoption. Beta roll and X support remain deferred.

[Verified profile experiment](../model-alpha/experiments/contact_profile_study/PROFILE_STUDY_RESULTS.md).


## Current direction: cube-first and editable geometry

The sphere neighborhood and ten physics diagnostics are preserved; numerical sensitivity prevents treating every coarse-step miss as a design boundary. No new geometry is adopted. The user authorizes arm, wrist, joint and wheel revisions. Begin with a cube and stable accessible support, then a directional block; keep spheres for later slip/rolling tests.

[Cube-first task](../../project-notes/CUBE_FIRST_BENCHMARK.md); [progress](../../project-notes/PROGRESS.md).
