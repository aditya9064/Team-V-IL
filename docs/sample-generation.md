# Dataset 3 sample generation

## First capture gate

Reproduce the unchanged starter before varying the scene. Use Isaac Sim's Python for `capture_scene.py`, not host Python. Table captures are 3840 × 2160 for cube stacking. Preserve the supplied calibration and physical geometry. Require readable RGB, camera intrinsics/extrinsics and evaluation ground truth. No rendered samples have yet been produced by Team V.

## Initial pilot

Generate 12 independently captured scenes: 4 nominal scenes, 4 valid position/rotation variations and 4 modest lighting variations. Begin with calibrated table-camera settings. Confirm arrangements are physically valid, in the workspace and non-overlapping. Preserve six 10 mm cubes with two of each color and the supplied distractors. Freeze one variation at a time so errors can be diagnosed. These counts are a pilot design, not completed captures.

## Quality and realism

Keep original lossless RGB captures. Reject blank frames, clipping, missing assets, incomplete shader loading, implausible contact and cubes outside the usable view. Inspect every pilot image at full resolution. Retain difficult but valid scenes rather than filtering only for model success. Check the physical camera setup with CSI before treating visual realism as hardware fidelity. Simulator friction and several finger dimensions remain estimates.

## Labels and provenance

Record scene ID, split, random seed, starter version, Isaac version, camera parameters, lighting settings and asset hashes. Ground truth may create training labels and score predictions; it must never supply inference grasp commands. Label cube identity/color, visible bounds or masks when exported, tabletop pose and visibility. The existing capture is not a complete annotated dataset exporter; add masks/bounds only through verified simulator outputs.

## Splits and progression

Keep all frames from one scene/run in one split. Reserve independent layouts/seeds for validation and final simulation testing. Separate CSI real evaluation from synthetic training. Expand beyond the pilot only after quality checks and baseline reproduction; choose dataset size using measured learning curves and rendering cost, not an arbitrary large batch.

## Compute

Host dependencies and tests run locally. Rendering requires an available Isaac installation and compatible GPU. No paid instance is started automatically. Use a free hosted API for inference only after validating an adapter; it cannot replace Isaac rendering. Prefer shared GB10/Spark or a benchmarked L40S Isaac launchable, and stop idle instances.
