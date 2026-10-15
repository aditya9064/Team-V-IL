# FANUC synthetic dataset tooling

Status: initial capture wrapper prepared; varied renders and training export are not yet verified.

`capture_dataset.py` wraps the provided v05 capture without editing it. It varies the four movable cubes, all six yaw angles, and light intensity by 0.7–1.3. It preserves camera calibration, cube dimensions, gripper geometry, two fixed yellow positions, and distractors. Seeds and parameters are saved. These constrained variations do not represent the full physical operating distribution.

Start with three pilot seeds, inspect images and labels, then expand to 60 independent scenes. Keep entire scenes together in train/validation/test splits; crops and augmentations from one scene must stay in that scene's split. Do not use the five development scenes as a final independent test set.

Training export must project the eight cube corners using camera calibration and the actual saved world pose, emit normalized bounding boxes and color classes, and validate boxes against rendered images. Such boxes are projected/amodal labels, not measured visible masks. Occluded objects require visibility handling before use. Pose training requires separate XY/yaw labels and square-symmetry-aware error, not bounding boxes alone.

## CSI information and collection checklist

- Robot and controller exact models; approved external command interface/protocol, coordinate frames, units, tool/user frames, and a sample program.
- Actual installed gripper, jaw travel, hooked-tip geometry, TCP, payload, and permitted hardware changes.
- Camera model, lens, RGB resolution/FPS, exposure/white balance, mounting pose, distortion/intrinsic calibration, and camera-to-robot calibration with measured residuals.
- Cube dimensions/colors/materials, caps/tubes, table height/material, reachable workspace, placement target and allowed task sequences.
- Supervision, speed/force/workspace restrictions, emergency stop and reset procedures, and allowed tests.
- Permission for a first small real dataset: diverse cube arrangements and rotations, empty scenes, caps/tubes without cubes, representative lighting/shadows, and approved occlusions. Save raw images with calibration version, lighting, object identities and measured poses where available. Record successful and failed supervised trials with timestamps and robot state.
- Decide transfer acceptance with mentors: localization and yaw tolerances, pickup/placement success rate, task completion, latency, and recovery/stop behavior. Do not infer tolerances from synthetic accuracy alone.

Before CSI: validate variations, export labels, train a lightweight baseline, evaluate on held-out synthetic scenes, run multi-scene physics tests, document failures and freeze a reproducible environment. Keep hardware actuation disabled until the controller integration and supervised procedure are agreed.
