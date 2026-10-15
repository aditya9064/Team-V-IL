# Shared context for teammates and AI assistants

Updated October 15, 2026. Treat this as the current project handoff, not as proof that proposed experiments or physical deployment have been completed.

## Goal and manufacturing scenario

Prepare small-part kits with a FANUC ER-4iA arm. Parts arrive on a work surface; a recipe specifies required colours and destination slots. Re-observe between picks, detect changed or missing inventory, verify grasps and placements, and recover conservatively from failures.

Initial targets are upright 10 mm cubes; caps and vials are distractors. Arbitrary-object 6D grasping and conveyor interception are outside the implemented scope. In a physical cell, replenishment or rearrangement must follow approved stopped-robot procedures. Simulation disturbances do not authorize anyone to enter a moving robot's workspace.

Organizers emphasized that static planning from one initial table image is already solved in the supplied twin. Our extension focuses on feedback, changed scenes and recovery. Preserve the original six-cube stacking baseline separately from the new flat-slot kitting task.

## Architecture

1. Capture table, side and wrist RGB with camera metadata and measured robot joint state.
2. Detect cube colour/location with the existing geometric baseline; refine planar position/yaw through image-only silhouette fitting.
3. Match observed inventory to a recipe. Missing inventory yields bounded WAIT.
4. Take a fresh pre-dispatch observation; replan if the target changed.
5. Preflight the complete pick/place trajectory using existing robot FK/IK and joint limits.
6. Execute one pick. Require source disappearance **and positive held-part camera evidence** in two fresh observations.
7. Place into a separate flat slot. Require two fresh table observations and complementary side-camera evidence of the expected colour/pose.
8. Commit the slot, observe again, and repeat. Permit one retreat/re-observe retry after a camera-confirmed empty grasp. Uncertain outcomes or exhausted retry produce HOLD.

Table RGB supplies source/destination poses. The rendered side view can see the held cube. The present wrist mounting cannot, so it remains diagnostic rather than positive pickup proof. An equivalent physical CSI side view has not been confirmed.

## Stack and platforms

- Isaac Sim 6.0.0.0, Python 3.12, official FANUC v05 starter, Rosie T4 scheduler allocations.
- OpenCV, NumPy, SciPy and USD camera metadata for segmentation, projection/backprojection and bounded planar silhouette fitting.
- Current runtime detector: HSV/geometric baseline, **not a trained neural detector or Cosmos output**.
- Mentor training pilot: PyTorch 2.10 / torchvision 0.25, Faster R-CNN ResNet-50 FPN v2 with COCO initialization and small-object settings. Mentor handles H100 training; trained results/checkpoint are pending.
- Brev is an optional cloud GPU platform. A limited benchmark was run and its instance stopped afterward. Do not infer zero storage charges or launch paid compute without a bounded authorized task and shutdown plan.
- Cosmos is a potential ecosystem component, not required by the current geometric runtime. Do not describe geometric predictions as independent VLM pose estimation.
- GB10/CSI access information is shared privately. Physical inference performance and the approved robot-control interface remain unvalidated.

## Gripper and calibration

The starter models an SMC MHZL2-16D2 gripper: 6 mm stroke per jaw, 12 mm straight-face opening, existing hooked fingers. The present 10 mm cube scope uses these fingers; redesign is conditional on a demonstrated need for larger parts.

The starter's `docs/CALIBRATION_V05.md` documents supplied geometry/camera assumptions. It is not a completed calibration of our real deployment. At CSI confirm intrinsics/distortion, camera-to-robot transforms, table plane, tool centre point, jaw opening, object dimensions, physical camera coverage and the approved control interface. Do not invent controller commands or assume simulator interfaces transfer directly.

## Verified progress

- Original stack demo: six successful simulated pickups/placements, zero reported trajectory IK failures, 1,656 frames / 55.2 seconds of 1080p video.
- Corrected data pilot: 78 unique scenes / 432 boxes. Train 56 images / 324 boxes; validation 11 / 54; test 11 / 54. One empty image per split.
- An earlier label-coordinate bug was corrected. Legacy `synthetic-v1` / `synthetic-crops-v1` exports are obsolete. Corrected overlays and patches were reviewed; this does not prove real-world generalization.
- Training code passed forward/backward/inference smoke tests on positive and empty samples. Zero optimizer training steps were performed; no trained model is claimed.
- Dynamic pilot 284262: one camera-verified cube pickup and placement, independent simulator score agreed, approximately 0.33 mm slot-centre error. One simulation result is not physical accuracy or a reliability estimate.
- Gate cases 284267: empty-table and missing-colour trials waited without picking; stale-observation trial held before motion. Motion suite 284274: moved target completed after REPLAN; a single missed grasp completed after exactly two attempts; repeated missed grasp held after two failed attempts with no third. Its six-part case held at the first yellow-cube lift on the strict camera-stillness gate. That run did not complete the six-part task.
- Diagnostic pilots 284285/284286/284288 preserved the stillness hold. A reported wrist velocity persisted despite essentially constant measured joint positions. Higher velocity iterations and a smaller timestep did not resolve it. See `dynamic-loop/STILLNESS-DIAGNOSIS.md`; PGS pilot 284291 then passed one yellow pickup/placement with all strict gates active. Suite 284292 held after three verified placements and a fourth verified pickup on insufficient side-camera placement coverage. A separate inclusive-raster geometry defect was identified and corrected with five analytic/rejection tests; thresholds remain unchanged. Rerun 284300 completed all six pickups and placements with runtime camera verification and matching independent score. Fifty accepted captures passed the strict stillness audit; single-miss recovery, repeated-miss HOLD and moved-target REPLAN/COMPLETE also passed under the current configuration. The full audit covered 87 accepted captures. Read `dynamic-loop/validation-report.md` and the machine-readable results for source/report hashes and provenance.
- Sixteen dynamic tests (including five geometry/rejection regressions) and the original 33 starter tests passed on Rosie. Seven replay and eleven supervisor tests passed locally. CPU tests alone do not validate GPU physics or model performance.
- No physical-arm testing or real-world calibration has been completed.

## Training handoff and data rules

The separate `Team-V-Detector-Pilot-v2.zip` contains 1920 x 864 RGB crops, labels, camera/crop mappings, QA sheets, training/evaluation scripts and instructions. It is approximately 23 MB and was sent to the mentor by the user. Image classes: 0 yellow, 1 blue, 2 red; the torchvision loader maps these to foreground IDs 1, 2, 3, with background 0.

Ask the mentor to return the checkpoint, exact configuration, validation/test metrics and representative predictions/errors. The detector predicts objects; it does not itself control the arm. Integration must preserve calibration and verification gates.

Expand data based on measured failures and CSI appearance/calibration. Keep related scene/layout families out of multiple splits. Review visibility, clipping, box geometry, colour balance and empty scenes before each release. Augmented copies are not independent scenes.

## Rules for continuing work

- Read applicable starter `AGENTS.md` instructions before editing the supplied twin. Preserve its original task and geometry; keep the adaptive extension separate.
- Runtime targets/outcomes must use RGB, calibration, joint feedback and the recipe. Simulator object poses and ground truth are for explicit test-harness setup/fault injection and independent post-run evaluation only.
- Preserve fit, freshness, inventory, pose, motion and outcome checks. Diagnose failures rather than weakening thresholds to get a pass.
- Source disappearance alone is not pickup proof. Slurm `COMPLETED` or process exit zero alone is not experiment success.
- Save all attempts, holds and injected faults. Use fresh output directories and inspect reports, RGB evidence and independent simulator scores.
- Current thresholds and RGB verification are provisional simulation methods. No collision certification, hardware safety controller or physical robot adapter is implemented.
- Keep credentials, private correspondence, raw supplied assets, generated datasets and model weights out of this public repository. Do not send messages or spend cloud credits without authorization.
- Use relative paths or user-provided environment paths. Rosie scripts default to the current user's home. Expensive simulations belong on scheduler GPU allocations, not management/login nodes.
- State separately what was implemented, host-tested, GPU-tested, trained and physically tested. Never present a planned step as complete.

## Detector plug-in and additional situation tests

Five new situations and the six-part regression passed exact simulation checks in jobs 284313/284319: occupied destination, missing side stream, late arrival, blocked pickup view and moved completed part. The first arrival fixture failed and is preserved; only its test setup changed before the successful rerun. See `dynamic-loop/additional-validation-report.md` and its JSON audit for mixed-job provenance, source/report hashes, 79 accepted camera observations and retained failure evidence. Thirty-three plug-in/fixture tests and the original 33 starter tests passed on Rosie.

The interface supports the mentor's expected checkpoint format, RGB crop/class mapping and strict pose/outcome gates. Its actual-architecture CUDA smoke used untrained random weights, zero training steps and zero motion. Motion trials used the geometric backend. No trained checkpoint has been received or deployed. Read `dynamic-loop/MODEL-PLUGIN.md` before selecting a learned backend. The completed trial requires no additional GPU allocation.

## Next steps

The stillness/rasterization fixes, six-part trial and additional destination/camera/arrival/inventory cases are validated in bounded simulation trials. Next obtain the mentor checkpoint and validation metrics, evaluate it offline, and run a single-part simulation pilot behind the same strict checks. Broader quantities/layouts and explicit wrong-part evidence remain useful future tests; do not assume these are covered by the six recorded cases. Complete approved CSI calibration, confirm camera views/control interfaces, and perform single-part hardware trials before scaling up.

The repository README links each component. Obtain the official starter and image package through approved team storage; they are intentionally external. Share this file and the relevant component README with a new AI assistant, then ask it to inspect current code and actual reports before continuing.
