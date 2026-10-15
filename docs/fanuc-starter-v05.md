# Dataset 3 starter v0.5: intake and compute choices

Inspected October 10, 2026. Source: `fanuc_er4ia_hackathon_starter_v05.zip` supplied by the team. Extracted unchanged under `data/external/fanuc-v05/fanuc_er4ia_sim/` (Git-ignored).

## What is supplied

39 files: procedural cell construction, calibrated scene configuration, five geometry assets, hybrid perception, grasp/stack planning, forward/inverse kinematics, execution verification, launcher scripts, setup instructions and host tests. Task: stack six rotated 10 mm cubes yellow → yellow → blue → blue → red → red; caps and tubes are distractors.

No recorded dataset, captures, videos, trained weights or saved runs are included. FANUC robot USD is loaded from Isaac's asset service/cache; its referenced files are not bundled. Cosmos framework and model snapshot are separate dependencies. Procedural textures are generated locally.

The supplied setup reports validation on Linux, Isaac Sim 6.0 and RTX A6000. This is the author's validation, not Team V's reproduced result. Smaller GPUs and Windows are not benchmarked in the package.

## Architecture

`capture_scene.py` → `perceive.py` / `cube_pose.py` → `plan_sort.py` → `sort_scene.py` → execution and perception reports.

Cosmos detects/classifies cubes; calibrated color segmentation and square-silhouette fitting measure position and yaw. Cosmos receives these measurements. This is hybrid perception, not independent VLM pose estimation. Ground truth is reserved for evaluation. Preserve confidence, inventory, pickup and support checks.

Important geometry: SMC MHZL2-16D2 actuator, 6 mm travel per jaw, 12 mm straight-face maximum gap. This differs from the earlier generic gripper assumption in our planning; use the package-specific geometry and confirm against CSI hardware before physical deployment.

## Cost rule

Use local CPU for preprocessing, host tests, planning and report analysis. Test hosted NVIDIA model APIs before provisioning separate inference GPUs. An API backend is an integration task: the current starter loads local CUDA Cosmos weights; it does not automatically use hosted endpoints. Confirm image inputs, output schema, limits and availability before replacing that backend.

Use shared MSOE GB10/UWM Spark when access and software compatibility are confirmed. For cloud simulation, benchmark the Isaac launchable on the least expensive compatible GPU; the supplied guide budgets L40S Isaac workloads around $3/hour and training around $2/hour. Check actual console rates. Never select the roughly $17/hour video-generation environment for this workload. Do not assume the whole simulation can run through a free model API.

No paid instance has been started by this inspection. Stop instances between active jobs, save results first, and account for stopped-disk storage charges.

## Local verification

Ran the four supplied unittest modules with bundled Python 3.12. Three runtime-configuration tests passed. The other three modules failed to import because OpenCV (`cv2`) is missing. No Cosmos inference, Isaac rendering, PhysX execution or real hardware test was performed.

Next: prepare an isolated host environment using the supplied requirements; run all host tests and texture generation; confirm Isaac asset access; reproduce the unchanged baseline in a fresh run directory. Require `task_complete: true`, six verified pickups and six verified placements in the execution report. An exit code alone is insufficient.

## References

- Package README, SETUP, MODEL_NOTES and CALIBRATION_V05 in the extracted directory.
- [Competition resources](https://msoe-maic.com/library/?article=Rockwell-NVIDIA-Fall-2026-Resources)
- [Compute guide](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute)
