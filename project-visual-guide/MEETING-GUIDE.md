# Team V mentor guide

Read the 12 slide images in order. The drawings explain concepts; they are not photographs, test evidence, engineering CAD or training images. Current project status and website requirements were checked October 11, 2026. Images were generated with the built-in image-generation tool. Slide briefs and generation prompts are in generation-manifest.json.

## Slides 1–4: what we are building

The target is the standalone CSI FANUC robot, separate from the vial production line. Our first task uses six upright 10 mm cubes, two each of yellow, blue and red, with caps and tubes as distractors. The supplied simulation stacks the cubes yellow-yellow-blue-blue-red-red. The physical task, geometry and controller must be confirmed with CSI.

Our first system already detects cubes using color and image geometry and has completed one simulated scene with six verified pickups and placements. A rendered repeat passed its full-task assertions. This is preliminary evidence, not proof of reliable operation across layouts or on hardware. We have not trained a neural detector. Camera-based runtime outcome checks and recovery remain planned. Some conceptual illustrations include other cube colors; actual first-task classes remain yellow, blue and red.

A trained detector will identify cubes and give approximate image locations. Calibration and geometry convert those observations into precise robot positions and angles. The planner checks the proposal before motion. After each move, the final system should look again and determine whether the correct object moved. One fresh-pose retry is proposed for a known recoverable miss; uncertain state or exhausted budget causes a stop. CSI must approve physical actions and recovery limits.

## Slide 5: images, training, validation and testing

Generating a sample means Isaac renders a camera image and saves known object poses, camera calibration and generation settings. Training means adjusting a model using examples. Those are different jobs.

We generated 60 varied scenes and exported projected cube boxes. Forty-eight belong to training, six to validation and six to test. Validation helps select a model and settings. The test set checks the frozen choice. Crops and augmentations from the same source scene remain in the same split.

Sixty scenes are enough to exercise the pipeline and start a baseline, not establish real-world reliability. Yellow positions and distractor positions remain fixed. Visibility checks and full-resolution label review remain incomplete. Next data decisions should address specific failures: empty scenes, absent cubes, varied distractors, changed illumination and approved occlusions. Changing a camera requires matching calibration. Avoid color changes that invalidate class labels.

## Slides 6–9: tools in plain language

| Tool/resource | Meaning | Role for us |
|---|---|---|
| Digital twin | A computer representation of equipment and workspace | Supplied FANUC v05 starting point; verify match to physical setup |
| Isaac Sim | Simulator for robot physics, cameras and images | Our running virtual environment |
| Isaac Lab | Tools for robot learning experiments using simulation | Optional later; we are not doing reinforcement learning now |
| Omniverse / OpenUSD | NVIDIA ecosystem and scene format for assembling 3D worlds | Underlying twin/scene infrastructure |
| Replicator | Isaac/Omniverse synthetic-data tools for randomization and labels | Planned scalable generation and visibility-aware export |
| Emulate3D Connector | Connects factory simulation with 3D workflows | Primarily the separate Rockwell production-line twin, not our core task |
| Rosie | MSOE's shared computing cluster | Runs Slurm jobs independently of our laptop |
| T4 | GPU available on Rosie | Successfully renders our Isaac 6.0 cell |
| H100 | Powerful GPU for AI computation | Mentor will run model training; not the supported RTX camera-rendering route |
| DGX Spark / Dell GB10 | Shared NVIDIA computing systems at UWM/MSOE | Access and scheduling must be confirmed; UWM has Spark, MSOE has GB10 |
| Brev | Cloud platform for renting GPU machines | Optional rendering benchmark once organizer-funded credits arrive |
| L40S / RTX PRO 6000 | GPUs recommended by MAIC for Isaac rendering | Brev options; console price and availability must be checked |
| CSI workstation | Windows workstation with RTX PRO 5000 Blackwell, Nucleus, Omniverse and Emulate3D | In-person resource; reservation process requires confirmation |
| Nucleus | Shared storage/service for 3D scene assets | CSI infrastructure; no unapproved connection |
| Cosmos | Models for physical-scene reasoning and visual generation | Optional semantic baseline or outcome reasoning; not used in our successful demo |
| Cosmos Edge / Nano | Model variants with different input/capacity requirements | Select endpoint based on task and current model limits |
| build.nvidia.com | Hosted model APIs | Try Cosmos without managing model GPUs; current quotas/limits apply |
| API | A defined way for software to ask another service for results | Keep keys out of GitHub and chat |
| PyTorch / torchvision | Libraries for training and running models | Proposed Faster R-CNN detector baseline |
| OpenCV / NumPy / SciPy | Image processing, numerical math and optimization | Existing color detection and silhouette refinement |
| Gemini / Antigravity | Recommended free coding-agent starting point | Helps write/debug code; verify its output |
| Claude / Codex | Other coding agents | Requested Claude capacity for long development runs; no current Codex plan request |
| TAO | NVIDIA toolkit for vision-model development | Optional inspection/model tooling |
| DEFT AOI | Automated visual-inspection workflow | More relevant to defect inspection than cube grasping |
| VisualChangeNet | Vision tooling for visual change/inspection problems | Optional; not required for Dataset 3 baseline |
| FLUX | Generative image model | Optional appearance augmentation, with label revalidation |
| Physical AI Data Factory augmentation | Pipeline for augmenting visual data | Optional after defining a measured data gap |
| NuRec | Reconstructs 3D environments from real observations | Possible real-to-sim resource, not a current dependency |
| NeMoClaw / NeMo Agent Toolkit | Tools for organizing AI-agent workflows | Optional developer/agent orchestration; not robot safety control |
| Factory Operations FOX | Architecture/blueprint for factory AI operations | Background reference, not a turnkey implementation for our cell |
| SOP tools | Analyze adherence to sequences or operating procedures | Could inspire task-progress checks; not integrated |
| Kumo-TS | Time-series forecasting tooling | Relevant to time-series problems, not current cube localization |
| ROS 2 / FANUC driver | Robotics communication software | Use only if CSI confirms installed controller support and approves interface |
| FactoryTalk / OPC UA | Industrial software and communication interfaces | Approval required; do not connect independently |
| GitHub / ALL AI Network | Code collaboration and competition submissions | Use official team repository; exclude secrets, large datasets and checkpoints |
| Slack | Organizer communication and test scheduling | #data-and-testbed-requests, tag Shamar Webster |

The site recommends RTX PRO 6000 when available, or L40S as a fallback, for Isaac on Brev. L40S simulation is listed around $3/hour as a planning estimate. Our current Brev organization shows zero credits and no instance. No cloud speed benchmark has been performed. A $17/hour video-generation environment is not needed for our current workload.

The website says Cosmos 3 does not provide image-to-image editing. It lists ChatGPT/Gemini/FLUX for that category. Generated appearance changes can move object boundaries or alter colors, so they cannot automatically inherit trusted labels.

## Slide 10: CSI meeting

Confirm actual gripper, fingers, TCP, robot/controller version, approved command interface, camera intrinsics/distortion and camera-to-robot calibration. The website describes SCHUNK, while our v05 starter models SMC MHZL2-16D2. Resolve the discrepancy before physical execution.

Ask what views, object variations, supervised trial counts and operating limits are available. Request observation/calibration first, integration next and a final evaluation slot, subject to CSI availability. No guaranteed number of visits is published.

The overview says supplied real production-line data is for inspection rather than training, while the collection guide discusses mixed real/synthetic evaluation. Clarify Dataset 3 permission. Default to synthetic training plus approved real calibration and held-out physical evaluation. Do not fine-tune on CSI images without that clarification.

## Slide 11: competition plan

October 14, 11:59 PM CT: short shared team resource/support plan. Required form fields cover team/project, dataset, problem/approach, additional data, requested compute and justification. Optional fields cover coding-agent plans, mentor support and questions. Our existing saved submission needs team review and corrections; this guide does not approve or submit it.

November 1, 11:59 PM CT: about five-minute video, one-page summary, repository and references/credits. Published expectations include real-hardware operation, a specific problem and manufacturer business case, measured baseline comparison and limitations. November 5: live presentations at MSOE Diercks Hall, 5–8 PM. Detailed judging weights remain pending.

## Slide 12: future meeting agendas

Meeting 1 — scope and ownership: agree on approved cube task, tech stack, controller unknowns, and paired workstreams for simulation/data, perception, control/verification, evaluation/demo. Every task gets an owner, reviewer, due date and test.

Meeting 2 — data and mentor handoff: review full-resolution labels, negatives and variation coverage; freeze scene splits; decide image preprocessing and class mapping; complete training script, environment, validation metrics and checkpoint-return instructions. Mentor handles H100 execution.

Meeting 3 — CSI preparation: resolve hardware mismatch, clarify data permission, confirm command protocol/calibration, book visit, prepare read-only observation and supervised test checklist.

Meeting 4 — evidence and failure handling: compare fixed-motion, geometric and learned baselines on matched scenes; log every attempt; agree on stop conditions and approved retry budget. Report pick/placement success, full-task completion, errors, cycle time and interventions with sample counts.

Meeting 5 — final demonstration: freeze versions, record approved hardware evidence, explain baseline improvement and limits, estimate manufacturer value using measured correction/setup times, and prepare video/summary/repository references.

## Sources

- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026
- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Resources
- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute
- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Data-Collection
- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting
- https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Coding-Agents
- https://www.nvidia.com/en-us/ai/cosmos/
- Team's supplied Rosie logs and FANUC v05 starter.
