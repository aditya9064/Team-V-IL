# Team V: official requirements audit and final-week strategy

Checked 25 October 2026. This document separates published requirements, current platform behavior, unresolved conflicts and our recommendations. It does not authorize a robot run, a purchase, a hardware change, a CAD revision or a submission.

## Decision recommended

Build a **recipe-driven assembly cell for a small family of industrial sensor enclosures**. Demonstrate useful product assembly, changes in part location, an approved recipe change, outcome verification and bounded recovery on the real FANUC cell.

The printed PCB is a dummy. The first product demonstrates mechanical assembly; it must not be described as a functioning sensor, an electrically tested board or a certified industrial instrument. Keep the board separate and intact inside its carrier. Geometry changes, including the separate size proposal awaiting an email reply, remain separate decisions.

Our strongest presentation would show the same perception, planning and execution software completing a second approved recipe without manually teaching another fixed path. This is a proposed differentiator, not an additional official requirement or a guaranteed judging bonus.

## Audit coverage

The MAIC frontend identifies its public content source as `https://msoe-ai-club.github.io/maic-content/manifest.json`. The current index lists **812 files** and is dated 15 October. Every current-event guide was fetched and read: overview, Dataset 1, Dataset 2, Dataset 3, collection/testing, compute, coding agents, tools/resources. Their eight metadata records were also fetched. All **96 unique links** in those guides were indexed.

Additional primary sources reviewed: the current participant dashboard reached from ManipBench, the official robot-client README and policy interface, both submission templates, camera calibration and frame definitions, the UWM event page, and the public final-submission landing page. The client tree was pinned to `ff7534e730f2b3db261c7f22d3d9b56a40c00eed`.

**Limits:** this is not a claim that every unrelated MAIC research article, every external tutorial/video, a private API or private Slack announcement was read. The final form requires sign-in before its editable fields appear; no form was filled or submitted. Numerical judging weights remain unpublished in the discovered current-event guides. Raw source receipts, content hashes and access gaps are retained privately in `/private/tmp/team-v-maic-audit-20261025/`.

## Published event requirements

The [overview](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026) requires a real-equipment demonstration, a specific problem, a business case applicable to other manufacturers, baseline comparisons and an honest account of failures. Final materials are an approximately five-minute video, one-page summary, repository link and credits/references, due **1 November, 11:59 PM Central**. The public [submission form](https://dashboard.all-ai-network.org/s/V28NHK) confirms that one teammate can submit and teammates can update before that deadline. The MAIC finals listing says **5 November, 5–8 PM, Diercks Hall**. Detailed category weights and award categories are pending.

Do not import a rubric from MAIC's earlier Persistent, Xorbix or other competitions. A platform rollout score and the final hackathon judging criteria are different things.

## What Dataset 3 actually asks

The [Dataset 3 guide](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting) addresses fixed-motion robots that fail when object positions, clutter or grasps change. It calls for camera-informed reliable grasping/sorting with a digital twin, starting with repeatability and then testing one variation at a time. Suggested measurements include pick success, placement accuracy, cycle time, recovery and unfamiliar objects. A run/settings/model log is expected; a single good clip does not establish reliability. Six cubes, sixty images and a prescribed stack are not specified. Assembly is an elective task direction, supported by the current platform's allowance for custom manipulation tasks.

## Requirements-to-evidence map

| Item | Authority | Evidence we need | Current implication |
|---|---|---|---|
| Real equipment operation | Event requirement | Approved physical-cell rollout recordings and logs | Simulation and mock readiness alone do not satisfy it. |
| Useful problem | Event requirement | One named manufacturing process and customer pain | Explain variable-batch enclosure assembly and changeover effort. |
| Baseline improvement | Event requirement | Same task/conditions, versioned baseline and adaptive results | Count every attempt and intervention. |
| Transfer beyond CSI | Event requirement | Adoption requirements, setup effort, supported variation and costs | A second recipe and a documented onboarding process are stronger evidence than a scalability claim. |
| Reliability under variation | Dataset 3 direction | Declared starting layouts and failure cases | New object locations must affect targets before pickup. |
| Camera/twin use | Dataset 3 direction | Actual cell views, correct coordinates, reproducible generation | Extra simulation cameras cannot stand in for physical inputs. |
| Custom objects/fingers | Reviewed option | Declared hardware and organizer review | Printed parts are not automatically approved for a real run. |
| Final package | Event requirement | Working links, video, summary, repository, credits | Reserve time for packaging and upload verification. |

## The current physical platform

The [participant dashboard](https://hackathon.sidclab.org/) evaluates a policy that runs on our machine and exchanges observations/actions with the cell. It reports whole-rollout success/failure, interventions, faults and successful completion time. The operator sets up declared scenes. Slots are 30 minutes. Current cameras are table, side and wrist; RGB is resized as a whole frame. The gripper command is binary open/close. The FAQ describes the current SMC MHZL2-16D2 with custom 76 × 12 × 6 mm fingers and 6 mm travel per jaw. It warns that real target following lags the twin by about 0.8 seconds. These are platform statements, not guarantees that our current policy is compatible.

The [official client](https://github.com/SIDCLab/PhysicalAI-Hackathon-2026) supplies `policy.py`, a format/connection mock, submission configuration, calibration and recorded rollout access. `model_type` is descriptive and does not affect scoring. The [submission template](https://github.com/SIDCLab/PhysicalAI-Hackathon-2026/blob/main/submission/submission.yaml) defaults to ten rollouts and separately declares whether objects and layouts vary. Non-default scenes need placement instructions. We should not edit `cell_client.py`, mistake the mock for a physical success test, or commit its private request/token files.

## Conflicts and gaps requiring resolution

| Issue | What conflicts | Required resolution |
|---|---|---|
| Gripper identity | MAIC Dataset 3 still lists SCHUNK; current cell FAQ says SMC. | Use the actual current cell specification and mentor confirmation. Do not design around the stale name. |
| Tool reference point | Live FAQ says TCP is 10 mm past fingertips; pinned client `FRAMES.md` and policy comments call it the fingertip. | Ask the cell mentor which current UTOOL/UF0 and offset are active; verify before physical targets. Never guess the 10 mm correction. |
| Finals venue | MAIC says MSOE Diercks Hall; general UWM promotion says CSI. | Confirm current event logistics with organizers. The November 1 submission deadline is separately confirmed by the form. |
| Training data permission | Overview directs synthetic training; general collection guide also discusses combined-data experiments. | Keep supplied real evaluation data out of training unless organizers explicitly clarify that use. |
| Compute allocation | Public guide still says allocation pending; team has an email assigning $500. | Treat the team's allocation as context, and verify remaining balance/rate in the actual account. |
| Team size | General UWM promotion says 4–5; Team V has an assigned larger roster. | Confirm roster in the event workspace rather than treating marketing text as a new eligibility ruling. |

The [frame definition file](https://github.com/SIDCLab/PhysicalAI-Hackathon-2026/blob/main/calibration/FRAMES.md) and the live FAQ must be reconciled before robot motion. Correct frame units and tool offsets are a prerequisite, not a cosmetic documentation detail.

## What the other guides mean for us

- [Collection and testing](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Data-Collection): schedule supervised access; freeze a test version, define start/stop procedures and record every attempt. Do not alter cameras, controllers, network connections or attach hardware without review. Keep raw data unchanged and datasets/checkpoints/credentials out of public Git. The arm's existing safety systems remain in charge.
- [Dataset 1](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Vial-Inspection): final vial/fixture defect inspection, with missed defects especially important. It is a different task and its defect labels are not our assembly labels. Reuse the idea of final outcome checking, without claiming our separate cell is the production-line inspection station.
- [Dataset 2](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Fluid-Mixing): relates fill/color anomalies to recipe history and carryover. Its twin lacks tanks and approximates liquid color. Its fluid limits and sequencing belong to that line. We should not add fluid-process control to this week's assembly scope.
- [Compute](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute): Brev, Spark/GB10 and APIs are options. Compare total workload cost, preserve outputs and stop paid instances when unused. The linked RTX recommendations are not evidence of a measured speedup for our particular code. No cloud machine was started by this audit.
- [Coding agents](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Coding-Agents): agents help create and document work; generated code still needs review and reproducibility. Agent subscriptions are support resources, not evidence that a robot task works.
- [Resources](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Resources): explicitly a toolbox rather than a required stack. Isaac Sim/Replicator fit synthetic vision data; Isaac Lab fits policy-learning experiments; Cosmos fits optional visual reasoning; NeMo tools fit optional orchestration; TAO/Visual ChangeNet target inspection; NuRec targets reconstruction. FactoryTalk/OPC UA access requires approval. Adding every tool is not a competition requirement.

## Physical adapter checks before deployment

Use the official client contract rather than assuming that simulator commands can be sent unchanged. Confirm whole-frame resize and camera intrinsics; source-frame units and the unresolved tool offset; FANUC joint convention; action timing and measured feedback lag; the binary gripper transition after a target is reached; and which completion evidence the cameras can supply. The published observation contract does not provide a force/torque observation, so do not claim measured force-controlled insertion from it.

Only the nest is prepositioned in our proposed assembly. The robot places the independent base, already-loaded carrier and lid. This prototype does not solder the board, electrically test it, tighten screws or certify an enclosure. A stable, correctly seated mechanical stack is the first acceptance scope; securing/fastening must be specified and separately validated before calling it a finished industrial product.

## Our three immediate engineering gaps

These conclusions combine the public contract with the frozen local V63 source; they are our engineering assessment, not organizer judgments.

1. **Perception uses extra simulation views.** `AssemblyOverview` and `inspection_x/y` exist in our assembly prototype, while the real API exposes table/side/wrist. Their images may help diagnose physics, but they do not prove that the deployed estimator can operate on the real observations. Adapt and qualify perception against the actual views and calibration.
2. **Grasp commands need a physical interpretation.** The simulator accepts specific jaw-position targets; the real API accepts open/close. Verify how actual object-limited closure behaves before relying on simulation preload assumptions. Do not change the supplied gripper to hide a mapping problem.
3. **A fixed pickup cannot handle shuffled parts.** The original variation trials retained nominal pickup targets. The revised RGB pickup has not yet executed a grasp because its camera freshness check stopped it. We still need validated pre-pick pose estimation, actual held-pose handling, and completion checks.

The nominal three-part simulation completed motion in an earlier run, but its support-force audit remained inconclusive. V61 variation runs recorded pickup contact-limit stops; their native audits remained incomplete because the final failing-step PCB record was missing. The latest V63 stopped before pickup. None is a reliable physical-assembly qualification. CPU test counts and scheduler exit zero do not replace those missing results.

## Proposed architecture

**Approved order → supported recipe → actual camera observations → part identity/pose → validated robot skill → observed outcome → next step or bounded recovery.**

- **Operator layer:** choose supported product, quantity and approved assembly region. Show progress, incomplete parts and why execution paused. Keep the black/white/orange visual direction.
- **Recipe layer:** component IDs, dependency order, allowed orientation, grasp feature, placement reference, workspace limits and completion tests. Reject an impossible order or unsupported dimension change instead of trying it physically.
- **Perception:** a small synthetic-trained detector for identification where useful; calibrated image/CAD or depth geometry for pose. Existing cube weights alone do not recognize custom housing parts. Ten thousand images are not ten thousand action demonstrations.
- **Execution:** explicit, reviewed skills for approach, grasp, lift, transfer, align, place and release, connected through the official policy API. Start with the existing controller, not a new end-to-end robot-learning stack.
- **Verification:** table view for inventory and positions; wrist view for close observations where validated; side view for held/released state. A closed gripper command is not proof of a held part. Use fresh observations between dependent actions.
- **Recovery:** one declared retry or an explicit hold, depending on the failure. Missing parts, uncertain pose or blocked destinations should not produce a claimed success.
- **Evidence:** source/model/CAD/recipe versions, all attempts, success criteria, time, interventions and failure reasons.

## The demonstration to aim for

Use one approved enclosure family first. Lay its independent base, loaded carrier and lid at varied declared locations. Select its recipe and assembly region. Show the cell finding the parts, building the product and checking the result. Then change a supported order configuration without teaching another fixed motion sequence.

If the first product is qualified early enough, demonstrate one second component variant that shares the verified grasp and mating interfaces. A second recipe must have a meaningful configuration difference; changing only the display name is not product generalization.

Include a declared missing-part or failed-pick case. The system should identify the issue and recover within its budget or stop clearly. A moved-part test must use an organizer-approved setup while the arm is stopped; nobody should reach into moving machinery for the demonstration. Record failures and interventions rather than editing them out of the measured results.

## What scalability should mean in the presentation

| Claim | Demonstration needed | Limit to state |
|---|---|---|
| Location variation | Same components, several held-out layouts, fresh image-derived pickup targets | Only the tested workspace and pose range. |
| Recipe changeover | Change supported order, quantity or destination through a recipe; measure setup effort | The component dependency order is validated, not arbitrary. |
| New product variant | A second approved SKU using stable interfaces; document CAD, grasp and verification additions | New geometry still requires clearance and physical qualification. |
| Recovery | Declared empty grasp, missing component or occupied destination tests | Bounded cases/retries, not universal fault recovery. |
| Deployment | Same policy contract through the actual cell interface | No unsupported cameras or proprietary controller changes. |

The business argument is reduced setup effort and fewer manual rescues during small-batch changeovers. Measure those quantities. An estimated return-on-investment calculation must list cycle count, labor rate, intervention time and deployment cost assumptions separately from observed results. Do not invent production savings from a handful of lab trials.

## Final-week execution and decision gates

| Target date | Priority | Required evidence / decision |
|---|---|---|
| 25 Oct | Lock scope and actual interface | One product family; next supervised test slot; current TCP/camera/gripper contract; preserve pending size approval. |
| 26 Oct | One real-input skill path | Actual-view observation and correctly framed target; open/close grasp; pick/hold/place/release checks in simulation; official mock connectivity checked. |
| 27 Oct | Early supervised physical pilot | Record the actual approved grasp/task, intervention and calibration issues. Do not wait until the final day to discover interface incompatibility. |
| 28 Oct | Repeated baseline/adaptive trials | Prefer equal declared layout sets; use ten attempts per compared version if time is approved, otherwise disclose unequal sample counts. |
| 29 Oct | One changeover and fault behavior | Supported recipe/part variant only after core success; missing-part/failed-pick handling with explicit acceptance. |
| 30 Oct | Freeze demonstrated release | No new tools or products; reproduce results and capture clear evidence. |
| 31 Oct | Finish submission package | Five-minute video, one-page summary, clean reproducible repository and all credits. Leave upload margin. |
| 1 Nov | Verify final entry | User-authorized submission, working links and saved receipt before 11:59 PM CT. |

These are proposed work targets, not reserved robot slots. The critical input is the next approved physical-cell slot and the exact task/hardware the operator will accept.

**Go/no-go rule:** if the next isolated grasp or interface check fails, diagnose that evidence; do not spend the remaining week rerunning whole assemblies. Freeze new geometry/tool additions until the fault is understood. If full assembly cannot be qualified by the physical pilot, email the user with a concrete reduced demonstration and its limitations for a decision; do not silently relabel kitting as finished assembly.

## Parallel work, without a separate integration lead

Suggested allocation, subject to team availability:

- **Aditya + Khushal:** actual-camera data, labels, part identification/pose and held-out layout definitions. Publish a small input/output example immediately.
- **Koushik + Levi + Shritej:** current-finger grasp/release, mechanical interfaces, policy adapter and repeatable robot skills. Use explicitly labelled test targets while perception is being qualified.
- **Thanvi + Yuktha:** recipe/operator workflow, trial log, baseline comparisons, evidence, business case and final materials. Keep mock data visibly distinct from actual run data.

Agree now on a shared part record: ID, pose/frame, confidence/uncertainty, observation identity and recipe version. Agree on a skill result: completed, retryable failure or hold, with evidence/reason. This lets the groups work concurrently without pretending placeholder outputs are a validated system.

## Immediate questions to resolve

1. When is the next approved real-arm slot, and has the task/custom printed hardware been accepted?
2. Which exact TCP/tool offset and cell calibration version should the physical adapter use?
3. Did organizers publish updated weights or logistics in private announcements after the public guides?
4. Which size proposal, if any, is approved? The other chat owns that approval email; no duplicate size change is authorized here.

Root's email-update workflow is enabled separately. Meaningful decisions, changes, results and questions go to `miriyalaa@msoe.edu`; only verified replies to the relevant question guide dependent work. Silence is not approval. Monitoring requires Codex to remain running and connected.
