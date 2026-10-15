# Physical AI for the Factory Floor requirements

Source review date: October 13, 2026. Sources are the eight MAIC pages listed in [Resource links](RESOURCE-LINKS.md). Published requirements and restrictions below are distinguished from suggested approaches. Pricing and access notices describe the website at review time, not a guarantee of current account access.

## Purpose and manufacturing context

Rockwell Automation, NVIDIA, MSOE AI Club and UWM Connected Systems Institute run a four-week Physical AI hackathon. Teams should improve a specific real process, measure the improvement and demonstrate a working solution on physical equipment.

CSI's production line is a high-mix, low-volume vial filling, mixing, inspection and placement testbed using Rockwell equipment. The Dataset 3 FANUC arm is a **separate cell, not connected to that production line**. Do not describe our FANUC demo as an integrated vial-line deployment.

Sponsors emphasize faster changeovers, reliability, less setup work and operator trust. Explain relevance to other manufacturers, including adoption hardware and setup costs. The overview cites approximately 3% AI inspection adoption from the 2020 Annual Business Survey and $32–58 billion annual defective-product costs from a NIST survey. These are website-reported background claims, not verified savings for our project.

Focusing on one strong idea or dataset is recommended, not mandatory. The resource list is optional guidance, not a required technology stack.

## Deadlines and deliverables

| Date and time | Requirement or event |
| --- | --- |
| October 8, 4–6 PM | Kickoff at Rockwell HQ, 1201 S 2nd St, Milwaukee; transportation provided from both campuses. |
| October 14, 11:59 PM Central | Submit the team plan through the designated form. |
| November 1, 11:59 PM Central | Submit final video, summary, repository link and references/credits. |
| November 5, 5–8 PM | Finals, live presentations and prizes at MSOE Diercks Hall; further details pending. |

The team plan is a short resource request, not a graded paper or binding contract. Any teammate can submit it or update the same shared submission before the deadline. Plans can change after inspecting data.

The plan asks for:

1. Team name and members.
2. Selected dataset or combination.
3. Specific problem description.
4. Additional data needed and why it is not already supplied.
5. Compute: workload, hardware, hours, requested Brev credits and timing; Spark, MSOE GB10/Rosie, CSI workstation or build APIs as appropriate.
6. Agent tools/credits and their justification.
7. Mentor support needed.
8. Specific organizer questions.

Final submission includes an approximately five-minute video, one-page project summary, team repository link, and references/credits for data, models, code and tools. The site does not specify a detailed file-format convention or a weighted score sheet on these pages.

## Published judging expectations

**No official numerical rubric or category weights are available in the reviewed pages.** The site says detailed criteria, weights and award categories are still being finalized. Judges represent Rockwell, NVIDIA, MSOE and UWM.

| Published expectation | Evidence we should prepare |
| --- | --- |
| Solution operates on real CSI line or FANUC hardware | Supervised physical demonstration and trial records. Simulation is supporting evidence. |
| Specific problem and why it matters | Defined failure mode, current workflow and manufacturing impact. |
| Business case for other manufacturers | Baseline scrap/rework/downtime/labor burden, measured improvement, adoption hardware, setup/changeover effort and operator trust. Distinguish measured values from estimates. |
| Measured results against a baseline | Repeatable test protocol, denominators, success/failure metrics, latency and comparison. |
| Clear limits and remaining failures | Failure examples, conditions outside scope and intervention/recovery records. |

The evidence column is our internal checklist, not an official scoring scheme. A video alone does not establish reliability.

Prize pool: $5,000 cash, RTX 5080, RTX 5070 and sponsor swag. Cash split and additional award categories remain pending.

## Account and repository setup

Create an ALL AI Network account using the university email where possible and the email associated with AI Club points. Existing MSOE/UWM teams are pre-created and university-email registration should assign students automatically; other students may create/join teams.

Connect GitHub from the profile, then accept the team repository invitation. Official team repositories are visible to sponsors. Confirm everyone who needs the supplied data can open it. Join Slack announcements and the selected dataset channel. Review all four guides before requesting resources.

## Dataset 3 problem statement and constraints

Industrial arms programmed for fixed motions cannot adapt well when object positions change, clutter appears or a grasp is imperfect. Use multiple camera views and a digital twin to make a standalone FANUC arm grasp and sort reliably. Once basic operation works, teams may explore harder objects, sorting rules or customized fingers.

Website-provided equipment:

- FANUC ER-4iA arm.
- SCHUNK pneumatic two-finger gripper.
- Two overhead cameras and one wrist camera.
- Arm/work-area digital twin.
- Current finger CAD and access to design/print replacement fingers.

Define success before increasing difficulty: picks per attempt, placement accuracy, cycle time, recovery after failed grasps and performance on unseen objects are suggested metrics. Log settings, randomization, model versions and physical outcomes.

Recommended sequence: existing objects/fingers in simulation → repeated varied positions → one added variation at a time (orientation, object, clutter, lighting) → small physical comparison → then policy, perception or finger changes.

Physical-arm commands and controller changes require plant manager and assigned mentor involvement. Printed fingers require clearance and safe-operation review before attachment. The page still says the twin will release within 24 hours and calibration/CAD/access instructions will be posted later; this conflicts with our already supplied and running v05 package and should be treated as a stale notice.

## Dataset 1 problem statement and data

Inspect visible cap/body defects and fixture occupancy after each placement at the final station. Existing side inspection can miss cap defects; caps can move later, and robot picks/placements can fail. **Missed defects matter more than false alarms in judging for this dataset.**

Two fixtures: G for good and S for scrap, 30 numbered positions each. Fill order is the rightmost column top-to-bottom, then move left. Interior gaps may indicate missed placement; trailing empty positions may be normal until the order is complete. Use placement count/order-complete signal when available.

Labels apply to the target numbered aperture, not the entire crop:

| Label | Meaning |
| --- | --- |
| pass | Visible cap and body acceptable; does not establish hidden properties. |
| missing_cap | Vial present with open mouth; transparent body is not an empty aperture. |
| crooked_cap | Cap tilted or unseated; leaning vial alone is insufficient. |
| damaged_vial | Visible target-body damage; neighboring transparent bodies may require scene context. |
| no_vial | Empty target aperture; error status depends on fill progress. |

Provided: full real camera scenes, labeled/unlabeled position crops, fixture coordinates/IDs, browser labeling tool and digital twin. Labeling workflow: select scene/fixture/position, inspect crop in context, edit class and observation, mark reviewed, export review JSON.

Suggested methods: twin-based synthetic training; lightweight TAO Visual ChangeNet; Cosmos in-context inspection; fill-pattern logic; DEFT evaluate/diagnose/augment/retrain/gate loop.

Visual ChangeNet compares an inspection crop with a golden good reference. Match pose, position, crop, scale and lighting; scores are distances, not probabilities. Higher distance favors NO_PASS. Report false-alarm rate at a threshold catching every known defect, inspect correct/incorrect pairs, change one variable at a time. TAO setup/backbones may need NGC/Hugging Face accounts.

Keep a test set out of prompt examples, augmentation decisions and training. Report per-class missed defects and false alarms, and test complete fixture scenes, not only crops. A held-out real test release is a possibility, not guaranteed.

## Dataset 2 problem statement and data

Detect color/fill problems and explain likely causes using recipe order, tank state and carryover. Merely flagging an unexpected appearance is not the full objective. Shared upstream mixing and one nozzle leave residual liquid that affects the first vial of the next recipe. The last vial of a batch is suggested as a clean color reference; second/third vials flush residue.

Four tanks left-to-right: T-100, T-200, T-300, T-400. T-300 was out of service during collection. Labeled data uses T-100 clear, T-200 orange, T-400 blue, but colors can change; recipes identify tank numbers, not colors. Up to three recipes queued/stored at once, 10 recipes covered by swapping the queue. Dispense order T-200 → T-400 → T-100, skipping unused tanks.

Operating restrictions:

- Total vial fill no more than 80 mL.
- A component can be configured 0–80 mL, but a selected component should practically be at least 10 mL.
- Do not use more than 50 mL from T-200; it can fault and require reset.
- Do not use T-300; configuration issue can require reset.
- Do not change recipes without plant-manager guidance.

Provided: three synchronized fixed-camera streams (vial_fill, tank_level, cap_seal), 640×480, video 30 fps and stills around every two seconds; five synchronized video IDs; labels for 30 vials/10 recipes (three each), recipe volumes/order, fill start/end and per-frame fill indicators. Other data and tank/cap views are unlabeled. Twin models fill and cap/seal stations with matched cameras; tanks are not modeled and fluid color is approximate. Barcodes may be absent or inconsistently oriented; production does not rely on them.

Suggested methods: simple color/fill baseline; synchronize views and audit offsets/drift/missing frames; prior-recipe context; Cosmos reasoning; synthetic variation; more labeled test examples. Existing Teledyne color/weight check recognizes colors but does not reliably confirm intended recipe or predict carryover.

Split by production run/time window to avoid neighboring-frame leakage. Suggested leave-one-recipe-out evaluation; evaluate causal explanations separately from detection. Report missed defects and false alarms separately.

## Data use rules and unresolved conflict

The overview and Dataset 1/2 pages explicitly direct teams to **look at supplied real line data but avoid training on it; generate training data with the twins**. This supports commissioning a new high-mix product before real examples exist.

The collection guide also suggests comparing real-only, synthetic-only and combined training. That conflicts with the more specific supplied-real-data restriction. Do not interpret the generic comparison suggestion as permission. Ask about the exact collected dataset and proposed training/calibration/testing use before real-data training. Dataset 3 real-data permissions also need a specific answer; the organizer email did not grant blanket training permission.

## CSI dos and don’ts

Allowed with the stated scheduling/review conditions:

- Visit to observe the line and arms.
- Record using our equipment after the setup is reviewed.
- Request specific CSI images/videos/operating examples.
- Generate samples with supplied twins.
- Request supervised physical testing of a finished system.

Prohibited without the required approval/review:

- Moving, re-aiming, disconnecting or reconfiguring existing cameras.
- Changing PLCs, robot controllers, FactoryTalk, safety systems, recipes or production programs.
- Connecting hardware to the industrial network.
- Placing objects that could collide with robots or interfere with production.
- Attaching printed fingers or other components before review.
- Capturing unnecessary people or sensitive information.

Observe first; touch equipment only with explicit plant-manager approval. Access must not disrupt production, controls or safety. Testing happens only after CSI confirms a slot and assigns an on-site supervisor.

## Visit and data request template

Post in #data-and-testbed-requests and tag Shamar Webster; ask early. Collection is not guaranteed. Email fallback: webste63@uwm.edu.

Include team/dataset; question to answer; exact recording; positive/counterexamples; sensor/framing/resolution/FPS; operating conditions/sequence; sample number/duration; labels/metadata; needed date; acceptance criteria.

Before visiting, specify target, variations, constants, labeler/timing, frame-to-run/trial identifiers, held-out runs and storage. Run a small pilot and check timestamps/files/labels while equipment is still available.

Before physical testing: freeze model/code; list inputs/outputs; prepare start/stop; define metrics; test failure handling offline; make the test work without plant-control changes. Request duration and attendees. Log **every attempt**, including versions, conditions, predictions, expected outcomes, latency and operator intervention.

Keep raw files unchanged. Separate cleaned data, labels and generated files; record origin, generation settings, collectors and sharing terms. Keep large datasets, checkpoints, account credentials and API keys out of Git. Include a small sample and access README for authorized teammates.

## Compute access and cost guidance

| Workload | Website hardware suggestion | Approximate planning rate |
| --- | --- | --- |
| Cosmos Edge inference | T4 | $0.50/hour |
| AOI training, Cosmos Nano inference, NuRec | RTX PRO 6000 preferred; L40S fallback | L40S $2/hour; RTX PRO 6000 roughly twice comparable rate |
| Isaac synthetic data / Isaac Lab | RTX PRO 6000 preferred; L40S fallback | L40S $3/hour; RTX PRO 6000 roughly twice comparable rate |
| Cosmos Edge fine-tuning | One H100 | $5/hour |
| Cosmos Nano LoRA | Four A100 80 GB | $8/hour |
| Cosmos video launchable | C2.5T | $17/hour |

These are estimates from the site, not price quotes. Check console before launch and measure total runtime/credits rather than assuming the faster GPU wins. RTX PRO 4500 is suggested if 6000 unavailable. AWS preferred, especially L40S, due to newer CUDA environments.

Brev captain receives a team organization invitation. Initial credits were not fixed on the site. Choose environments with Start/Stop; running machines spend credits while idle. Provisioning may take ten minutes; inspect Compute if launch screen stalls. Use exact instance names for `brev shell` and `brev copy`. Headless Isaac saves images/video for local viewing. Use documented Secure Links for sharing.

Stop after work: compute billing stops after shutdown, storage persists and still costs money. Delete only after backing up needed files; deletion removes disk permanently. Do not leave instances running overnight unless an active job and budget justify it. Keep keys out of code/team chat. Do not use the $17/hour environment for work available on a free API or cheaper machine.

Free build.nvidia.com endpoints are a first option for in-context inspection/video reasoning; create an API key and check model limits. Ten UWM DGX Spark and ten MSOE Dell GB10 systems are listed; MSOE access via Rosie, connection/scheduling notices pending. CSI has an **in-person-only** Windows 11 RTX PRO 5000 Blackwell workstation with Omniverse, Emulate3D and Nucleus; reservation/preloaded-file details pending. The organizer email places that workstation with Datasets 1–2.

Compute requests should state workload, hardware, hours, credits, timing and the benchmark that resolves uncertainty.

## Recommended tools and meanings

| Tool | Website role | Team V interpretation |
| --- | --- | --- |
| Isaac Sim / Omniverse / OpenUSD | Digital twins, labeled images/video/actions; robot simulation | Main simulation and synthetic-data foundation already working. |
| Replicator | Vary camera, lights, materials, objects and export labels | Relevant for controlled dataset generation. |
| Isaac Lab | Robot policy training | Optional later; not required for a deterministic pick/recovery controller. |
| Brev | GPU hosting and prepared launchables | Compute location, not an image-generation algorithm. |
| Cosmos | In-context image/video reasoning and generative workflows | Optional experiments with clear task and QA; not physical-arm command authority. |
| TAO / Visual ChangeNet | Dedicated inspection model training | Mainly Dataset 1 guidance, not automatically our cube detector. |
| DEFT AOI | Evaluate, diagnose, augment, retrain and gate | Useful experiment loop regardless of model. |
| FLUX / Physical AI Data Factory | Generative image augmentation | Label geometry/visibility must be rechecked after edits. |
| NuRec | Reconstruct scenes from images/lidar | Optional real-to-sim work. |
| NeMoClaw / NeMo Agent Toolkit | Agent orchestration | Optional; use only for an identified need. |
| FOX / FactoryTalk Optix OPC UA | Factory architecture and approved data integration | FOX is context, not complete testbed code. Plant access requires approval. |
| Kumo-TS | Time-series forecasting | Only relevant if appropriate sensor/time-series inputs exist. |
| FANUC ROS 2 driver | Robot interface reference | Confirm actual controller compatibility with CSI before selecting it. |

The MAIC table describes Edge for a single image and Nano for multiple images. It lists Cosmos T2I/I2V/T2V/I2T workflows and explicitly says **not I2I**; I2I is listed separately for tools such as FLUX. It lists video limits of Edge 480p/150 frames and Nano 720p/400 frames, and Spark training as Edge SFT/Nano LoRA. These are archived MAIC claims: verify exact model/endpoint capabilities in linked NVIDIA documentation when implementing. Do not conflate a reasoner with a generator.

In-context learning supplies examples in the prompt instead of first training a dedicated model. It is a quick baseline for rare classes, but latency/cost may favor a small trained model for edge throughput.

Before adding a tool, specify the failure addressed, inputs/outputs, compute/access and comparison with the baseline. Otherwise keep the pipeline simple.

## Coding agents

MAIC recommends starting free with Gemini in Antigravity CLI (`agy`), says Gemini 3.7 Flash was tested on the problems, and says older Gemini CLI personal-account support ended June 18, 2026. Installation, remote login and API-key instructions are linked in the resource index. These third-party product claims have not been independently verified in this reference.

The site lists a US student AI Pro offer (18+, SheerID, personal Google account and school verification), one year free, payment method required, $19.99/month renewal, redemption by December 31, 2026. Confirm current Google terms before enrollment.

If requesting Claude/Codex, state tool, $100/$20 plan, number, teammates and concrete capacity/use reason beyond free tools. The site expects possible one/two $100 and a few $20 seats per team; **not a guaranteed allocation**. Review agent-written code and save reproducible runs. Remote agents may be installed on GPU machines; protect credentials.

## Help channels and contacts

| Need | Channel/contact |
| --- | --- |
| Updates | #hackathon-announcements |
| General event/account/team/website | #hackathon-general; Brett Storoe, storoeb@msoe.edu |
| Dataset questions | #dataset-1-vial-inspection, #dataset-2-fluid-mixing, #dataset-3-grasping-sorting |
| Line/data context | Brett Storoe, Dr. Derek Riley, Tanner Cellio; storoeb@msoe.edu, driley@nvidia.com |
| Visit/data/physical test | #data-and-testbed-requests; Shamar Webster, webste63@uwm.edu |
| Compute | #compute-questions; Dan Schneider, danschneider@nvidia.com |

For a blocking issue, say it is blocking in the first line; include attempts, error and needed resolution date. The overview does not assign one exact mentor to each team. The supplied review email identifies **Dr. Zeng (zhenzeng@uwm.edu)** as our primary FANUC/connected-GB10 mentor and Dr. Riley for the twin.

## Items to confirm rather than assume

1. Official weighted rubric, award categories and finals logistics when published.
2. Actual physical gripper: website SCHUNK versus starter SMC model, mounting, gap/stroke and approved finger changes.
3. Exact camera/calibration files and physical interface/version compatibility from supplied assets and CSI.
4. Real-data collection/calibration/test/training permission for our specific use, given conflicting general guidance.
5. Brev credit activation, GB10 connection/scheduling and agent allocations; pending notices do not establish access.
6. CSI slot, attendees, supervisor and permitted test protocol.

Team V's next decisions should prioritize the dynamic manufacturing scenario and camera/recovery architecture before indiscriminately scaling image count. See [Organizer feedback and implications](TEAM-V-IMPLICATIONS.md).
