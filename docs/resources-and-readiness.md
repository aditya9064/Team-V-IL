> Historical planning/research document. For current scope, resource status, verified results and next steps, read [START-HERE.md](START-HERE.md). Statements below describe an earlier project stage.

# Resources and readiness

Checked October 8, 2026. Current working direction: Dataset 3 as the core, with Dataset 1 as a conditional quality-inspection extension. This is not a confirmed physical integration. The robot is separate from the vial line, and compatible objects, gripper geometry, camera views, and supervised operation need organizer confirmation.

## Readiness verdict

We have enough to begin data engineering, inspection research, evaluation design, and integration scaffolding. We do not yet have all the inputs or confirmed access required to complete a real FANUC demonstration. Neither models nor physical capabilities have been validated by Team V. The user confirms that no competition resources have been activated yet, including the event/team access, Slack, compute, and CSI bookings.

## Activation checklist

1. All Team V participants are from MSOE. Each teammate registers with their MSOE email through [event signup](https://dashboard.all-ai-network.org/j/L36HEH), confirms assignment to the organizer-created team, connects their GitHub profile, and accepts the team-repository invitation. Verify the user-provided repository is accepted or link/migrate work to the organizer-managed team repository as instructed.
2. Join the hackathon Slack using the [overview's invitation](https://msoe-maic.com/library/?nav=Competitions&article=Rockwell-NVIDIA-Fall-2026). Open announcements, the robotics channel, data/testbed requests, and compute questions. Confirm which listed channels already exist.
3. Name a captain, technical integrator, and testbed coordinator. Identify robotics/ROS, CAD, simulation, and computer-vision skills and weekly availability.
4. Send the reviewed requests in [organizer-requests.md](organizer-requests.md): prioritize FANUC files, approved target objects, controller/camera interfaces, and an early supervised visit.
5. Captain obtains the Brev team-organization invitation and confirms credit allocation. Prioritize MSOE Dell GB10 access through Rosie as the campus compute route, subject to readiness and scheduling. Confirm CSI reservations separately: the physical FANUC setup is at UWM CSI. Establish the team spending limit before starting billed instances.
6. Submit the short resource plan by October 14, 11:59 PM CT, including named users for any requested agent seats. Personal account sign-ins, student verification, and acceptance of account terms are completed by the relevant teammates.
7. Once a permitted GPU workspace is accessible, benchmark a small scene before batch generation or training. Request the initial $300-equivalent planning tranche only if the benchmark and organizer advice support it.

## Resource inventory

| Resource | Organizer provision | Team V's verified status | Intended use |
| --- | --- | --- | --- |
| Vial dataset | Three cameras, five runs, recipe/timing labels | Downloaded and structurally audited: 15 videos, 4,647 images, 30 recipe-labeled vials | Understand real imagery; evaluation and labeling, subject to track rules |
| Vial-line twin | Tray inspection and fill stations, camera JSONs, previews, render script | Downloaded and extracted; static geometry and fill animation, no physics | Synthetic inspection images; camera/material variation after generator development |
| Dataset 1-specific crops, defect labels, position coordinates, labeling tool | Described on Dataset 1 page | Their complete package and access path have not been verified; raw camera footage is not an equivalent replacement | Inspection baseline and independent real evaluation |
| FANUC arm | ER-4iA industrial arm at CSI | Offered for supervised testing; no Team V session or interface verified | Physical grasping, placement verification, and recovery |
| Gripper and cameras | SCHUNK pneumatic two-finger gripper, two overhead cameras, wrist camera | Hardware described; camera streams, gripper controls, and calibration not obtained | Localization, grasp selection, action verification |
| Robot twin and technical files | Arm/work-area twin, camera samples, calibration, finger CAD | Robotics guide says release/access pending; absent from supplied root folder listing | Robot simulation, coordinate mapping, object/gripper compatibility |
| Custom fingers | Access to design/print replacement fingers | Review and print logistics not confirmed | Only if current fingers cannot safely handle approved targets |
| Brev | Team organization, cloud GPU credits, launchables | Captain invitation and allocation unknown; initial credit amount not set in guide | First GPU route: Isaac Sim/Isaac Lab on L40S; training as needed |
| UWM systems | Ten shared NVIDIA DGX Spark systems | Connection/scheduling instructions pending; Team V allocation unknown | Local inference and suitable simulation/training workloads |
| MSOE systems | Ten shared Dell GB10 systems accessed through Rosie | Connection/scheduling instructions pending; Team V access unknown | Alternative to cloud compute after availability and workload checks |
| CSI workstation | Windows 11, RTX PRO 5000 Blackwell, Omniverse, Emulate3D, Nucleus | In-person only; reservation details pending | Rockwell-twin work, synthetic data, optional reconstruction |
| NVIDIA model APIs | Free build.nvidia.com endpoints for Cosmos generation/reasoning | No team API credentials configured | Optional inspection/explanation baseline; use only permitted data and account limits |
| Coding agents | Free Antigravity/Gemini route; possible sponsored Claude/Codex seats | Existing Team V subscriptions unknown; paid sponsor seats are conditional requests | Coding, debugging, evaluation, reproducibility; review generated work |
| Mentors | Rockwell/NVIDIA support via relevant Slack channels | No exact mentor assigned; contact process available | Resolve robot controls, simulation, inspection, and deployment questions |
| CSI visits and extra data | Scheduled observations, approved collection, supervised tests | No booking established; extra collection is not guaranteed | Pilot early and reserve a frozen final hardware evaluation |
| Team workspace/repository | ALL AI Network team management, GitHub invitations, submissions | User repository cloned; organizer-repository linkage unknown | Team collaboration and sponsor-visible submission |

## What to use first

1. Keep the existing local data audit and raw files as the starting reference.
2. Obtain the FANUC package, camera/robot calibration, and supported command interface. Confirm controller/software compatibility with a mentor before choosing a ROS deployment.
3. Use the supplied Isaac Sim launchable on a start/stop-capable L40S environment once team access and budget are known. First run: render and simulate existing-object pick-and-place. Benchmark time and cost before batch generation.
4. Build camera-based grasp selection, action verification, and bounded recovery before adding learned robot policies. Isaac Lab is useful if a defined failure calls for policy training and the twin supports it; it is not automatically required.
5. Use the vial twin for synthetic inspection experiments. TAO/Visual ChangeNet and a simpler vision baseline are candidates; select based on a concrete comparison, not the size of the tool stack. Confirm accounts/model prerequisites if TAO is selected.
6. Validate the inspection layer on the robot's actual approved objects and viewpoints. The overhead tray model cannot be assumed to transfer to wrist or robot-cell cameras.
7. Test free reasoner APIs only as a measured auxiliary component. Keep grasp execution and verified defect decisions grounded in the tested pipeline.
8. Use Spark/GB10 or the CSI workstation where access and runtime compatibility make them useful. NuRec, generative augmentation, NeMoClaw, and video generation remain optional until a baseline failure justifies them.

The user's instruction to use available resources authorizes preparation and appropriate use. It does not establish an unknown GPU spending limit, grant plant-control access, or confirm third-party accounts. No paid instances, subscriptions, messages, or hardware actions have been initiated.

## Critical dependencies for 3 + 1

- FANUC twin, real camera examples, calibration, finger CAD, existing object dimensions, supported robot/gripper interface, and compatible software versions.
- Mentor-approved target objects and sorting layout. Confirm whether inspection vials may be handled, whether representative inspection objects are allowed, and whether gripper changes are needed. If representative objects are used, describe that demonstration accurately rather than claiming a connected vial-line integration.
- Complete Dataset 1 labels/tool/position-map package, plus a reserved real test set. Confirm allowed calibration, prompt examples, threshold tuning, and use of real data.
- Approved camera input and a supervised physical pilot, followed by a final test slot before November 1.
- Team account access, GPU allocation, and one owner for each implementation stream.

If vial handling is not feasible, keep Dataset 3's adaptive grasping and recovery as the core and evaluate inspection separately. Do not force a weak cross-dataset link into the physical demonstration.

## Planning compute request

Provisional first tranche, not an allocation or spending authorization:

- 60 L40S hours for rendering/simulation at the guide's approximately $3/hour: $180.
- 20 L40S hours for inspection training/inference at approximately $2/hour: $40.
- Approximately $80 planning allowance for reruns and storage: request about $300 equivalent in credits, with units and actual console rates confirmed by organizers.

Run a small benchmark and revise the estimate before committing the full budget. Use one shared team environment where practical and stop it between sessions. Do not begin costly video generation or large-model fine-tuning as the default.

## Eight-person ownership

| Pair | Initial responsibility |
| --- | --- |
| A | Robot twin, scene loading, synthetic data, reproducible simulation trials |
| B | Perception, quality-inspection baseline, uncertainty, evaluation splits |
| C | Grasp planning, execution adapter, verification, bounded recovery |
| D | Replay/live integration, hardware coordination, trial logs, business case and demo |

Assign a technical integrator and testbed coordinator across these pairs. Confirm actual teammate skills and weekly availability before assigning people. One integrated replay should run before the hardware visit.

## Contacts and request process

- Shamar Webster, #data-and-testbed-requests: visits, extra collection, physical tests; email fallback webste63@uwm.edu.
- Brett Storoe, Dr. Derek Riley, Tanner Cellio, relevant dataset channel: equipment and dataset questions; listed email fallbacks storoeb@msoe.edu and driley@nvidia.com.
- Dan Schneider, #compute-questions: Brev/GPU access; email fallback danschneider@nvidia.com.
- Brett Storoe, #hackathon-general: team, website, GitHub invitation, or Slack issues.

Ready-to-send drafts are in [organizer-requests.md](organizer-requests.md). They have not been sent.

Team plan deadline: October 14, 11:59 PM CT. Final submission: November 1, 11:59 PM CT. Finals: November 5.

## Sources

- [Competition overview and team-plan process](https://msoe-maic.com/library/?nav=Competitions&article=Rockwell-NVIDIA-Fall-2026)
- [Robotics](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Grasping-Sorting)
- [Inspection](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Vial-Inspection)
- [Compute](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Compute)
- [Coding support](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Coding-Agents)
- [Data collection and testing](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Data-Collection)
- [Tools and tutorials](https://msoe-maic.com/library?article=Rockwell-NVIDIA-Fall-2026-Resources)
- [Shared Drive folder](https://drive.google.com/drive/folders/1GWuHV2WUNLztq_Fga6xBH7qFzhxDaac8)

Published provisions are distinct from Team V's activated access and successful use.
