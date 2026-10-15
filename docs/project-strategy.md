# Project strategy: adaptive robotic inspection and sorting

Working direction as of October 8, 2026: Dataset 3 is the core, with Dataset 1 as a conditional inspection extension. The user prefers an ambitious robotics demonstration and is exploring a two-dataset solution. All participants are from MSOE; the team has approximately eight members. The full team has not yet confirmed target objects, skills, time commitments, or hardware feasibility. The user confirms that no competition resources have been activated yet.

## Intended contribution

A robot uses vision to choose a grasp, verifies what happened, and recovers after a failed attempt. If the physical setup supports approved inspection objects, add synthetic-trained quality inspection that determines the sorting destination. The strongest demonstration completes this entire task on the real FANUC arm and shows measured improvement over an approved baseline.

The FANUC workcell is separate from the vial line. The inspection extension requires compatible objects, gripper geometry, camera views, calibration, and mentor approval. Do not assume the arm may handle production vials or that a tray-camera classifier transfers to wrist-camera images. If the inspection extension is infeasible, retain a strong Dataset 3 core rather than claiming a connected manufacturing line.

## Milestones

| Deadline | Evidence |
| --- | --- |
| October 9–10 | Team registration/Slack, named integrator and test coordinator, request FANUC assets and pilot access |
| October 11–14 | Existing-object simulated pick-and-place, known control/calibration interface, approved object plan, compute benchmark; submit resource plan by October 14, 11:59 PM CT |
| October 15–21 | End-to-end replay/simulation, varied-position trials, first supervised physical pilot |
| October 22–25 | Verified grasp/placement feedback and recovery; inspection extension only if feasible and baseline works |
| October 26–29 | Freeze code/model and run held-out physical evaluation, including all attempts and failures |
| October 30–November 1 | Approximately five-minute video, one-page summary, repository, references; submit by November 1, 11:59 PM CT |
| November 2–5 | Rehearse and present the tested release at finals |

These are team targets, not confirmed equipment bookings. If the FANUC assets and supervised access do not have a credible route before October 14, revisit scope with organizers. The [mixing option](mixing-option.md) and Dataset 1-only inspection remain alternatives.

## Eight-person implementation split

| Pair | Ownership | First deliverable |
| --- | --- | --- |
| A | Robot twin, scene loading, synthetic generator, simulation trials | Existing-object simulated pick-and-place and reproducible trial manifest |
| B | Perception, conditional inspection layer, uncertainty, evaluation | Localization/quality baseline with separated development and final test inputs |
| C | Grasp planning, execution adapter, action verification, bounded recovery | Planner/executor in simulation or replay with explicit success/failure outcomes |
| D | Integration, testbed coordination, trial logs, business case, demo | One integrated replay and precise hardware test request |

Name a technical integrator and testbed coordinator across pairs. Assign actual teammates after skills and availability are known. Use stable object/trial IDs, timestamps, coordinate-frame identifiers, and versioned input/output schemas.

## Evaluation

Measure successful picks, correct placements, full-task completion, cycle time, recovery success, and operator interventions. Compare against the existing approved baseline and separate gains from vision, verification, recovery, and inspection. Report denominators and every attempt; a successful video alone does not establish reliability.

Keep object arrangements, conditions, and final physical trials out of development tuning. Begin with existing approved objects and fingers, then vary position and orientation before adding clutter or new object types. If inspection is included, report missed defects, false alarms, REVIEW rate, and correct quality-based sorting separately from successful grasping.

Train perception using the appropriate digital twin and generated labels. Confirm permitted uses of real data with organizers. The downloaded vial twin has static geometry and a prerecorded fill animation; it is not the FANUC twin. The robot twin's physics and control capabilities have not yet been inspected by Team V.

## Immediate dependencies

The [resource inventory](resources-and-readiness.md) tracks what is supplied, pending, and unactivated. [Organizer request drafts](organizer-requests.md) specify the assets, object compatibility, compute, and supervised tests needed. They have not been sent.

The essential next physical milestone is a reliable existing-object pick-and-place. The signature capability is verified action and recovery. Quality-directed sorting is the conditional extension.
