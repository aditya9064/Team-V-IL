> Historical planning/research document. For current scope, resource status, verified results and next steps, read [START-HERE.md](START-HERE.md). Statements below describe an earlier project stage.

# Organizer request drafts

Prepared October 8, 2026. Team V has approximately eight members, all from MSOE. These are drafts only; no message or resource request has been sent. Team V should confirm its official team name and project direction before sending.

## Robotics package and feasible inspection extension

Destination: #dataset-3-grasping-sorting. Ask Brett Storoe, Dr. Derek Riley, or Tanner Cellio for routing to the robot mentor.

Team V is investigating Dataset 3 as our core project: camera-based grasping and sorting with action verification and recovery after a failed pick. We are also considering Dataset 1 as an inspection layer, conditional on compatible objects and approved hardware use.

We have downloaded and inspected the supplied vial dataset and vial-line twin. The shared folder currently has no FANUC package, and the robotics page still lists the download and access details as pending. To build an existing-object pick-and-place baseline, could you provide or confirm:

1. The FANUC arm/work-area twin, required Isaac Sim version, scene units, existing object assets, and current finger CAD.
2. Sample images from the overhead/wrist cameras, intrinsic/extrinsic calibration, robot-to-camera transforms, and the supported live camera interface.
3. The supported robot and gripper command interface, controller/software requirements, and the mentor who will supervise integration.
4. Whether this setup may handle approved inspection vials or other inspection objects. What objects, workspace layout, and gripper changes would be permitted?
5. The location of Dataset 1's labeled crops, fixture position maps, and labeling tool, and whether they are separate from the current raw camera archive.

Our initial target is a repeatable existing-object pick-and-place by the end of week one, before adding quality-based sorting. Please let us know which resources are available now and what is still being finalized.

## Supervised observation and testing

Destination: #data-and-testbed-requests. Tag Shamar Webster.

Team V requests an initial supervised visit for Dataset 3, ideally October 9–14, to understand the existing pick-and-place setup, confirm permitted camera inputs and robot/gripper interfaces, and determine whether a quality-inspection extension is physically feasible. We will begin with observation and the existing approved objects and fingers.

We would also like to discuss a pilot around October 18–21 and a final evaluation October 26–29, subject to your schedule. Before the pilot we will provide a frozen code/model version, input/output description, start/stop procedure, simulation/replay results, and failure-handling plan. We will not independently change cameras, plant controls, or robot configurations.

The planned evaluation compares the approved baseline with our vision-based system using supervisor-approved object positions/orientations. We will record all attempts, successful picks, correct placements, recovery outcomes, cycle time, prediction latency, and interventions. Could you confirm the available trial duration, permitted variations, maximum attendees, and how to reserve an untouched final evaluation set?

## GPU access and allocation

Destination: #compute-questions. Tag Dan Schneider.

Team V plans to use the provided Isaac Sim/Isaac Lab L40S launchable for robot simulation and synthetic inspection data, with lightweight inspection training where needed. Could you confirm the captain's team-organization invitation, initial Brev allocation, credit units, and access to a start/stop-capable AWS L40S environment?

Our provisional first tranche is 60 hours of simulation/rendering and 20 hours of training/inference. At the guide's planning rates this is approximately $220, and we propose requesting about $300 equivalent including reruns/storage. We will benchmark a small job and revise this estimate before large runs. We also request the availability and booking process for shared Spark/GB10 systems and the CSI workstation. Which is the preferred route for this project?

## Real-data boundary and evaluation

Destination: #dataset-1-vial-inspection, with a relevant dataset mentor.

Team V intends to train inspection models using synthetic examples from the provided twin, then evaluate on real imagery. Could you clarify which uses of real examples are permitted for camera/material calibration, threshold selection, prompt examples, development evaluation, and augmentation decisions? The track pages direct teams to avoid real-data training, while the collection guide also discusses combined-data comparisons.

Can organizers supply or reserve a held-out defect set and review the ground-truth labels? We also need to confirm how inspection targets can be evaluated from the FANUC cell's camera viewpoints if a combined Dataset 3 + 1 demonstration is approved.
