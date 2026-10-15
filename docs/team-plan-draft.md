# Team V — team plan draft

Prepared October 8, 2026. Due October 14, 11:59 PM Central. Draft for review; not submitted. The live form requires sign-in before its input fields are visible, so the sections below are prepared content rather than a verified field-by-field mapping.

## Team

Working name: Team V. Eight participants. The user identified the Teams channel Innovation Lab Group 20 as the roster source and confirmed that its channel owner, Liam Salas, is not a team member. The team roster is:

- Shritej Gunda
- Aditya Miriyala
- Koushik Cherukuri
- Thanvi Gorrepati
- Khushal Lingamaneni
- Levi Eloy Santos
- Yuktha Koganti
- Venkata Reddy Karri

All participants are from MSOE according to the user. Confirm the dashboard team assignment, captain, and individual roles before final submission.

## Project: vision-guided sorting with verified grasping and recovery

Dataset 3 is our core. We aim to make the supplied FANUC robot adapt to changing object positions, verify whether a pick and placement succeeded, and recover from a failed attempt within a fixed retry budget. This addresses a practical limitation of fixed-motion automation: small variations can lead to failed picks, misplaced objects, and operator intervention.

Our first milestone is repeatable pick-and-place using the existing approved objects and gripper fingers in the supplied digital twin. We will add camera-based localization, outcome verification, and bounded recovery, then compare the system with the mentor-approved baseline on the physical arm under supervision.

Dataset 1 is a conditional extension: synthetic-trained quality inspection could choose the sorting destination if organizers approve compatible objects, cameras, and gripper use. The FANUC cell is separate from the vial line. We will prioritize a reliable robotics core and will not claim an integrated vial production line without demonstrating one.

## Initial approach

1. Inspect the robot twin, camera examples, calibration, and supported command interfaces when released.
2. Reproduce the existing-object baseline, first in simulation and then in an early supervised physical pilot.
3. Build perception, grasp selection, execution, outcome verification, and a bounded recovery loop behind an adapter for the approved interface.
4. Vary position and orientation before introducing clutter or new objects. Freeze final test arrangements before evaluating.
5. Train any learned perception models on synthetic data generated from the appropriate twin. Confirm permitted real-data calibration and development use with organizers.

## Evidence and business case

Record every attempt, including failures. Report first-attempt pick success, correct placement, completed tasks, cycle time, recovery rate, and operator interventions, with counts and denominators. Compare baseline, vision, verification, and recovery variants on matched conditions. If inspection is included, separately measure missed defects, false alarms, review rate, and quality-based sorting accuracy.

For manufacturers, the intended value is fewer manual corrections and less reprogramming when object placement varies. We will quantify this from measured trials rather than assume cost savings or deployment readiness.

## Requested resources

- FANUC arm/work-area twin, object assets, finger CAD, required Isaac Sim version, real camera samples and calibration, approved robot/gripper interface, and a robotics mentor.
- Ideally two or three CSI visits: an initial observation/setup visit, followed by one or two supervised model-testing visits. Availability is unconfirmed. A Teams question about this has been sent to Brett Storoe.
- Brev team access and a provisional request of about $300 equivalent: 60 L40S simulation/rendering hours and 20 training/inference hours, plus reruns/storage. This is a planning estimate, not an allocation. Benchmark a small run and confirm actual rates before scaling.
- MSOE GB10 access through Rosie when ready, as an alternative compute route.
- Complete Dataset 1 labels, fixture coordinates, and labeling tool only if the inspection extension is pursued.

## Timeline

- By October 14: activate team access, obtain robot assets/interfaces, establish the baseline route, submit this plan.
- October 15–21: integrated simulation/replay and early supervised physical pilot, subject to bookings.
- October 22–25: verification and recovery; conditional inspection extension after the core works.
- October 26–29: frozen final physical evaluation with all attempts logged.
- By November 1: approximately five-minute video, one-page summary, repository, and references.
- November 5: finals presentation.

## Ownership

Four pairs: simulation/synthetic data; perception/evaluation; planning/verification/recovery; integration/testbed/demo. Select a technical integrator and testbed coordinator. Named assignments depend on member skills and availability.

## Remaining details needed to fill the form

Official dashboard team name and captain; confirmed scope and target objects; named owners and available time; actual resource allocation; any requested paid agent seats and named users. Member names have been retrieved from Teams. No results or bookings are claimed yet.

## Live form access status

The Chrome session is signed in, but the team plan says to join the team before submitting. The event workspace matches assignment by university email. Its Add Email dialog requires a six-digit verification code. The user must verify the MSOE email before the team entry and actual plan fields become accessible. No new team was created and no plan was submitted.
