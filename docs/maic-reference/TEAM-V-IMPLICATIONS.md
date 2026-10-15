# Organizer feedback and Team V implications

The feedback below comes from the review email pasted by the user on October 13, 2026, separately from the public website. The proposed response is planning guidance, not an approved physical test or a claim that the new features are implemented.

## Organizer feedback

- Our problem aligns with the hackathon, but selected parts must fit the standard fingers, or fingers must be redesigned for those parts. Explore grasps in the twin.
- Define a manufacturing scenario: how objects arrive and why we move or sort them.
- Static sorting from the initial table-camera image is already solved in the supplied twin. Address dynamic situations, use the camera views meaningfully and demonstrate broader manufacturing impact.
- Review provided assets before asking for missing data. Arrange a CSI visit and develop our own calibration strategy.
- Dr. Zeng, zhenzeng@uwm.edu, is the primary FANUC and connected-GB10 contact. Digital-twin questions go to Dr. Riley, driley@nvidia.com.
- build.nvidia.com can be used immediately. Brev, MSOE GB10 and agent access details come in separate messages. The CSI Emulate3D workstation is associated with Datasets 1–2, so it is not justified by our committed FANUC scope.
- Object variations are open if consistent with manufacturing and the hackathon theme. Caps, vials and cubes from the supplied assets can be used in the separate FANUC cell.
- Questions about packages, calibration, CAD and interfaces must be specific after inspecting the twin. Generic availability and real-data-use questions were too broad.
- Organizers invite questions via email/Slack before November 1; mentors respond to concrete questions rather than one blanket assignment.

## Proposed revised problem statement

In a small-batch manufacturing cell, parts arrive at varying positions and may shift between robot actions. A robot using only the first image can attempt a stale target, miss a grasp or leave a part incorrectly placed. We propose a FANUC sorting system that observes the current workspace before each pick, selects a compatible part, verifies pickup and placement, and either retries once under an approved procedure or stops for operator help. We will compare it with the supplied static-plan baseline using controlled changes and logged attempts.

Arrival mechanism, exact part, destination rule and permitted disturbances remain decisions to confirm. Do not claim a real conveyor connection or real-time moving-object interception.

## Proposed camera responsibilities

| Camera | Purpose to validate |
| --- | --- |
| Main overhead | Detect part identity/pose and refresh target before each pick. |
| Second overhead | Check placement and workspace changes with complementary coverage. |
| Wrist | Confirm close-range alignment or grasp/part presence where visibility supports it. |

These roles need actual camera samples, calibration and visibility tests; three cameras alone do not establish three useful signals.

## Finger decisions

Keep fitting small parts as the first baseline. Redesign is justified by a specific required grasp that the standard fingers cannot perform, not by novelty alone. Review supplied mounting/CAD, choose contact shape, model clearance and collision geometry, and compare repeated lift/transport/release trials. Physical gripper identity is unresolved: website says SCHUNK, starter models SMC. Obtain actual mounting, stroke/opening and permitted material/load details before fabricating.

Potential shapes include flat faces for cubes or shallow V-grooves for a selected cylinder. Outward-offset fingers may accommodate wider parts but also increase the minimum closed gap; they do not increase actuator stroke. A simulation pass does not validate printed strength. CSI/mentor review is required before physical attachment.

## Evidence we have and gaps

- Isaac Sim is running on Rosie T4; 33 starter tests passed.
- Sixty synthetic scenes exported with 48/6/6 scene-level splits; crop/visibility QA is incomplete.
- Geometric refinement performed well on five development scenes, not real camera images.
- One simulated six-cube execution demonstrated six verified pickups/placements; rendered video exists. This is the static baseline, not the new dynamic system.
- No trained neural detector, completed training handoff, CSI physical trial, calibrated multi-camera runtime verifier or recovery controller yet.

## Before more large dataset jobs

1. Agree on manufacturing arrival and sorting purpose, selected part and finger compatibility.
2. Define dynamic scenarios: shifted target between actions, failed pickup, uncertain view, misplaced part and new arrival. Start with stationary parts during each action.
3. Define baseline comparisons and metrics: attempts, pickup/placement success, stale-target rejection, recovery success, intervention rate and cycle time.
4. Assign camera roles and calibration strategy.
5. Freeze the dataset specification: randomized layouts/counts/yaws, camera/light/background variations, distractors, occlusion policy, image/box/mask/pose labels, generation metadata and scene-level held-out splits.
6. Build persistent simulation capture to amortize startup, then benchmark images/hour and valid samples/hour before choosing Rosie versus Brev.
7. Package a reproducible training/evaluation handoff for the mentor; keep calibration/model changes versioned.
8. Prepare a specific CSI collection/test request with stop procedure, supervision and all-attempt logging.

The plan has now been revised in [Project plan v2](../PROJECT-PLAN-V2.md), with an [organizer response draft](../ORGANIZER-RESPONSE-DRAFT.md). Earlier plans are marked superseded; visual slides/diagrams still describe the older design.
