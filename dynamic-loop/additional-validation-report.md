# Additional failure tests and detector plug-in — October 15, 2026

The robot passed five new situation tests plus a complete six-part kit test in simulation. It waited for an arriving part, stopped when a destination was occupied or a camera was unavailable, and stopped when camera proof was blocked or a previously placed part moved. The trained detector is not available yet: every motion trial used the existing colour-and-shape detector.

## What each test showed

| Situation | Verified behavior | Evidence job |
|---|---|---|
| Destination already occupied | Stopped before any pick; unconfirmed_destination_occupied | 284313 |
| Side-camera image missing | Stopped before any pick; camera_image_missing:side; independent scoring still saved | 284313 |
| Required part arrives late | First WAIT, then a new image detects the part; one attempt, one verified pickup and placement, COMPLETE | 284319 |
| Side view blocked after pickup | Two uncertain observations; stopped without placing or retrying | 284313 |
| A completed part is moved 15 mm | First placement was verified; next observation detected the change and stopped before a second pick | 284313 |
| Full six-part kit | Six camera-verified pickups and placements; independent simulator checks agreed for all six slots | 284313 |

An intentional stop is a pass only when its exact cause, timing and attempt count match the test. A scheduler success code alone is not proof. Read [the machine-readable audit](additional-validation-results.json) for exact report and source hashes, observation hashes, fault-injection provenance and individual measurements. The original [284300 validation](validation-report.md) remains a separate historical record.

## Retained failure and repair

The first late-arrival test in 284313 failed. The test setup wrote a new position to already-kinematic parts, but the parts remained visible at their authored positions. The robot never reached WAIT. Both grasp attempts were correctly camera-confirmed empty, and it stopped after the allowed retry. That result is preserved as a failed fixture trial, not relabeled as a pass. Consequently the overall 284313 batch correctly exited FAILED (1:0), despite its other five tests passing.

The fixture repair authors the parked positions before physics initialization, saves the originals, then restores one part's authored position and releases its kinematic setting after WAIT. A mocked-stage regression covers ordering and one-time arrival. The fresh runtime trial 284319 then passed WAIT -> fresh image -> one verified pick/place -> COMPLETE with independent scoring agreement. The two source snapshots and run folders are preserved. Runtime code hashes agree across the trials except for this explicit fixture change; the acceptance checker also now requires parking provenance.

## Checks remained strict

Across the six accepted cases, 79 complete camera observations passed the saved six-sample stillness audit. The largest raw joint speed was 0.003458332 rad/s, below the unchanged 0.02 limit. The largest settled tool-position error was 0.426831 mm, below 2 mm. Pose silhouette IoU remained at least 0.85. PGS physics, 1/60 s physics steps, 1/30 s rendering, articulation iterations 32/1, geometry, gains, masses and motion durations were unchanged.

Freshness includes image processing time. Backend identity, sensor progression, inventory, positive camera evidence, previous placements and bounded retry checks remain active. Simulator object positions are used only in test setup, deliberate disturbances and independent post-run scoring. Runtime targets and outcomes use RGB, camera calibration, measured robot joints and the recipe.

All 33 plug-in/fixture tests passed in the allocated rerun. The original 33 starter tests also passed on Rosie; 21 pure contract/supervisor tests and the mocked arrival fixture passed locally. Saved camera checksums were verified for 276 RGB files, including the retained failed fixture's 39 frames. Camera sheets were visually reviewed. Full RGB and reports remain in the preserved Rosie run folders; only compact QA and audit summaries are published here.

## What the model plug-in proves

The interface loads the mentor's expected Faster R-CNN architecture, verifies checkpoint/config identity and maps the training crop back to full-camera pixels. It retains the image-based pose and camera outcome gates and has no silent fallback when a learned model is selected but invalid. See [MODEL-PLUGIN.md](MODEL-PLUGIN.md).

A CUDA smoke on the actual architecture used explicitly untrained random weights. It passed in 7.65 seconds with zero training steps, zero motion commands, zero accepted proposals and 100 low-score rejections. This establishes loading, RGB/crop conversion and GPU inference wiring only. It does not establish trained-model accuracy. The deployment template's threshold and checkpoint hash remain empty until the mentor supplies validated artifacts.

## Camera review

- [Six-part source, empty-hand, pickup and placement views](evidence/additional-nominal-six-284313.jpg)
- [Late-arrival successful pickup and placement](evidence/arrival-complete-284319.jpg)
- [Blocked destination](evidence/additional-occupied_destination-284313.jpg)
- [Missing side stream: retained table images](evidence/additional-missing_view-284313.jpg)
- [Blocked side view produces uncertain pickup proof](evidence/additional-occluded_view-284313.jpg)
- [First placement before the deliberate displacement](evidence/additional-placed_moved-284313.jpg)
- [Retained failed arrival fixture: two empty grasps](evidence/arrival-failed-fixture-284313.jpg)

The displaced-part sheet shows the earlier verified placement; the later stop and independent failed final-slot score are recorded in the JSON. Enlarged crops are diagnostic views, not added image detail.

## Limits and next work

These are individual controlled simulation trials, not a measured reliability rate or physical deployment. Cubes remain upright known 10 mm parts; the present wrist view does not supply positive held-part proof. Confirm the physical camera views, calibration, gripper setup and approved control interface at CSI. When the mentor returns the checkpoint and validation metrics, first evaluate it offline, then run a bounded simulation pilot with these same checks. Broader quantities, layouts and arrival patterns should be separate recorded trials; six parts are the current repeatable recipe, not an organizer limit.
