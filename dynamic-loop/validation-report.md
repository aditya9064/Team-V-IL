# Dynamic kitting validation — October 15, 2026

The six-part adaptive flat-slot trial passed on Rosie T4 job **284300**, together with single-miss recovery, repeated-miss hold and moved-target replanning. Saved runtime camera evidence and independent post-run simulator scoring agree. The job completed in 25 minutes 45 seconds. Slurm exit status alone was not used as proof.

| Case | Verified outcome | Provenance |
|---|---|---|
| Six-part kit | COMPLETE: six pickups and six placements, six independent slot scores agree | 284300 |
| One missed grasp | COMPLETE: first attempt camera-confirmed empty, one retry, one verified pick/place | 284300 |
| Two missed grasps | HOLD: empty_grasp_retry_exhausted after two confirmed-empty attempts, no third attempt | 284300 |
| Moved target | REPLAN after injected 25 mm shift, then COMPLETE with one verified pick/place | 284300 |
| Empty table | WAIT_FOR_PART, no motion attempt | 284267 |
| Missing recipe colour | WAIT_FOR_PART, no motion attempt | 284267 |
| Stale observation | HOLD before motion | 284267 |

The three early observation-gate cases reuse prior reports with explicit source-suite and runtime hashes. They are not claimed as rerun under the latest solver. The relevant supervisor and source detector logic is unchanged. [Machine-readable results](validation-results.json) contain report/runtime hashes, counts and individual provenance.

## Fixes and unchanged gates

The TGS solver reported persistent wrist velocity while measured joint positions barely changed. More velocity iterations and a smaller timestep did not resolve the hold. PGS at the original 1/60-second timestep and original 32/1 iteration counts resolved it in the yellow pilot and current suite. Solver readback is saved. Geometry, masses, drive gains and motion durations were preserved. See [the investigation](STILLNESS-DIAGNOSIS.md).

A second defect inflated tiny side-camera comparison regions by rounding polygon vertices and filling inclusive borders. Continuous pixel-centre sampling now matches the existing pose fitter. The retained fourth-placement failure in 284292 remains a failed run; offline re-analysis was diagnosis, not a runtime success claim.

The raw camera-stillness gate remains below 0.02 rad/s for six consecutive rendered samples; source silhouette IoU remains at least 0.85; endpoint TCP error remains at most 2 mm. Freshness, view skew, inventory, duplicate-target, two-frame outcome and one-retry checks remain active. Phase endpoint position convergence is separate from camera stillness during contact.

An independent audit checked **87 accepted captures**, six speed samples per capture, increasing sensor timestamps/frame identifiers, view skew, source fit, endpoint tracking and runtime hashes across the four current cases. Peak accepted raw joint speed was 0.003762 rad/s (rounded upward); maximum settled TCP endpoint error was 0.426841 mm (rounded upward). These are simulator telemetry, not physical accuracy claims.

## Checks and camera QA

All 16 dynamic CPU tests passed on Rosie, including five analytic rasterization and colour/occlusion rejection tests. The original 33 starter tests also passed. Seven replay and eleven supervisor tests passed locally; the bundled client interpreter lacks OpenCV, so geometry tests are reported as Rosie results.

The current six-part, one-miss, repeated-miss and moved-target QA sheets were visually reviewed. Full frames, calibration metadata, traces, all attempts and failed-run directories remain on Rosie. Compact reports and QA were downloaded; raw datasets and videos are excluded from Git.

- [Six-part camera evidence](evidence/six-complete-284300.jpg)
- [One-miss recovery](evidence/miss-once-284300.jpg)
- [Repeated-miss hold](evidence/miss-twice-284300.jpg)
- [Moved-target completion](evidence/moved-284300.jpg)
- [Retained fourth-placement failure](evidence/six-held-284292.jpg)

## Limits and next work

These are bounded simulations with known upright 10 mm cubes. They do not establish reliability across layouts, arbitrary obstacles, camera changes or a physical cell. The wrist view remains diagnostic; positive pickup evidence came from side RGB. No physical adapter, collision certification, hardware safety implementation or real-world calibration is validated. Simulator object state was used only by the separate fixture/fault-injection/scoring harness.

Next, evaluate a mentor-returned detector checkpoint behind the same observation contract, expand tests for changed inventory/occupied destinations and uncertain views, and perform approved CSI camera/TCP/table calibration before supervised single-part hardware trials. Training and physical deployment remain separate milestones.
