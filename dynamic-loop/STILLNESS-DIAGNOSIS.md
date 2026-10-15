# Strict camera-stillness investigation — October 15, 2026

Six-part run 284274 stopped at its first yellow-cube lift. Diagnostic pilots 284285 and 284286 preserved the hold and saved rejected RGB and joint/TCP traces. Rejected frames are marked `accepted_for_runtime=false`; they never authorize a runtime action or count as success.

At the original 1/60-second physics timestep, J6 reported approximately -0.026 rad/s over almost eight simulated seconds, while measured joint position spanned only about 1e-7 rad. Increasing velocity iterations from one to eight in pilot 284286 did not fix the mismatch: J6 remained around -0.028 rad/s, with position span about 6e-8 rad. This disproves the first candidate fix. The failed runs are retained.

NVIDIA's [Implicit Spring Joint Drives](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/_downloads/6acf3afb8f69452757e0e766b5a22978/implicitDrives.pdf), page 5, explains that TGS reports velocity at its final position substep; for stiff drives this may disagree with velocity over the complete timestep. This is consistent with our traces, rather than proof of a camera delay or real physical accuracy. NVIDIA's [stability guide](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/articulation_stability.html) recommends testing smaller physics steps on a reproducible failure.

Failed candidate pilot 284288 used a 1/240-second physics timestep, original 32 position / 1 velocity solver iterations, unchanged geometry/masses/drive gains and unchanged acceptance thresholds. Eight physics steps per 30 Hz motion command preserve command duration; four substeps per historical 60 Hz settle unit preserve settling duration. Rendered captures remain 30 Hz. This is a numerical simulation change, not a hardware calibration.

The raw joint-speed gate remains below 0.02 rad/s for six consecutive rendered samples. Source silhouette IoU remains at least 0.85; freshness, measured 2 mm TCP tracking, camera outcome verification, inventory and bounded retry gates remain active. A candidate is accepted only after runtime camera evidence and independent simulator scoring agree. Pilot 284288 also held: its rejected lift sample reported J6 speed near -0.043 rad/s. It did not fix the issue.

Pilot 284291 tested PGS instead of TGS, at the original 1/60-second timestep and original 32/1 solver iterations. The installed PhysicsContext API supports both solver types. Solver type is set before reset, read back afterward, and saved in physics-configuration.json. All motion durations, geometry, drive gains and raw feedback gates remain unchanged. Successful captures now retain their six accepted stillness samples for audit.

Pilot 284291 passed: one camera-verified yellow pickup and placement, matching independent scoring. All 10 accepted captures retained six speed samples; their peak was 0.001830454 rad/s. Both lift captures reported approximately 0.0018303 rad/s, with side-colour coverage approximately 0.50 and 0.497 (required 0.35). Destination side coverage was approximately 0.762 and 0.755 (required 0.65). The evidence sheet was visually reviewed.

Suite 284292 passed stillness through four pickups, then held after three verified placements on a separate placement-region rasterization defect. The strict hold was preserved. See validation-report.md for the geometric diagnosis.

Six-part rerun 284300 subsequently completed all six slots with camera verification and matching independent score. Its 50 accepted captures passed the unchanged raw stillness gate. The single-miss, repeated-miss and moved-target regressions also passed under the current configuration. All 87 accepted captures passed the unchanged speed/timing audit. See validation-report.md for exact outcomes and provenance.
