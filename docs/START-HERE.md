# Team V shared context

Updated 25 October 2026. Start here when continuing the project.

## Goal

Build an adaptable FANUC assembly demonstration. The prototype is a sensor enclosure: base, tray preloaded with an intact separate dummy PCB, then lid. Keep the supplied SMC MHZL2-16D2 gripper and current fingers unchanged.

## Verified progress and limits

- Cube kitting: six camera-verified simulated pickups/placements and recovery tests. No physical-arm validation.
- Original-size enclosure: one nominal run completed three-part motion; strict final support-force evidence was inconclusive. Offset variations failed contact checks.
- Larger enclosure: CAD, finger access, sampled reach and 63 CPU checks pass. No larger-part simulator pickup has run yet.
- IndustReal: one calibrated preheld deterministic insertion/release baseline passed. A later learned tuning cycle passed, but other tuning cycles failed. All 30 final-test seeds remain unexposed. No reliable policy or enclosure/hardware qualification.

## Current components

- [Larger-part prototype](../sensor-assembly/option-a/README.md): test base, loaded tray and lid separately using binary open/close, 40 mm lift and a fixed two-second hold.
- [IndustReal](../industreal/README.md) and [experimental policy](../industreal-policy/README.md).
- [Camera diagnostic](../sensor-assembly/camera-diagnostic/README.md): CPU-reviewed, unrun native image/timestamp association test. No invented time offset or relaxed freshness threshold.
- [Project checklist](PROJECT-CHECKLIST.md) and [official requirements audit](maic-reference/2026-10-25/REQUIREMENTS-AND-STRATEGY.md).

## Working rules

Preserve failures and strict contact/motion/stillness/PCB/inventory checks. Simulator truth may initialize fixtures and independently score or stop; it must not secretly correct targets. Keep the PCB separate, loose and intact, with no added grasp hole, glue or simulated following. Review a finite protocol and resource cap; use source hashes, a fresh queue check and durable receipts. Never blindly resubmit after a timeout. Videos stay on Rosie. Printing, physical actuation, purchases and external communication need applicable user authorization. No printing or hardware readiness is qualified.

Keep the learning runtime separate from the assembly environment. Physical deployment still needs camera calibration, confirmed TCP, binary-gripper mapping, real motion-delay handling and a booked robot slot. Debug cameras and simulator force observations are not physical perception evidence.

Datasets, weights, supplied licensed assets, videos, credentials and private correspondence are excluded. See [publication scope](PUBLICATION-SCOPE.md) and [third-party notices](../THIRD-PARTY.md). Existing dataset/training/kitting tools remain in their original directories.

The published final submission is due November 1 at 11:59 PM Central: five-minute video, one-page summary and repository/credits. The audit documents access gaps and missing detailed judging weights; verify organizer updates.
