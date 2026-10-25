# Experimental learned FANUC peg insertion policy

## Current result —25 October2026

This published package is the earlier experimental policy source (profile02), not a promotion of the later GPU tuning candidates. Training produced two PPO updates from300 simulator transitions. A later profile completed one strict learned insertion/release cycle, but subsequent tuning cycles failed; all30 final-test seeds remain unexposed. No reliable learned policy is qualified. Newer numerical experiments remain separate until reviewed and accepted.

## External assets and setup

Licensed Factory peg/socket assets, the exported FANUC robot, generated bindings, original source snapshots, weights and videos are excluded from Git. Obtain the official assets/runtime separately and recreate the local export/binding before simulator execution. This public source tree is not a standalone simulator installation, and source-doctor checks are expected to report missing assets until setup is complete. Preserve the vendor BSD notices; see [third-party notices](../THIRD-PARTY.md).


This separate package preserves the verified calibrated baseline in ../industreal.
It uses NVIDIA Factory/IndustReal's public environment, keypoint rewards and
curriculum utilities, with a small goal-conditioned linear Gaussian actor and
custom clipped PPO. It does not load the original Franka checkpoint or reproduce
the original SDF reward/full SAPU algorithm.

The first policy is warm-started by fitting synthetic examples of the verified
Cartesian servo, then fine-tuned using actual simulator rewards. The release supervisor is fixed. During150insertionsteps the actor sees one
fixed seated goal and must generate the approach/corrections itself. There is
no moving approach waypoint trajectory; a0.4mm/controlstep velocity envelope
is enforced separately. One-time ideal calibration remains an assumption.
Every arm action after reset must come from the fitted checkpoint; deterministic
baseline action overrides are forbidden. The checkpoint is frozen during each
rollout, with observations/actions/weights hashes recorded.

The peg begins preheld. A one-time ideal simulator fixture/grasp calibration is
frozen after grip requalification. Actor inputs use TCP error to a phase goal,
simulated contact forces and previous actions, with no live peg pose. Those
calibration/force observations are simulation assumptions, not a demonstrated
camera or physical force sensor system. Physical hardware and full module
assembly are outside this benchmark.

experiment-plan.json freezes distinct training, tuning and final-test seeds,
changed-start ranges and unchanged physical pass gates before simulator runs.
Training loss or reward is never an insertion pass. Every reported qualification
must complete insertion, measured jaw release,40mm withdrawal, a fixed3s settling
pause and15strict final samples with fully audited contacts. Failures stay in
the denominator. A frozen checkpoint must reach at least29/30 held-out passes
without a contact-guard failure before the planned reliability milestone.

CURRENT STATUS: experimental source prepared; no learned-policy simulator result
or reliable learned insertion is claimed yet. The passing deterministic baseline
288315 remains separate and unchanged.