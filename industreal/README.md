# IndustReal → FANUC insertion port

## Current result —25 October2026

The calibrated, preheld deterministic insertion baseline passed one strict simulated cycle (job288315): insertion, measured jaw release,40mm withdrawal,3s settling and15 strict final samples. It is not camera-driven pickup, a trained policy, full PCB-housing assembly, statistical reliability or hardware validation. Older "pending" notes below are retained as chronological development history.

## External assets and setup

Licensed Factory peg/socket assets, the exported FANUC robot, generated bindings, original source snapshots, weights and videos are excluded from Git. Obtain the official assets/runtime separately and recreate the local export/binding before simulator execution. This public source tree is not a standalone simulator installation, and source-doctor checks are expected to report missing assets until setup is complete. Preserve the vendor BSD notices; see [third-party notices](../THIRD-PARTY.md).


This package contains an isolated implementation. It reuses NVIDIA's public Factory
peg/hole assets, insertion environment, multi-scale keypoint reward and PPO
configuration, plus the original IndustReal sampling-based curriculum functions.
It adapts the controller to six FANUC joints and the existing SMC jaw signs and
stroke. The original cube task and the shared module-assembly experiments are
not edited.

**Current evidence: GPU contact data now pass independent point-to-force-matrix
and net-force cross-checks. Job 288084 completed a 75-control-step hold and 5mm lift,
with 5.91 micrometres of peg/TCP relative drift, maximum49.6 micrometres
contact depth, bilateral force on all600physicsframes and no socket support.
A second seeded reset is checking this delivered revision. No trained policy, successful insertion, complete module
assembly, or physical hardware validation is claimed.** The actor uses TCP feedback, a noisy simulator-derived fixture
estimate and previous actions. That synthetic fixture estimate is a training
assumption, not camera inference. Simulator part poses also supply explicit
training reward/critic/setup/scoring inputs; this is not a vision-qualified or
physical deployment system.

## What is implemented

- Pinned upstream source with retained BSD licenses and per-file SHA-256 hashes.
- A bounded six-joint Cartesian-to-position adapter; no Franka torque commands,
  artificial inertia offsets or changes to original finger collision geometry.
- A preheld 8 mm peg benchmark, noisy pose inputs and progressive reset height.
- Read-only raw PhysX contact records with a 0.25 mm depth guard. No contact
  reports means unknown, never proof of collision-free motion.
- An independent insertion/release scorer requiring radial clearance, depth,
  tilt, contact evidence and a release hold. It cannot accept full assembly.
- A robot exporter, source/dependency doctor, zero-action smoke launcher and
  access to NVIDIA's reviewed PPO training driver.

The first port runs **one** environment. Vectorized FANUC reset/contact binding
and throughput work are future changes, not implemented capabilities. The
Factory keypoint reward is used here. The 2023 mesh-SDF reward and full SAPU
transition-filtering algorithm are retained as reference source but are not
ported/executed by this task; the contact guard is explicitly SAPU-inspired.

## Run host checks

From this directory, with host Python containing PyTorch and NumPy:

```bash
python3 -m unittest discover -s tests -v
python3 -m team_v_industreal.doctor
```

The doctor reports missing simulator dependencies without launching Isaac.

## Export our actual robot and fingers

Use the existing Isaac Sim 6 environment and supplied v05 starter. Export writes
a new self-contained asset, normalizes only the root translation, and retains
the original gripper shapes/materials/drives. `--starter` must be the extracted
`fanuc_er4ia_sim` directory. `FANUC_ROBOT_USD` can select the approved local asset.

```bash
python export_robot.py --starter "$TEAM_V_STARTER" --out "$TEAM_V_EXPORT"
```

The original 1.8mm/jaw opening and 2mm grasp depth failed the first hold test.
The current candidate uses a 30mm shaft grasp and 3.8mm opening per jaw with
0.5s of gravity-free closure settling during explicit episode setup. The
original supplied jaw geometry, mass, friction and drives stay unchanged.
A nominal spring-force calculation is not a measured grasp force. Earlier
20mm and 30mm candidates slipped and touched the socket; these are preserved
as failed trials. The 5mm diagnostic lift remains inside the 50mm action box.
These parameters passed the initial simulated hold/lift qualification;
this is a preheld benchmark and does not demonstrate picking up a loose peg.
Export success alone is not a fit, mass, collision or grasp certificate.
Generated robot assets and source snapshots must stay out of the public repo.

## Runtime compatibility and GPU admission

Reference source is Isaac Lab v2.3.0, commit in `upstream-manifest.json`. Its
documented runtime is Isaac Sim 5.1.0 and Python 3.11. The supplied FANUC starter
and the working assembly installation use Isaac Sim 6.0 / Python 3.12. Use that
existing installation only for the robot export; prepare a **separate** Python
3.11 / Sim 5.1 environment for this pinned learning port. The exported asset's
cross-version compatibility and grip must still be tested in simulation.

The doctor now checks the selected runtime versions and SHA-256 fingerprints
of the pinned Lab environment and quaternion-math APIs before startup. Lab's
v2.3.0 release identifies its Python distribution as `isaaclab==0.47.2`; these
are different version labels. An installed Lab 3.x cannot replace it directly:
this port expects the 2.x WXYZ quaternion convention. Matching versions and
fingerprints are only preflight evidence, never proof that the simulation ran.
A missing dependency or mismatching runtime stops the launcher before Isaac
startup. The working assembly installation must stay intact.

The existing assembly coordinator owns the sole teaching T4. This package does
not submit, cancel or modify scheduler jobs. Admit an IndustReal smoke only after
that coordinator hands over the GPU and a fresh queue check shows no overlap.
Use an allocated session; the launcher's `SLURM_JOB_ID` check is not itself GPU
admission or proof of available resources.

Inside that allocation and the compatible Isaac/Lab Python environment:

```bash
python launch.py --mode smoke --binding "$TEAM_V_EXPORT/binding.json" \
  --out "$TEAM_V_RUN" --steps 30
```

Only after inspecting the actual reset/grip/contact traces and smoke report:

```bash
python launch.py --mode train --binding "$TEAM_V_EXPORT/binding.json" \
  --isaaclab-root "$TEAM_V_ISAACLAB" --out "$TEAM_V_TRAIN_RUN" --iterations 1 \
  --smoke-report "$TEAM_V_RUN/smoke-report.json"
```

This delegates to the pinned NVIDIA RL-Games training driver, at one environment
and one initial PPO iteration. One iteration is a training integration check,
not proof of a useful learned policy. No vendor Franka checkpoint is claimed
compatible with this changed controller and observation/task distribution.
The zero-action smoke requires reported peg/finger contact and at most 1 mm
relative peg/TCP translation change. This is a new peg-benchmark startup gate,
not a change to the module assembly's existing 0.020 mm lid gate or proof of
insertion accuracy. Training requires the matching prior smoke report.

The smoke also flushes `measured-state.jsonl` after reset and every control step.
It records measured and commanded joints separately, TCP/part poses, raw-contact
audit summaries, and peg alignment in the socket frame. Episode auto-reset
samples are explicitly labelled. A jaw-opening request never becomes release
evidence in this recorder. The trace supports debugging; it does not certify
insertion, release, a learned policy, or complete assembly.

The contact reader enables PhysX contact processing and reads each jaw and
socket through separate native GPU contact views. Signed contact scalars are
multiplied by their normals and must reconstruct the independently queried
pair-force matrix. The total held-part force must also agree with the sum of
known jaw/socket forces; unexpected force stops the episode. Raw callbacks
remain separately counted and are unverified in this GPU tensor runtime.
Static table colliders are unsupported as GPU filter targets in this version.
Depth coverage therefore covers the indexed peg/jaw and peg/socket contacts;
net-force residuals detect unaccounted loaded contacts, not all geometric
contacts or a global collision-free certificate.

The qualification retains a 0.25mm penetration limit, 1mm relative-drift
limit, sustained bilateral jaw force and no socket support. No attachment,
per-step peg repositioning or acceptance-threshold relaxation is used.
`--grip-lift-m .005 --steps 75` commands a 5mm TCP-feedback lift after 30 hold
steps. Privileged reset contact transients are retained separately; episode
qualification starts with gravity restored. The candidate restores the pinned
Factory 120Hz TGS solver with 192 position iterations for this isolated
benchmark, while the controller remains at 15Hz. These solver settings differ
from the earlier 60Hz PGS startup diagnostic and do not modify shared trials.

## Qualification sequence

1. Export and verify asset dependencies, six arm/two jaw bindings and original
   full finger geometry/material/mass/drive properties in the actual runtime.
2. Confirm one feasible reset, grasp hold, finite actions and contact reports.
3. Run a bounded training smoke; preserve failed attempts and exact seed/source.
4. Train only after runtime and throughput evidence justify the allocation.
5. Compare the learned policy with the baseline on identical unseen seeded
   offsets. Keep evaluation offsets fixed and separate from the curriculum.
6. Verify seating, release and independent scoring before a benchmark success
   claim. The custom lid/base and full sequence require separate qualification.

No physical FANUC interface is implemented. Its controller options and supported
online command interface still need confirmation in the hardware workstream.

Sources: [IndustReal](https://research.nvidia.com/publication/2023-05_industreal-transferring-contact-rich-assembly-tasks-simulation-reality),
[Factory source](https://github.com/isaac-sim/IsaacLab/tree/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab_tasks/isaaclab_tasks/direct/factory),
[original algorithms](https://github.com/isaac-sim/IsaacGymEnvs/blob/main/isaacgymenvs/tasks/industreal/industreal_algo_utils.py).

## Calibrated insertion extension (prepared, not yet simulator-qualified)

`insert.py` is a deterministic physics-validation baseline in the public
NVIDIA Factory/IndustReal task. It is explicitly **not a trained policy**.
A one-time privileged simulator fixture/grasp calibration is frozen after
reset. Targets then use that frozen record plus measured TCP feedback; live
part truth supplies independent STOP/scoring only. The current candidate requalifies a24mm shaft grasp with3.6mm opening per jaw for2seconds, aligns, descends,
opens the measured jaws, withdraws15mm and verifies released support.
The original57micrometre radial,1mm depth,1degree tilt,0.25mm penetration
and0.5second minimum released-hold scoring limits are retained.

The insertion-only benchmark gives the geometrically/material-identical
immovable table a kinematic rigid-body identity so native GPU contact views
can distinguish table support from unaccounted force. Its actual pose must
remain fixed. This is a reported model/configuration change from the static
collider used in the passing grip-only tests. Peg/table contact is allowed
only inside the explicit307um radial capture envelope at seated depth; final released fit remains57um. Extra jaw/socket and jaw/table
views reject fixture contact by the fingers. The task records measured jaw
opening/load, released gravity support and tool withdrawal. These checks do
not constitute a global collision-free certificate for every robot link.
The preceding grip qualification remains evidence for source09eb372c;
this insertion extension requires its own actual simulator result.

The public hole mesh is40mm square and25mm tall above its seating plane.
The prior30mm grasp would put fingertips5mm below that rim at full seating.
The first actual insertion candidate used20mm grasp depth: the tips remained5mm
above the rim, with3.8mm jaw opening. All supplied geometry/drives were
unchanged. This different grasp must pass the original hold/drift/force gates
in the same run before a frozen calibration is taken and insertion begins.

The first insertion diagnostic288129 stopped before release when radial error
exceeded57um near the seat. Recorded-pose decomposition at the preceding
valid sample found approximately51um of armX tracking error and100um of
in-gripY displacement. The next candidate uses24mm grasp depth (1mm fingertip
clearance above the25mm socket rim) and3.6mm jaw opening. It also adds bounded
joint-command integration to the diagnostic adapter to reject static servo
load bias, capped5mrad from measured joints. Original authored drives,
materials, masses and all independent fit/contact limits remain unchanged.
This candidate still needs actual simulator verification.

The second actual diagnostic288140 rejected a held pose at62.9um radial
error before release. Arm tracking bias fell below1um, while the grip showed
oscillatory rather than constant residual deflection. The next candidate
uses8velocity solver iterations and a smooth final descent, then allows
release only inside a307um radial capture envelope (57um clearance plus the
existing250um maximum contact penetration) at the original1mm seated depth
and1degree tilt bounds. This is an intermediate trajectory permission, not
a changed success criterion. Final released scoring is still57um radial,
1mm depth,1degree tilt,250um penetration, measured gravity support and at
least0.5s released hold after measured jaw opening and15mm tool withdrawal.
A CPU regression explicitly proves a63um captured pose still fails final
scoring. Prior failed trials retain their original criteria and outcomes.

The third actual insertion trial288160 reached a39.9um held radial fit,
then failed the original guard on the first instantaneous jaw-opening step.
Measured pre-release peg velocity was oscillatory; abrupt full opening released
stored grip/contact motion and displaced the peg outside the capture envelope.
The next candidate commands a4second smoothstep opening from the original
3.6mm grasp target to the unchanged6mm full per-jaw stroke. It changes jaw
commands only, without changing authored drive gains, damping, mass/friction,
geometry or velocities of the peg. The arm holds the same frozen seated target.
The final released fit/support/contact gates remain unchanged. This correction
requires its own actual simulator trial;288160 remains failed.

The original NVIDIA socket mesh has a9mm bore (4.5mm radius atz0/24mm),
although Factory Hole8mm declares8.1mm. Its~0.507mm physical clearance
matches the prior release displacement. The original asset is preserved in
fixtures/factory_hole_original_9mm.usd. This candidate explicitly replaces
only the socket inner surface with the declared8.1mm bore:432inner vertices
move radially inward0.45mm; the1mm mouth chamfer, outer vertices, faces,
topology, materials, mass, fixed joint, robot and gripper are preserved.
The derived asset is checksum-bound and its measured57um radial clearance
matches the unchanged final criterion. This is a disclosed FIXTURE GEOMETRY
CORRECTION, not proof that the unchanged native9mm fixture meets57um.
Previous failed trials keep their nativefixture, criteria and failed outcomes.
A new single5min prospective fixture-qualification job follows the705seconds
spent in the prior insertion batch; no unbounded retry or policy training.

Fixturequalification288168 reached measured jaw release at55.3um radial
fit,1.3um depth and0.029deg tilt, but hit the250s child timeout atstep237,
so it remains FAILED without completedwithdrawal/releasedhold. The next
candidate copies each contactbuffer once, retaining everyindexedactivecontact
through its maximum end index, and builds the same first8debug samples from
those CPUarrays. No active contact,force/depth check orphysicsstep is skipped.
One prospective3minute completion test follows the967actual priorGPUseconds.

The first complete-withdrawal trial288301 measured15.0002mm retreat and
held released radialfit~55um, but failed the original0.1rad/s instantaneous
angular-stationary check during the final hold. It remains FAILED. The next
trajectory withdraws40mm (measuredminimum39mm), placing the fingertip beyond
the50mm peg plus both5mm contact margins, then waits a fixed3seconds before
the unchanged15sample strict hold. All active contacts remain audited during
settling; no body damping, velocities, geometry or acceptance bounds change.
This pause is fixed in advance, not selected from successful measured samples.

Clearance/settling trial288310 completed40mm withdrawal and fixed3s settle,
but failed the unchanged0.1rad/s stationary check atstep320;radial55um and
tilt0.00265deg remained small. It remains FAILED. The next candidate refines
peg/socket numerical SDF fields to1024 cells along longestextent (48.8/39.1um
spacing), sparse subgrids6, masking old baked buffers beforephysics to force
recooking. This is a reported COLLISION-DISCRETIZATION change, not a CAD,
material, inertia, drive, damping, timestep or acceptance change. The default
256-cell field has195/156um spacing, coarser than57um clearance. Jitter being
caused by that discretization is a hypothesis pending actual qualification.
Source: NVIDIA PhysxSDFMeshCollisionAPI documentation (gridspacing=AABB/resolution):
https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/physxschema/class_physx_schema_physx_s_d_f_mesh_collision_a_p_i.html