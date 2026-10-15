> Historical planning/research document. For current scope, resource status, verified results and next steps, read [START-HERE.md](START-HERE.md). Statements below describe an earlier project stage.

# Alternative project option: CHROMA

Earlier Dataset 2 proposal, retained for reference. The current working direction is Dataset 3 with a conditional Dataset 1 extension; see [project-strategy.md](project-strategy.md) and [resource readiness](resources-and-readiness.md).

Proposed direction, October 8, 2026. Not yet validated or selected by the full team. The team has approximately eight members; skills and weekly availability remain unknown.

The supplied dataset and twin have now been inspected. See [dataset findings](dataset-findings.md): the twin has no physics, and the recipe labels do not provide quality/cause verdicts. These are first-week feasibility dependencies.

## Thesis

Build a recipe-aware physical AI assistant for Dataset 2 that detects fill/color problems, forecasts carryover risk before a recipe transition, and recommends a lower-risk sequence for operator review. Show the full path from synthetic training to live-line evidence.

The competition guide describes a concrete gap: the existing inspection recognizes known colors without reliably establishing whether they match the intended recipe. It describes the first vial after a changeover being affected by residue. Team V's proposed contribution is to test whether recipe history improves detection and prediction, then whether those predictions support useful production decisions.

The strongest defensible final claim would be: "On held-out physical runs, our recipe-aware system improved detection of recipe mismatches over a color-only baseline; a supervised sequence experiment measured the effect of our recommendations on rejected vials." Replace this with the actual measured result. Never claim prevention, causal attribution, or savings based only on simulation or replay.

## The demonstration

1. Show a real recipe transition and the available synchronized camera views.
2. Before the first new vial is filled, display a forecast based only on information available then: previous recipe, intended next recipe, dispense order, and observed tank state.
3. Track the fill; compare observed color and level with the expected recipe and show PASS, REVIEW, or a detected mismatch with evidence.
4. Explain the leading hypothesis with timestamps and recipe metadata. Mark carryover as a hypothesis unless an independent label or controlled experiment supports it.
5. Present an alternative order for the queued recipes, including uncertainty and operating constraints. With at most three queued recipes, enumerate the six orders; a complex scheduler is unnecessary.
6. Show real trial counts, misses, false alarms, latency, failure examples, and comparison with the baseline. If an approved intervention was performed, show its measured result separately from forecasts.

The system initially operates in observation mode. It does not independently write recipes or plant controls. Approved supervisor-led experiments can test sequence recommendations.

## Technical scope

### Required core

- Ingest the three camera streams and recipe metadata. Audit timestamps, missing frames, offsets, and drift. Preserve stable run, recipe, vial, and event IDs.
- Establish a simple color/level measurement baseline using fixed regions and explicit lighting sensitivity tests. Do not claim physical volume accuracy without a validated mapping.
- Generate labeled fill/color variants with the supplied Isaac Sim twin. Keep generation parameters, seeds, asset versions, and labels. Inspect renders for invalid states and label corruption.
- Train a small perception model on synthetic data for robust vial/liquid localization or fill-state extraction only if it addresses a measured baseline weakness.
- Build a bounded, interpretable carryover surrogate using recipe history and dispense order. Start with explicit assumptions and parameter ranges, not a large model. The supplied twin does not establish fluid dynamics; add a clearly identified process surrogate if needed, and test its predictions on real runs.
- Fuse perception and process context to forecast transition risk and detect a mismatch. Expose uncertain or unsupported cases as REVIEW. Report automated coverage as well as performance.
- Log evidence and render a compact operator interface with synchronized views, the expected/observed result, and a recommendation.

Tank colors can change. Treat current tank-camera color inference as a separate component; the twin has no tanks, so use an interpretable camera measurement initially rather than assuming synthetic tank assets exist. If tank observations are unreliable, require operator-confirmed tank identities and report that dependency.

Do not assume linear RGB interpolation yields physically correct mixture colors. Confirm usable color relationships and allowed calibration with mentors. Vary the process assumptions to expose uncertainty rather than displaying unsupported precision.

### Optional extensions, only after the core works

- Use the available Cosmos reasoner to summarize verified measurements and footage. Compare it with deterministic explanations and score unsupported assertions. It must not be the authority for a pass decision or a verified cause.
- Select targeted synthetic variations based on a designated development split, then show whether they improve a separate frozen real test split.
- Demonstrate an unseen recipe transition and a lighting change with the pipeline frozen.
- Test sequence recommendations in a supervised paired experiment. Account for initial residual state, batch order, and rinse/reset conditions so the comparison is interpretable.

## Feasibility gates and fallback

| Gate | Evidence required | Decision |
| --- | --- | --- |
| By October 10 | Open real files and twin, map recipe/vial timestamps, inspect useful camera crops; identify someone who can own simulation | Continue if data is usable; resolve access immediately otherwise |
| By October 12 | Runnable replay baseline and sample synthetic renders; clear route to supervised live camera input and testing | Prefer Dataset 1 if mixing data, rendering, or physical access has no credible route before team-plan deadline |
| Before October 14 plan | Mentor answer on allowed calibration, test split, recipe metadata, permitted sequence experiments, and scheduling | Commit to detection + forecasting; make intervention conditional on approved access |
| By October 21 | First physical observation-mode run plus evidence of recipe-history benefit on development evaluation | Drop sequence optimization if predictions are unreliable; retain recipe-aware inspection with measured limitations |
| By October 25 | Frozen evaluation protocol and scheduled final hardware run | Stop adding architecture; protect hardware evidence and reproducibility |

Fallback within Dataset 2 is a recipe-aware fill/color inspector with evidence-based carryover warnings. Dataset 1 fallback is a synthetic-trained fixture inspector with position context, uncertainty review, and targeted synthetic generation for missed defects. Switching tracks after October 14 would be expensive, so resolve primary risks before then.

## Evaluation contract

Define the protocol before optimizing the system. Distinguish synthetic training, development data, and final evaluation. Obtain organizer clarification before using real data to fit calibration, thresholds, process parameters, or model behavior. Keep a final physical run set out of prompt examples, tuning, and error-driven augmentation.

Use vial-level outcomes rather than counting neighboring video frames as independent examples. Split by production run/time window and hold out recipe groups. Report sample counts and uncertainty; 30 labeled vials cannot support broad production-grade claims.

Compare:

1. Color-only inspection baseline.
2. Synthetic-trained perception with current recipe context.
3. The same system plus previous-recipe history and process context.
4. Optional reasoner explanations, scored separately from detection.

For each, report missed mismatches, false alarms, per-condition results, detection delay, per-vial latency, REVIEW rate, and automated coverage. Predeclare how REVIEW is counted; routing a defect to review is not equivalent to automatically identifying it.

For transition forecasts, evaluate before-fill predictions against subsequent vial outcomes, compare with a simple "first vial after every recipe change is risky" baseline, and include repeat-recipe and clean-transition counterexamples. A model that merely repeats the first-vial rule has not established an improvement.

For explanations, have a mentor or independent reviewer label supported causes where possible; otherwise evaluate consistency with known metadata and call the output a hypothesis. Images alone do not prove carryover caused a mismatch.

For sequence recommendations, separate predicted savings, simulation results, replay estimates, and physically observed savings. Include every physical attempt, software/model version, initial condition, expected result, outcome, timing, and supervisor intervention. Record naturally occurring defects and any authorized seeded faults separately.

Working objectives, not promised results: reduce misses at a comparable false-alarm rate, improve transition forecasting over the simple first-vial rule, and stay within the measured line-cycle budget. Set numerical targets after a baseline and mentor input establish credible denominators and requirements.

## Eight-person work split

| Pair | Ownership | First deliverable | Shared interface |
| --- | --- | --- | --- |
| A: simulation and data | Twin, synthetic generator, data manifests | Reproducible render batch and aligned real-data index | Images/events with stable IDs and generation metadata |
| B: perception and evaluation | Color/level baseline, small perception model, frozen test protocol | Per-vial baseline report with failure examples | Observations with confidence, timestamp, and source |
| C: process modeling and planning | Expected state, carryover forecast, sequence scoring | Explicit process assumptions and baseline forecast comparison | Forecasts using only pre-fill information |
| D: integration and physical proof | Live inputs, interface, test coordination, business case, demo | End-to-end replay and hardware test request draft | Operator output and immutable trial log |

Name one technical integrator and one testbed coordinator across the pairs. Agree on one event schema and a replay dataset on day one. Each pair should integrate daily; eight disconnected notebooks will not produce a credible hardware demo.

## Schedule

- October 8–10: confirm team skills, audit data/twin, define problem and metrics, draft precise testbed request.
- October 11–14: baseline, synthetic pilot, mentor review, direction decision, team plan and resource request.
- October 15–21: synthetic pipeline, perception/process comparison, integrated replay, first supervised physical test.
- October 22–25: targeted improvements, optional sequence experiment, deployment latency measurement, freeze evaluation design.
- October 26–29: frozen hardware runs, ablations, failure analysis, deployment/business evidence.
- October 30–November 1: one-page summary, approximately five-minute video, reproducible repository, references, submit before 11:59 PM CT November 1.
- November 2–5: rehearse finals; retain the tested release and disclose any later changes.

## Draft team-plan content

**Dataset:** Dataset 2, Fluid Mixing Station.

**Problem:** The line's existing color check does not reliably determine whether a vial matches its intended recipe, and residual fluid can affect the first vial after a recipe change. Team V proposes a recipe-aware assistant that combines synchronized camera observations, synthetic-trained perception, and an explicit process model to detect mismatches and forecast risky changeovers. A stretch goal is to recommend lower-risk orders for queued recipes, tested only with plant-manager approval.

**Additional data/support:** Request timestamp-aligned recipe/dispense events, current tank identities, permitted read-only live camera input, repeated recipe transitions with independently reviewed outcome labels, and a separate reserved final run. Ask for counterexamples: repeated same-recipe fills, clean transitions, and naturally occurring fill problems unrelated to carryover. Confirm whether a supervisor-led recipe-order experiment is feasible, and define initial residual-state/reset conditions with the plant manager.

**Visits/tests:** Request a short observation and input-integration visit in week one, an observation-mode pilot by October 21, and a frozen final test October 26–29. These are requested dates, not bookings. The test must run without changing plant controls.

**Compute estimate:** Provisional 40 hours of L40S rendering at the guide's approximately $3/hour plus 20 hours of training/inference at approximately $2/hour: about $160. Request about $200 equivalent in Brev credits as a planning allowance, with actual credit units, displayed rates, storage, and allocation confirmed by organizers. Revise after a small rendering benchmark. Request shared Spark/GB10 inference access if available. Begin optional reasoner tests with the free endpoint. Do not launch paid compute until team access and budget are established.

**Mentors:** Process/controls mentor for mixing and recipe semantics; Isaac Sim mentor for faithful synthetic data; vision mentor for transparent liquids and transfer to real imagery; CSI supervisor for permitted inputs and controlled tests.

**Agent tools:** Inventory teammates' existing access first. Request specific users and plans only where free tools or current subscriptions leave a concrete capacity gap; do not invent a need for eight paid seats.

## Business case and judge evidence

Measure defective/rejected vials per transition, false rejects, operator review burden, latency, and setup effort for a new recipe. Ask a mentor for actual or explicitly illustrative cost inputs. Express economics as attributable scrap/rework reduction minus compute, installation, maintenance, and review costs; do not double-count saved scrap and added throughput.

For other manufacturers, discuss shared dispensing paths, changing tank contents, new recipes, camera variation, and the commissioning cost of building the twin. Show what must be recalibrated and where the solution will not transfer. Avoid claiming pharmaceutical qualification or general industrial readiness from a water-based demonstration.

The final evidence package should include a hardware recording, fixed test manifests, baseline/ablation table, all-attempt trial log, a few concrete failures, code/model release identifiers, synthetic generation provenance, timing measurements, and a business estimate with its assumptions.

## Immediate organizer questions

1. What real-data uses are allowed for calibration, threshold selection, process-parameter estimation, development evaluation, and prompt examples?
2. Is there an organizer-held-out set or can a final physical run be reserved?
3. Can we receive recipe events and synchronized cameras during a live observation-mode test without plant-control changes?
4. Can the plant manager run a controlled comparison of approved recipe sequences, with documented initial residual state?
5. What is the earliest supervised pilot slot and how are final test slots allocated?
6. Does the supplied twin include carryover dynamics, or should we explicitly add a process surrogate?
7. What are the final judging weights, permitted team size, and accepted submission-repository requirements?

Requests are drafts only; no messages, resource purchases, test bookings, or submissions have been sent.

Sources and verified competition facts are recorded in [competition-brief.md](competition-brief.md).
