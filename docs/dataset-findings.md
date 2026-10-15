# Dataset findings and project decision

Inspected October 8, 2026. Sources: the supplied dataset and twin archives in the [shared Drive folder](https://drive.google.com/drive/folders/1GWuHV2WUNLztq_Fga6xBH7qFzhxDaac8), their READMEs, camera JSONs, rendering source, recipe CSV/JSON, image indexes, and selected images. This is an initial feasibility review, not a model evaluation.

## What is actually available

- Both archives are downloaded and extracted locally. Dataset: 709,560,806 compressed bytes, 726,440,519 uncompressed bytes. Twin: 212,620,610 compressed bytes, 748,598,654 uncompressed bytes.
- The dataset has 15 MP4 files across three cameras and five runs, and 4,647 indexed JPEG images. Every indexed image and expected video filename exists; no duplicate indexed filenames were found.
- The video manifest describes 64 vials in total, of which 30 have supplied recipe/timing labels. The 30 labeled vials occupy one run, with 10 recipes and three vials each.
- There are 845 frame-label rows: 509 inside labeled fill intervals and 336 outside them. These are correlated frames, not 845 independent examples.
- Per-vial labels contain recipe, intended component volumes, dispense order, and fill timing. They do not include an explicit quality verdict or verified carryover cause. Additional independently reviewed outcome labels are necessary to measure defect detection and explanation correctness.
- All supplied per-vial rows pass the audit's component-sum, stated recipe-volume, duration, interval-overlap, and frame/time checks. These checks do not establish that labels align visually with every event.
- Cap/seal still-image coverage for run 1 is sparse: five assigned images, plus one image marked unassigned. Tank and fill views each have 558 images for run 1. Use continuous video where sampled still coverage is insufficient.

Machine-readable structural audit: [dataset-audit.json](dataset-audit.json). Audit code: [audit_dataset.py](../scripts/audit_dataset.py).

## Twin capabilities

The twin README explicitly states there is no physics. It has static geometry and a prerecorded fill animation, spanning frames 1–686 at 30 fps. Glass and liquid materials are approximations. It provides a fill station and tray inspection station, two camera configurations, RTX preview images, and an example RGB renderer tested by the provider with Isaac Sim 6.0.

The example renderer opens a stage, selects the camera and animation time, captures RGB, and applies cap/seal lens correction where needed. It is not a configurable recipe simulator, carryover simulator, randomized labeled-data generator, or robotic control policy. Fill parts have custom synthetic label attributes, but a training generator still needs to map those attributes into usable annotations and vary meaningful states.

Team V has not yet run Isaac Sim or validated renders after changing materials or geometry. A camera-matched scene is valuable for synthetic perception training, but it is not evidence that a process model predicts physical mixtures.

## Visual and timing observations

A real overhead tray image and the supplied fill-station render were inspected. Three sampled fill images near the labeled end times for vials 7–9 were also opened. One chosen image had no visible vial under the nozzle, so event boundaries and offsets need video-level spot checks before extracting "finished-vial" examples. This observation does not establish that the supplied labels are wrong.

Synchronizing clocks does not establish vial identity between the fill station and downstream tray inspection: transport and placement occur later. Cross-station association needs event/placement context, not only nearest timestamps.

## Recommendation

This paragraph records the initial data-based assessment. The later discussion favors Dataset 3 with a conditional Dataset 1 extension; see the current [project strategy](project-strategy.md). The vial dataset findings below remain relevant to the original Dataset 2 option.

The initial recommendation favored Dataset 2, conditional on the first-week gates in the [mixing option](mixing-option.md). It offers a concrete path from recipe-aware inspection to forecasting and potentially preventing waste. The available recipe history, volumes, synchronized views, and matched fill camera support that investigation.

The ambitious element is an experimentally supported process model and decision aid. It must beat simple color inspection and the rule that flags every first vial after a recipe change. Adding a large reasoner alone does not establish that contribution.

Before committing by October 14, establish usable timing alignment, independently reviewed quality outcomes, an allowed route to process calibration, a synthetic-rendering pilot, and scheduled supervised testing. Make physical sequence intervention a stretch goal until approved. If those gates fail, choose Dataset 1's fixture inspection instead. Dataset 3 becomes attractive if the team demonstrates existing robotics expertise and early confirmed arm/twin access; its assets were not present in the supplied top-level listing during this review.
