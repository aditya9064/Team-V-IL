# Model plug-in and additional failure trials

Completed October 15, 2026. Five new situation tests and the six-part regression passed their exact simulation acceptance checks, using explicit evidence from jobs **284313** and **284319**. The failed first arrival fixture is retained and explained. The overall 284313 batch exited nonzero because of that fixture; its replacement passed in a fresh isolated run.

Read [the results and limits](additional-validation-report.md), [machine-readable audit](additional-validation-results.json), and [model handoff instructions](MODEL-PLUGIN.md). Every motion used the current geometric detector. No trained checkpoint or physical-arm result is claimed.

The completed suite used one teaching T4 for 27m34s; the arrival-only rerun used one teaching T4 for 3m27s. Allocations did not overlap. No H100 or Brev jobs were submitted. Both isolated source snapshots, original logs, RGB and reports remain on Rosie. No further GPU job is needed for these cases.
