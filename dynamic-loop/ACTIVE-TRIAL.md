# Validation complete — October 15, 2026

Job 284300 completed all four current cases: nominal-six (six verified pick/place outcomes), miss_once (one confirmed-empty attempt then successful retry), miss_twice (two confirmed-empty attempts then HOLD, no third attempt), and moved (REPLAN then verified pick/place). Independent scoring agrees for all successful slots. All 87 accepted captures passed the strict raw-speed/timestamp/fit audit. Full source and report hashes are recorded in validation-results.json.

Earlier empty/missing/stale gate cases from 284267 are retained with explicit provenance. All failed diagnostic/run directories are preserved. See validation-report.md for evidence, unchanged thresholds, tests and remaining limits. No GPU job is needed for this completed validation. No physical robot testing is claimed.
