# Persistent capture pilot

First version written October14. Host tests check known starter compatibility and reject labels that disagree with saved stage state. Rosie GPU pilots283767(3scenes) and283770(10scenes) passed on October14. All60 cube patches in the ten-scene batch were visually inspected; cameras, labels, stage positions and repeated-seed metadata passed offline checks. Ten-scene elapsed4m38s, mean sample23.757s after startup. This demonstrates ten-scene lifecycle operation, not unlimited stability, real-arm transfer or a fair cross-GPU speed comparison. Evidence: benchmark-results/rosie-persistent-20261014/REPORT.md.

One application is created for the batch. Each scene stops/clears the previous world, pauses its camera, creates a new stage, rebuilds the original cell and executes the unchanged capture logic apart from lifecycle/lighting adaptation. It retains six known-size cubes, fixed yellow positions and distractors, fresh seeded yaw/light variation, 4K camera and 90 render frames. It preserves starter code and refuses an incompatible capture script. This first pilot amortizes application startup, not stage-building cost. Full dataset diversification and visibility annotations remain later work.

## Pilot on the working Rosie environment

Upload persistent_capture.py and persistent_pilot.sh to `$HOME/team-v-tests` through authorized SSH/SCP (now working) or the Files route. Do not bypass the previous browser terminal security restriction. Confirm actual paths before running, then submit the default three-scene lifecycle test:

```bash
mkdir -p "$HOME/team-v-logs"
sbatch --output="$HOME/team-v-logs/persistent-%j.log" \
  "$HOME/team-v-tests/persistent_pilot.sh"
```

Require the report's complete=true, valid_samples=3 and `TEAM_V_PERSISTENT_PILOT_PASS`, not just Slurm exit zero. Inspect logs for camera/reset/Physics errors and images at full resolution. Check distinct layouts and identical camera calibration across scenes. Ground truth/state only validate generation, not runtime targeting.

If the three-scene reset/quality checks pass, run the ten-scene pilot:

```bash
sbatch --export=ALL,PILOT_COUNT=10 \
  --output="$HOME/team-v-logs/persistent-%j.log" \
  "$HOME/team-v-tests/persistent_pilot.sh"
```

Each job saves under `$HOME/team-v-samples/persistent-pilot-JOBID`, with per-scene artifacts and batch_report.json. Startup, per-scene reset/capture/validation and total time are separate. This uses one T4, 4 CPUs, 32 GB, 20-minute scheduler ceiling. GPU validation and comparison with the original process-per-scene pipeline are required before a larger batch.

## Work after the pilot

1. Compare these same seeds with original captures and verify label/camera agreement.
2. If stage reset or sensor cleanup fails, repair lifecycle handling rather than dropping quality gates.
3. Profile startup, stage build, warmup, rendering and export before optimizing more.
4. Add full cube-layout/count and distractor variations with valid workspace/reachability checks.
5. Audit visible labels and freeze a first training package for the mentor.

Brev remains stopped. Do not restart it for an untested bulk job. Its existing headless container works but the launcher streaming startup was terminated during the earlier benchmark; inspect competing processes and version differences before another paid pilot.
