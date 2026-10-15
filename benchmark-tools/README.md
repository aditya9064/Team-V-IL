# Limited Rosie and Brev image-generation benchmark

Prepared October 14, 2026. No paid machine launched and no host benchmark run yet. The organizer/user confirms $500 allocated; current browser account cannot inspect Billing because it lacks admin/owner visibility. Assigned organization: https://brev.nvidia.com/org/org-3KTRLJbtGxwXejypzHKCcBGcFow/environments .

## Protocol and spending limit

Run the SAME starter, capture script, seeds 424201–424203, table camera at 3840x2160 RGB, scene settings and 90 render warmup frames on both hosts. This first small benchmark measures the CURRENT process-per-scene pipeline, with three sequential launches. It does not claim persistent-loop speed. Report each sample separately because asset/shader caches and first startup differ. Do not clear global/shared caches. Keep the original images for visual comparison; labels/JSON validation alone is insufficient.

Observed AWS price: RTX PRO 4500 $3.02/hour, RTX PRO 6000 $4.04/hour. Prefer one 4500 for the low-cost pilot if the Isaac launchable supports it. Rates/availability may change; inspect final configuration. Never launch multiple instances or the $17/hour Cosmos video launchable for this task.

Target total spend at most $2 for this pilot, including provisioning, setup, idle compute and storage. At $3.02/hour, 30 minutes is about $1.51 compute; at $4.04/hour, 20 minutes is about $1.35. These are estimates, not a billing guarantee or automatic cap. Confirm billing increments/storage and start/stop support at deployment. The benchmark script has a 15-minute aggregate execution timeout. It kills capture processes on timeout and writes failure evidence, BUT DOES NOT stop cloud compute. Stop the instance immediately after the run or any setup failure. Arrange an operator/independent stop timer before launching; do not start a machine that will wait for someone to prepare code. If setup risks exceeding the pilot envelope, stop and resolve locally. Back up results before eventual deletion.

## Files

- benchmark_capture.py: 3 captures, timeout, GPU/source metadata, artifact and label checks, benchmark.json.
- capture_dataset.py: unchanged copy of the existing varied-scene driver.
- rosie_benchmark.sh: 1 T4 allocation, 4 CPUs, 32 GB, 20-minute Slurm ceiling.
- compare_benchmarks.py: refuses incomplete/mismatched sources before computing timing ratio.

Needs an extracted unchanged v05 starter, its dependencies/assets, an Isaac-compatible interpreter, and Pillow in the benchmark controller environment. Use the provided README/SETUP; do not reinstall the working Rosie environment. Brev launchable must match Isaac version/settings or results are a comparison of different environments, not isolated GPU speed. Obtain the same starter through the supplied authorized file source. Do not send datasets, credentials or this private handoff to a public launchable.

## Rosie setup and run

Upload this benchmark folder through the approved Rosie Files interface to `$HOME/team-v-benchmark`. Direct terminal automation was previously blocked; do not bypass it. The participant can run:

```bash
mkdir -p "$HOME/team-v-benchmarks" "$HOME/team-v-logs"
sbatch --output="$HOME/team-v-logs/benchmark-%j.log" \
  "$HOME/team-v-benchmark/rosie_benchmark.sh"
```

After it finishes, inspect `benchmark.json`, all three seed logs/images and Slurm accounting. Success needs `complete: true` and three valid samples, not merely exit code zero. Paths assume the original participant's working environment; teammates adapt them.

## Brev preparation and run

Before paid launch, have the ZIP/scripts ready, account access, connection method, stop control and timer. Choose the Isaac Sim/Isaac Lab launchable from MAIC and check GPU/rate/start-stop/version. Existing organization Compute showed no resources at inspection. No environment has been deployed by this preparation.

Connect using the exact instance instructions. Find the actual Isaac-compatible Python or python.sh path from the environment documentation; do not assume the Rosie virtual environment works on Brev. Confirm Isaac 6.0.0.0, Linux x86_64, working camera/render dependencies and matching starter content. Record software versions and storage settings in the results. Do not download huge extra models or train during this rendering test.

Run with real paths (replace the shell variables below with discovered paths):

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
"$BENCHMARK_PYTHON" "$BENCHMARK_DIR/benchmark_capture.py" \
  --python "$ISAAC_PYTHON" \
  --starter "$STARTER_DIR" \
  --out "$RESULTS_DIR" \
  --host-label brev-rtx-pro-4500 \
  --rate-per-hour 3.02
```

Use the actual rate if it changed. `BENCHMARK_PYTHON` needs Pillow; `ISAAC_PYTHON` must be an executable accepted by the starter. Record instance start/stop/billing timestamps separately. Stop in the Brev console after capture; ending a Python process does not stop the instance. Keep paid setup time visible, not hidden in a speed claim.

## Compare and decide

Download each benchmark.json and the three images to authorized local storage, then:

```bash
python3 compare_benchmarks.py /actual/rosie/benchmark.json /actual/brev/benchmark.json
```

Review scene/camera agreement, full-resolution objects, rendering artifacts, valid image counts, sample wall time, script-level throughput, first-run versus later-run times, billable total and cost per valid scene. The script-cost field excludes provisioning/setup/idle/storage. Rosie cost zero in this report means no Brev credit charge, not free institutional compute.

This pilot can show whether the current capture pipeline differs across hosts. Three samples do not establish stable long-run throughput. Persistent capture is the next optimization to validate separately; compare persistent mode against persistent mode on both hosts before extrapolating thousands of images. No training starts from this benchmark.
