# Rosie outputs (copied Oct 18, 2026 from `~/adaptivevision`)

| Path | What |
| --- | --- |
| `results_and_scripts/` | Result tables as written on Rosie (BASELINE, PERCEPTION, RESULTS, RESULTS_V2, SIM_RESULTS), `baseline_results.jsonl` (per-image baseline), `TASKS.md` (job board), small test scripts |
| `runs/detect/<run>/` | Ultralytics training output per run: `results.csv`, curves, confusion matrix, batch previews, `args.yaml` (weights excluded — see docs/DATA.md) |
| `runs/perception/` | Perception evaluation per model (JSON + text + log) and overlay frames |
| `runs/sim/` | Per-episode JSON/TXT summaries of the simulated robot runs (images excluded) |
| `runs/eval_v2.*`, `runs/eval_runs/` | Cross-evaluation v1 vs v2 |
| `baseline_set*/` | Lists/labels of the held-out baseline images (images excluded) |
| `gallery/` | Overview images of generated twin scenes |
| `transfer_tools/` | Transfer batch helper scripts as deployed on Rosie |
| `logs.tar.gz` | All Slurm job logs (`tar xzf logs.tar.gz`) |
