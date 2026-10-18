# GB10 outputs (copied Oct 18, 2026 from `spark1:~/adaptivevision`)

- `*.sh`, `*.py`, `*.log` at the top: Cosmos3-Nano smoke tests (including the failed attempts that led to
  `transformers==4.57.1` and the Triton fix) and the Cosmos-vs-YOLO baseline runs (`baseline_ext*.sh`).
- `transfer/`: Cosmos Transfer runner scripts as deployed, the 11-setting sweep (`sweep_out*/`, before/after
  white balance) and **batch #2** (`batch2_out/`: `qc_train.json`, sample outputs with their edge/vis control
  images, all per-image specs in `specs_and_run_json.tar.gz`).
- The HF token file and the 49 GB of model caches are deliberately not included.
