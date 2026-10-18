# Running steps 2–7 on ROSIE

## Quick start: one command, fully automatic

```bash
cd ~ && tar xzf AdaptiveVision_rosie.tar.gz && cd fanuc_er4ia_sim
nano rosie/config.env        # optional: set MAIL_USER to your email for fail/finish emails
./rosie/run_all.sh           # submits every stage as a chained Slurm job; then log out
```

It runs: setup → container (if missing) → twin test → 20 test images → **automatic label check**
→ 10,000 images on T4s → split (1k / 5k / all) → training on H100 (all, 1k, 5k) → `~/adaptivevision/RESULTS.txt`.
A failed stage cancels everything after it, so a bad label check never wastes GPU hours.
If the container is not built yet and no pull is running, `export NGC_API_KEY=...` first.
Rerunning is safe: finished images are kept and generation resumes.

The sections below run the same stages one at a time, for debugging.

Everything runs from the Rosie shell (dashboard → **Clusters → Rosie Shell Access**).
Heavy work goes to GPU nodes via Slurm. The login node only sets things up and submits jobs.

## 0. Upload and unpack (once)

Upload `AdaptiveVision_rosie.tar.gz` with the dashboard's **Files** app, then:

```bash
tar xzf AdaptiveVision_rosie.tar.gz && cd fanuc_er4ia_sim
./rosie/av.sh check
```

`rosie/config.env` is already filled from Rosie's `check` output (2026-10-15):

| Setting | Value | Why |
| --- | --- | --- |
| `ISAAC_PARTITION` / `ISAAC_GRES` | `teaching` / `gpu:t4:1` | the T4 is Rosie's only GPU with RT cores; Isaac can't render on V100/H100 |
| `TRAIN_PARTITION` / `TRAIN_GRES` | `dgxh100` / `gpu:h100:1` | access confirmed; use `teaching` / `gpu:t4:1` if the H100 queue is long |
| `DATA_ROOT` | `~/adaptivevision` | `/data` and `/scratch` are read-only for students |

The T4 is older and slower than the RTX A6000 the starter was validated on. `twin` and `gen-test` show whether it works and how fast.

## 1. Environments and container (once)

```bash
./rosie/av.sh setup                      # conda envs, textures, 48 host tests, YOLO weights
export NGC_API_KEY=...                   # from ngc.nvidia.com → Setup → API key; never save it in a file
./rosie/av.sh pull-isaac                 # ~20 GB Isaac Sim 6.0 image → $DATA_ROOT/containers
```

## 2–3. Twin check (GPU job)

```bash
./rosie/av.sh twin
./rosie/av.sh status                     # wait until the job leaves the queue
```

Look at `$DATA_ROOT/runs/twin_<jobid>/tablecam` in the Files app. You should see the cell with the cubes on the table.
If the log (`$DATA_ROOT/logs/av-isaac-*.out`) says the robot asset can't be fetched, the GPU node has no internet:
download the ER-4iA asset on the login node and set `FANUC_ROBOT_USD` in `config.env`.

## 4–5. Synthetic images + labels

```bash
./rosie/av.sh gen-test                   # 20 frames; the log prints seconds per frame
./rosie/av.sh preview                    # draws labels into $DATA_ROOT/synth/chunk_0000/preview
```

**Open the preview images before going further.** Every box must sit on its object.
Then size the full run from the measured speed: `frames per chunk × seconds per frame` must fit in `GEN_TIME`.

```bash
./rosie/av.sh gen                        # NUM_CHUNKS × CHUNK_SIZE frames as a job array
./rosie/av.sh status                     # per-chunk frame counts and disk use
```

Chunks resume: resubmitting `gen` skips frames that already exist.

## 6. Split

```bash
./rosie/av.sh split                      # all frames      → datasets/cubes_all
./rosie/av.sh split 1000                 # first 1,000     → datasets/cubes_1000
./rosie/av.sh split 5000                 # first 5,000     → datasets/cubes_5000
```

Smaller sets are nested inside larger ones with the same train/val assignment, so results are comparable.

## 7. Train

```bash
./rosie/av.sh train cubes_1000 --epochs 60     # quick first model
./rosie/av.sh train cubes_all
```

Results: `$DATA_ROOT/runs/detect/<name>/` — `summary.json` (mAP per class), `results.csv`, plots, `weights/best.pt`.

## Data-scaling experiment (for the presentation)

Train on `cubes_1000`, `cubes_5000` and `cubes_all`, then plot validation mAP against frame count.
If the curve is flat between 5,000 and 10,000, more synthetic data won't help. Effort should go into realism instead:
lighting, materials and camera noise matched to real photos.

## Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| `sbatch: error: invalid partition` | `ISAAC_PARTITION` / `TRAIN_PARTITION` in config.env |
| Isaac log: `No device could be created` / Vulkan errors | GPU without RT cores, or a driver too old for Isaac Sim 6.0 |
| `pull-isaac`: unauthorized | `NGC_API_KEY` missing or expired |
| Job ends at once, log mentions EULA | container env vars; check `isaac_exec` in `common.sh` |
| Training can't download weights | rerun `./rosie/av.sh setup` on the login node |
