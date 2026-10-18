# AdaptiveVision — Team V

Rockwell Automation + NVIDIA **Physical AI Hackathon for the Factory Floor** (UWM CSI + MSOE, Fall 2026),
**Dataset 3: FANUC ER-4iA grasping and sorting.**

A vision-guided pick-and-sort cell: a digital twin in Isaac Sim produces labelled training images,
a YOLO detector finds the cubes from three cameras, a grasp planner turns detections into picks,
and (next) a ManipBench policy drives the real FANUC arm at CSI.

> **This branch (`DEVELOPMENT`) is the full AdaptiveVision project as of Oct 18, 2026**: every script we
> wrote on the Mac, Rosie, the GB10 and Brev, plus the results, logs and samples. The earlier Team V
> kitting work is still on `main`.

## Where we are (Oct 18, 2026)

| Area | State |
| --- | --- |
| Digital twin + synthetic data | **Done.** Isaac Sim 6.0 on Rosie T4 GPUs. v1 = ~8.5k easy frames, v2 = ~20k hard frames (6 scene types, 3 cameras, towers, clutter, occlusion-aware labels, automatic label check). |
| Detector | **Done.** YOLO11m v2, F1 **0.995** on 300 held-out twin images vs **0.093** for the organizers' Cosmos3-Nano baseline, 0.06 s vs 50 s per image. |
| Robot in simulation | **Done.** 100/100 six-cube towers built (v1, v2, v2 snapshot), 0.21 mm / 1.35° median placement error, 55 s per tower. |
| Realism (Cosmos Transfer 2.5) | **Done, not yet used for training.** 2,752 photo-real versions of twin frames pass our colour gate (99% keep cube colour). |
| Real cell (CSI visit Oct 17) | **Learned.** Real robot is driven through **ManipBench**, 3 real runs per team; the scene differs from the starter twin (coloured bins, rounded cubes, pencil lines). |
| Real-robot policy (ManipBench adapter) | **Not started.** Critical path. Needs `fanuc_er4ia_cell_client.zip` from organizers. |
| Twin v3 matching the real cell | **Not started.** |
| Video / 1-page summary | **Not started.** Due **Nov 1, 11:59 PM CT**. Finals Nov 5. |

Full status, the real-cell findings, the product idea and the task list: **[docs/HANDOFF.md](docs/HANDOFF.md)**.

## Quick start

```bash
git clone -b DEVELOPMENT https://github.com/aditya9064/Team-V-IL.git
cd Team-V-IL

# 1. Unit tests (any laptop, no GPU) — 48 tests
cd fanuc_er4ia_sim
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m unittest test_sort_planning test_cube_perception test_cube_pose test_runtime_config synth.test_synth

# 2. Detector tests + try the trained model on a photo (needs ultralytics)
pip install ultralytics
python -m unittest test_perceive_yolo test_real_photo_test          # 13 tests
python real_photo_test.py ../csi_kit/photos/example_sim_1080p.jpg \
    --weights ../models/cubes_v2.pt --camera table --out /tmp/av_test
```

Isaac Sim, training and Cosmos need a GPU machine — see [docs/COMPUTE.md](docs/COMPUTE.md) (Rosie, GB10, Brev).

## Repository map

| Folder | What it is |
| --- | --- |
| [`fanuc_er4ia_sim/`](fanuc_er4ia_sim) | **Main code.** Organizer starter kit v0.5 + our additions: `perceive_yolo.py` (YOLO perception), `real_photo_test.py`, `synth/` (v2 scene generator, labels, training, evaluation), `rosie/` (Slurm pipeline and job scripts). Rules for coding agents: `AGENTS.md`. |
| [`tools/`](tools) | Baselines (`run_baseline.py`, `baseline_metrics.py`), status mail, and `transfer_trial/` (Cosmos Transfer: white balance, batch builder, colour gate, Brev/GB10 runners). |
| [`csi_kit/`](csi_kit) | Offline real-photo test kit used at CSI (`run_test.sh`, checklist). |
| [`models/`](models) | Trained detectors: `cubes_v2.pt` (main, 41 MB) and `cubes_v1.pt`. |
| [`results/`](results) | Final result tables (baseline, perception, sim robot, training). Start here for numbers. |
| [`rosie_outputs/`](rosie_outputs) | Everything Rosie produced except bulk images: result files, training curves/plots, perception and sim summaries, all job logs (`logs.tar.gz`). |
| [`gb10_outputs/`](gb10_outputs) | GB10 scripts and logs: Cosmos3-Nano baseline, Cosmos Transfer sweeps and batch #2 (QC, samples). |
| [`brev_outputs/`](brev_outputs) | Brev A100 Transfer trial #1 and batch #1: findings, logs, QC, before/after samples. |
| [`rtx_test/`](rtx_test) | Render-quality test (T4 vs L40S real-time vs path-traced). |
| [`sim_eval_mac/`](sim_eval_mac) | Early perception/sim evaluation JSON from the Mac. |
| [`data_samples/`](data_samples) | 45 sample frames + labels + metadata from the v1 and v2 synthetic datasets. |
| [`real_photos/`](real_photos) | Phone photos of the real cell from the CSI visit (Oct 17) and YOLO overlays. |
| [`videos/`](videos) | Simulated robot sorting videos (email size) and posters. |
| [`organizer/`](organizer) | Organizer starter kit `fanuc_er4ia_hackathon_starter_v05.zip` and the explainer pages. |
| [`docs/`](docs) | Handoff, results, compute runbooks, data locations, factory research, project plan. |

Large data that cannot live in Git (synthetic datasets, full-size videos, all Transfer images,
model checkpoints) is listed with exact locations in **[docs/DATA.md](docs/DATA.md)**.

## Results at a glance

All numbers are on **synthetic / digital-twin data**; nothing has run on the real arm yet.
Details: [docs/RESULTS.md](docs/RESULTS.md).

| Detector (300 held-out twin images, IoU ≥ 0.5) | Precision | Recall | F1 | s / image |
| --- | --- | --- | --- | --- |
| **YOLO v2** (ours, `models/cubes_v2.pt`) | 0.993 | 0.997 | **0.995** | 0.06 |
| YOLO v2s (yolo11s, 19 MB) | 0.988 | 0.997 | 0.993 | 0.04 |
| YOLO v1 (easy scenes only) | 0.614 | 0.783 | 0.689 | 0.06 |
| Cosmos3-Nano (starter-kit baseline) | 0.128 | 0.073 | 0.093 | 50.2 |

| Robot in Isaac Sim | Towers complete | Picks / placements | Position error (median) | Yaw error (median) |
| --- | --- | --- | --- | --- |
| YOLO v1 | 20 / 20 | 120 / 120 | 0.21 mm | 1.35° |
| YOLO v2 | 20 / 20 | 120 / 120 | 0.21 mm | 1.35° |
| YOLO v2 snapshot | 60 / 60 | 360 / 360 | 0.21 mm | 1.35° |

Known weakness: the grasp planner refuses ~30% of detected cubes as "uncertain" (crowded or
touching cubes) — safe, but cubes are left behind.

## Rules every teammate must follow

- **No commands to the physical arm and no controller/PLC changes** without the plant manager and our
  assigned mentor. Real runs go only through ManipBench. Don't move or re-aim the cell cameras.
- Real cell data is for **looking and testing only** — training data must come from the digital twin.
- Never use simulator ground truth to choose grasps; never weaken a test or quality gate to make it pass (`fanuc_er4ia_sim/AGENTS.md`).
- Shared machines (Rosie, GB10): stay in your own folders and job limits; don't look at other teams' work.
- **Never commit secrets** (NGC key, Hugging Face token, passwords, SSH keys). Delete Brev machines when done — they bill while running.

## Key dates and contacts

- Final submission **Nov 1, 11:59 PM CT**: ~5-min video, 1-page summary, repo link, credits. Finals **Nov 5**, 5–8 PM, MSOE Diercks Hall.
- Slack: `#dataset-3-grasping-sorting` (Dataset 3, Zhen Zeng), `#data-and-testbed-requests` (robot time and the ManipBench client, Shamar Webster).
- Event page: https://uwm.edu/csi/?p=4080 · Real-robot interface: https://manipbench.com
