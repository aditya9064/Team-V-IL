# Results (as of Oct 18, 2026)

Everything below is measured on **synthetic / digital-twin data**. Nothing has run on the real arm yet.
Raw files: [`results/`](../results). Training curves and plots per run: `rosie_outputs/runs/detect/<run>/`.

## Models

| Name in results | Architecture | Training data | File |
| --- | --- | --- | --- |
| `cubes_all` = **v1** | YOLO11m, imgsz 1664 | v1 easy scenes, 7,274 train imgs | `models/cubes_v1.pt` |
| `cubes_v2` = **v2** | YOLO11m, imgsz 1664, 120 epochs | v2 hard scenes, 16,982 train imgs | weights on Rosie (DATA.md) |
| `cubes_v2snap_1016_e102` | v2 at epoch 102 | same | **`models/cubes_v2.pt`** (main model; optimizer stripped, detections identical) |
| `cubes_v1v2` | YOLO11m | v1 + v2 | weights on Rosie |
| `cubes_v2s` | **YOLO11s** (19 MB), 60 epochs | v2 | weights on Rosie |

Classes: `cube_red, cube_yellow, cube_blue, cap, vial`.

## 1. Detection vs the organizers' baseline — `results/BASELINE.txt`

300 held-out v2 + v1 images never used in training, IoU ≥ 0.5 (an 80-image set in the same file agrees).

| Detector | Precision | Recall | F1 | s / image | F1 table | F1 side | F1 wrist |
| --- | --- | --- | --- | --- | --- | --- | --- |
| YOLO v2 | 0.993 | 0.998 | **0.995** | 0.14 | 0.996 | 0.997 | 0.981 |
| YOLO v2 snapshot (`models/cubes_v2.pt`) | 0.993 | 0.997 | **0.995** | 0.06 | 0.996 | 0.997 | 0.976 |
| YOLO v1v2 | 0.991 | 0.997 | 0.994 | 0.13 | 0.996 | 0.994 | 0.973 |
| YOLO v2s (small) | 0.988 | 0.997 | 0.993 | 0.04 | 0.996 | 0.991 | 0.971 |
| YOLO v1 | 0.614 | 0.783 | 0.689 | 0.06 | 0.753 | 0.482 | 0.514 |
| Cosmos3-Nano (starter prompt) | 0.128 | 0.073 | **0.093** | 50.18 | 0.091 | 0.052 | 0.232 |

Cosmos ran on the GB10; YOLO timings include model load warm-up differences between runs.

## 2. Training (synthetic validation) — `results/RESULTS_v1.txt`, `results/RESULTS_V2.txt`

| Run | Train images | mAP50 | mAP50-95 |
| --- | --- | --- | --- |
| cubes_1000 | 857 | 0.995 | 0.993 |
| cubes_5000 | 4,255 | 0.995 | 0.995 |
| cubes_all (v1) | 7,274 | 0.995 | 0.995 |
| cubes_v2 | 16,982 | 0.995 | 0.985 |

Cross-evaluation (mAP50-95): v1 scores 0.995 on easy scenes but **0.728** on the hard v2 scenes;
v2 and v1v2 score 0.995 / 0.985. Easy-scene data saturates after ~1k images — scene variety, not volume, mattered.

## 3. Pose accuracy per camera — `results/PERCEPTION.txt`

Full perception (YOLO + `refine_pose`) against twin ground truth; colour measured on single, non-stacked cubes.

| v2 snapshot | Table | Side | Wrist |
| --- | --- | --- | --- |
| Cube recall | 0.977 | 0.823 | 0.967 |
| Cube precision | 0.985 | 0.855 | 0.958 |
| Colour accuracy | 0.985 | 1.0 | 0.984 |
| Position error, median / p90 | 0.24 / 1.02 mm | 0.80 / 4.97 mm | 0.27 / 2.70 mm |
| Yaw error, median / p90 | 1.39 / 4.70° | 4.82 / 14.36° | 1.64 / 6.61° |
| **Pose-ready share** (planner accepts) | **0.698** | 0.561 | 0.158 |

v1 wrist recall was 0.523 → v2 0.967. **Weak spot:** only ~70% of table-camera cubes are "pose ready";
the planner refuses the rest (`localisation_uncertain`, `cube_pose_missing`), mostly touching/crowded cubes.

## 4. Robot in simulation — `results/SIM_RESULTS.txt`

Full loop in Isaac Sim (capture → YOLO → plan → pick/place), one random layout per seed, six-cube tower task.

| Model | Episodes | Towers complete | Picks | Placements | Position error median / p90 | Yaw error median / p90 | Time / tower |
| --- | --- | --- | --- | --- | --- | --- | --- |
| v1 | 20 | 20/20 | 120/120 | 120/120 | 0.21 / 0.32 mm | 1.35 / 2.17° | 55.2 s |
| v2 | 20 | 20/20 | 120/120 | 120/120 | 0.21 / 0.32 mm | 1.35 / 2.11° | 55.2 s |
| v2 snapshot | 60 | 60/60 | 360/360 | 360/360 | 0.21 / 0.31 mm | 1.35 / 2.25° | 55.2 s |

Per-episode JSON: `rosie_outputs/runs/sim/`. Videos: `videos/`.

## 5. Cosmos Transfer 2.5 (photo-realism) — `brev_outputs/FINDINGS.txt`

| Setting | Cube colour kept | Gate pass |
| --- | --- | --- |
| Trial 1: edge control only, colour words in prompt | 26% | 0/20 |
| White balance + edge 1.0 + vis 1.0, sigma_max 80, neutral prompt (**recipe**) | 100% | 4/6 |
| Batch #1, Brev 2× A100 (2,164 imgs) | 99.0% | 1,805 (83%; table 96%, wrist 96%, side 35%) |
| Batch #2, GB10 (1,000 imgs) | 99% | 947 (95%) |

Side-camera gate failures are false alarms (yellow robot arm re-shaded, no cube changed colour); the gate is kept strict.
**2,752 gated images** are ready for detector v3 (not used yet). Before/after samples:
`brev_outputs/batch/samples_before_after/`, `gb10_outputs/transfer/batch2_out/samples/`.

## 6. Rendering test — `rtx_test/`

| Renderer | Speed | Look |
| --- | --- | --- |
| T4 real-time (Rosie) | 5.5 s / frame | grainy → fixed with median3 denoise |
| T4 path tracing | 44 s / frame | lags one scene behind labels (caught by the label check) |
| L40S real-time (Brev) | 0.3 s / frame | clean, blotchy denoise patches |
| L40S path tracing, 64 spp | ~35 s / frame | photo-like, labels aligned |

## 7. First real photos (phone, CSI, Oct 17) — `real_photos/csi_1017/`

YOLO v2 found the real red and blue cubes in the wide shot (confidence 0.94 / 0.93) and ignored the
coloured bins; it missed tiny or far-away cubes in odd views. These are phone photos, not the cell cameras.
