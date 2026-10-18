# Trained detectors

| File | What | Notes |
| --- | --- | --- |
| `cubes_v2.pt` | **Main model.** YOLO11m, imgsz 1664, trained on the v2 synthetic dataset (Rosie run `cubes_v2snap_1016_e102`, epoch 102) | Optimizer state stripped (121 MB → 41 MB); detections identical to the original. F1 0.995 on 300 held-out twin images. SHA-256 `18a5ad33…a249c9` |
| `cubes_v1.pt` | YOLO11m trained on v1 easy scenes (Rosie run `cubes_all_1016_0059`) | Baseline for "why v2 matters" (F1 0.689 on hard scenes) |

Classes: `0 cube_red, 1 cube_yellow, 2 cube_blue, 3 cap, 4 vial`. Use with `fanuc_er4ia_sim/perceive_yolo.py`
or `real_photo_test.py --weights models/cubes_v2.pt`. Other checkpoints (v2 final, v1v2, v2s small) are on Rosie — see `docs/DATA.md`.
