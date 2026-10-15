# Team V — mentor detector training pilot v2

This handoff trains a **2D colour-cube detector** for the FANUC small-part kitting project. It predicts boxes and yellow/blue/red classes. It does not command the robot or replace camera calibration, pose refinement, grasp planning, feedback, or the CSI operating procedure.

## What is in this release

78 unique synthetic scenes, 432 cube labels, and three scenes with no cubes. Crops are 1920×864 RGB pixels, copied without resizing from 3840×2160 simulation frames. Each split includes an empty scene and examples with 1, 3, 6, and 8 cubes.

| Split | Images | Cube boxes | Empty images |
|---|---:|---:|---:|
| Train | 56 | 324 | 1 |
| Validation | 11 | 54 | 1 |
| Test | 11 | 54 | 1 |

`images/`, `labels/`, and `manifest.json` are the training inputs. `metadata/` preserves camera parameters, full-frame crop offsets, generation settings, cube pose, and label provenance for later analysis. **The model does not read ground-truth poses or generation settings.** `qa/` contains label overlays and enlarged cube patches. `data_validation.json`, `model_smoke.json`, and `verification/` record the completed release checks.

The text labels use `class cx cy width height`, normalized to the **cropped** image dimensions. Classes are **0 yellow, 1 blue, 2 red**. The dataset adapter maps them to torchvision **1 yellow, 2 blue, 3 red**, reserving 0 for background. Empty label files are intentional negative examples; keep them.

## Label verification and correction

An earlier exporter treated each USD cube origin as its centre. The starter actually places the origin at the bottom face. Those old `synthetic-v1` / `synthetic-crops-v1` boxes are obsolete. This release projects the saved orientation and bottom-origin geometry correctly, then obtains visible RGB component bounds and checks them against the projected silhouette. It rejects low agreement (IoU below 0.85), clipping, overlapping boxes, incorrect counts/colours/poses, and duplicate scenes/images.

Every saved crop is reopened and checked for identical pixel data. Normalized crop labels are decoded and translated back to their full-frame bounds within 0.001 pixels. Crop conversion preserves every accepted cube; it does not drop objects. Source captures are unchanged. See `data_validation.json` for the final human review status and scope.

## Train on the mentor's H100

Use a separate Python 3.12 environment, not the robot's Isaac environment. The following versions match the smoke-tested Rosie stack; choose the appropriate official PyTorch CUDA wheel if your H100 host differs.

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu128
python -m pip install -r training/requirements.txt
python -m pip freeze > mentor-environment.txt
cd training
python -m unittest -v test_data
python train.py --data .. --out ../runs/detector-pilot --epochs 30 --batch-size 2 --workers 4
```

The script fine-tunes the official COCO-pretrained Faster R-CNN ResNet50 FPN v2 weights (about 167 MB downloaded on the first training run). Initial settings use smaller anchors for small objects, retain the crop detail, and apply modest brightness/contrast/saturation variation plus correct horizontal flipping. Hue is preserved because colour is the class. These are starting hyperparameters, not an optimized recipe.

AMP defaults to BF16 on supported CUDA devices (including H100), FP16 otherwise, and disabled on CPU. Set `--amp none` for a full-precision troubleshooting run. The T4 smoke checks model forward/backward and empty targets; H100 BF16 throughput is not benchmarked. Batch size 2 is a conservative starting point; increase only after monitoring memory. Seeds and settings are logged, but GPU results are not promised to be bitwise reproducible.

Training evaluates **validation** after every epoch and saves `best.pt` using validation AP50; `last.pt`, loss history, validation predictions, and all COCO metrics are also saved. It never uses the test split for training or model selection. No resume logic is supplied; each experiment requires a fresh output directory. Outputs can be large because both checkpoints include optimizer state.

After selecting the recipe and best checkpoint, evaluate the held-out test split:

```bash
python evaluate.py --data .. --checkpoint ../runs/detector-pilot/best.pt \
  --split test --out ../runs/test-evaluation
```

Please return `best.pt`, `config.json`, `history.jsonl`, validation metrics, final test metrics/predictions, and `mentor-environment.txt`. Load only checkpoints from this trusted training workflow. `evaluate.py` uses restricted `weights_only=True` loading.

## What to measure

The evaluator reports COCO AP (IoU 0.50–0.95), AP50, AP75, per-colour AP50, legacy/diverse source subsets, and false detections on empty scenes at confidence 0.5. Background hardware, caps, and vials remain unlabeled distractors. Small-object recall and empty-scene errors matter more than a single headline score. Preserve split membership when experimenting.

All scenes use the same simulator/assets. The legacy 60 have repeated yellow positions and fixed distractors; the 18 new scenes add inventory, position/yaw, light, neutral surface, distractor, and limited camera variations. Scenes are unique and groups do not cross splits, but shared rendering and correlated legacy positions make metrics optimistic. The package is a **small pilot for initial fine-tuning and pipeline validation**, not proof of generalization or readiness for the physical arm. No neural model has been trained as part of this handoff.

Only upright, separated 10 mm cubes are covered. Occluded/clipped objects are rejected. This release does not establish grasp reachability, changing-object tracking, arbitrary backgrounds, real camera behaviour, or CSI transfer. More diverse synthetic scenes and a separately held-out real-camera test are still needed before deployment.

## Restore full-frame camera coordinates

The fixed crop rectangle is `[960,1080,2880,1944]`, using exclusive right/bottom bounds. Add **960 to x** and **1080 to y** before using the original camera calibration. `detector_data.crop_to_full` implements this. Full-frame camera parameters remain in each metadata file; do not treat them as crop-camera intrinsics. Detection centre alone is not an exact cube pose; retain the separate geometric pose/refinement stage for grasping.

## Reproduce and inspect

`python training/verify_release.py --data .` checks hashes, image dimensions, all targets, split/group uniqueness, and inventory totals. `training/test_data.py` covers class mapping, empty targets, crop restoration, horizontal flipping, invalid boxes, and COCO perfect/empty prediction scoring. The smoke test runs one reduced-resolution untrained forward/backward pass, with **zero optimizer steps** and no pretrained download. It establishes software compatibility, not model accuracy.

The complete preprocessing source and generation QA helper are in `provenance/`. Their original Rosie paths are recorded for provenance only; use the relative paths above for training. Do not copy any SSH keys, passwords, or account credentials into this package.

Official references: [PyTorch installation versions](https://pytorch.org/get-started/previous-versions/), [torchvision 0.25 Faster R-CNN v2](https://docs.pytorch.org/vision/0.25/models/generated/torchvision.models.detection.fasterrcnn_resnet50_fpn_v2.html), [detection dataset tutorial](https://docs.pytorch.org/tutorials/intermediate/torchvision_tutorial.html), [COCO evaluator](https://github.com/cocodataset/cocoapi).
