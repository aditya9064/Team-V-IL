# Synthetic data → first detector

Steps 2–7 of the team plan, built on the organizer starter (v0.5).
Run every command from the `fanuc_er4ia_sim/` root. **On ROSIE use `rosie/av.sh` instead; see `rosie/README.md`.**

| Step | Command / file | Where it runs | Status |
| --- | --- | --- | --- |
| 2. Dataset 3 files | Starter v0.5: `README.md`, `docs/`, `assets/` | — | Reviewed |
| 3. Run the twin | `"$ISAAC/python.sh" build_scene.py --headless --shoot runs/shoot` | Rosie T4 (Isaac Sim 6.0) | Done (Oct 15) |
| 4. Synthetic images | v1 `synth/generate_synthetic.py`, v2 `synth/generate_v2.py` | Rosie T4 | Done: v1 ~8.5k, v2 ~20k frames |
| 5. Labels | `synth/labels.py`, written alongside each image | (inside step 4) | Host-tested |
| 6. Train/val split | `synth/split_dataset.py` | Any machine | Host-tested |
| 7. Train | `synth/train_detector.py` (`rosie/av.sh train` on ROSIE) | Rosie H100 | Done: see `results/` |

## What gets randomised (`synth/layout.py`)

- Cube count and colour per frame (each of 6 slots present with p = 0.85, colour red/yellow/blue)
- Position and yaw of cubes, caps and tubes over x 0.20–0.50 m, y −0.25–0.32 m, with no footprint overlap
- Dome and three rect-light intensities (×0.4–1.6) and colour tint (±8 %)
- Table-camera pose jitter (σ 4 mm eye, 3 mm target), standing in for calibration error

The 4K table render is cropped to a fixed 1664×896 workspace window, where a cube spans about 25–40 px.
Labels are boxes projected from the poses the script authored.

## Run it

```bash
# 3. twin sanity check: one frame from each camera
"$ISAAC/python.sh" build_scene.py --headless --shoot runs/shoot

# 4–5. images + labels; chunks may run on different machines and resume
"$ISAAC/python.sh" synth/generate_synthetic.py --out synth_out/a --count 20      # look at these first
"$ISAAC/python.sh" synth/generate_synthetic.py --out synth_out/a --count 1500

# 6. split (symlinks; add --copy before moving the dataset to another machine)
.venv/bin/python synth/split_dataset.py synth_out/a --out datasets/cubes_v1

# 7. train
python -m venv .venv-train && .venv-train/bin/pip install -r synth/requirements-train.txt
.venv-train/bin/python synth/train_detector.py --data datasets/cubes_v1/data.yaml --name cubes_v1
# on ROSIE: ./rosie/av.sh train cubes_v1   (see rosie/README.md)
```

Before the full 1500 frames, open the 20 test images and draw a few labels over them to check alignment.
A box offset error here would silently corrupt every label.

## Tests

```bash
.venv/bin/python -m unittest synth.test_synth
```

## Rules this follows

- Training images come only from the twin. Real line photos are for testing only (event rules).
- Twin ground truth produces training labels only. It never commands a grasp (`AGENTS.md`).
- Starter files are unchanged; everything new lives in `synth/`.

## Known limits

- Labels ignore occlusion by taller neighbours (caps, tubes) in the oblique view.
- Only the table camera is generated; side and wrist views are not.
- The arm is parked out of view, so it never occludes objects in training images.
- Lighting randomisation scales the existing rig; it does not model the real room's lights.
