# AdaptiveVision — current tasks (updated Fri Oct 16, 5:00 PM)

Live progress: type `avstatus` in this terminal.

## Running now (automatic, nothing to do)

| # | Task | Where | Expected done | Result file |
|---|------|-------|---------------|-------------|
| 1 | Train YOLO on v2 hard images (`cubes_v2`) | Rosie H100, job 284699 (epoch 100/120) | ~7 PM | |
| 2 | Train YOLO on v1+v2 mixed (`cubes_v1v2`) | Rosie H100, job 284700 (epoch 69/120) | ~1 AM | |
| 3 | Test all models on easy + hard sets (`av-eval`) | Rosie, job 284701 (after 1+2) | ~2 AM | |
| 4 | Write v1-vs-v2 report (`av-report`) | Rosie, job 284702 (after 3) | ~2:30 AM | `~/adaptivevision/RESULTS_V2.txt` |
| 5 | Score new models vs Cosmos on GB10 (`av-gb10`) | Rosie job 284799 -> GB10 (after 1+2); EARLY v2-only job 284927 | v2 ~8 PM, all ~2 AM | `~/adaptivevision/BASELINE.txt` |
| 6 | Cosmos Transfer round 2: looser colour control + white balance | GB10 docker | ~4:00 PM | GB10 `~/adaptivevision/transfer/sweep_out*/` |
| 7 | B1-B3 accuracy, 600 held-out frames x 3 models (`av-percep`) | Rosie T4, job 284890 (after 1+2); EARLY v1+v2 job 284928 | v1+v2 ~9 PM, all ~4 AM | `~/adaptivevision/PERCEPTION.txt` |
| 8 | ROBOT BASELINE: 20 sim episodes, YOLO v1 (`av-sim`) | Rosie T4, job 284913 (7/20 done) | ~6:30 PM | `~/adaptivevision/SIM_RESULTS.txt` |
| 9 | ROBOT BASELINE: 20 sim episodes, YOLO v2 (`av-sim`) | Rosie T4, job 284914 (after 1) | ~9:30 PM | `~/adaptivevision/SIM_RESULTS.txt` |
| 10 | Extended Cosmos-vs-YOLO baseline, 300 more images | GB10 | ~8 PM | GB10 `baseline_set_ext/BASELINE.txt` |
| 11 | Cosmos Transfer: 2,000 train + 200 held-out photo-real images | Brev 2x A100 | ~8 PM | copied back + machines deleted |
| 12 | RTX re-render test: DONE. Path-traced = photo-like, labels exact (35 s/frame) | Brev L40S | done | Mac ~/Desktop/AdaptiveVision/rtx_test/ |
| 13 | Demo video (robot stacking, table + side views) | Rosie T4, job 284915 | ~5:45 PM | Mac ~/Desktop/AdaptiveVision/videos/ |

## Done today

- v2 dataset: 20,000 synthetic images (table/side/wrist), label check passed.
- Cosmos3-Nano running on GB10 (fixed transformers + torch issues).
- Cosmos vs YOLO v1 baseline on 80 held-out images. YOLO v1: F1 0.73 overall
  (easy 1.00, hard table 0.75, side 0.50, wrist 0.44), 0.07 s/image.
- Brev: Cosmos Transfer trial #1 on A100 (~$1.30 of $500). Realism much better,
  but only 26% of cube colours kept -> not usable yet; trial #2 fixes this.

- B1-B3 built: `perceive_yolo.py` (YOLO -> same detections.json as Cosmos) + starter geometry.
  v1 model, 230 held-out frames, table cam: 88% cubes found, colour 96%, position 0.27 mm
  median (92% within 2 mm), yaw 1.5 deg median (88% within 5 deg); 33% of cubes refused by
  the planner (crowded/touching cubes) -> main thing to improve.
- Cosmos Transfer trial #2: colour control ("vis") keeps 100% of cube colours (vs 26%);
  white balance removes the purple tint. Round 2 is choosing the best realism setting.

- FIRST FULL ROBOT RUNS IN THE TWIN (YOLO v1): tower complete 2/2, 12/12 picks, 12/12 placements,
  55 s per tower (9.2 s/pick), position 0.18 mm, yaw 1.4 deg, colours 18/18.

## Next (after the runs above)

- Pick the best Transfer setting; run it on more images; keep only images that pass
  the colour/hallucination gate (`transfer_qc.py`).
- Put YOLO into the starter pick-and-sort pipeline (same 4K crop as the real camera).
- Simulated pick-and-stack + failure recovery; measure picks/attempt and cycle time.
- Demo video + 1-page summary (due Sun Nov 1, 11:59 PM CT).

## Waiting on you

1. Email Zhen Zeng: real-arm interface, SMC vs SCHUNK gripper, which cube colours exist.
2. Post to Shamar (#data-and-testbed-requests): 40-60 real camera images + supervised test slot.
3. Delete the old NGC API key on ngc.nvidia.com.
4. Ask teammates before using the shared Brev credits ($500, team org UWM/MSOE-Team-1).
