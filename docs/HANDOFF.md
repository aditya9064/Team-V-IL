# Team handoff — AdaptiveVision (as of Oct 18, 2026)

Read this first if you are picking up work. Numbers live in [RESULTS.md](RESULTS.md), machines in
[COMPUTE.md](COMPUTE.md), big data in [DATA.md](DATA.md).

## 1. The challenge

- Rockwell + NVIDIA Physical AI Hackathon, **Dataset 3: FANUC ER-4iA grasping and sorting**.
- Every project must end with a **run on the real robot** at CSI.
- Real cell data is for looking and testing only; **training data must come from a digital twin**.
- Judges want: a specific problem, a business case for other manufacturers, measured results vs a
  baseline (including where it fails). Judging weights are not published yet.
- Submission **Nov 1, 11:59 PM CT** (~5-min video, 1-page summary, repo, credits). Finals **Nov 5**.

## 2. What we did (timeline)

| Date | What happened |
| --- | --- |
| Oct 8 | Kickoff at Rockwell HQ. |
| Oct 15 | Analysed the organizer starter kit v0.5 (`organizer/`). Built the Rosie Slurm pipeline (`fanuc_er4ia_sim/rosie/`): Isaac Sim 6.0 in Singularity on T4, training on H100. Fixed Rosie issues (Vulkan libs in Singularity, `~/.local` torch shadowing, job-end GPU kills). |
| Oct 15–16 | **v1 dataset** (~8.5k easy frames) + YOLO11m training: synthetic mAP50-95 0.995, but only 0.728 on hard scenes. |
| Oct 16 | **v2 dataset** (`synth/generate_v2.py`, ~20k frames): 6 scene modes (scatter, cluster, tower, crowded, sparse, edge), up to 12 cubes, towers, robot arm in view, occlusion-aware labels, distractors, flat tape look-alikes, lighting changes, sensor effects, **table + side + wrist cameras**. Automatic label check gates every chunk. |
| Oct 16 | **Baseline:** organizer's Cosmos3-Nano perception run on the GB10 (needed `transformers==4.57.1` and Triton fixes). **Cosmos Transfer 2.5** trials on Brev A100 and the GB10 — found a colour-safe recipe (white balance → edge + vis control → colour gate). **RTX test** on a Brev L40S (real-time vs path-traced rendering). |
| Oct 16 | **Robot in sim:** `rosie/jobs/sim_job.sh` runs capture → `perceive_yolo.py` → `plan_sort.py` → `sort_scene.py`. 20/20 towers with v1 and with v2. |
| Oct 16–17 | Overnight: v2 / v1v2 / v2s training on H100, 300-image held-out baseline, 60 more sim towers (60/60), Transfer batch #1 on Brev (1,805 gated images) and batch #2 on the GB10 (947 gated). Built the CSI real-photo kit (`real_photo_test.py`, `csi_kit/`). Brev total spend ≈ $25; all Brev machines deleted. |
| Oct 17 | **CSI visit** (photos in `real_photos/csi_1017/`). Learned the real robot is driven through ManipBench and the scene differs from the twin (section 3). First real-photo test: YOLO v2 found the real red/blue cubes (conf 0.94 / 0.93) and ignored the bins; missed tiny/far cubes. |
| Oct 17–18 | Chose the real task (sort cubes into colour bins), researched real factory limitations (section 4), wrote this handoff. |

## 3. The real cell (CSI, Oct 17)

The setup is **task-independent**: teams choose objects and task and may 3D-print objects, bins or fingers.

| Topic | What we saw / were told |
| --- | --- |
| Robot | FANUC ER-4iA, controller R-30iB **Mate** Plus |
| Gripper | SMC MHZL2-16D2 two-finger pneumatic (6 mm stroke per jaw, 0–12 mm straight-face gap, hooked tips 4–16 mm) |
| Cameras | 3: top, side, wrist (silver USB-C module under the gripper) |
| Table | White surface with pencil guide lines, a paper seam, CAUTION tape |
| Objects | Red and blue cubes with rounded edges — **size not confirmed (10 or 16 mm?)** |
| Bins | 3D-printed bins: blue, orange, green, yellow (yellow holds M6×30 screws). **No red bin.** |

### ManipBench — how code drives the real robot (https://manipbench.com)

- We write a closed-loop policy `act(obs)` that runs **on our machine**.
- `obs` = camera images + joint angles + fingertip pose + a text instruction.
- Action = `"joints"` (7 values, deg) or `"tcp"` (mm / deg + gripper).
- Safety limits: **≤ 8 mm and ≤ 5° per step**, 50 mm/s, fingertip ≥ ~2 mm above the surface, ~0.8 s lag.
- `uv run run.py --mock` tests locally; `--submit` then book a slot. **3 real runs per team**, scored by
  lab operators, recorded as a LeRobotDataset.
- Client: **`fanuc_er4ia_cell_client.zip` must be requested in `#data-and-testbed-requests`** (not done yet).
- CAD on the site: SMC gripper, M6×30 screw, work-surface texture.

## 4. The idea (proposed — team to confirm)

> **A sorting robot you re-task with one sentence, that checks its own work and recovers without stopping the line.**

| Factory limitation | Evidence | Our answer |
| --- | --- | --- |
| Small error rates compound | 99.5% vision × 99.5% grip → only ~36% chance to empty a 100-part bin without a failure ([Control Engineering](https://www.controleng.com/articles/what-happens-when-automation-systems-fail)); robots repeat the same failed grasp ([UC Berkeley, CASE 2020](https://arxiv.org/abs/2007.10420)) | Wrist camera confirms the grip, top camera confirms the cube is in the right bin; on failure retry with a **different** grasp |
| Downtime is expensive | ~$2.3M per hour in automotive ([Siemens TCOD 2024](https://assets.new.siemens.com/siemens/assets/api/uuid:1b43afb5-2d07-47f7-9eb7-893fe7d0bc59/TCOD-2024_original.pdf)) | Recover by itself instead of stopping or failing silently |
| Reprogramming per product change | Integration costs ~4–5× the robot ([MIT Sloan](https://mitsloan.mit.edu/node/29144)) | Sort rule comes from the ManipBench instruction text ("blue cubes to the blue bin, red to orange") |
| Skills shortage | Up to 1.9M US manufacturing jobs may go unfilled by 2033 ([Deloitte](https://www.deloitte.com/us/en/Industries/industrial-construction/articles/future-of-manufacturing-skills-gap-study.html)) | No robot programmer needed to change the task |

Sim-only bonus: a **camera-bump check** (refuse to run if a camera moved). Full research: [FACTORY_RESEARCH.md](FACTORY_RESEARCH.md).

## 5. What's left — task list

Critical path: **client zip → policy adapter → mock test → 3 real runs**, all before Nov 1.

| # | Task | Notes / where to start |
| --- | --- | --- |
| 1 | Request `fanuc_er4ia_cell_client.zip` | `#data-and-testbed-requests`. Blocks all real-robot work. |
| 2 | Send open questions (section 6) to the CSI student worker | Answers set cube/bin sizes and adapter units. |
| 3 | Confirm idea + sort rule (red has no bin) | Team decision. |
| 4 | **ManipBench policy adapter** | Reuse `perceive_yolo.py` + `plan_sort.py`; turn each pick into ≤ 8 mm / 5° TCP steps + gripper commands; pass `run.py --mock`. |
| 5 | Self-check + recovery | Wrist camera grip check, top camera bin check, different-grasp retry. Today `sort_scene.py` aborts on any failure. |
| 6 | Instruction parser | Instruction text → bin mapping. |
| 7 | **Twin v3** matching the real cell | Coloured bins as new classes (`bin_blue/orange/green/yellow`), pencil-line + seam table texture, rounded cubes 10–20 mm, screws. Start in `synth/scenarios.py`, `synth/generate_v2.py`, `synth/labels.py` (`hull_points` uses `CUBE_EDGE`), `cell.py` (`_make_bin`). |
| 8 | Detector v3 | v3 twin frames + the 2,752 gated Transfer images; re-run baselines (`tools/run_baseline.py`). |
| 9 | Fix the 30% "uncertain" refusals | `plan_sort.py` reasons `localisation_uncertain`, `cube_pose_missing` (see `results/PERCEPTION.txt`). |
| 10 | Book and run 3 real-robot slots; keep the recorded episodes | Required by the rules. |
| 11 | Video, 1-page summary, credits | Robot videos in `videos/`; full-size ones listed in DATA.md. |

## 6. Open questions for the organizers / CSI

- Cube edge size (10 or 16 mm?) and material; bin outer size and height.
- ManipBench: camera names, resolution, frame rate; are camera calibrations in `obs`? Which frame are TCP
  poses in (base / UF0)? Gripper command range? `act()` rate, episode length, how to signal "done"?
- How to grasp a 10–16 mm cube with the ~2 mm minimum fingertip height?
- Who places objects for our runs; can we choose the layout or bring 3D-printed objects/bins?
- Scoring, slot length, open dates before Nov 1.
- Have the cameras moved since the starter-kit v0.5 calibration? May we save frames for testing?
- Does `run.py --mock` work on macOS (Apple Silicon)?

## 7. Lessons learned (save yourself hours)

- Rosie: any job ending on a T4 node can kill GPU processes of other jobs on that node → our jobs are
  resumable and the pipeline refills missing chunks. Max 4 running jobs / 24 h per job.
- Rosie: export `PYTHONNOUSERSITE=1` (old `~/.local` torch breaks envs) — already in `rosie/common.sh`.
- T4 real-time rendering is speckled → `AV_RENDER_DENOISE=median3`; path tracing on T4 lags labels by a frame.
- Ultralytics `cache=ram` needs ~4.1 MB per 1664×896 image — disable above ~10k images.
- Cosmos Transfer flips cube colours unless the **vis** control is on (weight 1.0); always run `transfer_qc.py`.
- `pkill -f` / `pgrep -f` inside `ssh "…"` matches the ssh command itself — use `ps … | grep "[b]ash script.sh"`.
