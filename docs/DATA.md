# Data — what is in Git and where the big data lives

Git holds code, results, logs and **samples**. The full datasets are tens of GB, so they stay on
**Rosie** under Shritej's account in `/home/ad.msoe.edu/gundas/adaptivevision/` (below: `~/adaptivevision`).

## In this repo

| Folder | Content |
| --- | --- |
| `organizer/` | Organizer starter kit `fanuc_er4ia_hackathon_starter_v05.zip` (robot/gripper assets, twin, Cosmos perception, sort demo) + explainer pages |
| `data_samples/synth/` | 5 chunks × 3 frames (15) of the **v1** dataset: image, YOLO label, metadata |
| `data_samples/synth_v2/` | 10 chunks × 3 frames (30) of the **v2** dataset + each chunk's `dataset.json` (camera, mode) |
| `data_samples/datasets/cubes_v2/data.yaml` | YOLO dataset config (classes) |
| `models/` | `cubes_v2.pt` (main detector), `cubes_v1.pt` |
| `real_photos/csi_1017/` | Phone photos of the real cell + YOLO v2 overlays (testing only, never training) |
| `brev_outputs/`, `gb10_outputs/` | Transfer QC reports, logs, before/after samples |

Label format: YOLO txt (`class cx cy w h`, normalised); classes `0 cube_red, 1 cube_yellow, 2 cube_blue, 3 cap, 4 vial`.
Table images are 1664×896 crops of a 3840×2160 render.

## On Rosie (`~/adaptivevision`)

| Path | What | Size |
| --- | --- | --- |
| `synth/chunk_*/{images,labels,meta}` | **v1** synthetic dataset (easy scenes, table camera) | 25,685 files (~8.5k frames) |
| `synth_v2/chunk_*/{images,labels,meta}` | **v2** synthetic dataset (6 scene modes, table/side/wrist) | 60,738 files (~20k frames) |
| `datasets/{cubes_1000,cubes_5000,cubes_all,cubes_v2,cubes_v1v2}/` | Train/val splits used for training (`data.yaml` inside) | — |
| `runs/detect/<run>/weights/{best,last}.pt` | All trained checkpoints (v1, v2, v1v2, v2s, subsets) | 11 runs |
| `runs/perception/` | Per-frame perception evaluation | — |
| `runs/sim/<model>/seed_<n>/` | Sim robot episodes (JSON + capture images) | 4.1 GB |
| `runs/sim_video/` | Rendered demo episodes (full-HD frames + video) | 15 GB |
| `baseline_set/`, `baseline_set_ext/` | 80 + 300 held-out images used for the Cosmos-vs-YOLO baseline | 0.6 GB |
| `transfer_batch/`, `transfer_batch2/` | White-balanced inputs sent to Cosmos Transfer | 6,400 files |
| `team_share/transfer_brev_batch1/` | Brev batch #1 Transfer outputs (`b0`, `b1`) + originals (`src0`, `src1`) + QC JSON | ~1.5 GB |
| `team_share/transfer_gb10_batch2/` | GB10 batch #2 Transfer outputs + `qc_train.json` | ~1,000 images |
| `team_share/videos_full/` | Full-size robot videos (1.6 GB / 1.0 GB) and share versions | 2.7 GB |
| `team_share/mac_eval/` | Mac-side evaluation samples and the full RTX render test | ~0.6 GB |
| `containers/isaac-sim.sif` | Isaac Sim 6.0 Singularity image | — |

Which Transfer images are usable: only those with `"pass": true` in the matching `qc_*.json`
(2,752 in total; 1,805 Brev + 947 GB10).

## Getting access

The data folders in the table above are **already readable by every Rosie user** (read-only), so there is
nothing to request. Log in to Rosie with your own MSOE account (username = the part of your email before
`@`) and read or copy by full path. `~gundas` and `~gundas/adaptivevision` can be entered but not listed,
so use the exact paths from the table.

```bash
# on Rosie, as yourself
ls /home/ad.msoe.edu/gundas/adaptivevision/team_share
mkdir -p ~/adaptivevision/datasets
rsync -a /home/ad.msoe.edu/gundas/adaptivevision/datasets/cubes_v2 ~/adaptivevision/datasets/
```

To train, copy what you need into your own `DATA_ROOT` (set in your `rosie/config.env`). The shared folders
are read-only, so YOLO cannot write its label cache there. The split images in `datasets/<name>/images` are
**symlinks** into `synth`/`synth_v2`, so `rsync -a` copies only the links (~0.5 GB, still readable); after
copying, change `path:` in the copied `data.yaml` to your own folder. Add `-L` to rsync for real file copies
(tens of GB).

Why not per-person ACLs: Rosie home is NFSv4 on NetApp, `setfacl` returns *Operation not supported*, and
`nfs4_setfacl` is not installed. To undo the sharing (as Shritej):
`chmod o-x ~ ~/adaptivevision && chmod -R o-rwx ~/adaptivevision/{synth,synth_v2,datasets,runs,baseline_set,baseline_set_ext,transfer_batch,transfer_batch2,team_share}`.
Regenerating instead of copying: `./rosie/run_v2.sh` rebuilds v2 from scratch (~several hours on 4 T4s).
