# Compute runbook — Rosie, GB10, Brev, Mac

| Machine | Used for | Access |
| --- | --- | --- |
| **Rosie** (MSOE Slurm cluster) | Isaac Sim rendering (T4), detector training (H100), sim robot episodes | Your own MSOE Rosie account; SSH or OnDemand |
| **GB10** (`spark1.ai.msoe.edu`, DGX Spark, aarch64) | Cosmos3-Nano baseline, Cosmos Transfer 2.5 batches (Docker) | Hackathon team account — ask Shritej; never put the password in Git |
| **Brev** (NVIDIA cloud, org `UWM/MSOE-Team-1`, $500 shared credits) | Cosmos Transfer batch #1 (2× A100), RTX render test (L40S) | Brev CLI; **instances bill while running — delete when done** |
| Mac / any laptop | Unit tests, real-photo kit, video encoding, orchestration | — |

## Rosie

Code lives in `~/fanuc_er4ia_sim` (this repo's `fanuc_er4ia_sim/`), data in `DATA_ROOT=~/adaptivevision`.
Full guide: [`fanuc_er4ia_sim/rosie/README.md`](../fanuc_er4ia_sim/rosie/README.md). The only file to edit is
`fanuc_er4ia_sim/rosie/config.env`.

```bash
# copy code to Rosie (from the repo root on your laptop)
tar --no-mac-metadata --no-xattrs -czf av.tar.gz fanuc_er4ia_sim && scp av.tar.gz rosie:~
ssh rosie 'tar xzf av.tar.gz'          # extract over the folder; never rm -rf it

# on Rosie, from ~/fanuc_er4ia_sim
./rosie/av.sh check                     # partitions, GPUs, storage
./rosie/av.sh setup                     # envs + 48 host tests + YOLO weights
NGC_API_KEY=... ./rosie/av.sh pull-isaac   # Isaac Sim 6.0 container (once; key from ngc.nvidia.com, never commit it)
./rosie/run_all.sh                      # v1 chain: twin → test frames → label check → gen → split → train → report
./rosie/run_v2.sh                       # v2 chain (hard scenes, 3 cameras) + GB10 baseline + perception eval
./rosie/av.sh status                    # queue, frame counts, disk
bash rosie/status_all.sh                # one-screen status (alias `avstatus`)
```

Individual jobs (`rosie/jobs/*.sh`) document their arguments in the header, e.g.
`sim_job.sh <weights | run name> <first seed> <count> [--render]` (render episodes need `--time` ≥ 4 h).

Facts and limits:
- T4 nodes (`teaching` partition, `gpu:t4:1`) are the only GPUs with RT cores → Isaac must run there. H100 = `dgxh100`, `gpu:h100:1`.
- Student QOS: **4 running jobs**, **24 h** per job. One job may hold several GPUs. Use `--dependency`, not `SBATCH_DEPENDENCY`.
- First Isaac run per driver compiles shaders for 20+ minutes. Real-time render ≈ 5.5 s / frame on a T4.
- A job ending on a T4 node can kill GPU processes of other jobs on that node — generation is chunked and resumable; the pipeline refills missing chunks.
- `/data` and `/scratch` are read-only for students — everything is under `$HOME`.

## GB10 (DGX Spark)

Layout under `~/adaptivevision` (scripts and logs copied to [`gb10_outputs/`](../gb10_outputs)):

| Path | What |
| --- | --- |
| `envs/cosmos3` | Cosmos3-Nano env: **`transformers==4.57.1`** (newer versions load garbage weights) and torch native Triton ops deregistered (no `Python.h`) |
| `run_baseline.py`, `baseline_ext.sh`, `baseline_ext2.sh` | Cosmos vs YOLO baseline on `baseline_set_ext` (300 imgs) |
| `cosmos-transfer2.5/` + Docker image `av-transfer:nightly` | Cosmos Transfer 2.5 |
| `transfer/gb10_run.sh` → `gb10_batch.sh` / `gb10_sweep.sh` | Run Transfer in the container (`--gpus all`); resumable chunks; HF token read from `transfer/.hf_token` (chmod 600, **not in Git**) |
| `transfer/batch2_out/` | Batch #2 outputs (947 gated) + `qc_train.json` |

Be polite: it is shared with other teams — check GPU load first, stay in your folder, never inspect others' processes or files.

## Cosmos Transfer recipe (colour-safe)

1. `tools/transfer_trial/make_batch.py` (on Rosie) picks frames, applies `white_balance.py`, shards them.
2. Transfer with **edge 1.0 + vis 1.0 (very_low blur), sigma_max 80, neutral prompt** (`prompts_neutral.json`, config `edgevis_s80` in `sweep_specs.py`).
3. `transfer_qc.py` gate: every labelled cube keeps its colour **and** no new cube-coloured area appears outside the boxes. Only passing images may be used for training.

## Brev

Scripts: `tools/transfer_trial/{launch_batch.sh,batch_run.sh,brev_finish.sh,rtx_test.sh}`. Lessons:
- Pin Python 3.10 for Cosmos Transfer.
- The Isaac Sim Docker image runs as uid 1234 with home `/isaac-sim`: mount writable dirs at `/isaac-sim/.cache` and `/isaac-sim/.nv`, `sudo chmod -R a+rwX` host dirs.
- After a run: copy outputs, shred the HF token on the machine, `brev delete <instance>`. `brev_finish.sh` does all three.
- Spend so far ≈ $25 (trial $1.3, L40S $3.1, 2× A100 ~$20). No instances are running.
