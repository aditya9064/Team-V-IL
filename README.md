# Team V — Adaptive small-part kitting

Physical AI for the Factory Floor Hackathon, UWM/MSOE, 2026. We are building camera-supervised kit preparation using the FANUC ER-4iA digital twin and SMC MHZL2-16D2 gripper.

**Start with [the shared project context](docs/START-HERE.md)** for the goal, architecture, verified progress, limitations and instructions for teammates or AI assistants.

## Status — October 15, 2026

- Original stacking demo: six successful simulated pickups and placements; a 55.2-second video.
- Corrected training pilot: 78 scenes / 432 cube labels, packaged for the mentor. Training results are pending.
- Adaptive flat-slot pilot: one camera-verified pickup and placement on Rosie, independently checked against simulator truth (job 284262).
- Empty-table, missing-part and stale-observation trials passed. A displaced target triggered replanning followed by an approach-position hold. The remaining fault tests and six-part adaptive kit are **not yet validated**.
- No physical-arm calibration or execution has been validated.

## Repository map

| Directory | Purpose |
|---|---|
| [dynamic-loop](dynamic-loop/README.md) | Fresh observations, per-pick planning, outcome verification and bounded recovery |
| [dataset-tools](dataset-tools/README.md) | Scene variation, persistent Isaac capture, corrected labels and QA |
| [mentor-training](mentor-training/README.md) | Training-data packaging, detector training/evaluation and smoke tests |
| [benchmark-tools](benchmark-tools/README.md) | Limited Rosie/Brev image-capture benchmarks |
| [src/team_v](src/team_v) | Earlier simulator-independent replay/evaluation foundation |
| [docs](docs) | Requirements, research and historical proposals |
| [data](data/README.md) | External-data instructions; raw assets are not in Git |

Historical proposals describe earlier decisions. The shared context and actual execution evidence take precedence for current status. Legacy `synthetic-v1` and `synthetic-crops-v1` labels are obsolete; use the corrected mentor v2 package.

## Get started

```bash
git clone https://github.com/aditya9064/Team-V-IL.git
cd Team-V-IL
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m unittest discover -s tests -v
(cd dynamic-loop && python -m unittest -v test_supervisor)
```

Replay and supervisor tests use the standard library. Rendering/physics require the separate Isaac-compatible Python environment and official FANUC v05 starter. Follow the component READMEs; a plain host Python cannot run simulation scripts.

## Updating an existing clone

```bash
git fetch origin
git switch main
git pull --ff-only origin main
```

If an older empty checkout has no local `main`, use `git switch -c main --track origin/main` after fetching. Preserve local work before switching branches. Use feature branches and review new changes before merging.

Credentials, private correspondence, raw organizer assets, generated datasets/videos, model weights, caches and Python environments are excluded. Training images and larger visual deliverables are shared separately; cloning this repository does not download them.
