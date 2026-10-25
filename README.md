# Team V — adaptable small-part assembly

Physical AI for the Factory Floor Hackathon, UWM/MSOE, 2026. FANUC ER-4iA with the supplied SMC MHZL2-16D2 gripper and current fingers.

**Start with [the shared context](docs/START-HERE.md)** and [the project checklist](docs/PROJECT-CHECKLIST.md).

## Status — 25 October 2026

- Larger enclosure: five CAD models and 63 CPU checks pass. Actual larger-part pickup tests remain pending.
- Original-size enclosure: one nominal three-part motion sequence completed; strict final support-force evidence was inconclusive. Offset variations failed contact checks.
- IndustReal: one calibrated preheld deterministic insertion/release baseline passed. Learned insertion remains experimental, with failures and no held-out reliability result.
- Cube kitting: six camera-verified simulated pickups/placements and bounded recovery tests were validated earlier.
- No complete enclosure assembly, print readiness or physical-robot success is qualified.

## Repository map

| Directory | Purpose |
|---|---|
| [sensor-assembly/option-a](sensor-assembly/option-a/README.md) | Current larger-part CAD, known-fixture pickup protocol and independent checks |
| [sensor-assembly/historical-assembly-baseline](sensor-assembly/historical-assembly-baseline/README.md) | Retained original-size source and regression fixtures |
| [sensor-assembly/camera-diagnostic](sensor-assembly/camera-diagnostic/README.md) | CPU-reviewed observation-only camera preparation |
| [industreal](industreal/README.md) | Deterministic insertion port and separate runtime requirements |
| [industreal-policy](industreal-policy/README.md) | Experimental learned insertion source; no reliable policy claim |
| [dynamic-loop](dynamic-loop/README.md) | Cube kitting, observation gates and bounded recovery |
| [dataset-tools](dataset-tools/README.md) | Synthetic scene generation and label QA |
| [mentor-training](mentor-training/README.md) | Detector training and evaluation tools |
| [benchmark-tools](benchmark-tools/README.md) | Limited Rosie/Brev capture benchmarks |
| [docs](docs/START-HERE.md) | Shared context, checklist and official requirements review |

## Quick host checks

    git clone https://github.com/aditya9064/Team-V-IL.git
    cd Team-V-IL
    PYTHONPATH=src python3 -m unittest discover -s tests -v
    (cd dynamic-loop && python3 -m unittest test_supervisor test_detector_contract)

The 28 checks above do not launch a simulator. Follow the component README for its dependencies and tests. Assembly rendering requires the separately obtained official starter and Isaac-compatible environment. The learning port requires a separate pinned environment; preserve the working assembly installation.

Credentials, private correspondence, datasets, weights, supplied licensed assets and videos are excluded. See [publication scope](docs/PUBLICATION-SCOPE.md) and [third-party notices](THIRD-PARTY.md). The current larger-part publication is a metadata-redacted derivative of the retained reviewed candidate, not a newly executed simulator result. Historical debug fixtures may have private paths redacted and are not replacements for original run evidence.

Legacy synthetic-v1 labels are obsolete; use the corrected training package and its component instructions. Preserve local work before pulling or switching branches.
