# Third-party components

The project uses a separately installed NVIDIA Isaac Sim environment and the official FANUC ER-4iA starter supplied for the hackathon. Supplied robot/gripper assets, textures, SDK files and simulator installations are not redistributed here. Obtain them through the organizers or the relevant vendor distribution.

The IndustReal integration contains selected NVIDIA Isaac Lab/Factory and original IndustReal reference code. Retain the BSD-3-Clause notice in `industreal/vendor/factory/LICENSE`, `industreal-policy/vendor/factory/LICENSE` and the per-file copyright/license headers. The original algorithms module includes its NVIDIA BSD terms in its header. These notices apply to those components; they do not license the omitted simulator or binary assets.

Pinned Isaac Lab reference: release2.3.0, commit `3c6e67bb5c7ada942a6d1884ab69338f57596f77`. See the integration READMEs for separate runtime requirements. Do not install the learning runtime over the working assembly simulator environment.

Official requirements and other referenced sources are linked in `docs/maic-reference/2026-10-25/REQUIREMENTS-AND-STRATEGY.md`.
