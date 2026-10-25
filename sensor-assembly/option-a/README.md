# Option A: three isolated larger-part pickups

Approved actual user message01a17468-12f1-78d2-8e9e-bb89cf502d12. Current supplied SMC gripper/fingers remain unchanged. Targets use authored known fixture positions.

Run base, loaded tray with separate intact dynamic PCB, and lid once each. Binary open/close only; lift40mm; fixed120-step/two-second hold. Original physics, raw contact, endpoint, stillness and continuous PCB limits remain. No placement/release/full assembly, learned perception, camera freshness, physical-hardware or print qualification.

One teaching T4 allocation capped at15min; each simulator case250s+20s kill grace, independent post-case CPU audit15s+2s kill grace, zero automatic retries. All-state UID admission rejects active/queued/unknown jobs and independently verifies retained terminal records. The durable receipt prevents resubmission after uncertain responses.

Actual videos stay on Rosie. Independent CLI verifies raw source/mechanical/media evidence; actual video must still be viewed. CPU checks and scheduler exit0 never imply a pickup pass.

## Prepare dependencies and run CPU checks

Obtain the official FANUC v05 starter separately; supplied SDK/assets are not redistributed. In a Python environment containing NumPy/SciPy and an FFmpeg decoder, run:

    PYTHONDONTWRITEBYTECODE=1 python prepare_source.py --starter /path/to/fanuc_er4ia_sim
    PYTHONDONTWRITEBYTECODE=1 python -W error -m unittest test_option_a test_admission test_pickup_integration test_option_a_pickup_trace_checker

Preparation copies five hash-pinned dependency files and creates a local execution manifest; it does not start a job. Supplied dependencies and local execution manifests are ignored by Git. The publication removes a private approval reference and updates matching file identities; geometry and numeric gates are unchanged. This publication derivative has not run a GPU experiment. Review source and obtain applicable account/resource authorization before scheduler use. Meshes in cad/ are not print approval.
