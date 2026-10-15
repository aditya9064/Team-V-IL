# Detector plug-in and model handoff

The current motion trials still use the geometric detector. No trained checkpoint has been received. The new torchvision back end is prepared for the mentor's Faster R-CNN ResNet-50 FPN v2 checkpoint; its predictions remain proposals, never unverified grasp commands.

The pipeline is: full BGR camera frame -> RGB training crop -> CNN boxes/foreground labels -> full-camera pixel coordinates -> existing image-only silhouette pose fit -> unchanged supervisor and camera outcome checks. There is no silent geometric fallback when the learned backend is requested and fails. A high model score never substitutes for pose IoU >=0.85 or positive pickup/placement evidence.

## Required mentor return

Return best.pt from the existing training recipe, its config.json, validation-selected score threshold with provenance, held-out test results and representative prediction failures. The checkpoint must contain model state and the recipe config. The loader uses torch.load(weights_only=True), strict state loading, class/architecture checks and an explicit SHA-256 checksum. It never downloads pretrained weights during deployment. Unsupported checkpoint formats fail before target motion.

The training pilot uses full frames 3840x2160 and crop rectangle [960,1080,2880,1944], yielding RGB crops 1920x864. Foreground labels are 1 yellow, 2 blue, 3 red; label 0 is reserved for background. Dataset text labels 0..2 are not model output IDs. These verified preprocessing dimensions are camera-specific, not a CSI calibration.

Copy deployment.example.json to a new deployment config. Fill checkpoint_sha256, score_threshold and threshold_source from the returned model/validation artifacts. The template deliberately contains null placeholders and will be rejected until complete. Do not select the threshold on the held-out test set. The untrained smoke's threshold 1.0 is solely for rejecting all smoke proposals, not a recommendation for deployment.

```bash
python runner.py --starter "$FANUC_STARTER" --out /your/fresh/run \
  --backend torchvision --checkpoint /your/best.pt \
  --deployment /your/deployment.json --training-code /your/Team-V-IL/mentor-training
```

Use the Isaac-compatible Python, actual paths and an allocated GPU for a simulation run. Default backend is explicitly geometric. Supplying checkpoint options without selecting torchvision is rejected. Backend identity pins weights, config and architecture hashes, and the supervisor expects that fixed identity. Crop resizing/mapping is explicit; invalid shapes/classes/nonfinite or out-of-frame boxes fail the frame. Low-score proposals and insufficient pose fits are recorded as rejected. Duplicate fitted proposals hold rather than being silently merged. Existing capture-age limits still include inference time.

## Tests and honest scope

Pure contract tests cover crop offsets, resizing, foreground/background labels, malformed values, thresholds and identity pinning. Loading/RGB tests use a tiny explicitly untrained fixture. The bounded GPU smoke constructs the actual architecture with pretrained=False, saves/loads random weights, and tests CUDA inference on existing rendered RGB with no labels or motion. None of these tests establish trained-model accuracy.

Motion/failure trials continue with the geometric backend while waiting for the mentor checkpoint. First evaluate the returned model offline; then run a bounded simulation pilot behind the same strict checks. Real-camera calibration and approved CSI control interfaces remain required before physical deployment.

The plug-in's six-part geometric regression and five additional situation checks are now recorded in [additional-validation-report.md](additional-validation-report.md). These validate the interface and existing controller behavior, not a trained checkpoint. The additional launcher requires 33 tests and supports an optional subset of case names; set TEAM_V_DYNAMIC_CODE to the reviewed installed source directory before submitting. Default execution runs all six cases serially on one T4. Do not launch a new trial without a bounded authorized need.
