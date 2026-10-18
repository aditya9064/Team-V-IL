"""Smoke test: Cosmos3-Nano with the starter's own prompt (run on the GB10).

    HF_HOME=~/adaptivevision/hf python cosmos_smoke.py <starter dir> <image>
"""
import sys
import time

sys.path.insert(0, sys.argv[1])
import perceive  # noqa: E402  (starter module, needs the path above)
import torch  # noqa: E402
import torch._native.registry as native_ops  # noqa: E402

# torch 2.14 routes bmm & co. to Triton kernels that need Python.h at runtime;
# the GB10 has no python3-dev (and no sudo), so use the stock CUDA kernels.
native_ops.deregister_op_overrides(
    disable_op_symbols=["bmm", "_foreach_mm", "linalg_polar", "linalg_polar.out"])

t0 = time.time()
model, processor = perceive.load_model()
t1 = time.time()
reply = perceive.ask(model, processor, sys.argv[2], perceive.PROMPT)
t2 = time.time()
print(f"load {t1 - t0:.0f} s | answer {t2 - t1:.0f} s | "
      f"peak GPU memory {torch.cuda.max_memory_allocated() / 1e9:.1f} GB")
print("REPLY:", reply[:1500])
print("PARSED:", perceive.parse_json(reply))
