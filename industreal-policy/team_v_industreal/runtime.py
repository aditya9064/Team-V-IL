"""Check the pinned runtime without importing Isaac or initializing CUDA.

Matching metadata and source is a prerequisite, never simulation evidence.
The 2.x port uses WXYZ quaternions; loading a 3.x Lab is not interchangeable.
"""
import hashlib
import importlib.metadata
import importlib.util
from pathlib import Path
import sys


LAB_API_FILES = {
    "envs/direct_rl_env.py": "52f16e81cbde14abfffcb3946b612ba9f78b3d6394e54c6f5a13af67976f6bba",
    "utils/math.py": "48f7052ed30d9e7ae21e0e20ac963ccb52f784a076f1a602cae1c641e923e3cd",
}


def inspect_runtime(versions=None, python_version=None, isaaclab_root=None):
    """Accept only the selected Python 3.11 / Sim 5.1 / pinned Lab 2.3 stack."""
    if versions is None:
        versions = {}
        for name in ("isaacsim", "isaaclab", "numpy", "torch", "gymnasium", "rl-games"):
            try:
                versions[name] = importlib.metadata.version(name)
            except importlib.metadata.PackageNotFoundError:
                versions[name] = None
    if python_version is None:
        python_version = tuple(sys.version_info[:2])
    errors = []
    if tuple(python_version) != (3, 11):
        errors.append("This pinned port requires the isolated Python 3.11 environment")
    if (versions.get("isaacsim") or "").split(".")[:3] != ["5", "1", "0"]:
        errors.append("This pinned port requires Isaac Sim 5.1.0, not the assembly's Sim 6 runtime")
    # The Lab v2.3.0 release's Python distribution is version 0.47.2.
    if versions.get("isaaclab") != "0.47.2":
        errors.append("Isaac Lab distribution must be 0.47.2 from pinned release v2.3.0")
    if (versions.get("numpy") or "").split(".")[0] != "1":
        errors.append("The selected Lab/Sim runtime requires NumPy 1.x")
    if versions.get("gymnasium") != "1.2.0":
        errors.append("Pinned Isaac Lab requires Gymnasium 1.2.0")
    for name in ("torch", "rl-games"):
        if not versions.get(name):
            errors.append("Missing runtime distribution: " + name)
    if isaaclab_root is None:
        try:
            spec = importlib.util.find_spec("isaaclab")
            if spec and spec.origin:
                isaaclab_root = Path(spec.origin).parent
        except (ImportError, ValueError):
            pass
    if isaaclab_root is None:
        errors.append("Pinned Isaac Lab API source is unavailable")
    else:
        for name, expected in LAB_API_FILES.items():
            path = Path(isaaclab_root) / name
            try:
                matches = hashlib.sha256(path.read_bytes()).hexdigest() == expected
            except OSError:
                matches = False
            if not matches:
                errors.append("Pinned Isaac Lab API source mismatch: " + name)
    return {
        "profile": "python311-sim510-lab230-wxyz",
        "package_versions": dict(versions),
        "runtime_errors": errors,
        "runtime_profile_matches": not errors,
        "runtime_compatibility_verified": False,
        "simulation_executed": False,
    }
