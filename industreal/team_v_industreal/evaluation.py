"""Independent insertion scoring; training reward is never acceptance evidence."""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class InsertionLimits:
    radial_clearance_m: float = (0.0081-0.007986)/2
    depth_error_m: float = 0.001
    maximum_tilt_rad: float = math.radians(1)
    maximum_contact_depth_m: float = 0.00025
    minimum_release_hold_s: float = 0.5


def score(samples, *, limits=InsertionLimits()):
    """Requires actual released samples and contact evidence, never reward/exit code."""
    reasons = []
    if len(samples) < 2:
        reasons.append("insufficient_samples")
    times = []
    socket_contact_observed = False
    for sample in samples:
        for key in ("time_s", "radial_error_m", "depth_error_m", "tilt_rad", "max_contact_depth_m"):
            value = sample.get(key)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value):
                reasons.append("missing_or_nonfinite_"+key)
        if reasons:
            continue
        times.append(sample["time_s"])
        if sample.get("released") is not True:
            reasons.append("not_released")
        if sample.get("contact_monitor_valid") is not True:
            reasons.append("contact_monitor_unverified")
        socket_contact_observed |= sample.get("socket_contact_observed") is True
        if abs(sample["radial_error_m"]) > limits.radial_clearance_m:
            reasons.append("radial_clearance_exceeded")
        if abs(sample["depth_error_m"]) > limits.depth_error_m:
            reasons.append("depth_error_exceeded")
        if abs(sample["tilt_rad"]) > limits.maximum_tilt_rad:
            reasons.append("tilt_exceeded")
        if not 0 <= sample["max_contact_depth_m"] <= limits.maximum_contact_depth_m:
            reasons.append("contact_depth_exceeded")
    if times and (any(b <= a for a,b in zip(times,times[1:])) or times[-1]-times[0] < limits.minimum_release_hold_s):
        reasons.append("invalid_release_hold_duration")
    if not socket_contact_observed:
        reasons.append("no_socket_contact_evidence")
    return {"status": "FAILED" if reasons else "INSERTION_BENCHMARK_PASS",
            "reasons": sorted(set(reasons)), "limits": asdict(limits),
            "scope": "simulated peg insertion and release only",
            "full_assembly_accepted": False, "physical_hardware_tested": False}


def within_capture_envelope(radial_error_m,depth_error_m,tilt_rad,*,limits=InsertionLimits()):
    """Intermediate release/support permission, never insertion acceptance.

    The part must already reach the seated depth inside the cavity. The
    radial capture margin includes only the existing maximum allowed contact
    penetration. Final released scoring still requires the original clearance.
    """
    values=[radial_error_m,depth_error_m,tilt_rad]
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):return False
    return (0<=radial_error_m<=limits.radial_clearance_m+limits.maximum_contact_depth_m
            and abs(depth_error_m)<=limits.depth_error_m and abs(tilt_rad)<=limits.maximum_tilt_rad)
