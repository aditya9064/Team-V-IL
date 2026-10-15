"""Deterministic task evaluation from explicit fixture observations.

Fixture observations are supplied outcomes, not inferred camera measurements.
This module does not connect to a robot, simulator, GPU, or external API.
"""
from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class Policy:
    max_retries: int = 1
    max_frame_age_ms: float = 500.0

    def __post_init__(self):
        if type(self.max_retries) is not int or self.max_retries < 0:
            raise ValueError("max_retries must be a nonnegative integer")
        if not math.isfinite(self.max_frame_age_ms) or self.max_frame_age_ms < 0:
            raise ValueError("max_frame_age_ms must be finite and nonnegative")


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{label} must be a finite nonnegative number")


def validate_scenario(data):
    """Reject malformed fixtures before producing any report."""
    if data.get("schema_version") != 1 or data.get("evidence_kind") != "mock":
        raise ValueError("Expected schema_version 1 and evidence_kind mock")
    tasks = data.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("tasks must be a nonempty list")
    seen = set()
    for task in tasks:
        for key in ("task_id", "condition", "object_id", "destination"):
            if not isinstance(task.get(key), str) or not task[key].strip():
                raise ValueError(f"Missing nonempty {key}")
        if task["task_id"] in seen:
            raise ValueError("Duplicate task_id")
        seen.add(task["task_id"])
        observations = task.get("observations")
        if not isinstance(observations, list) or not observations:
            raise ValueError("Each task requires observations")
        for obs in observations:
            _number(obs.get("frame_age_ms"), "frame_age_ms")
            if type(obs.get("proposal_valid")) is not bool:
                raise ValueError("proposal_valid must be boolean")
            if obs.get("execution") not in ("completed", "timeout", "fault"):
                raise ValueError("Unknown execution outcome")
            for key in ("pick", "placement"):
                if obs.get(key) not in ("success", "failure", "uncertain"):
                    raise ValueError(f"Unknown {key} verdict")
            if type(obs.get("recoverable")) is not bool:
                raise ValueError("recoverable must be boolean")
            _number(obs.get("duration_s"), "duration_s")


def run_scenario(data, policy=Policy()):
    """Apply bounded recovery to mock observations and return auditable records."""
    validate_scenario(data)
    results, events = [], []
    for task in data["tasks"]:
        attempts = 0
        successful_picks = 0
        placement_attempts = 0
        duration = 0.0
        final = "review"
        reason = "observations_exhausted"
        recovery_attempted = False
        eligible_failure = False
        for index, obs in enumerate(task["observations"]):
            event = {"task_id": task["task_id"], "observation_index": index,
                     "attempt_number": attempts + 1, "evidence_kind": "mock",
                     "frame_age_ms": obs["frame_age_ms"]}
            if obs["frame_age_ms"] > policy.max_frame_age_ms:
                reason = "stale_observation"
                events.append({**event, "state": "review", "reason": reason,
                               "execution_requested": False})
                break
            if not obs["proposal_valid"]:
                reason = "invalid_proposal"
                events.append({**event, "state": "review", "reason": reason,
                               "execution_requested": False})
                break
            recovery_attempted = attempts > 0
            attempts += 1
            duration += obs["duration_s"]
            event.update(execution_requested=True, execution=obs["execution"],
                         pick=obs["pick"], placement=obs["placement"],
                         duration_s=obs["duration_s"])
            if obs["execution"] != "completed":
                final, reason = "fault", "execution_" + obs["execution"]
            elif "uncertain" in (obs["pick"], obs["placement"]):
                reason = "uncertain_outcome"
            else:
                pick_ok = obs["pick"] == "success"
                successful_picks += int(pick_ok)
                placement_attempts += int(pick_ok)
                if pick_ok and obs["placement"] == "success":
                    final, reason = "complete", "verified_transfer"
                elif obs["recoverable"]:
                    eligible_failure = True
                    if attempts <= policy.max_retries:
                        events.append({**event, "state": "retry", "reason": "eligible_failure"})
                        continue
                    reason = "retry_budget_exhausted"
                else:
                    reason = "nonrecoverable_failure"
            events.append({**event, "state": final, "reason": reason})
            break
        else:
            events.append({"task_id": task["task_id"], "state": "review",
                           "reason": reason, "execution_requested": False,
                           "evidence_kind": "mock"})
        results.append({"task_id": task["task_id"], "condition": task["condition"],
                        "object_id": task["object_id"], "destination": task["destination"],
                        "status": final, "reason": reason, "attempts": attempts,
                        "successful_picks": successful_picks,
                        "placement_attempts": placement_attempts,
                        "duration_s": duration, "eligible_failure": eligible_failure,
                        "recovery_attempted": recovery_attempted,
                        "recovered": final == "complete" and recovery_attempted,
                        "operator_review_required": final != "complete"})
    complete = sum(r["status"] == "complete" for r in results)
    recovery_count = sum(r["recovery_attempted"] for r in results)
    recovered = sum(r["recovered"] for r in results)
    return {"schema_version": 1, "evidence_kind": "mock", "mode": "replay",
            "notice": "Software fixture outcomes only; not measured robot performance.",
            "policy": asdict(policy), "tasks": results, "events": events,
            "metrics": {"tasks_started": len(results), "tasks_completed": complete,
                        "task_completion_rate": complete / len(results),
                        "execution_attempts": sum(r["attempts"] for r in results),
                        "successful_picks": sum(r["successful_picks"] for r in results),
                        "recovery_tasks_attempted": recovery_count,
                        "recovery_tasks_completed": recovered,
                        "recovery_success_rate": recovered / recovery_count if recovery_count else None,
                        "tasks_requiring_review": len(results) - complete}}
