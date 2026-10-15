import copy
import unittest
from team_v.replay import Policy, run_scenario


def scenario(*observations):
    return {"schema_version": 1, "evidence_kind": "mock", "tasks": [{
        "task_id": "t1", "condition": "fixture", "object_id": "block", "destination": "bin",
        "observations": list(observations)}]}


def observation(**changes):
    data = dict(frame_age_ms=20, proposal_valid=True, execution="completed",
                pick="success", placement="success", recoverable=False, duration_s=2)
    data.update(changes)
    return data


class ReplayTests(unittest.TestCase):
    def test_success_requires_both_verdicts(self):
        for changes in ({"pick": "failure"}, {"placement": "failure"}, {"pick": "uncertain"}):
            self.assertEqual(run_scenario(scenario(observation(**changes)))["metrics"]["tasks_completed"], 0)

    def test_recovery_denominator_and_budget(self):
        data = scenario(observation(pick="failure", placement="failure", recoverable=True), observation())
        report = run_scenario(data)
        self.assertEqual(report["metrics"]["execution_attempts"], 2)
        self.assertEqual(report["metrics"]["recovery_success_rate"], 1)
        baseline = run_scenario(data, Policy(max_retries=0))
        self.assertEqual(baseline["metrics"]["execution_attempts"], 1)
        self.assertIsNone(baseline["metrics"]["recovery_success_rate"])

    def test_timeout_never_retries(self):
        report = run_scenario(scenario(observation(execution="timeout", recoverable=True), observation()))
        self.assertEqual(report["tasks"][0]["status"], "fault")
        self.assertEqual(report["metrics"]["execution_attempts"], 1)

    def test_stale_and_invalid_do_not_execute(self):
        for changes in ({"frame_age_ms": 501}, {"proposal_valid": False}):
            report = run_scenario(scenario(observation(**changes)))
            self.assertEqual(report["metrics"]["execution_attempts"], 0)
            self.assertFalse(report["events"][0]["execution_requested"])

    def test_uncertainty_never_retries(self):
        report = run_scenario(scenario(observation(placement="uncertain", recoverable=True), observation()))
        self.assertEqual(report["metrics"]["execution_attempts"], 1)

    def test_missing_followup_is_review(self):
        report = run_scenario(scenario(observation(pick="failure", placement="failure", recoverable=True)))
        self.assertEqual(report["tasks"][0]["reason"], "observations_exhausted")
        self.assertTrue(report["tasks"][0]["operator_review_required"])

    def test_malformed_data_rejected(self):
        for changes in ({"duration_s": -1}, {"frame_age_ms": float("nan")}, {"recoverable": "yes"}):
            with self.assertRaises(ValueError):
                run_scenario(scenario(observation(**changes)))
        data = scenario(observation())
        data["tasks"].append(copy.deepcopy(data["tasks"][0]))
        with self.assertRaises(ValueError):
            run_scenario(data)


if __name__ == "__main__":
    unittest.main()
