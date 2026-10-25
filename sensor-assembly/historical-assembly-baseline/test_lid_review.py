import unittest
from build_lid_pose_review import acceptance


class ReviewScopeTests(unittest.TestCase):
    def test_precontact_cannot_be_promoted_to_full(self):
        self.assertEqual(acceptance({'diagnostic_stage': 'precontact', 'diagnostic_stage_accepted': 'precontact', 'full_assembly_accepted': True}, True), (True, False))

    def test_runtime_claim_without_checker_cannot_pass(self):
        self.assertEqual(acceptance({'diagnostic_stage': 'release', 'diagnostic_stage_accepted': 'release', 'full_assembly_accepted': True}, False), (False, False))

    def test_release_requires_independent_acceptance_and_scope(self):
        self.assertEqual(acceptance({'diagnostic_stage': 'release', 'diagnostic_stage_accepted': 'release', 'full_assembly_accepted': True}, True), (True, True))


if __name__ == '__main__':
    unittest.main()
