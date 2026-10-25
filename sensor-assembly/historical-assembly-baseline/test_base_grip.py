import json,unittest,hashlib
from pathlib import Path
class BaseGripChecks(unittest.TestCase):
 def setUp(self):
  r=Path(__file__).parent;self.p=json.loads((r/'base-grasp-proof.json').read_text());self.g=json.loads((r/'trial-mesh.json').read_text());self.d=json.loads((r/'design.json').read_text())
 def test_only_base_changes_geometry(self):
  for n,h in self.p['unchanged_part_hashes'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
 def test_support_positive_and_wrong_height_collision_negative(self):
  self.assertEqual(self.p['nominal_initial_ledge_interference_area_mm2'],0);self.assertGreater(self.p['wrong_tip_plus_1mm_interference_area_mm2'],0);self.assertGreater(self.p['source_profile_seating_drop_mm'],.05);self.assertLess(self.p['source_profile_seating_drop_mm'],.25)
 def test_base_post_fits_current_jaws(self):
  c=self.g['grip_commands']['base'];self.assertAlmostEqual(c['nominal_contact_per_jaw_mm'],2.0000152587890625);self.assertEqual(c['extra_travel_per_jaw_mm'],.05);self.assertEqual(c['penetration_stop_limit_mm'],.25);self.assertGreater(c['closed_grasp_command_per_jaw_m'],0);self.assertLess(c['closed_grasp_command_per_jaw_m'],.006)
 def test_tolerance_corners_clear_notch(self):
  self.assertEqual(len(self.p['notch_tolerance_checks']),54);self.assertGreater(self.p['minimum_notch_tolerance_corner_margin_mm'],.7)
 def test_base_ledge_probe_corresponds_to_physical_height(self):
  p=next(p for p in self.g['geometry_probes'] if p['label']=='base_shoulder_top');self.assertAlmostEqual(p['expected_distance_mm'],100-18.36);self.assertEqual(p['maximum_distance_error_mm'],.03)

class BaseGuideChecks(unittest.TestCase):
 def test_guides_clear_original_blade_and_detect_misalignment(self):
  p=json.loads(Path(__file__).with_name('base-guide-proof.json').read_text());self.assertEqual(p['source_finger_blade_x_mm'],6.);self.assertEqual(p['slot_width_x_mm'],6.4);self.assertEqual(p['aligned_guide_interference_mm3'],0);self.assertGreater(p['negative_misaligned_guide_interference_mm3'],1);self.assertEqual(p['nominal_each_side_clearance_mm'],.2)
