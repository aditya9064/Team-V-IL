import json,hashlib,unittest
from pathlib import Path
from payload_checks import horizontal_seating_normals,placement_supports,horizontal_support_pairs
from assembly_checks import role_pair,contact_violation
class LidSeating(unittest.TestCase):
 def setUp(self):
  self.r=Path(__file__).parent;self.p=json.loads((self.r/'lid-seating-proof.json').read_text());self.g=json.loads((self.r/'trial-mesh.json').read_text())
 def test_all_729_relative_corners_have_contact_offset_margin(self):
  self.assertEqual(self.p['relative_placement_corners'],729);self.assertGreater(self.p['minimum_base_floor_margin_mm'],.4);self.assertGreater(self.p['minimum_tray_clearance_mm'],.4);self.assertGreater(self.p['wrong5mm_tray_collision_area_mm2'],0.)
 def test_support_surface_horizontal_phase_gated_not_lateral_or_old_wall(self):
  pair=role_pair(('cover','ear'),('base','floor'));self.assertEqual(placement_supports()['cover'],{pair});self.assertIn(pair,horizontal_support_pairs());self.assertNotIn(role_pair(('cover','ear'),('base','wall')),horizontal_support_pairs());self.assertFalse(horizontal_seating_normals([[1,0,0]]));self.assertTrue(contact_violation(True,-.000251,0,.25));self.assertAlmostEqual(self.p['actual_support_area_mm2'],25.)
 def test_proven_base_and_board_exact_and_runtime_only_declared_pair(self):
  for n,h in self.p['unchanged_parts_sha256'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
  for n,h in self.p['source_runtime_sha256'].items():self.assertEqual(hashlib.sha256((self.r/n).read_bytes()).hexdigest(),h)
  self.assertTrue(self.p['no_extra_support_exemption'] and self.p['pcb_unchanged'])
 def test_physical_thresholds_and_closure_unchanged(self):
  self.assertTrue(self.p['limits_unchanged'] and self.p['payload_limits_unchanged'] and self.p['grip_commands_unchanged']);self.assertEqual(self.g['limits']['maximum_intended_penetration_mm'],.25);self.assertEqual(self.g['limits']['placement_xy_mm'],.75)
