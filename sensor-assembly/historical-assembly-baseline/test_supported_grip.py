import json,unittest,hashlib
from pathlib import Path
class SupportedGripChecks(unittest.TestCase):
 def setUp(self):
  self.root=Path(__file__).parent;self.p=json.loads((self.root/'grasp-interface-proof.json').read_text());self.g=json.loads((self.root/'trial-mesh.json').read_text())
 def test_noncarrier_parts_unchanged(self):
  for n,h in self.p['unchanged_noncarrier_parts'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
 def test_source_hook_clears_ledge_and_wrong_height_does_not(self):
  self.assertEqual(self.p['shoulder_initial_interference_area_mm2'],0);self.assertGreater(self.p['wrong_tip_height_plus_1mm_interference_area_mm2'],0);self.assertGreater(self.p['source_profile_seating_drop_mm'],.05);self.assertLess(self.p['source_profile_seating_drop_mm'],.25)
 def test_existing_stroke_preload_limits_and_lid_clearance_preserved(self):
  c=self.g['grip_commands']['carrier'];self.assertEqual(c['extra_travel_per_jaw_mm'],.05);self.assertEqual(c['penetration_stop_limit_mm'],.25);self.assertGreater(c['closed_grasp_command_per_jaw_m'],0);self.assertLess(c['closed_grasp_command_per_jaw_m'],.006);self.assertGreaterEqual(self.p['lid_clearance_mm'],2.5)

class EntryAndSlotMarginChecks(unittest.TestCase):
 def test_fingertip_has_two_mm_tracking_margin_above_bridge(self):
  d=json.loads(Path(__file__).with_name('design.json').read_text());self.assertGreater(d['carrier']['tcp_above_bottom_mm']-11.,2.);self.assertGreaterEqual(d['lid_internal_clearance_above_tray_handle_mm'],2.5)
 def test_actual_bridge_leg_meshes_clear_base_slot(self):
  import math
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());legs=[p for p in g['parts']['carrier']['colliders'] if p['role']=='bridge_leg']
  halfwidth=max(abs(v[0]) for p in legs for v in p['vertices_mm']);self.assertAlmostEqual(halfwidth,7.)
  # Existing18mm base slot;0.75mm XY and1degree yaw are unchanged placement limits.
  worst=halfwidth*math.cos(math.radians(1))+29*math.sin(math.radians(1))+.75
  self.assertGreaterEqual(9-halfwidth,2.);self.assertGreater(9-worst,.7)

class ProbeInteriorChecks(unittest.TestCase):
 def test_narrowed_post_probe_is_inside_material_not_edge(self):
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());v=next(v for v in g['geometry_probes'] if v['label']=='negative_usb_bridge_post');self.assertEqual(v['start'][0],.006);self.assertEqual(v['expected_distance_mm'],87.);self.assertEqual(v['maximum_distance_error_mm'],.03)

class LidCentreProbeChecks(unittest.TestCase):
 def test_centre_bar_probe_uses_bar_height_not_flange(self):
  g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text());v=next(v for v in g['geometry_probes'] if v['part']=='cover' and v['label']=='spool');stem=next(v for v in g['parts']['cover']['colliders'] if v['role']=='grip_stem');top=max(v[2] for v in stem['vertices_mm']);self.assertAlmostEqual(v['expected_distance_mm'],v['start'][2]*1000-top,places=4);self.assertEqual(v['maximum_distance_error_mm'],.03)
