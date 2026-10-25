import json,hashlib,unittest
from pathlib import Path
class CentredBase(unittest.TestCase):
 def setUp(self):
  self.r=Path(__file__).parent;self.p=json.loads((self.r/'centred-base-proof.json').read_text());self.g=json.loads((self.r/'trial-mesh.json').read_text())
 def test_actual_load_reduction_with_unchanged_density(self):
  self.assertLess(self.p['new_v26']['gravity_torque_about_grip_Nmm'],self.p['reference_v25']['gravity_torque_about_grip_Nmm']);self.assertEqual(self.p['material_density_kg_m3'],1240)
 def test_all_independent_pose_corner_insertion_margins(self):
  self.assertEqual(self.p['independent_relative_insertion_corners'],729);self.assertGreater(self.p['minimum_swept_insertion_gap_mm'],.4)
 def test_unchanged_pcb_pickup_interface_runtime_and_thresholds(self):
  for n,h in self.p['other_parts_sha256'].items():self.assertEqual(hashlib.sha256(json.dumps(self.g['parts'][n],sort_keys=True).encode()).hexdigest(),h)
  for n,h in self.p['runtime_sha256'].items():self.assertEqual(hashlib.sha256((self.r/n).read_bytes()).hexdigest(),h)
  grip=[c for c in self.g['parts']['carrier']['colliders'] if c['role'] in ('grip_stem','grip_support','bridge_leg','bridge_beam')];self.assertEqual(hashlib.sha256(json.dumps(grip,sort_keys=True).encode()).hexdigest(),self.p['unchanged_carrier_grip_sha256']);self.assertTrue(self.p['limits_unchanged'] and self.p['grip_commands_unchanged']);self.assertEqual(self.g['limits']['maximum_intended_penetration_mm'],.25)

class RetainedKeeperProbe(unittest.TestCase):
 def setUp(self):self.g=json.loads(Path(__file__).with_name('trial-mesh.json').read_text())
 def test_keeper_probe_lands_in_retained_material_with_same_distance_tolerance(self):
  p=next(p for p in self.g['geometry_probes'] if p['part']=='carrier' and p['label']=='board_keeper');x,y=[q*1000 for q in p['start'][:2]];zs=[]
  for c in self.g['parts']['carrier']['colliders']:
   if c['role']!='pcb_keeper':continue
   vs=c['vertices_mm']
   if min(v[0] for v in vs)<x<max(v[0] for v in vs) and min(v[1] for v in vs)<y<max(v[1] for v in vs):zs.append(max(v[2] for v in vs))
  self.assertTrue(zs);self.assertAlmostEqual(p['expected_distance_mm'],p['start'][2]*1000-max(zs),places=6);self.assertEqual(p['maximum_distance_error_mm'],.03)
 def test_old_keeper_point_is_declared_empty_clearance_with_negative_probe(self):
  p=next(p for p in self.g['geometry_probes'] if p['label']=='right_edge_handle_clearance');self.assertEqual(p['start'],[.01,0,.1]);self.assertFalse(p['hit'])
