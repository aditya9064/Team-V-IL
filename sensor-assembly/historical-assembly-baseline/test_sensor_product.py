import json,unittest
from pathlib import Path
from payload_checks import initial_supports_for_case
from assembly_checks import role_pair
class SensorProductChecks(unittest.TestCase):
 def setUp(self):self.d=json.loads(Path(__file__).with_name('design.json').read_text())
 def test_printed_board_is_not_real_electronics(self):
  b=self.d['board_print_assumptions'];self.assertTrue(b['not_functional'] and b['real_component_height_unverified'] and b['real_mass_unverified']);self.assertEqual(b['model_xy_mm'],[17.76,43.16])
 def test_no_gripper_change_and_no_print_release(self):
  self.assertFalse(self.d['current_fingers_changed']);self.assertFalse(self.d['print_release'])
 def test_reduced_base_lever_and_opposite_tray_exit(self):
  self.assertEqual(self.d['base']['handle_centre_xy'],[16.5,0]);self.assertEqual(self.d['carrier']['side_entry_mm'],-18.)
 def test_all_cad_sweeps_and_fits_pass(self):
  self.assertGreaterEqual(len(self.d['cad_checks']),100);self.assertTrue(all(v['passed'] and v['intersection_mm3']<1e-4 for v in self.d['cad_checks']));self.assertTrue(all(v['interference_mm3']<1e-4 for v in self.d['nominal_product_fit']))
if __name__=='__main__':unittest.main()

class InitialFixtureCaseChecks(unittest.TestCase):
 def test_preplaced_loaded_base_has_floor_support(self):
  self.assertIn(role_pair(('base','floor'),('nest','floor')),initial_supports_for_case('loaded-carrier'))
 def test_full_trial_base_is_not_preplaced(self):
  self.assertNotIn(role_pair(('base','floor'),('nest','floor')),initial_supports_for_case('full-assembly'))
 def test_no_sidewall_or_robot_exemption(self):
  allowed=initial_supports_for_case('loaded-carrier')
  self.assertNotIn(role_pair(('base','wall'),('nest','wall')),allowed)
  with self.assertRaises(ValueError):initial_supports_for_case('typo')
