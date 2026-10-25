import unittest,ast
from pathlib import Path
class InitializationCoverageChecks(unittest.TestCase):
 def test_monitor_is_installed_before_any_physics_reset(self):
  s=Path(__file__).with_name('holder_assembly_trial.py').read_text();tree=ast.parse(s)
  reset=[];subscription=[];application=[]
  for node in ast.walk(tree):
   if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
    if isinstance(node.func.value,ast.Name) and node.func.value.id=='world' and node.func.attr=='reset':reset.append(node.lineno)
    if node.func.attr=='subscribe_contact_report_events':subscription.append(node.lineno)
    if node.func.attr=='Apply' and isinstance(node.func.value,ast.Attribute) and node.func.value.attr=='PhysxContactReportAPI':application.append(node.lineno)
  self.assertEqual(len(reset),1);self.assertEqual(len(subscription),1);self.assertEqual(len(application),1)
  self.assertLess(application[0],reset[0]);self.assertLess(subscription[0],reset[0])
if __name__=='__main__':unittest.main()
