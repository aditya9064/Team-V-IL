"""Bind native full startup and immutable physical/controller parent sources."""
import ast,copy,hashlib,json,unittest
from pathlib import Path
from types import SimpleNamespace
from test_full_motion_dispatch import loop_node
H=Path(__file__).resolve().parent
def tree(name):return ast.parse((H/name).read_text())
def part_loop(t):return next(n for n in ast.walk(t) if isinstance(n,ast.For) and ast.unparse(n.target)=='(index, name)' and ast.unparse(n.iter)=='enumerate(physical_parts)')
class Migration(unittest.TestCase):
 def test_frozen_parents_and_unchanged_original_CAD(self):
  for name,h in {'full-prefix-parent-v53.py':'3f04770b8e4099fdaee59886b5af92521eafbe2c60f7f115da025d335c87a0e0','lid-parent-v59.py':'c315b3a399b47ea85bee81a660105234532f49b9f202e43dc421c284ad64100b','nest-mesh.json':'728f96ba4be877e8d768ab57a5505a5c2c6be08e750791014274241d3bb9fcd5','nest-design.json':'3808dbe0fd8bbda6f6990e46b796cde1d38055c4fdf84450cccd39a858acc62a'}.items():self.assertEqual(hashlib.sha256((H/name).read_bytes()).hexdigest(),h,name)
 def test_all_body_construction_is_exact_original_full_case(self):
  new=part_loop(tree('full_observed_assembly_trial.py'));old=copy.deepcopy(part_loop(tree('full-prefix-parent-v53.py')))
  for node in ast.walk(old):
   if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='xy':node.value=ast.parse("layout['pickup_body_centres'][name]",mode='eval').body
   if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name) and node.targets[0].id=='z':node.value=ast.parse("surface+(0. if name=='nest' else .0003)",mode='eval').body
  self.assertEqual(ast.dump(new),ast.dump(old))
 def test_actual_initial_coordinates_are_separate_products_and_loose_PCB(self):
  loop=part_loop(tree('full_observed_assembly_trial.py'));g=json.loads((H/'nest-mesh.json').read_text());d=json.loads((H/'nest-design.json').read_text());surface=.8949;actual={}
  # Execute only actual coordinate expressions, not a duplicate recipe.
  expressions=loop.body[:6]
  for name in ['base','carrier','pcb','cover','nest']:
   s={'layout':g['layout_m'],'design':d,'geometry':g,'surface':surface,'name':name}
   selected=[n for n in expressions if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('xy','z') for t in n.targets) or isinstance(n,ast.If) and ast.unparse(n.test)=="name == 'pcb'"]
   exec(compile(ast.Module(body=selected,type_ignores=[]),'actual-initial-coordinates','exec'),s);actual[name]=(s['xy'],s['z'])
  self.assertEqual(actual['base'][0],[.22,-.14]);self.assertEqual(actual['carrier'][0],actual['pcb'][0]);self.assertEqual(actual['cover'][0],[.43,.04]);self.assertEqual(actual['nest'][0],[.3,.1]);self.assertAlmostEqual(actual['pcb'][1]-actual['carrier'][1],(d['carrier']['floor_thickness']+g['payload_initial_gap_mm'])*.001)
 def test_support_pairs_and_inventory_start_with_only_nest(self):
  nodes=list(ast.walk(tree('full_observed_assembly_trial.py')))
  support=next(n for n in nodes if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='support_pairs' for t in n.targets));self.assertEqual(ast.unparse(support.value),"initial_supports_for_case('full-assembly')")
  placed=next(n for n in nodes if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='placed' for t in n.targets));self.assertEqual(ast.literal_eval(placed.value),['nest'])
 def test_no_early_lid_capture_and_fixed_release_reference(self):
  loop=loop_node(H/'full_observed_assembly_trial.py');lines=ast.unparse(loop);self.assertLess(lines.index("state['isolated_contact_capture'] = False"),lines.index("move(prefix + '_orient'"));self.assertLess(lines.index("state['isolated_contact_capture'] = True"),lines.index("close = move(prefix + '_close'"));self.assertEqual(lines.count("state['released_reference'] = None"),1)
 def test_policy_full_phase_and_checkpoint_scope(self):
  from check_full_observed_assembly import PHASES,DURATIONS
  p=json.loads((H/'full_motion_policy.json').read_text());self.assertEqual(p['phases'],PHASES);self.assertEqual(p['durations_s'],DURATIONS);self.assertEqual(len(PHASES),41);self.assertEqual(len(p['checkpoints']),52);self.assertEqual(p['only_preplaced_fixture'],['nest']);self.assertFalse(p['full_assembly_accepted_by_runtime'])
if __name__=='__main__':unittest.main()
