"""Synthetic offline checker regressions; no simulator or acceptance evidence."""
import ast,copy,json,unittest
from pathlib import Path
from unittest.mock import patch
import check_supported_saddle_grip as C
import test_paired_lid_grip as F
import paired_trace_integrity as T
HERE=Path(__file__).parent
G=json.loads((HERE/'lid-saddle-mesh.json').read_text());D=json.loads((HERE/'lid-saddle-design.json').read_text())
def identity(variant='supported-saddle'):
    r=F.identity_fixture();r['task']='isolated_lid_supported_saddle_diagnostic';r['grip_geometry_experiment'].update(variant=variant,extension_mm=0.,supported_saddle_proof_sha256=C.digest(HERE/'supported-saddle-proof.json'),supported_saddle_clearance_sha256=C.digest(HERE/'supported-saddle-clearance.json'))
    mesh,design,mass=C.VARIANTS[variant]
    r.update(mesh_source_filename=mesh,design_source_filename=design,mass_reference_source_filename=mass,mesh_sha256=C.digest(HERE/mesh),design_sha256=C.digest(HERE/design))
    return r
def physics():
    r=F.physics_fixture();reference=json.loads((HERE/'lid-saddle-mass-reference.json').read_text());r['installed_full_body_mass_properties']=reference
    for v in reference.values():v.update(validated=True,physics_time_advanced_s=0.)
    for name,b in r['separate_body_audit'].items():b['mass_kg']=G['parts'][name]['volume_mm3']*1e-9*1240;r['physical_collider_counts'][name]=len(G['parts'][name]['colliders'])
    r['cooked_geometry_probes']=[]
    for p in G['geometry_probes']:
        q={'part':p['part'],'label':p['label'],'expected_part_hit':p['hit'],'actual_part_hit':p['hit'],'distance_pass':True}
        if p['hit']:q.update(expected_distance_mm=p['expected_distance_mm'],actual_distance_mm=p['expected_distance_mm'],maximum_distance_error_mm=p.get('maximum_distance_error_mm',.03))
        r['cooked_geometry_probes'].append(q)
    return r
class SaddleTests(unittest.TestCase):
    def test_original_and_saddle_identity(self):
        C.check_identity(identity('original'),F.G,HERE,1,'original');C.check_identity(identity(),G,HERE,1,'supported-saddle')
    def test_wrong_source_variant_rejected(self):
        r=identity();r['mesh_sha256']=C.digest(HERE/'nest-mesh.json')
        with self.assertRaises(C.AuditError):C.check_identity(r,G,HERE,1,'supported-saddle')
    def test_unrelated_stem_extension_rejected(self):
        r=identity();r['grip_geometry_experiment']['extension_mm']=3.
        with self.assertRaises(C.AuditError):C.check_identity(r,G,HERE,1,'supported-saddle')
    def test_new_mass_and_probes_pass(self):C.check_original_physics(physics(),G,D,True,HERE,'supported-saddle')
    def test_old_mass_cannot_substitute_for_new_lid(self):
        r=physics();r['installed_full_body_mass_properties']['cover']=copy.deepcopy(F.physics_fixture()['installed_full_body_mass_properties']['cover'])
        with self.assertRaises(C.AuditError):C.check_original_physics(r,G,D,True,HERE,'supported-saddle')
    def test_missing_new_probe_rejected(self):
        r=physics();r['cooked_geometry_probes'].pop()
        with self.assertRaises(C.AuditError):C.check_original_physics(r,G,D,True,HERE,'supported-saddle')
    def test_stale_self_consistent_guide_hit_rejected(self):
        r=physics();p=next(p for p in r['cooked_geometry_probes'] if p['label']=='lid_carrier_guide_inner_1');p.update(expected_part_hit=True,actual_part_hit=True,expected_distance_mm=3.2,actual_distance_mm=3.2,maximum_distance_error_mm=.03)
        with self.assertRaises(C.AuditError):C.check_original_physics(r,G,D,True,HERE,'supported-saddle')
    def test_contact_limit_preserved(self):
        r=physics();r['contact_summary']['synthetic grip']['minimum_separation_mm']=-.251
        with self.assertRaises(C.AuditError):C.check_original_physics(r,G,D,True,HERE,'supported-saddle')
    def test_failed_archived_hold_still_fails_for_same_lid_feet(self):
        r={'incoming_lid_stability':F.D['incoming'],'diagnostic_trace':F.D['pose_trace_level_and_hold'],'lid_contact_trace':{'last_existing_snapshot':F.D['post_step']}}
        self.assertGreater(C.check_incoming(r,G,HERE)['fixed_maximum_mm'],.02)
    def test_runtime_exact_reverse_and_controller_AST(self):
        now=(HERE/'isolated_lid_grip_trial.py').read_text();text=now
        for p in reversed(json.loads((HERE/'v57-runtime-patches.json').read_text())):
            self.assertEqual(text.count(p['new']),1);text=text.replace(p['new'],p['old'])
        self.assertEqual(text,(HERE/'v55_isolated_lid_grip_trial.py').read_text())
        normalize=lambda t:t.replace("'SIMULATION | lid grip '+a.geometry_variant+' | current fingers'","'SIMULATION | lid post '+a.geometry_variant+' | current fingers'")
        fs=lambda t:{n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(normalize(t))) if isinstance(n,ast.FunctionDef) and n.name!='main'}
        self.assertEqual(fs(now),fs(text))
    def test_missing_or_wrong_proof_binding_rejected_before_trace(self):
        for key in ['supported_saddle_proof_sha256','supported_saddle_clearance_sha256']:
            r=identity();r.update(status='FAILED',error='synthetic unrelated failure',payload_contact_trace={'complete':True,'errors':[],'capture_scope_reached':False,'rows':[]});r.pop('diagnostic_stage_accepted');r['grip_geometry_experiment'][key]='0'*64
            with patch.object(C,'verify_source',return_value=(G,D)),patch.object(T,'validate_trace',side_effect=AssertionError('Trace must not be reached')):
                with self.assertRaises(C.AuditError):C.validate_report(r,HERE,C.digest(HERE/'isolated_lid_grip_trial.py'),1,False,'supported-saddle')
if __name__=='__main__':unittest.main()
