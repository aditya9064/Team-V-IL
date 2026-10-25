import ast,copy,json,math,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from enum import IntEnum
from lid_contact_trace import ContactTrace,selected_pair,ABSENCE
from check_contact_trace_lid_precontact import validate_trace,verify_source

COVER='/World/Assembly_cover';LEFT='/World/ER4iA/JawL';RIGHT='/World/ER4iA/JawR'
PATHS={1:COVER,2:LEFT,3:RIGHT,4:COVER+'/C_053_grip_support',5:LEFT+'/Collision/Part02',6:RIGHT+'/Collision/Part68',7:'/World/Assembly_base',8:'/World/Assembly_base/C_009_floor'}
ROLES={PATHS[4]:('cover','grip_support'),PATHS[8]:('base','floor')}
class Event(IntEnum):FOUND=1;PERSISTS=2;LOST=3
def header(reverse=False,lost=False):
    return NS(actor0=2 if reverse else 1,actor1=1 if reverse else 2,collider0=5 if reverse else 4,collider1=4 if reverse else 5,
              type=Event.LOST if lost else Event.PERSISTS,stage_id=7,contact_data_offset=-1 if lost else 1,num_contact_data=0 if lost else 1)
def point():return NS(position=[.3,.1,1.02],normal=[0,-1,0],impulse=[-.001,.002,-.003],separation=-.00003,face_index0=4)
def snapshot(step,time):return {'step':step,'sim_time':time,'actor_world_matrices':{a:[[1.,0,0,0],[0,1.,0,0],[0,0,1.,0],[.3,.1,1.0,1.]] for a in (COVER,LEFT,RIGHT)}}
def rowstart(t,step,time,index,activate=True):
    s=snapshot(step,time);t.note_existing_snapshot(s);t.begin_step(s,index,'3_cover_level_held',activate)
def capture(t,h,data,time):
    token=t.begin_callback(t.current['state_step'],'3_cover_level_held',time);t.observe_header(token,h,data,PATHS.__getitem__,ROLES);t.end_callback(token)

class LidContactTraceTests(unittest.TestCase):
    def test_both_orders_exact_subtrees_and_raw_signs(self):
        t=ContactTrace(COVER,[LEFT,RIGHT]);rowstart(t,10,1.,0)
        for reverse in (False,True):capture(t,header(reverse),[None,point()],1.)
        t.end_step(10,1+1/60);t.finalize('FAILED','original gate')
        hs=t.value['rows'][0]['headers'];self.assertEqual(len(hs),2)
        self.assertEqual(hs[0]['actors'],[COVER,LEFT]);self.assertEqual(hs[1]['actors'],[LEFT,COVER]);self.assertEqual(hs[1]['selection']['cover_index'],1)
        for h in hs:
            p=h['points'][0];self.assertEqual(p['normal_world_raw'],[0,-1,0]);self.assertEqual(p['impulse_Ns_raw'],[-.001,.002,-.003]);self.assertAlmostEqual(p['separation_mm'],-.03);self.assertIn('PERSISTS',h['event_type_repr'])
            self.assertEqual(p['optional_fields']['face_index0'],{'available':True,'value':4});self.assertEqual(p['optional_fields']['face_index1'],{'available':False,'value':None})
        self.assertIsNone(selected_pair([COVER,LEFT+'Extra'],[PATHS[4],LEFT+'Extra/Collision/x'],COVER,[LEFT,RIGHT]))
        self.assertIsNone(selected_pair([COVER,PATHS[7]],[PATHS[4],PATHS[8]],COVER,[LEFT,RIGHT]))

    def test_lost_zero_points_and_two_callbacks_keep_headers(self):
        class NoData:
            def __getitem__(self,index):raise AssertionError('Zero-point header read data')
        t=ContactTrace(COVER,[LEFT,RIGHT]);rowstart(t,10,1.,0)
        capture(t,header(lost=True),NoData(),1.);capture(t,header(lost=True),NoData(),1.)
        t.end_step(10,1+1/60)
        row=t.value['rows'][0];self.assertEqual(row['callback_count'],2);self.assertEqual(row['cover_jaw_header_count'],2)
        self.assertEqual([x['callback_sequence'] for x in row['headers']],[1,2]);self.assertTrue(all(h['event_type_code']==3 and h['points']==[] and h['num_contact_data']==0 for h in row['headers']))

    def test_no_callback_empty_callback_and_unrelated_are_explicit_unknown(self):
        t=ContactTrace(COVER,[LEFT,RIGHT]);rowstart(t,10,1.,0);t.end_step(10,1+1/60)
        rowstart(t,11,1+1/60,1);token=t.begin_callback(11,'phase',1+1/60);t.end_callback(token);t.end_step(11,1+2/60)
        rowstart(t,12,1+2/60,2);h=header();h.actor1=7;h.collider1=8;capture(t,h,[None,point()],1+2/60);t.end_step(12,1+3/60)
        rows=t.value['rows'];self.assertEqual([r['callback_count'] for r in rows],[0,1,1]);self.assertEqual([r['header_count_total'] for r in rows],[0,0,1]);self.assertEqual([r['cover_jaw_header_count'] for r in rows],[0,0,0]);self.assertTrue(all(r['reported_record_state']=='NO_REPORTED_RECORD' for r in rows));self.assertIn('unknown',ABSENCE)

    def test_invalid_telemetry_isolated_and_no_truncation(self):
        t=ContactTrace(COVER,[LEFT,RIGHT]);rowstart(t,10,1.,0);token=t.begin_callback(10,'phase',1.);bad=point();bad.impulse=[0,float('nan'),0]
        original_processing=[]
        for h,data in [(header(),[None,bad]),(header(),[None,point()])]:
            t.observe_header(token,h,data,PATHS.__getitem__,ROLES);original_processing.append(h)
        t.end_callback(token);t.end_step(10,1+1/60);t.finalize('FAILED','original gate')
        self.assertEqual(len(original_processing),2);self.assertFalse(t.value['complete']);self.assertEqual(len(t.value['errors']),1);self.assertEqual(t.value['header_count_total'],2);self.assertEqual(t.value['selected_header_count'],1)
        self.assertEqual(t.value['rows'][0]['headers'][0]['header_ordinal'],1);json.dumps(t.value,allow_nan=False)
        q=ContactTrace(COVER,[LEFT,RIGHT]);rowstart(q,10,1.,0);q.finalize('FAILED','step failure');self.assertFalse(q.value['complete'])

    def synthetic_report(self):
        t=ContactTrace(COVER,[LEFT,RIGHT]);pre=snapshot(10,1.);t.begin_step(pre,0,'phase',True);capture(t,header(),[None,point()],1.);capture(t,header(lost=True),[],1.);t.end_step(10,1+1/60)
        second=snapshot(11,1+1/60);t.note_existing_snapshot(second);t.begin_step(second,1,'phase',False);t.end_step(11,1+2/60)
        post=snapshot(11,1+2/60);t.note_existing_snapshot(post);t.finalize('FAILED','Lid grasp per-foot3D drift veto')
        return {'diagnostic_stage':'precontact','full_assembly_accepted':False,'physical_hardware_test':False,'print_release':False,'status':'FAILED','error':'Lid grasp per-foot3D drift veto',
                'lid_contact_trace_provenance':{'control_or_truth_steering':False,'extra_physics_queries_or_registration':False,'runtime_logging_IO_per_step':False},'lid_contact_trace':t.value,
                'incoming_lid_stability':{'start_step':9,'start_sim_time':1.},'diagnostic_trace':[pre,second],'contact_timing_diagnostics':{'post_step':post}}

    def test_failed_outcome_checker_never_promotes_and_rejects_missing_bad_data(self):
        r=self.synthetic_report();v=validate_trace(r);self.assertEqual(v['original_outcome'],'FAILED');self.assertFalse(v['mechanical_success_checked_here']);self.assertEqual(v['selected_headers'],2);self.assertEqual(v['selected_points'],1);self.assertEqual(v['zero_matching_header_steps'],1)
        for change in (lambda x:x['lid_contact_trace']['rows'].pop(),lambda x:x['lid_contact_trace']['rows'][0].update(cover_jaw_header_count=0),lambda x:x['lid_contact_trace']['rows'][0]['headers'][0]['points'][0]['impulse_Ns_raw'].__setitem__(0,float('nan')),lambda x:x['lid_contact_trace']['rows'][0]['pre_step_snapshot_link'].update(index=1)):
            bad=copy.deepcopy(r);change(bad)
            with self.assertRaises((AssertionError,IndexError,ValueError)):validate_trace(bad)

    def test_source_exact_reversible_parent_identity_and_no_extra_physics_calls(self):
        p=Path(__file__).parent;verify_source(p);old=ast.parse((p/'multistart_lid_precontact_trial.py').read_text());new=ast.parse((p/'contact_trace_lid_precontact_trial.py').read_text())
        def funcs(tree):return {n.name:ast.dump(n) for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
        a,b=funcs(old),funcs(new)
        for name in a:
            if name not in ('main','contact_pose_snapshot','on_contacts','advance'):self.assertEqual(a[name],b[name],name)
        def calls(tree):return [ast.dump(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and (isinstance(n.func.value,ast.Name) and n.func.value.id in ('world','arm','stage') or n.func.attr in ('Apply','subscribe_contact_report_events'))]
        self.assertEqual(calls(old),calls(new));helper=ast.parse((p/'lid_contact_trace.py').read_text());self.assertEqual([n.names[0].name for n in ast.walk(helper) if isinstance(n,ast.Import)],['math'])

if __name__=='__main__':unittest.main()
