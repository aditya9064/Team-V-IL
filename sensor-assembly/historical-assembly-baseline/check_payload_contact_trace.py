"""Validate telemetry integrity for a failed OR precontact-only original outcome.

Never promotes failure to pass and does not certify mechanical success.
"""
import argparse,hashlib,json,math
from pathlib import Path
from payload_contact_trace import SCHEMA,ABSENCE,selected_pair

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def finite_tree(value):
    if isinstance(value,float):assert math.isfinite(value),'Non-finite JSON number'
    elif isinstance(value,dict):
        for x in value.values():finite_tree(x)
    elif isinstance(value,list):
        for x in value:finite_tree(x)

def verify_source(source):
    source=Path(source)
    allowed={'assembly_suite_on_rosie.sh','assembly_review_on_rosie.sh','snapshot-sha256.json'}
    for n,h in json.loads((source/'v52-parent-sha256.json').read_text()).items():
        assert digest(source/n)==h or n in allowed,n
    parent=(source/'contact_trace_lid_precontact_trial.py').read_text()
    child=(source/'payload_contact_lid_precontact_trial.py').read_text()
    for old,new in reversed(json.loads((source/'payload-contact-runtime-patches.json').read_text())):
        assert child.count(new)==1;child=child.replace(new,old)
    assert child==parent,'Payload patches must recover exact V52 runtime'
    contract=json.loads((source/'PAYLOAD-CONTACT-CONTRACT.json').read_text())
    for n,h in contract['immutable_candidate_sha256'].items():assert digest(source/n)==h,n
    for n,h in json.loads((source/'snapshot-sha256.json').read_text()).items():assert digest(source/n)==h,n
    return digest(source/'payload_contact_lid_precontact_trial.py')

def validate_cooked(value,status):
    assert value['schema']=='POST_TERMINAL_PAYLOAD_COOKING_REPRESENTATION_V1'
    assert value['original_status']==status and value['complete'] is True and not value['errors']
    assert value['physics_or_target_use'] is False and value['may_obtain_or_recompute_cached_cooking_data'] is True
    assert value['kind']=='cooking-service convex representation, NOT direct runtime PxShape readback'
    finite_tree(value)
    assert value['sim_time_before_s']==value['sim_time_after_s']
    assert value['expected_paths'] and sorted(value['colliders'])==value['expected_paths']
    for path,item in value['colliders'].items():
        assert path.startswith(item['actor']+'/') and item['actor'] in ('/World/Assembly_pcb','/World/Assembly_carrier')
        assert item['callback_count']==1 and item['result_valid'] is True and item['convexes']
        for key in ('mesh_to_world_matrix','actor_to_world_matrix','mesh_to_actor_matrix'):
            assert len(item[key])==4 and all(len(row)==4 for row in item[key])
        for convex in item['convexes']:
            vertices=convex['vertices_mesh_local'];indices=convex['indices']
            assert len(vertices)>=4 and all(len(v)==3 for v in vertices) and convex['polygons']
            assert all(isinstance(i,int) and not isinstance(i,bool) and 0<=i<len(vertices) for i in indices)
            for face in convex['polygons']:
                assert face['index_base']>=0 and face['num_vertices']>=3 and face['index_base']+face['num_vertices']<=len(indices)
    return len(value['colliders'])

def validate_trace(r,source=None):
    """The returned success means trace integrity only; original outcome retained."""
    assert r['diagnostic_stage']=='precontact' and r['full_assembly_accepted'] is False
    assert r['physical_hardware_test'] is False and r['print_release'] is False
    assert r['status'] in ('FAILED','DIAGNOSTIC_PASS')
    if r['status']=='FAILED':assert isinstance(r.get('error'),str) and r['error']
    else:assert r['diagnostic_stage_accepted']=='precontact'
    p=r['payload_contact_trace_provenance'];t=r['payload_contact_trace']
    assert p['control_or_truth_steering'] is False and p['extra_physics_queries_or_registration'] is False and p['runtime_logging_IO_per_step'] is False
    assert t['schema']==SCHEMA and t['absence_semantics']==ABSENCE and t['physics_or_target_use'] is False
    assert t['complete'] is True and not t['errors'],'Telemetry incomplete; original outcome is unchanged'
    assert t['original_runtime_status']==r['status'] and t['original_runtime_error']==r.get('error')
    finite_tree(t)
    if source is not None:
        source=Path(source);assert verify_source(source)==r['trial_script_sha256']
        assert p['parent_runtime_sha256']==digest(source/'contact_trace_lid_precontact_trial.py')
        assert p['helper_sha256']==digest(source/'payload_contact_trace.py')
        assert p['cooked_reader_sha256']==digest(source/'cooked_payload_evidence.py')
        validate_cooked(r['payload_cooked_geometry'],r['status'])
        assert r['mesh_sha256']==digest(source/'nest-mesh.json') and r['design_sha256']==digest(source/'nest-design.json')
    rows=t['rows']
    if not rows:
        assert t['capture_scope_reached'] is False and not r['payload_contact_pose_trace']
        return {'telemetry_integrity_valid':True,'capture_scope_reached':False,'original_outcome':r['status'],'original_error':r.get('error'),'mechanical_success_checked_here':False,'full_assembly_accepted':False}
    assert t['capture_scope_reached'] is True
    first=r['payload_contact_pose_trace'][0];assert first['phase']=='2_carrier_orient'
    assert rows[0]['state_step']==first['step'] and rows[0]['pre_step_time_s']==first['sim_time']
    trace=r['payload_contact_pose_trace'];latest=t['last_existing_snapshot'];by_step={x['step']:x for x in trace}
    if r['status']=='FAILED':
        post=r['contact_timing_diagnostics']['post_step'];assert rows[-1]['state_step']==post['step']
        assert latest['step']==post['step'] and latest['sim_time']==post['sim_time'] and latest['actor_world_matrices']==post['actor_world_matrices']
    assert abs(latest['sim_time']-rows[-1]['post_step_time_s'])<=2e-6
    assert t['first_step']==rows[0]['state_step'] and t['last_step']==rows[-1]['state_step']
    assert t['first_pre_step_time_s']==rows[0]['pre_step_time_s'] and t['last_post_step_time_s']==rows[-1]['post_step_time_s']
    all_callbacks=[];total_headers=selected_headers=selected_points=0;previous=None;zero_rows=0
    rolemap=None
    if source is not None:
        mesh=json.loads((source/'nest-mesh.json').read_text());rolemap={f'/World/Assembly_{part}/C_{i:03d}_{c["role"]}':[part,c['role']] for part,v in mesh['parts'].items() for i,c in enumerate(v['colliders'])}
    for index,row in enumerate(rows):
        assert row['closed'] is True and abs(row['post_step_time_s']-row['pre_step_time_s']-1/60)<=2e-6
        if previous is not None:assert row['state_step']==previous['state_step']+1 and abs(row['pre_step_time_s']-previous['post_step_time_s'])<=2e-6
        previous=row;link=row['pre_step_snapshot_link'];assert link['collection']=='payload_contact_pose_trace'
        pre=trace[link['index']];assert pre['step']==link['step']==row['state_step'] and pre['sim_time']==link['sim_time']==row['pre_step_time_s']
        after=by_step.get(row['state_step']+1,latest);assert abs(after['sim_time']-row['post_step_time_s'])<=2e-6
        for actor in [t['pcb_root'],*t['carrier_roots']]:
            for snapshot in (pre,after):
                matrix=snapshot['actor_world_matrices'][actor];assert len(matrix)==4 and all(len(x)==4 for x in matrix);finite_tree(matrix)
        callbacks=row['callbacks'];assert row['callback_count']==len(callbacks)
        assert row['pcb_carrier_header_count']==len(row['headers']) and row['pcb_carrier_point_count']==sum(len(h['points']) for h in row['headers'])
        assert row['reported_record_state']==('REPORTED_RECORDS' if row['headers'] else 'NO_REPORTED_RECORD')
        zero_rows+=int(not row['headers']);assert row['header_count_total']==sum(c['header_count_total'] for c in callbacks)
        total_headers+=row['header_count_total'];selected_headers+=row['pcb_carrier_header_count'];selected_points+=row['pcb_carrier_point_count']
        for callback in callbacks:
            assert callback['closed'] is True and callback['row_index']==index and callback['state_step']==row['state_step']
            assert row['pre_step_time_s']-2e-6<=callback['callback_world_time_s']<=row['post_step_time_s']+2e-6
            selected=[h for h in row['headers'] if h['callback_sequence']==callback['sequence']]
            assert len(selected)==callback['pcb_carrier_header_count'] and sum(h['num_contact_data'] for h in selected)==callback['pcb_carrier_point_count']
            ordinals=[h['header_ordinal'] for h in selected];assert ordinals==sorted(set(ordinals)) and all(0<=i<callback['header_count_total'] for i in ordinals)
            for h in selected:
                assert h['state_step']==row['state_step'] and h['phase']==callback['phase'] and h['callback_world_time_s']==callback['callback_world_time_s']
                assert h['selection']==selected_pair(h['actors'],h['colliders'],t['pcb_root'],t['carrier_roots']) and h['selection'] is not None
                assert all(isinstance(x,str) and x.startswith('/') for x in h['actors']+h['colliders'])
                assert all(isinstance(x,str) and str(int(x))==x for x in h['actor_ids_decimal']+h['collider_ids_decimal'])
                assert isinstance(h['event_type_code'],int) and not isinstance(h['event_type_code'],bool) and isinstance(h['event_type_repr'],str)
                assert h['event_type_name']['available'] is (h['event_type_name']['value'] is not None)
                assert h['num_contact_data']==len(h['points']) and h['num_contact_data']>=0
                assert [v['raw_data_index'] for v in h['points']]==list(range(h['contact_data_offset'],h['contact_data_offset']+h['num_contact_data']))
                for point in h['points']:
                    assert all(len(point[k])==3 for k in ('position_world_m','normal_world_raw','impulse_Ns_raw'))
                    assert point['separation_mm']==point['separation_m']*1000
                    for optional in point['optional_fields'].values():assert optional['available'] is (optional['value'] is not None)
                assert h['stage_id']['available'] is (h['stage_id']['value'] is not None)
                if rolemap is not None:assert h['roles']==[rolemap.get(x) for x in h['colliders']]
            all_callbacks.append(callback)
    assert [x['sequence'] for x in all_callbacks]==list(range(1,len(all_callbacks)+1))
    assert t['callback_count']==len(all_callbacks) and t['header_count_total']==total_headers and t['selected_header_count']==selected_headers and t['selected_point_count']==selected_points
    return {'telemetry_integrity_valid':True,'capture_scope_reached':True,'steps':len(rows),'callbacks':len(all_callbacks),'selected_headers':selected_headers,'selected_points':selected_points,'zero_matching_header_steps':zero_rows,'absence_semantics':ABSENCE,'original_outcome':r['status'],'original_error':r.get('error'),'mechanical_success_checked_here':False,'full_assembly_accepted':False}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('report');args=parser.parse_args()
    result=validate_trace(json.loads(Path(args.report).read_text()),Path(__file__).parent)
    print('TEAM_V_PAYLOAD_CONTACT_TELEMETRY_INTEGRITY_ONLY',json.dumps(result,allow_nan=False))
