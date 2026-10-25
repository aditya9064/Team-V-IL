"""Only replace base estimator import/hash/provenance, preserving V50 control."""
import json
from pathlib import Path
p=Path(__file__).parent;s=(p/'continuous_lid_precontact_trial.py').read_text();patches=[]
def patch(a,b):
 global s
 assert s.count(a)==1,(a[:100],s.count(a));s=s.replace(a,b);patches.append([a,b])
patch('from held_pose_bounded import estimate,placement_from_held,project,front_edges','from held_pose_multistart import estimate\nfrom held_pose_bounded import placement_from_held,project,front_edges')
patch("report['held_pose_estimator_sha256']=digest(Path(__file__).with_name('held_pose_bounded.py'))", "report['held_pose_estimator_sha256']=digest(Path(__file__).with_name('held_pose_multistart.py'))\n    report['base_multistart_original_estimator_sha256']=digest(Path(__file__).with_name('held_pose_bounded.py'))\n    report['base_multistart_scope']='Fixed25 original-objective image-only multistart with mandatory original zero-seed image gate; cost is not an accuracy certificate. All original repeatability/accuracy and motion gates remain. Simulation precontact only.'")
(p/'multistart_lid_precontact_trial.py').write_text(s);(p/'multistart-runtime-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
s=(p/'check_continuous_lid_precontact.py').read_text();patches=[]
patch("[('continuous_lid_precontact_trial.py','trial_script_sha256')", "[('multistart_lid_precontact_trial.py','trial_script_sha256')")
patch("assert r['held_pose_estimator_sha256']==digest(source/'held_pose_bounded.py')", "assert r['held_pose_estimator_sha256']==digest(source/'held_pose_multistart.py')\n    assert r['base_multistart_original_estimator_sha256']==digest(source/'held_pose_bounded.py')\n    design=json.loads((source/'nest-design.json').read_text())\n    for observation in r['held_pose_observations']:validate_multistart_observation(observation,Path(a.report).parent,g['parts']['base']['visual']['vertices_mm'],design)")
patch('def main():', '''def validate_multistart_observation(o,evidence,vertices,design):
    import cv2,numpy as np
    from scipy.spatial.transform import Rotation
    import held_pose_bounded as original
    bank=[[0.]*6]
    for magnitude in (.5,1.):
        for axis in range(6):
            for sign in (-1,1):
                seed=[0.]*6;seed[axis]=sign*magnitude;bank.append(seed)
    a=o['multistart'];assert a['protocol_sha256']=='b22cb0055a6f462454a0933086f3c33513de573cb5db7f918cf7755e4fbf85c1'
    assert a['fixed_seed_parameters_mm_xyz_deg']==bank and a['candidate_count']==len(a['candidates'])==25
    assert a['original_zero_seed_gate_required'] is True and a['original_zero_seed_gate_pass'] is True
    assert a['truth_inputs_or_selection'] is False and a['early_stop'] is False and a['adaptive_seed_bank'] is False
    options={'bounds':[[-2,-2,-2,-3,-3,-3],[2,2,2,3,3,3]],'diff_step':.01,'x_scale':[1,1,1,1,1,1],'max_nfev':400,'ftol':1e-8,'xtol':1e-8,'gtol':1e-8}
    assert json.loads(json.dumps(a['solver_options']))==options
    models=o['camera_models'];images={k:cv2.cvtColor(cv2.imread(str(Path(evidence)/(o['label']+'-'+k+'.png'))),cv2.COLOR_BGR2RGB) for k in models}
    assert o['camera_model_sha256']=={k:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest() for k,v in models.items()}
    assert o['CAD_vertices_mm_sha256']==hashlib.sha256(np.ascontiguousarray(vertices,dtype=np.float64).tobytes()).hexdigest()
    assert o['image_RGB_sha256']=={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in images.items()}
    nominal=np.array(o['tip_world_m'])-np.array([*np.array(design['base']['handle_centre_xy'])*.001,design['base']['tcp_above_bottom_mm']*.001])
    assert np.allclose(nominal,o['nominal_position_m'],atol=1e-12,rtol=0)
    captured={}
    class Captured(Exception):pass
    def capture(fun,x0,**kwargs):captured.update(fun=fun,kwargs=kwargs);raise Captured
    previous=original.least_squares;original.least_squares=capture
    try:
        try:original.estimate(images,models,vertices,nominal)
        except Captured:pass
    finally:original.least_squares=previous
    assert json.loads(json.dumps(captured['kwargs']))==options
    fun=captured['fun'];closure=dict(zip(fun.__code__.co_freevars,[cell.cell_contents for cell in fun.__closure__]));views=closure['views'];eligible=[]
    for i,c in enumerate(a['candidates']):
        assert c['seed_index']==i and c['seed_parameters_mm_xyz_deg']==bank[i]
        if c['offset_mm_and_xyz_deg'] is None:
            assert c['image_eligible'] is False and c['cost'] is None and c['optimizer'] is None and len(c['rejection_reasons'])==1 and c['rejection_reasons'][0].startswith('invalid_solver_or_output:');continue
        x=np.array(c['offset_mm_and_xyz_deg']);assert x.shape==(6,) and np.isfinite(x).all();errors=fun(x);assert errors.shape==(280,) and np.isfinite(errors).all()
        opt=c['optimizer'];assert math.isfinite(c['cost']) and c['cost']>=0 and math.isclose(c['cost'],float(errors@errors/2),rel_tol=1e-8,abs_tol=1e-8) and opt['cost']==c['cost'] and opt['max_nfev']==400 and math.isfinite(opt['optimality']) and opt['optimality']>=0
        assert isinstance(opt['success'],bool) and isinstance(opt['status'],int) and not isinstance(opt['status'],bool) and isinstance(opt['message'],str) and isinstance(opt['nfev'],int) and not isinstance(opt['nfev'],bool) and isinstance(opt['njev'],int) and not isinstance(opt['njev'],bool) and 0<opt['nfev']<=400 and 0<opt['njev']<=400
        assert np.allclose(c['position_world_m'],nominal+x[:3]*.001,atol=1e-12,rtol=0) and np.allclose(c['rotation_world'],Rotation.from_euler('xyz',x[3:],degrees=True).as_matrix(),atol=1e-12,rtol=0)
        reasons=[]
        if not opt['success']:reasons.append('solver_not_successful')
        for j,(name,model,v,d,contour) in enumerate(views):
            e=errors[j*140:(j+1)*140];score=c['views'][name]
            expected={'edge_rms_px':float(np.sqrt(np.mean(e**2))),'edge_p95_px':float(np.percentile(e,95)),'edge_coverage_within_2px':float(np.mean(e<=2)),'observed_contour_points':len(contour)}
            for key,value in expected.items():assert math.isclose(score[key],value,rel_tol=1e-8,abs_tol=1e-8),key
            if expected['edge_p95_px']>2:reasons.append(name+':p95_exceeds_2px')
            if expected['edge_coverage_within_2px']<.95:reasons.append(name+':coverage_below_.95')
        if np.any(abs(x[:3])>=1.95):reasons.append('translation_near_original_bound')
        if np.any(abs(x[3:])>=2.95):reasons.append('rotation_near_original_bound')
        assert c['rejection_reasons']==reasons and c['image_eligible'] is (not reasons)
        assert math.isclose(c['rms_px'],float(np.sqrt(np.mean(errors**2))),rel_tol=1e-8,abs_tol=1e-8)
        if not reasons:eligible.append(c)
    assert a['candidates'][0]['image_eligible'] is True
    assert eligible;selected=min(eligible,key=lambda c:(c['cost'],c['seed_index']));assert a['selected_seed_index_IMAGE_ONLY']==selected['seed_index']
    for key in ('position_world_m','rotation_world','offset_mm_and_xyz_deg','views','rms_px','optimizer'):assert o[key]==selected[key]


def main():''')
(p/'check_multistart_lid_precontact.py').write_text(s);(p/'multistart-checker-patches.json').write_text(json.dumps(patches,indent=2)+'\n')
