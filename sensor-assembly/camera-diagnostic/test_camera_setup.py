"""Synthetic evidence negatives only; no simulator or historical report edits."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation

SOURCE=Path(__file__).resolve().parents[1]/'source'
sys.path.insert(0,str(SOURCE))
import check_camera_setup as C
from continuous_lid_command import independent_fk
from reliability_fixture import spawn_transform, new_record, record_spawn

G=json.loads((SOURCE/'nest-mesh.json').read_text())
D=json.loads((SOURCE/'nest-design.json').read_text())


def fixture():
    poses={}; authored=new_record('b',C.sha(SOURCE/'reliability_fixture.py'))
    for name in ('base','carrier','pcb','cover','nest'):
        z=.8949+(0 if name=='nest' else .0003)
        if name=='pcb':z+=(D['carrier']['floor_thickness']+G['payload_initial_gap_mm'])*.001
        tr=spawn_transform(name,G['layout_m'],z,'b')
        m=np.eye(4);m[:3,:3]=Rotation.from_quat(np.array(tr['orientation_wxyz'])[[1,2,3,0]]).as_matrix().T
        m[3,:3]=tr['position_m'];record_spawn(authored,name,[*G['layout_m']['pickup_body_centres'][name],z],tr,m.tolist())
        # Physically settled fixture, distinct from authored .3mm startup gap.
        poses[name]={'position_world_m':[tr['position_m'][0],tr['position_m'][1],z-(0 if name=='nest' else .0003)],
                     'orientation_wxyz':tr['orientation_wxyz']}
    sdk=Path(__file__).with_name('pinned-sdk')
    require_hashes={'ik.py':'6326a47a395c02944f4f2ae1231ee4bb082e5ceb06e039d79992f66d8673d795',
                    'fk.py':'f7aa7df14c8d2b8c2a4838846caa302fcc0976276d616769aa6730bcbff2abed'}
    for name,expected in require_hashes.items():assert C.sha(sdk/name)==expected
    sys.path.insert(0,str(sdk));import ik
    contract=json.loads(Path(__file__).with_name('HOME-COMMAND-CONTRACT.json').read_text())
    command,ok=ik.solve(np.deg2rad(contract['initial_home_seed_deg']),
        [.3,-.1,-.0051+G['layout_m']['travel_tip_height_above_table']], [0,0,-1],
        np.deg2rad(contract['joint_limits_deg']),tool_offset=(.293,0,0),spread_dir=[0,1,0],iters=500)
    assert ok
    q=np.asarray(list(command)+[0.,0.],dtype=np.float32).astype(float).tolist()
    R,t=independent_fk(q[:6],[.293,0,0],'float32');tip=(np.asarray(t)+[0,0,.9]).tolist()
    actors={}
    for name,v in poses.items():
        m=np.eye(4);m[:3,:3]=Rotation.from_quat(np.array(v['orientation_wxyz'])[[1,2,3,0]]).as_matrix().T
        m[3,:3]=v['position_world_m'];actors['/World/Assembly_'+name]=m.tolist()
    for jaw in ('JawL','JawR'):actors['/World/ER4iA/'+jaw]=np.eye(4).tolist()
    def snapshot(step):return {'step':step,'sim_time':(step-1)/60,'joint_positions':q.copy(),
                                'actor_world_matrices':copy.deepcopy(actors)}
    rows=[{'step':i,'sim_time':i/60,'phase':'initial_settling','actual_q_rad':q.copy(),
           'actual_qdot_rad_s':[0.]*8,'issued_arm_q_rad':command.tolist(),'issued_jaw_targets_m':{'jaw_l':0.,'jaw_r':0.},
           'tool_rotation_world':R.tolist(),'tip_world_m':tip.copy(),'poses':copy.deepcopy(poses)} for i in range(1,133)]
    trace=[{'step':i,'sim_time':i/60,'maximum_raw_joint_speed_rad_s':0.,
            'tip_robot_m':(np.array(tip)-[0,0,.9]).tolist()} for i in range(122,133,2)]
    return {'initial_pose_pilot':authored,'layout_m':G['layout_m'],
        'kinematic_binding':{'arm_joint_names':['J'+str(i) for i in range(1,7)],'arm_joint_indices':list(range(6)),
            'tool_offset_m':[.293,0.,0.],'table_top_z_m':.9,'joint_dtype':'float32'},
        'full_motion_observations':rows,'step_timing_audit':{'samples':132},
        'diagnostic_trace':[snapshot(i) for i in range(1,133)],'full_motion_last_snapshot':snapshot(133),
        'pose_readback_dtypes':{n:{'position':'float64','orientation':'float64'} for n in poses},
        'pose_readback_dtype_errors':[],'pose_readback_dtype_capture_counts':dict.fromkeys(poses,133),
        'placement_observations':[],'part_results':[],'part_order':[],'motion_sampling':{'phases':[]},
        'expected_motion_phases':[],'expected_checkpoint_phases':['initial'],
        'checkpoints':[{'phase':'initial','step':132,'sim_time':132/60,'poses':copy.deepcopy(poses),'stillness_trace':trace}],
        'camera_probe':{'configuration':{'runtime_state_step':132,'native_clock_before':{'step':144,'sim_time':132/60}}}}


class RawTests(unittest.TestCase):
    def setUp(self):self.r=fixture()
    def test_original_initial_setup_is_only_a_setup_certificate(self):
        result=C.audit_raw_setup(self.r,G,D)
        self.assertEqual(result['raw_rows'],132);self.assertEqual(result['pickup_steps'],0)
        C.audit_fixture(self.r,G,D,SOURCE)

    def test_missing_terminal_snapshot_is_not_retrofilled(self):
        del self.r['full_motion_last_snapshot']
        with self.assertRaises((KeyError,ValueError)):C.audit_raw_setup(self.r,G,D)
        self.assertNotIn('full_motion_last_snapshot',self.r)

    def test_missing_raw_step_rejected(self):
        self.r['full_motion_observations'].pop(60)
        with self.assertRaisesRegex(ValueError,'Missing existing post-step'):C.audit_raw_setup(self.r,G,D)

    def test_forged_actual_TCP_rejected_by_original_FK(self):
        self.r['full_motion_observations'][9]['tip_world_m'][0]+=.001
        with self.assertRaisesRegex(ValueError,'FK mismatch'):C.audit_raw_setup(self.r,G,D)

    def test_measured_PCB_escape_cannot_hide_behind_valid_raw_pose_binding(self):
        self.r['full_motion_observations'][9]['poses']['pcb']['position_world_m'][0]+=.000751
        self.r['diagnostic_trace'][10]['actor_world_matrices']['/World/Assembly_pcb'][3][0]+=.000751
        with self.assertRaisesRegex(ValueError,'Continuous loose PCB'):C.audit_raw_setup(self.r,G,D)

    def test_new_home_command_or_unapproved_phase_rejected(self):
        self.r['full_motion_observations'][9]['issued_arm_q_rad'][0]+=.001
        with self.assertRaisesRegex(ValueError,'command changed'):C.audit_raw_setup(self.r,G,D)
        self.r=fixture();self.r['full_motion_observations'][9]['phase']='1_base_close'
        with self.assertRaisesRegex(ValueError,'Unapproved robot phase'):C.audit_raw_setup(self.r,G,D)

    def test_constant_wrong_arm_command_is_not_blessed_as_original_home(self):
        for row in self.r['full_motion_observations']:row['issued_arm_q_rad']=[.123]*6
        with self.assertRaisesRegex(ValueError,'home goal/orientation'):C.audit_raw_setup(self.r,G,D)

    def test_constant_wrong_jaw_keys_or_targets_rejected(self):
        for value in ({'invented_jaw':.02},{'jaw_l':-.001,'jaw_r':.001}):
            r=fixture()
            for row in r['full_motion_observations']:row['issued_jaw_targets_m']=value.copy()
            with self.assertRaisesRegex(ValueError,'gripper-open keys/targets'):C.audit_raw_setup(r,G,D)

    def test_stillness_summary_must_equal_raw_speed(self):
        self.r['checkpoints'][0]['stillness_trace'][0]['maximum_raw_joint_speed_rad_s']=.01
        with self.assertRaisesRegex(ValueError,'Raw original camera stillness'):C.audit_raw_setup(self.r,G,D)

    def test_changed_spawn_gap_or_PCB_authorship_rejected(self):
        self.r['initial_pose_pilot']['authored']['base']['nominal_position_m'][2]-=.0003
        with self.assertRaisesRegex(ValueError,'spawn height'):C.audit_fixture(self.r,G,D,SOURCE)
        self.r=fixture();self.r['initial_pose_pilot']['authored']['pcb']['actual_USD_world_matrix'][3][0]+=.001
        with self.assertRaisesRegex(ValueError,'world pose mismatch'):C.audit_fixture(self.r,G,D,SOURCE)

    def test_config_clock_must_be_the_last_physics_row(self):
        self.r['camera_probe']['configuration']['native_clock_before']['sim_time']+=.001
        with self.assertRaisesRegex(ValueError,'last raw physics row'):C.audit_raw_setup(self.r,G,D)

    def test_manifest_rejects_tampering_and_unlisted_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            for name in ('camera_association_trial.py','nest-mesh.json','nest-design.json'):shutil.copy2(SOURCE/name,path/name)
            manifest={p.name:C.sha(p) for p in path.iterdir()}
            (path/'snapshot-sha256.json').write_text(json.dumps(manifest))
            digest=C.sha(path/'snapshot-sha256.json');C.verify_source(path,digest)
            (path/'unlisted.py').write_text('raise RuntimeError()')
            with self.assertRaisesRegex(ValueError,'omitted or added'):C.verify_source(path,digest)
            (path/'unlisted.py').unlink();(path/'camera_association_trial.py').write_text('# changed')
            with self.assertRaisesRegex(ValueError,'Source content'):C.verify_source(path,digest)


if __name__=='__main__':unittest.main()
