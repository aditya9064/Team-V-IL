"""FANUC position-controlled adaptation of NVIDIA's pinned Factory environment.

Only simulation: preheld peg reset, noisy socket-pose actor inputs, privileged
reward/critic state. There is no FANUC controller/network/hardware interface.
"""
import math
import os
from types import SimpleNamespace

import torch
import numpy as np
import carb
import isaacsim.core.utils.torch as torch_utils
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from vendor.factory.factory_env import FactoryEnv
from vendor.factory import factory_utils
from vendor.factory.factory_env_cfg import OBS_DIM_CFG, STATE_DIM_CFG
from .control import quaternion_rotate, quaternion_multiply, pose_error, shift_jacobian, joint_position_step, jaw_targets, integrate_joint_target
from .curriculum import get_new_max_disp
from .contacts import ContactAudit, active_prefix_length
from .evaluation import within_capture_envelope


class FanucInsertionEnv(FactoryEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        self.binding = cfg.fanuc_binding
        self.release_requested = False
        self.current_downward_disp = cfg.curriculum_initial_downward_disp
        self.curriculum_cfg = SimpleNamespace(rl=SimpleNamespace(
            curriculum_height_bound=[0.0,0.02], curriculum_height_step=[-0.001,0.002],
            curriculum_success_thresh=0.8, curriculum_failure_thresh=0.5))
        cfg.observation_space = sum(OBS_DIM_CFG[o] for o in cfg.obs_order)+cfg.action_space
        cfg.state_space = sum(STATE_DIM_CFG[o] for o in cfg.state_order)+cfg.action_space
        self.cfg_task = cfg.task
        if cfg.scene.num_envs != 1:
            raise ValueError('First FANUC port is qualified for one environment only')
        self.contact_audit = ContactAudit()
        self.last_contact_guard_failure=None
        self.diagnostic_target_tcp_pose=None
        self.diagnostic_release_opening=None
        self.tool_contact_audit=ContactAudit()
        DirectRLEnv.__init__(self,cfg,render_mode,**kwargs)
        # Intentionally do not invoke upstream's artificial robot inertia offset.
        self._init_tensors()
        self._set_default_dynamics_parameters()
        self._subscribe_contacts()

    def _subscribe_contacts(self):
        import omni.physx
        import omni.usd
        from pxr import PhysicsSchemaTools,UsdPhysics
        # Lab disables event processing unless a ContactSensor explicitly enables
        # it. This task uses native event/tensor readers, so enable it ourselves.
        settings=carb.settings.get_settings()
        settings.set_bool('/physics/disableContactProcessing',False)
        if settings.get_as_bool('/physics/disableContactProcessing'):
            raise RuntimeError('PhysX contact event processing is still disabled')
        self._contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
            lambda headers,data,*unused:self.contact_audit.ingest(headers,data,PhysicsSchemaTools.intToSdfPath))
        stage=omni.usd.get_context().get_stage()
        def rigid_body(role):
            paths=[str(p.GetPath()) for p in stage.Traverse()
                   if p.HasAPI(UsdPhysics.RigidBodyAPI) and role in str(p.GetPath()).split('/')]
            if len(paths)!=1:raise RuntimeError('Expected one rigid body for '+role+': '+str(paths))
            return paths[0]
        self._contact_pair_names=['JawL','JawR','FixedAsset']
        targets=['/World/envs/env_0/Robot/JawL','/World/envs/env_0/Robot/JawR',
                 rigid_body('FixedAsset')]
        # Static colliders are unsupported GPU filters in Sim5.1. Compare
        # total held-part force with all identified jaw/socket force matrices
        # to reject unaccounted force, while explicitly limiting depth coverage.
        # One target group per view preserves identity even when a compound
        # finger resolves to multiple filter shapes in this PhysX version.
        self._peg_contact_views={name:self.sim.physics_sim_view.create_rigid_contact_view(
            rigid_body('HeldAsset'),filter_patterns=[target],max_contact_data_count=4096)
            for name,target in zip(self._contact_pair_names,targets)}
        self._tool_contact_views={}
        if self.cfg.insertion_baseline:
            fixed=rigid_body('FixedAsset')
            table='/World/envs/env_0/Table'
            self._support_table_view=self.sim.physics_sim_view.create_rigid_body_view(table)
            self._support_table_reference=self._support_table_view.get_transforms().clone()
            self.maximum_table_displacement_m=0.
            self._peg_contact_views['Table']=self.sim.physics_sim_view.create_rigid_contact_view(
                rigid_body('HeldAsset'),filter_patterns=[table],max_contact_data_count=4096)
            for jaw in ('JawL','JawR'):
                for role,target in [('FixedAsset',fixed),('Table',table)]:
                    self._tool_contact_views[jaw+'/'+role]=self.sim.physics_sim_view.create_rigid_contact_view(
                        '/World/envs/env_0/Robot/'+jaw,filter_patterns=[target],max_contact_data_count=4096)
        self._last_contact_timestamp=None

    def _poll_contact_view(self):
        views=getattr(self,'_peg_contact_views',None)
        stamp=self._robot._data._sim_timestamp
        if views is None or stamp==self._last_contact_timestamp:return
        self._last_contact_timestamp=stamp
        try:
            names=[];all_counts=[];all_starts=[];all_distances=[];all_forces=[];all_normals=[];all_matrices=[]
            self.contact_tensor_snapshot={}
            for name,view in views.items():
                forces,_,normals,distances,counts,starts=view.get_contact_data(dt=self.physics_dt)
                matrix=view.get_contact_force_matrix(dt=self.physics_dt).detach().cpu().reshape(-1,3).tolist()
                counts=counts.detach().cpu().reshape(-1).tolist();starts=starts.detach().cpu().reshape(-1).tolist()
                prefix=active_prefix_length(counts,starts,min(forces.numel(),distances.numel(),normals.shape[0]))
                cpu_forces=forces.reshape(-1)[:prefix].detach().cpu().tolist()
                cpu_distances=distances.reshape(-1)[:prefix].detach().cpu().tolist()
                cpu_normals=normals.reshape(-1,3)[:prefix].detach().cpu().tolist()
                self.contact_tensor_snapshot[name]={"force_matrix_n_repr":repr([matrix]),"counts":[counts],"starts":[starts],"active_samples":[]}
                for count,start in zip(counts,starts):
                    if count:
                        for i in range(int(start),min(int(start+count),int(start)+8)):
                            self.contact_tensor_snapshot[name]["active_samples"].append(repr({"index":i,"force":[cpu_forces[i]],"separation":[cpu_distances[i]],"normal":cpu_normals[i]}))
                all_normals.extend(cpu_normals);all_matrices.extend(matrix)
                offset=len(all_distances)
                names.extend([name]*len(counts));all_counts.extend(counts)
                all_starts.extend([int(i)+offset for i in starts])
                all_distances.extend(cpu_distances);all_forces.extend(cpu_forces)
            if self.cfg.insertion_baseline:
                table_pose=self._support_table_view.get_transforms()
                table_shift=float(torch.linalg.vector_norm(table_pose[:,:3]-self._support_table_reference[:,:3],dim=-1).max())
                self.maximum_table_displacement_m=max(self.maximum_table_displacement_m,table_shift)
                if table_shift>1e-6:raise RuntimeError('Immovable support table moved')
                if ((table_pose[:,3:]-self._support_table_reference[:,3:]).abs()>1e-6).any():raise RuntimeError('Immovable support table rotated')
                relative=self.held_pos-self.fixed_pos
                radial=float(torch.linalg.vector_norm(relative[:,:2]))
                axis=quaternion_rotate(self.held_quat,torch.tensor([[0.,0.,1.]],device=self.device))
                tilt=float(torch.acos(axis[0,2].clamp(-1.,1.)))
                self.contact_audit.allow_table_support=within_capture_envelope(radial,float(relative[0,2]),tilt)
            self.contact_audit.ingest_tensor(names,all_counts,all_starts,all_distances,all_forces,all_normals,all_matrices,
                next(iter(views.values())).get_net_contact_forces(dt=self.physics_dt).detach().cpu().reshape(-1,3).sum(0).tolist())
        except Exception as exc:
            self.contact_audit.errors.append('PhysX contact tensor query: '+repr(exc))
        if getattr(self,'_tool_contact_views',None):
            try:
                names=[];counts_all=[];starts_all=[];forces_all=[];normals_all=[];distances_all=[];matrices_all=[]
                for name,view in self._tool_contact_views.items():
                    forces,_,normals,distances,counts,starts=view.get_contact_data(dt=self.physics_dt)
                    counts=counts.detach().cpu().reshape(-1).tolist();starts=starts.detach().cpu().reshape(-1).tolist()
                    matrix=view.get_contact_force_matrix(dt=self.physics_dt).detach().cpu().reshape(-1,3).tolist()
                    offset=len(forces_all)
                    names.extend([name]*len(counts));counts_all.extend(counts);starts_all.extend([int(i)+offset for i in starts]);matrices_all.extend(matrix)
                    prefix=active_prefix_length(counts,starts,min(forces.numel(),distances.numel(),normals.shape[0]))
                    forces_all.extend(forces.reshape(-1)[:prefix].detach().cpu().tolist())
                    normals_all.extend(normals.reshape(-1,3)[:prefix].detach().cpu().tolist())
                    distances_all.extend(distances.reshape(-1)[:prefix].detach().cpu().tolist())
                self.tool_contact_audit.ingest_tensor(names,counts_all,starts_all,distances_all,forces_all,normals_all,matrices_all)
                for name,vector in self.tool_contact_audit.last_pair_force_world_n.items():
                    if math.sqrt(sum(v*v for v in vector))>.001:
                        self.tool_contact_audit.unexpected.append({'tool_fixture_contact':name,'force_world_n':vector})
            except Exception as exc:self.tool_contact_audit.errors.append('Tool contact query: '+repr(exc))

    def _get_dones(self):
        terminated,timeout=super()._get_dones()
        if self.contact_audit.invalid or self.tool_contact_audit.invalid:
            self.last_contact_guard_failure={
                'audit':self.contact_audit.report(),
                'tool_audit':self.tool_contact_audit.report(),
                'tensor_snapshot':getattr(self,'contact_tensor_snapshot',None),
                'held_position_env_m':self.held_pos.detach().cpu().tolist(),
                'held_quaternion_wxyz':self.held_quat.detach().cpu().tolist(),
                'tcp_quaternion_wxyz':self.fingertip_midpoint_quat.detach().cpu().tolist(),
                'tcp_position_env_m':self.fingertip_midpoint_pos.detach().cpu().tolist(),
                'measured_joints':self.joint_pos.detach().cpu().tolist(),
                'state_phase':'BEFORE_AUTOMATIC_RESET'}
            terminated=torch.ones_like(terminated)
        return terminated,timeout

    def _get_rewards(self):
        reward=super()._get_rewards()
        # SAPU-inspired guard: invalid physical transitions receive no positive reward.
        # This is not claimed to reproduce the original mesh-SDF SAPU algorithm.
        if self.contact_audit.invalid or self.tool_contact_audit.invalid:
            reward=torch.full_like(reward,-1.)
        return reward

    def _get_curr_successes(self,success_threshold,check_rot=False):
        # Upstream's broad engagement shaping is retained, while final training
        # success requires the actual radial fit allowance. Independent release
        # scoring remains a separate operation.
        if self.contact_audit.invalid or self.tool_contact_audit.invalid:
            return torch.zeros(self.num_envs,dtype=torch.bool,device=self.device)
        if success_threshold>self.cfg_task.success_threshold:
            return super()._get_curr_successes(success_threshold,check_rot)
        relative=self.held_pos-self.fixed_pos
        radial=torch.linalg.vector_norm(relative[:,:2],dim=1)
        axis=quaternion_rotate(self.held_quat,torch.tensor([0.,0.,1.],device=self.device).repeat(self.num_envs,1))
        tilt=torch.acos(axis[:,2].clamp(-1.,1.))
        clearance=(self.cfg_task.fixed_asset_cfg.diameter-self.cfg_task.held_asset_cfg.diameter)/2
        return (radial<=clearance)&(relative[:,2].abs()<=self.cfg_task.fixed_asset_cfg.height*success_threshold)&(tilt<=math.radians(1.))

    def _setup_scene(self):
        # Enable before PhysX actors/contact reporters are initialized as well
        # as verifying it when the native read-only views are subscribed.
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing',False)
        self._robot = Articulation(self.cfg.robot)
        self._fixed_asset = Articulation(self.cfg_task.fixed_asset)
        self._held_asset = Articulation(self.cfg_task.held_asset)
        from .fixture import refine_sdf_meshes
        import omni.usd
        self.sdf_refinement=refine_sdf_meshes(omni.usd.get_context().get_stage())
        # Author reporting before PhysX creates its actor views.
        import omni.usd
        from pxr import UsdPhysics, PhysxSchema
        for prim in omni.usd.get_context().get_stage().Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        surface = self.binding['surface_dz_m']
        table = sim_utils.CuboidCfg(size=(1.13,1.56,0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.0002,rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.9,dynamic_friction=0.8))
        if self.cfg.insertion_baseline:
            table.rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True,disable_gravity=True)
        table.func('/World/envs/env_.*/Table',table,translation=(0.194,0.01,surface-0.025))
        if self.cfg.insertion_baseline:
            for prim in omni.usd.get_context().get_stage().Traverse():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) and 'Table' in str(prim.GetPath()).split('/'):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        self.scene.clone_environments(copy_from_source=False)
        if self.device=='cpu': self.scene.filter_collisions()
        self.scene.articulations.update(robot=self._robot,fixed_asset=self._fixed_asset,held_asset=self._held_asset)
        light=sim_utils.DomeLightCfg(intensity=1500,color=(0.75,0.75,0.75))
        light.func('/World/Light',light)

    def _init_tensors(self):
        arm,names=self._robot.find_joints(self.binding['arm_joints'],preserve_order=True)
        jaws,jaw_names=self._robot.find_joints(self.binding['jaw_joints'],preserve_order=True)
        if list(arm)!=list(range(6)) or list(jaws)!=[6,7] or names!=self.binding['arm_joints'] or jaw_names!=self.binding['jaw_joints']:
            raise RuntimeError('Unsupported DOF ordering: expected J1..J6, jaw_l, jaw_r; refusing motion')
        if not self._robot.is_fixed_base: raise RuntimeError('FANUC base must be fixed')
        self.wrist_body_idx=self._robot.body_names.index(self.binding['wrist_body'])
        self.ctrl_target_joint_pos=self._robot.data.default_joint_pos.clone()
        self.ema_factor=self.cfg.ctrl.ema_factor
        self.dead_zone_thresholds=None
        self.fixed_pos_obs_frame=torch.zeros((self.num_envs,3),device=self.device)
        self.init_fixed_pos_obs_noise=torch.zeros_like(self.fixed_pos_obs_frame)
        self.last_update_timestamp=0.0
        self.prev_fingertip_pos=torch.zeros_like(self.fixed_pos_obs_frame)
        self.prev_fingertip_quat=torch.tensor([1.,0.,0.,0.],device=self.device).repeat(self.num_envs,1)
        self.prev_joint_pos=torch.zeros((self.num_envs,6),device=self.device)
        self.ep_succeeded=torch.zeros(self.num_envs,dtype=torch.long,device=self.device)
        self.ep_success_times=torch.zeros_like(self.ep_succeeded)
        self.actions=torch.zeros((self.num_envs,6),device=self.device)
        self.prev_actions=torch.zeros_like(self.actions)

    def _set_default_dynamics_parameters(self):
        self.default_gains=torch.tensor(self.cfg.ctrl.default_task_prop_gains,device=self.device).repeat(self.num_envs,1)
        self.pos_threshold=torch.tensor(self.cfg.ctrl.pos_action_threshold,device=self.device).repeat(self.num_envs,1)
        self.rot_threshold=torch.tensor(self.cfg.ctrl.rot_action_threshold,device=self.device).repeat(self.num_envs,1)
        # Original authored FANUC/gripper friction, drives and inertias are retained.

    def _compute_intermediate_values(self,dt):
        self.fixed_pos=self._fixed_asset.data.root_pos_w-self.scene.env_origins
        self.fixed_quat=self._fixed_asset.data.root_quat_w
        self.held_pos=self._held_asset.data.root_pos_w-self.scene.env_origins
        self.held_quat=self._held_asset.data.root_quat_w
        wrist_q=self._robot.data.body_quat_w[:,self.wrist_body_idx]
        offset=torch.tensor(self.binding['tcp_offset_local_m'],device=self.device).repeat(self.num_envs,1)
        world_offset=quaternion_rotate(wrist_q,offset)
        self.fingertip_midpoint_pos=self._robot.data.body_pos_w[:,self.wrist_body_idx]+world_offset-self.scene.env_origins
        rotation=torch.tensor([math.sqrt(0.5),0.,math.sqrt(0.5),0.],device=self.device).repeat(self.num_envs,1)
        self.fingertip_midpoint_quat=quaternion_multiply(wrist_q,rotation)
        self.fingertip_midpoint_angvel=self._robot.data.body_ang_vel_w[:,self.wrist_body_idx]
        self.fingertip_midpoint_linvel=self._robot.data.body_lin_vel_w[:,self.wrist_body_idx]+torch.linalg.cross(self.fingertip_midpoint_angvel,world_offset,dim=-1)
        wrist_j=self._robot.root_physx_view.get_jacobians()[:,self.wrist_body_idx-1,:,:6]
        self.fingertip_midpoint_jacobian=shift_jacobian(wrist_j,world_offset)
        self.joint_pos=self._robot.data.joint_pos.clone()
        self.joint_vel=self._robot.data.joint_vel.clone()
        self.ee_linvel_fd=(self.fingertip_midpoint_pos-self.prev_fingertip_pos)/dt
        self.ee_angvel_fd=pose_error(self.prev_fingertip_pos,self.prev_fingertip_quat,self.fingertip_midpoint_pos,self.fingertip_midpoint_quat)[:,3:]/dt
        self.joint_vel_fd=(self.joint_pos[:,:6]-self.prev_joint_pos)/dt
        self.prev_fingertip_pos=self.fingertip_midpoint_pos.clone()
        self.prev_fingertip_quat=self.fingertip_midpoint_quat.clone()
        self.prev_joint_pos=self.joint_pos[:,:6].clone()
        self.last_update_timestamp=self._robot._data._sim_timestamp
        self._poll_contact_view()

    def set_learned_insertion_action(self,action,max_translation_m=.002):
        import os
        if os.environ.get('TEAM_V_INDUSTREAL_LEARNED_POLICY')!='1':raise RuntimeError('Learned-policy execution not enabled')
        from .learned_policy import action_target
        self._set_bounded_tcp_target(*action_target(self.fingertip_midpoint_pos,self.fingertip_midpoint_quat,action,max_translation_m))

    def set_calibrated_insertion_target(self,position,quaternion):
        import os
        if os.environ.get('TEAM_V_INDUSTREAL_LEARNED_POLICY')=='1':raise RuntimeError('Deterministic baseline override is forbidden during learned-policy execution')
        self._set_bounded_tcp_target(position,quaternion)

    def _set_bounded_tcp_target(self,position,quaternion):
        """Shared bounded pose-target application; no task-state action generation."""
        if not self.cfg.insertion_baseline:raise RuntimeError('Insertion baseline not enabled')
        if not torch.isfinite(position).all() or not torch.isfinite(quaternion).all():raise ValueError('Nonfinite target')
        if (torch.linalg.vector_norm(quaternion,dim=-1)<1e-8).any():raise ValueError('Zero target quaternion')
        frame=self.fixed_pos_obs_frame+self.init_fixed_pos_obs_noise
        if ((position-frame).abs()>.05).any():raise ValueError('Insertion target exceeds original50mm action box')
        if self.diagnostic_target_tcp_pose is None:self.ctrl_target_joint_pos[:,:6]=self.joint_pos[:,:6]
        self.diagnostic_target_tcp_pose=(position.clone(),quaternion.clone())

    def _apply_action(self):
        if self.diagnostic_target_tcp_pose is None:return super()._apply_action()
        if self.last_update_timestamp<self._robot._data._sim_timestamp:
            self._compute_intermediate_values(dt=self.physics_dt)
        self.generate_ctrl_signals(*self.diagnostic_target_tcp_pose,0.)

    def generate_ctrl_signals(self,ctrl_target_fingertip_midpoint_pos,ctrl_target_fingertip_midpoint_quat,ctrl_target_gripper_dof_pos):
        error=pose_error(self.fingertip_midpoint_pos,self.fingertip_midpoint_quat,
                         ctrl_target_fingertip_midpoint_pos,ctrl_target_fingertip_midpoint_quat)
        bounds=self._robot.data.soft_joint_pos_limits[:,:6]
        increment_target=joint_position_step(self.joint_pos[:,:6],self.fingertip_midpoint_jacobian,error,bounds[:,:,0],bounds[:,:,1])
        if self.cfg.insertion_baseline and self.diagnostic_target_tcp_pose is not None:
            self.ctrl_target_joint_pos[:,:6]=integrate_joint_target(self.joint_pos[:,:6],self.ctrl_target_joint_pos[:,:6],increment_target,bounds[:,:,0],bounds[:,:,1])
        else:self.ctrl_target_joint_pos[:,:6]=increment_target
        opening=0.006 if self.release_requested else self.binding['grasp_opening_per_jaw_m']
        if self.cfg.insertion_baseline and self.diagnostic_release_opening is not None:opening=self.diagnostic_release_opening
        self.ctrl_target_joint_pos[:,6:8]=torch.tensor(jaw_targets(opening),device=self.device)
        self._robot.set_joint_position_target(self.ctrl_target_joint_pos)

    def close_gripper_in_place(self):
        self.generate_ctrl_signals(self.fingertip_midpoint_pos,self.fingertip_midpoint_quat,0.)

    def _get_factory_obs_state_dict(self):
        obs,state=super()._get_factory_obs_state_dict()
        state['joint_pos']=self.joint_pos[:,:6]
        return obs,state

    def _set_franka_to_default_pose(self,joints,env_ids):
        # Legacy method name is required by the inherited reset; values are FANUC.
        q=self._robot.data.default_joint_pos[env_ids].clone()
        q[:,:6]=torch.tensor(self.binding['home_joints_rad'],device=self.device)
        q[:,6:8]=torch.tensor(jaw_targets(0.006),device=self.device)
        self.ctrl_target_joint_pos[env_ids]=q
        self._robot.write_joint_state_to_sim(q,torch.zeros_like(q),env_ids=env_ids)
        self._robot.set_joint_position_target(q,env_ids=env_ids)
        self._robot.reset(env_ids)
        self.step_sim_no_action()

    def set_pos_inverse_kinematics(self,ctrl_target_fingertip_midpoint_pos,ctrl_target_fingertip_midpoint_quat,env_ids):
        # Explicit training episode setup only. No object-truth corrections in policy steps.
        for _ in range(160):
            error=pose_error(self.fingertip_midpoint_pos,self.fingertip_midpoint_quat,ctrl_target_fingertip_midpoint_pos,ctrl_target_fingertip_midpoint_quat)
            selected=error[env_ids]
            if (torch.linalg.vector_norm(selected[:,:3],dim=1)<1e-3).all() and (torch.linalg.vector_norm(selected[:,3:],dim=1)<1e-3).all(): break
            bounds=self._robot.data.soft_joint_pos_limits[env_ids,:6]
            q=self.joint_pos[env_ids].clone()
            q[:,:6]=joint_position_step(q[:,:6],self.fingertip_midpoint_jacobian[env_ids],selected,bounds[:,:,0],bounds[:,:,1],max_step_rad=0.03,max_translation_m=0.01,max_rotation_rad=0.05)
            self.ctrl_target_joint_pos[env_ids]=q
            self._robot.write_joint_state_to_sim(q,torch.zeros_like(q),env_ids=env_ids)
            self._robot.set_joint_position_target(q,env_ids=env_ids)
            self.step_sim_no_action()
        error=pose_error(self.fingertip_midpoint_pos,self.fingertip_midpoint_quat,ctrl_target_fingertip_midpoint_pos,ctrl_target_fingertip_midpoint_quat)[env_ids]
        return error[:,:3],error[:,3:]

    def get_handheld_asset_relative_pose(self):
        # Grasp depth is an explicit benchmark parameter, not a Franka fingerpad length.
        pos=torch.zeros((self.num_envs,3),device=self.device)
        pos[:,2]=self.cfg_task.held_asset_cfg.height-self.binding['peg_grasp_depth_m']
        quat=torch.tensor([1.,0.,0.,0.],device=self.device).repeat(self.num_envs,1)
        return pos,quat

    def randomize_initial_state(self,env_ids):
        self.diagnostic_target_tcp_pose=None
        self.diagnostic_release_opening=None
        self.release_requested=False
        self.contact_audit.reset()
        self.tool_contact_audit.reset()
        # Five mm clear of the socket mouth at the easiest stage; up to 25 mm
        # clear at the hardest. Derived from the explicit peg grasp depth.
        self.cfg_task.hand_init_pos[2]=self.cfg_task.held_asset_cfg.height-self.binding['peg_grasp_depth_m']+0.025-self.current_downward_disp
        try:
            self._randomize_fanuc_episode(env_ids)
        finally:
            view=sim_utils.SimulationContext.instance().physics_sim_view
            view.set_gravity(carb.Float3(*self.cfg.sim.gravity))

    def _reset_idx(self,env_ids):
        if hasattr(self,'ep_succeeded') and env_ids is not None and len(env_ids)==self.num_envs and self.cfg.curriculum_enabled:
            rate=float(self.ep_succeeded.float().mean())
            self.current_downward_disp=get_new_max_disp(rate,self.curriculum_cfg,self.current_downward_disp)
        super()._reset_idx(env_ids)

    def begin_release(self,gradual=False):
        """Evaluation-only jaw opening; actual release still requires measurement."""
        if gradual and not self.cfg.insertion_baseline:raise RuntimeError('Gradual release is diagnostic-only')
        self.release_requested=True
        self.diagnostic_release_opening=self.binding['grasp_opening_per_jaw_m'] if gradual else None

    def set_release_opening_per_jaw(self,opening):
        if not self.cfg.insertion_baseline or not self.release_requested or self.diagnostic_release_opening is None:
            raise RuntimeError('Calibrated gradual release is not active')
        if not math.isfinite(opening) or not self.binding['grasp_opening_per_jaw_m']<=opening<=.006:
            raise ValueError('Release target exceeds the original grasp-to-stroke interval')
        self.diagnostic_release_opening=float(opening)

    def benchmark_provenance(self):
        return {'sdf_refinement':self.sdf_refinement,'fixture_geometry':self.cfg.fixture_geometry_record,'scope':'preheld peg insertion training','learned_policy_mode':os.environ.get('TEAM_V_INDUSTREAL_LEARNED_POLICY')=='1','actor_inputs':('TCP error to fixed calibrated goal, simulated contact forces and previous actions' if os.environ.get('TEAM_V_INDUSTREAL_LEARNED_POLICY')=='1' else 'TCP feedback, noisy fixture estimate, previous actions'),
                'privileged_state':('one-time ideal simulated grasp/fixture calibration; online part truth only for reward, STOP and independent scoring' if os.environ.get('TEAM_V_INDUSTREAL_LEARNED_POLICY')=='1' else 'simulated part state used only for training reward, critic and independent scoring'),
                'physical_hardware_tested':False,'full_assembly_accepted':False,
                'physics_dt_s':self.physics_dt,'control_dt_s':self.step_dt,
                'solver_type':self.cfg.sim.physx.solver_type,
                'solver_position_iterations':self.cfg.sim.physx.max_position_iteration_count,
                'solver_velocity_iterations':self.cfg.sim.physx.max_velocity_iteration_count,
                'current_curriculum_downward_disp_m':self.current_downward_disp,
                'contact_depth_audit_implemented':True,'contact_audit':self.contact_audit.report(),
                'tool_contact_audit':self.tool_contact_audit.report(),
                'table_mode':'immovable kinematic rigid body for GPU contact identity' if self.cfg.insertion_baseline else 'static collider',
                'maximum_table_displacement_m':getattr(self,'maximum_table_displacement_m',None),
                'diagnostic_joint_target_integration':self.cfg.insertion_baseline and self.diagnostic_target_tcp_pose is not None,
                'suppress_readback':carb.settings.get_settings().get_as_bool('/physics/suppressReadback'),
                'contact_processing_disabled':carb.settings.get_settings().get_as_bool('/physics/disableContactProcessing'),
                'grasp_opening_per_jaw_m':self.binding['grasp_opening_per_jaw_m'],
                'peg_grasp_depth_m':self.binding['peg_grasp_depth_m'],
                'reset_contact_audit':getattr(self,'reset_contact_audit',None),
                'contact_depth_coverage':('indexed peg/jaws/socket/table and jaws/socket/table' if self.cfg.insertion_baseline else 'indexed peg/jaw and peg/socket contacts; raw event coverage reported separately'),
                'runtime_validation':'UNVERIFIED'}

    # Adapted from pinned NVIDIA FactoryEnv.randomize_initial_state.
    def _randomize_fanuc_episode(self, env_ids):
            """Randomize initial state and perform any episode-level randomization."""
            # Disable gravity.
            physics_sim_view = sim_utils.SimulationContext.instance().physics_sim_view
            physics_sim_view.set_gravity(carb.Float3(0.0, 0.0, 0.0))
    
            # (1.) Randomize fixed asset pose.
            fixed_state = self._fixed_asset.data.default_root_state.clone()[env_ids]
            # (1.a.) Position
            rand_sample = torch.rand((len(env_ids), 3), dtype=torch.float32, device=self.device)
            fixed_pos_init_rand = 2 * (rand_sample - 0.5)  # [-1, 1]
            fixed_asset_init_pos_rand = torch.tensor(
                self.cfg_task.fixed_asset_init_pos_noise, dtype=torch.float32, device=self.device
            )
            fixed_pos_init_rand = fixed_pos_init_rand @ torch.diag(fixed_asset_init_pos_rand)
            fixed_state[:, 0:3] += fixed_pos_init_rand + self.scene.env_origins[env_ids]
            # (1.b.) Orientation
            fixed_orn_init_yaw = np.deg2rad(self.cfg_task.fixed_asset_init_orn_deg)
            fixed_orn_yaw_range = np.deg2rad(self.cfg_task.fixed_asset_init_orn_range_deg)
            rand_sample = torch.rand((len(env_ids), 3), dtype=torch.float32, device=self.device)
            fixed_orn_euler = fixed_orn_init_yaw + fixed_orn_yaw_range * rand_sample
            fixed_orn_euler[:, 0:2] = 0.0  # Only change yaw.
            fixed_orn_quat = torch_utils.quat_from_euler_xyz(
                fixed_orn_euler[:, 0], fixed_orn_euler[:, 1], fixed_orn_euler[:, 2]
            )
            fixed_state[:, 3:7] = fixed_orn_quat
            # (1.c.) Velocity
            fixed_state[:, 7:] = 0.0  # vel
            # (1.d.) Update values.
            self._fixed_asset.write_root_pose_to_sim(fixed_state[:, 0:7], env_ids=env_ids)
            self._fixed_asset.write_root_velocity_to_sim(fixed_state[:, 7:], env_ids=env_ids)
            self._fixed_asset.reset()
    
            # (1.e.) Noisy position observation.
            fixed_asset_pos_noise = torch.randn((len(env_ids), 3), dtype=torch.float32, device=self.device)
            fixed_asset_pos_rand = torch.tensor(self.cfg.obs_rand.fixed_asset_pos, dtype=torch.float32, device=self.device)
            fixed_asset_pos_noise = fixed_asset_pos_noise @ torch.diag(fixed_asset_pos_rand)
            self.init_fixed_pos_obs_noise[:] = fixed_asset_pos_noise
    
            self.step_sim_no_action()
    
            # Compute the frame on the bolt that would be used as observation: fixed_pos_obs_frame
            # For example, the tip of the bolt can be used as the observation frame
            fixed_tip_pos_local = torch.zeros((self.num_envs, 3), device=self.device)
            fixed_tip_pos_local[:, 2] += self.cfg_task.fixed_asset_cfg.height
            fixed_tip_pos_local[:, 2] += self.cfg_task.fixed_asset_cfg.base_height
            if self.cfg_task.name == "gear_mesh":
                fixed_tip_pos_local[:, 0] = self.cfg_task.fixed_asset_cfg.medium_gear_base_offset[0]
    
            _, fixed_tip_pos = torch_utils.tf_combine(
                self.fixed_quat,
                self.fixed_pos,
                torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1),
                fixed_tip_pos_local,
            )
            self.fixed_pos_obs_frame[:] = fixed_tip_pos
    
            # (2) Move gripper to randomizes location above fixed asset. Keep trying until IK succeeds.
            # (a) get position vector to target
            bad_envs = env_ids.clone()
            ik_attempt = 0
    
            hand_down_quat = torch.zeros((self.num_envs, 4), dtype=torch.float32, device=self.device)
            while True:
                n_bad = bad_envs.shape[0]
    
                above_fixed_pos = fixed_tip_pos.clone()
                above_fixed_pos[:, 2] += self.cfg_task.hand_init_pos[2]
    
                rand_sample = torch.rand((n_bad, 3), dtype=torch.float32, device=self.device)
                above_fixed_pos_rand = 2 * (rand_sample - 0.5)  # [-1, 1]
                hand_init_pos_rand = torch.tensor(self.cfg_task.hand_init_pos_noise, device=self.device)
                above_fixed_pos_rand = above_fixed_pos_rand @ torch.diag(hand_init_pos_rand)
                above_fixed_pos[bad_envs] += above_fixed_pos_rand
    
                # (b) get random orientation facing down
                hand_down_euler = (
                    torch.tensor(self.cfg_task.hand_init_orn, device=self.device).unsqueeze(0).repeat(n_bad, 1)
                )
    
                rand_sample = torch.rand((n_bad, 3), dtype=torch.float32, device=self.device)
                above_fixed_orn_noise = 2 * (rand_sample - 0.5)  # [-1, 1]
                hand_init_orn_rand = torch.tensor(self.cfg_task.hand_init_orn_noise, device=self.device)
                above_fixed_orn_noise = above_fixed_orn_noise @ torch.diag(hand_init_orn_rand)
                hand_down_euler += above_fixed_orn_noise
                hand_down_quat[bad_envs, :] = torch_utils.quat_from_euler_xyz(
                    roll=hand_down_euler[:, 0], pitch=hand_down_euler[:, 1], yaw=hand_down_euler[:, 2]
                )
    
                # (c) iterative IK Method
                pos_error, aa_error = self.set_pos_inverse_kinematics(
                    ctrl_target_fingertip_midpoint_pos=above_fixed_pos,
                    ctrl_target_fingertip_midpoint_quat=hand_down_quat,
                    env_ids=bad_envs,
                )
                pos_error = torch.linalg.norm(pos_error, dim=1) > 1e-3
                angle_error = torch.norm(aa_error, dim=1) > 1e-3
                any_error = torch.logical_or(pos_error, angle_error)
                bad_envs = bad_envs[any_error.nonzero(as_tuple=False).squeeze(-1)]
    
                # Check IK succeeded for all envs, otherwise try again for those envs
                if bad_envs.shape[0] == 0:
                    break
    
                self._set_franka_to_default_pose(
                    joints=[0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0], env_ids=bad_envs
                )
    
                ik_attempt += 1
                if ik_attempt >= 8:
                    raise RuntimeError("FANUC reset IK failed after 8 bounded attempts")
    
            self.step_sim_no_action()
    
            # Add flanking gears after servo (so arm doesn't move them).
            if self.cfg_task.name == "gear_mesh" and self.cfg_task.add_flanking_gears:
                small_gear_state = self._small_gear_asset.data.default_root_state.clone()[env_ids]
                small_gear_state[:, 0:7] = fixed_state[:, 0:7]
                small_gear_state[:, 7:] = 0.0  # vel
                self._small_gear_asset.write_root_pose_to_sim(small_gear_state[:, 0:7], env_ids=env_ids)
                self._small_gear_asset.write_root_velocity_to_sim(small_gear_state[:, 7:], env_ids=env_ids)
                self._small_gear_asset.reset()
    
                large_gear_state = self._large_gear_asset.data.default_root_state.clone()[env_ids]
                large_gear_state[:, 0:7] = fixed_state[:, 0:7]
                large_gear_state[:, 7:] = 0.0  # vel
                self._large_gear_asset.write_root_pose_to_sim(large_gear_state[:, 0:7], env_ids=env_ids)
                self._large_gear_asset.write_root_velocity_to_sim(large_gear_state[:, 7:], env_ids=env_ids)
                self._large_gear_asset.reset()
    
            # (3) Randomize asset-in-gripper location.
            # flip gripper z orientation
            flip_z_quat = torch.tensor([0.0, 0.0, 1.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1)
            fingertip_flipped_quat, fingertip_flipped_pos = torch_utils.tf_combine(
                q1=self.fingertip_midpoint_quat,
                t1=self.fingertip_midpoint_pos,
                q2=flip_z_quat,
                t2=torch.zeros((self.num_envs, 3), device=self.device),
            )
    
            # get default gripper in asset transform
            held_asset_relative_pos, held_asset_relative_quat = self.get_handheld_asset_relative_pose()
            asset_in_hand_quat, asset_in_hand_pos = torch_utils.tf_inverse(
                held_asset_relative_quat, held_asset_relative_pos
            )
    
            translated_held_asset_quat, translated_held_asset_pos = torch_utils.tf_combine(
                q1=fingertip_flipped_quat, t1=fingertip_flipped_pos, q2=asset_in_hand_quat, t2=asset_in_hand_pos
            )
    
            # Add asset in hand randomization
            rand_sample = torch.rand((self.num_envs, 3), dtype=torch.float32, device=self.device)
            held_asset_pos_noise = 2 * (rand_sample - 0.5)  # [-1, 1]
            if self.cfg_task.name == "gear_mesh":
                held_asset_pos_noise[:, 2] = -rand_sample[:, 2]  # [-1, 0]
    
            held_asset_pos_noise_level = torch.tensor(self.cfg_task.held_asset_pos_noise, device=self.device)
            held_asset_pos_noise = held_asset_pos_noise @ torch.diag(held_asset_pos_noise_level)
            translated_held_asset_quat, translated_held_asset_pos = torch_utils.tf_combine(
                q1=translated_held_asset_quat,
                t1=translated_held_asset_pos,
                q2=torch.tensor([1.0, 0.0, 0.0, 0.0], device=self.device).unsqueeze(0).repeat(self.num_envs, 1),
                t2=held_asset_pos_noise,
            )
    
            held_state = self._held_asset.data.default_root_state.clone()
            held_state[:, 0:3] = translated_held_asset_pos + self.scene.env_origins
            held_state[:, 3:7] = translated_held_asset_quat
            held_state[:, 7:] = 0.0
            self._held_asset.write_root_pose_to_sim(held_state[:, 0:7])
            self._held_asset.write_root_velocity_to_sim(held_state[:, 7:])
            self._held_asset.reset()
    
            #  Close hand
            # Set gains to use for quick resets.
            reset_task_prop_gains = torch.tensor(self.cfg.ctrl.reset_task_prop_gains, device=self.device).repeat(
                (self.num_envs, 1)
            )
            self.task_prop_gains = reset_task_prop_gains
            self.task_deriv_gains = factory_utils.get_deriv_gains(
                reset_task_prop_gains, self.cfg.ctrl.reset_rot_deriv_scale
            )
    
            self.step_sim_no_action()
    
            grasp_time = 0.0
            while grasp_time < 0.5:
                # SMC targets are applied by close_gripper_in_place().
                self.close_gripper_in_place()
                self.step_sim_no_action()
                grasp_time += self.sim.get_physics_dt()
    
            self.prev_joint_pos = self.joint_pos[:, 0:6].clone()
            self.prev_fingertip_pos = self.fingertip_midpoint_pos.clone()
            self.prev_fingertip_quat = self.fingertip_midpoint_quat.clone()
    
            # Set initial actions to involve no-movement. Needed for EMA/correct penalties.
            self.actions = torch.zeros_like(self.actions)
            self.prev_actions = torch.zeros_like(self.actions)
    
            # Zero initial velocity.
            self.ee_angvel_fd[:, :] = 0.0
            self.ee_linvel_fd[:, :] = 0.0
    
            # Set initial gains for the episode.
            self.task_prop_gains = self.default_gains
            self.task_deriv_gains = factory_utils.get_deriv_gains(self.default_gains)
    
            # Preserve privileged reset transients separately; qualification
            # starts with gravity restored and continuously recorded contacts.
            self.reset_contact_audit=self.contact_audit.report()
            self.contact_audit.reset()
            self.tool_contact_audit.reset()
            self._last_contact_timestamp=None
            physics_sim_view.set_gravity(carb.Float3(*self.cfg.sim.gravity))
