"""FANUC bindings for the pinned Factory task. Loaded after Isaac app startup."""
import json
import os
import hashlib
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass
from vendor.factory.factory_env_cfg import FactoryTaskPegInsertCfg


@configclass
class FanucPegInsertCfg(FactoryTaskPegInsertCfg):
    """Original NVIDIA peg/hole; FANUC and original SMC geometry from export."""
    def __post_init__(self):
        parent_hook = getattr(super(), "__post_init__", None)
        if parent_hook:
            parent_hook()
        path = os.environ.get("TEAM_V_FANUC_BINDING")
        if not path:
            raise RuntimeError("Set TEAM_V_FANUC_BINDING to export_robot.py's binding.json")
        self.fanuc_binding = json.loads(Path(path).read_text())
        b = self.fanuc_binding
        if b.get("schema") != 1 or b.get("arm_joints") != ["J"+str(i) for i in range(1,7)] or b.get("jaw_joints") != ["jaw_l", "jaw_r"]:
            raise ValueError("Unrecognized FANUC joint binding")
        if hashlib.sha256(Path(b['robot_usd']).read_bytes()).hexdigest()!=b['robot_usd_sha256']:
            raise ValueError('Exported FANUC asset checksum mismatch')
        self.insertion_baseline = os.environ.get("TEAM_V_INDUSTREAL_INSERTION_BASELINE")=="1"
        from .fixture import verified_fixture
        fixture_path,self.fixture_geometry_record=verified_fixture()
        self.task.fixed_asset.spawn.usd_path=fixture_path
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.0
        self.state_order = [x for x in self.state_order if x != "joint_pos"]
        self.ctrl.reset_joints = b["home_joints_rad"]
        self.ctrl.pos_action_threshold = [0.001]*3
        self.ctrl.rot_action_threshold = [0.01]*3
        self.ctrl.pos_action_bounds = [0.05]*3
        self.obs_rand.fixed_asset_pos = [0.001]*3
        self.task.hand_init_pos = [0.0, 0.0, 0.053]
        self.task.hand_init_pos_noise = [0.002, 0.002, 0.001]
        self.task.hand_init_orn_noise = [0.0, 0.0, 0.05235987755982989]
        self.task.fixed_asset_init_pos_noise = [0.002, 0.002, 0.0]
        self.task.fixed_asset_init_orn_range_deg = 0.0
        self.task.held_asset_pos_noise = [0.0, 0.0, 0.0]
        self.task.fixed_asset.init_state.pos = (0.35, 0.0, b["surface_dz_m"])
        # Peg gravity is enabled. A floating peg is not grip evidence.
        self.task.held_asset.spawn.rigid_props.disable_gravity = False
        for asset in (self.task.fixed_asset,self.task.held_asset):
            asset.spawn.rigid_props.solver_position_iteration_count = 192
            asset.spawn.rigid_props.solver_velocity_iteration_count = 8
        self.robot = ArticulationCfg(
            prim_path="/World/envs/env_.*/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=b["robot_usd"], activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=192,solver_velocity_iteration_count=8),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(solver_position_iteration_count=192,solver_velocity_iteration_count=8)),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0,0,0),
                joint_pos=dict(zip(b["arm_joints"],b["home_joints_rad"])) | {"jaw_l":0.0,"jaw_r":0.0}),
            actuators={"arm":ImplicitActuatorCfg(joint_names_expr=b["arm_joints"], stiffness=None, damping=None),
                       "jaws":ImplicitActuatorCfg(joint_names_expr=b["jaw_joints"], stiffness=None, damping=None)},
        )
        # Refine contact integration to240Hz while preserving15Hz control. This only
        # changes this isolated benchmark; authored CAD, drives, mass/friction
        # and acceptance limits stay unchanged. Control remains15Hz.
        self.sim.dt = 1/240
        self.decimation = 16
        self.sim.physx.solver_type = 1
        self.sim.physx.max_position_iteration_count = 192
        self.sim.physx.max_velocity_iteration_count = 8
        self.sim.physx.gpu_max_rigid_contact_count = 2**18
        self.sim.physx.gpu_max_rigid_patch_count = 2**18
        self.curriculum_enabled = True
        self.curriculum_initial_downward_disp = 0.02
