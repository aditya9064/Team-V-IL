"""Export the supplied FANUC/SMC twin into a self-contained research asset.

Run with the existing Isaac Sim 6 interpreter. Does not step simulation or move
a physical robot. Uses the original starter builder without editing it.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback


def relocate_scene_libraries(asset, robot_path='/ER4iA'):
    """Preserve robot-native libraries and relocate scene bindings with real USD.

    The FANUC asset already contains Looks. Moving /World/Looks to that name
    collides; a separate scope keeps both libraries and USD rewrites bindings.
    This changes namespace only, never material or physics attribute values.
    """
    from pxr import Usd, UsdGeom
    names=[n for n in ('Looks','Physics','PhysicsMaterials')
           if asset.GetPrimAtPath('/World/'+n)]
    if not names:
        return {}
    container=robot_path+'/ExportedSceneLibraries'
    if asset.GetPrimAtPath(container):
        raise RuntimeError('Reserved export library scope already exists: '+container)
    UsdGeom.Scope.Define(asset,container)
    relocated={}
    for name in names:
        source='/World/'+name;target=container+'/'+name
        editor=Usd.NamespaceEditor(asset)
        if not editor.MovePrimAtPath(source,target):
            raise RuntimeError('Cannot schedule library relocation: '+source+' -> '+target)
        allowed=editor.CanApplyEdits()
        if not allowed:
            raise RuntimeError('Cannot relocate material library '+source+' -> '+target+': '+str(getattr(allowed,'whyNot',allowed)))
        if not editor.ApplyEdits():
            raise RuntimeError('Library relocation did not apply: '+source)
        relocated[source]=target
    return relocated


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--starter',required=True,type=Path)
    ap.add_argument('--out',required=True,type=Path)
    ap.add_argument('--grasp-opening-per-jaw-m',type=float,default=0.0018)
    ap.add_argument('--peg-grasp-depth-m',type=float,default=0.002)
    a=ap.parse_args()
    if not 0<=a.grasp_opening_per_jaw_m<=0.006 or not 0<a.peg_grasp_depth_m<0.05:
        raise ValueError('Invalid benchmark grasp configuration')
    a.out.mkdir(parents=True,exist_ok=False)
    report={'status':'STARTING','simulator_stepped':False,'physical_hardware_tested':False}
    app=None
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True})
        import omni.usd
        from pxr import Usd,UsdGeom,Gf
        sys.path.insert(0,str(a.starter.resolve()))
        import cell,scene_config as C
        omni.usd.get_context().new_stage()
        stage=omni.usd.get_context().get_stage()
        cell.build(stage,with_objects=False)
        for path in [C.ROBOT_PRIM+'/J6_link',C.ROBOT_PRIM+'/JawL',C.ROBOT_PRIM+'/JawR']:
            if not stage.GetPrimAtPath(path): raise RuntimeError('Missing supplied robot/gripper body '+path)
        # Flatten references, then relocate with USD's namespace editor so joint and
        # material relationships follow the robot. No points, colliders or drives change.
        asset=Usd.Stage.Open(stage.Flatten())
        editor=Usd.NamespaceEditor(asset)
        if not editor.MovePrimAtPath(C.ROBOT_PRIM,'/ER4iA') or not editor.CanApplyEdits():
            raise RuntimeError('Cannot relocate exported robot')
        editor.ApplyEdits()
        report['library_namespace_before']={
            'robot_native':[n for n in ('Looks','Physics','PhysicsMaterials') if asset.GetPrimAtPath('/ER4iA/'+n)],
            'scene':[n for n in ('Looks','Physics','PhysicsMaterials') if asset.GetPrimAtPath('/World/'+n)]}
        report['relocated_scene_libraries']=relocate_scene_libraries(asset)
        root=asset.GetPrimAtPath('/ER4iA')
        xform=UsdGeom.Xformable(root)
        matrix=xform.GetLocalTransformation()
        matrix.SetTranslateOnly(Gf.Vec3d(0,0,0))
        xform.ClearXformOpOrder()
        xform.AddTransformOp(opSuffix='baseNormalized').Set(matrix)
        for prim in list(asset.GetPseudoRoot().GetChildren()):
            if prim.GetPath()!=root.GetPath(): asset.RemovePrim(prim.GetPath())
        asset.SetDefaultPrim(root)
        UsdGeom.SetStageMetersPerUnit(asset,1.0)
        UsdGeom.SetStageUpAxis(asset,UsdGeom.Tokens.z)
        # Any remaining external relationship is a broken export, not a warning.
        for prim in asset.Traverse():
            for rel in prim.GetRelationships():
                for target in rel.GetTargets():
                    if target.IsAbsolutePath() and not target.HasPrefix(root.GetPath()):
                        raise RuntimeError('External relationship after export: '+str(target))
        usd=a.out/'fanuc-smc.usda'
        asset.GetRootLayer().Export(str(usd))
        source_names=['scene_config.py','cell.py','gripper_geometry.py','fk.py','ik.py']
        binding={'schema':1,'robot_usd':str(usd.resolve()),'robot_usd_sha256':hashlib.sha256(usd.read_bytes()).hexdigest(),
                 'source_hashes':{n:hashlib.sha256((a.starter/n).read_bytes()).hexdigest() for n in source_names},
                 'arm_joints':['J'+str(i) for i in range(1,7)],'jaw_joints':['jaw_l','jaw_r'],
                 'wrist_body':'J6_link','home_joints_rad':[math.radians(C.HOME_POSE_DEG['J'+str(i)]) for i in range(1,7)],
                 'joint_limits_rad':[[math.radians(v) for v in C.JOINT_LIMITS_DEG['J'+str(i)]] for i in range(1,7)],
                 'tcp_offset_local_m':[float(v)*cell.tip_offset() for v in C.TOOL_AXIS],
                 'surface_dz_m':C.SURFACE_DZ,'grasp_opening_per_jaw_m':a.grasp_opening_per_jaw_m,
                 'peg_grasp_depth_m':a.peg_grasp_depth_m,'grasp_parameters_validated':False,
                 'scope':'unchanged supplied FANUC/SMC geometry; root translation normalized to base frame',
                 'export_is_not_physics_validation':True}
        (a.out/'binding.json').write_text(json.dumps(binding,indent=2)+'\n')
        report.update(status='ASSET_EXPORTED_NOT_SIMULATOR_VALIDATED',binding='binding.json')
    except Exception as exc:
        report.update(status='FAILED',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        (a.out/'export-report.json').write_text(json.dumps(report,indent=2)+'\n')
        sys.stdout.flush();sys.stderr.flush()
        if app is not None: app.close()


if __name__=='__main__': main()
