"""Genuine CPU OpenUSD regression tests, separate from simulator evidence."""
import unittest

try:
    from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf
except ImportError:
    Usd=None

from export_robot import relocate_scene_libraries


@unittest.skipIf(Usd is None,'Standalone CPU OpenUSD is required for these tests')
class LibraryRelocationTests(unittest.TestCase):
    def scene(self):
        stage=Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage,'/ER4iA')
        UsdGeom.Scope.Define(stage,'/ER4iA/Looks')
        native=UsdShade.Material.Define(stage,'/ER4iA/Looks/OriginalRobotMaterial')
        native.CreateInput('identity',Sdf.ValueTypeNames.String).Set('robot-native')
        original=UsdGeom.Cube.Define(stage,'/ER4iA/OriginalVisual')
        UsdShade.MaterialBindingAPI.Apply(original.GetPrim()).Bind(native)
        UsdGeom.Scope.Define(stage,'/World/Looks')
        material=UsdShade.Material.Define(stage,'/World/Looks/OriginalFingerMaterial')
        shader=UsdShade.Shader.Define(stage,'/World/Looks/OriginalFingerMaterial/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.2,.3,.4))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        finger=UsdGeom.Cube.Define(stage,'/ER4iA/OriginalFinger')
        finger.CreateSizeAttr(.0145)
        UsdPhysics.CollisionAPI.Apply(finger.GetPrim())
        UsdPhysics.MassAPI.Apply(finger.GetPrim()).CreateMassAttr(.05)
        UsdShade.MaterialBindingAPI.Apply(finger.GetPrim()).Bind(material)
        UsdGeom.Scope.Define(stage,'/World/Physics')
        physics=UsdShade.Material.Define(stage,'/World/Physics/OriginalFriction')
        material_api=UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
        material_api.CreateStaticFrictionAttr(.9);material_api.CreateDynamicFrictionAttr(.8)
        UsdShade.MaterialBindingAPI(finger.GetPrim()).Bind(physics,materialPurpose='physics')
        return stage

    def test_original_collision_reproduced_and_both_libraries_preserved(self):
        stage=self.scene()
        old=Usd.NamespaceEditor(stage)
        self.assertTrue(old.MovePrimAtPath('/World/Looks','/ER4iA/Looks'))
        self.assertFalse(bool(old.CanApplyEdits()))
        before=stage.GetPrimAtPath('/ER4iA/OriginalFinger')
        attributes={a.GetName():a.Get() for a in before.GetAttributes()}
        mapping=relocate_scene_libraries(stage)
        self.assertEqual(mapping['/World/Looks'],'/ER4iA/ExportedSceneLibraries/Looks')
        self.assertTrue(stage.GetPrimAtPath('/ER4iA/Looks/OriginalRobotMaterial'))
        after=stage.GetPrimAtPath('/ER4iA/OriginalFinger')
        self.assertEqual(attributes,{a.GetName():a.Get() for a in after.GetAttributes()})
        visual=UsdShade.MaterialBindingAPI(after).ComputeBoundMaterial()[0]
        self.assertEqual(str(visual.GetPath()),mapping['/World/Looks']+'/OriginalFingerMaterial')
        surface=visual.GetSurfaceOutput().GetConnectedSource()[0]
        self.assertEqual(str(surface.GetPrim().GetPath()),str(visual.GetPath())+'/Shader')
        physics=UsdShade.MaterialBindingAPI(after).ComputeBoundMaterial(materialPurpose='physics')[0]
        self.assertAlmostEqual(UsdPhysics.MaterialAPI(physics.GetPrim()).GetStaticFrictionAttr().Get(),.9,places=6)
        self.assertAlmostEqual(UsdPhysics.MaterialAPI(physics.GetPrim()).GetDynamicFrictionAttr().Get(),.8,places=6)
        original=stage.GetPrimAtPath('/ER4iA/OriginalVisual')
        native=UsdShade.MaterialBindingAPI(original).ComputeBoundMaterial()[0]
        self.assertEqual(str(native.GetPath()),'/ER4iA/Looks/OriginalRobotMaterial')

    def test_reserved_scope_is_never_overwritten(self):
        stage=self.scene()
        UsdGeom.Scope.Define(stage,'/ER4iA/ExportedSceneLibraries')
        before=stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(RuntimeError,'already exists'):
            relocate_scene_libraries(stage)
        self.assertEqual(stage.GetRootLayer().ExportToString(),before)


if __name__=='__main__':unittest.main()

@unittest.skipIf(Usd is None,'Standalone CPU OpenUSD is required for these tests')
class FixtureGeometryTests(unittest.TestCase):
 def test_only_declared_inner_bore_surface_changes_and_mesh_remains_closed(self):
  from pathlib import Path
  import math
  root=Path(__file__).parents[1]/'fixtures'
  a=Usd.Stage.Open(str(root/'factory_hole_original_9mm.usd'));b=Usd.Stage.Open(str(root/'factory_hole_declared_8p1mm.usda'))
  am=next(UsdGeom.Mesh(p) for p in a.Traverse() if p.IsA(UsdGeom.Mesh));bm=UsdGeom.Mesh(b.GetPrimAtPath(am.GetPath()))
  self.assertEqual(list(am.GetFaceVertexCountsAttr().Get()),list(bm.GetFaceVertexCountsAttr().Get()))
  self.assertEqual(list(am.GetFaceVertexIndicesAttr().Get()),list(bm.GetFaceVertexIndicesAttr().Get()))
  before=am.GetPointsAttr().Get();after=bm.GetPointsAttr().Get();self.assertEqual(len(before),len(after));changed=0
  for x,y in zip(before,after):
   r=math.hypot(x[0],x[1])
   if .00449<=r<=.00551 and x[2]>=-1e-6:
    changed+=1;self.assertAlmostEqual(math.hypot(y[0],y[1]),r-.00045,places=8);self.assertEqual(x[2],y[2])
   else:self.assertEqual(x,y)
  self.assertEqual(changed,432)
  for z,r in [(0,.00405),(.024,.00405),(.025,.00505)]:
   radii=[math.hypot(p[0],p[1]) for p in after if abs(p[2]-z)<1e-6 and .001<math.hypot(p[0],p[1])<.01]
   self.assertTrue(radii);self.assertTrue(all(abs(v-r)<1e-8 for v in radii))
  edges={};indices=list(bm.GetFaceVertexIndicesAttr().Get());offset=0
  for n in bm.GetFaceVertexCountsAttr().Get():
   face=indices[offset:offset+n];offset+=n
   for v,w in zip(face,face[1:]+face[:1]):
    key=tuple(sorted((v,w)));edges[key]=edges.get(key,0)+1
  self.assertTrue(all(n==2 for n in edges.values()))

@unittest.skipIf(Usd is None,'Standalone CPU OpenUSD is required for these tests')
class SdfRefinementTests(unittest.TestCase):
 def test_refinement_blocks_stale_collision_buffers_without_changing_geometry_or_robot(self):
  from team_v_industreal.fixture import refine_sdf_meshes
  stage=Usd.Stage.CreateInMemory();meshes=[]
  for role,length in [('HeldAsset',.05),('FixedAsset',.04),('Robot',.5)]:
   mesh=UsdGeom.Mesh.Define(stage,'/World/envs/env_0/'+role+'/mesh');mesh.CreatePointsAttr([Gf.Vec3f(0,0,0),Gf.Vec3f(length,0,0),Gf.Vec3f(0,.01,0)])
   mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2]);UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('sdf')
   schemas=mesh.GetPrim().GetMetadata('apiSchemas').GetAddedOrExplicitItems();mesh.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(list(schemas)+['PhysxSDFMeshCollisionAPI']))
   mesh.GetPrim().CreateAttribute('physxCookedData:triangleMesh:buffer',Sdf.ValueTypeNames.UCharArray).Set([1,2,3]);meshes.append(mesh)
  before=[list(m.GetPointsAttr().Get()) for m in meshes];records=refine_sdf_meshes(stage)
  self.assertEqual({r['role'] for r in records},{'HeldAsset','FixedAsset'})
  self.assertTrue(all(r['maximum_local_cell_spacing_m']<.000057 for r in records))
  for m in meshes[:2]:
   self.assertEqual(m.GetPrim().GetAttribute('physxSDFMeshCollision:sdfResolution').Get(),1024)
   self.assertIsNone(m.GetPrim().GetAttribute('physxCookedData:triangleMesh:buffer').Get())
  self.assertEqual([list(m.GetPointsAttr().Get()) for m in meshes],before)
  self.assertEqual(list(meshes[2].GetPrim().GetAttribute('physxCookedData:triangleMesh:buffer').Get()),[1,2,3])
