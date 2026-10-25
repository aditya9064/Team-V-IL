import numpy as np,math
def query_mass(stage,prim,mass,world):
 from omni.physx import get_physx_property_query_interface
 from omni.physx.bindings._physx import PhysxPropertyQueryResult,PhysxPropertyQueryMode
 from pxr import UsdUtils,PhysicsSchemaTools
 result={};start=float(world.current_time)
 def received(x):
  if x.result!=PhysxPropertyQueryResult.VALID:result['error']='PhysX mass query invalid: '+str(x.result);return
  q=list(x.principal_axes)
  result.update(mass_kg=float(x.mass),centre_of_mass_local_m=[float(v) for v in x.center_of_mass],diagonal_inertia_kg_m2=[float(v) for v in x.inertia],principal_axes_wxyz=[float(q[3]),*[float(v) for v in q[:3]]])
 get_physx_property_query_interface().query_prim(stage_id=UsdUtils.StageCache.Get().Insert(stage).ToLongInt(),prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPrimPath()),query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,rigid_body_fn=received,timeout_ms=2000)
 renders=0
 while not result and renders<30:
  world.render();renders+=1
 if float(world.current_time)!=start:raise RuntimeError('Mass query advanced physics')
 if not result or 'error' in result:raise RuntimeError(result.get('error','Mass query callback incomplete'))
 assert abs(result['mass_kg']-mass)<1e-8 and np.isfinite(result['centre_of_mass_local_m']).all() and all(math.isfinite(v) and v>0 for v in result['diagonal_inertia_kg_m2']);result.update(validated=True,physics_time_advanced_s=0.,render_only_callback_updates=renders);return result
