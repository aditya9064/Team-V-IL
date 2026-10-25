"""Post-terminal cooking-service representation. Never a runtime PxShape readback.

Uses the installed NVIDIA 110 ConvexMeshDataDemo synchronous request API. No
stage edits, force loading, reset, step, render, update or controller operations.
A request may obtain/recompute cached cooking data; it does not prove the exact
active low-level collision object. That limitation stays in the report.
"""
from payload_contact_trace import finite,integer,vector


def serialize_convex(raw):
    vertices=[vector((v.x,v.y,v.z)) for v in raw.vertices]
    indices=[integer(i) for i in raw.indices];polygons=[]
    for face in raw.polygons:
        base=integer(face.index_base);count=integer(face.num_vertices)
        if base<0 or count<3 or base+count>len(indices):raise ValueError('Invalid cooked polygon slice')
        polygons.append({'index_base':base,'num_vertices':count})
    if len(vertices)<4 or not polygons or any(i<0 or i>=len(vertices) for i in indices):raise ValueError('Invalid cooked convex topology')
    return {'vertices_mesh_local':vertices,'indices':indices,'polygons':polygons}


def read_cooked_payload(stage,world,roots,original_status):
    evidence={'schema':'POST_TERMINAL_PAYLOAD_COOKING_REPRESENTATION_V1','original_status':str(original_status),'complete':True,'errors':[],
              'kind':'cooking-service convex representation, NOT direct runtime PxShape readback',
              'timing':'after original terminal motion outcome; no further physics or target issue',
              'may_obtain_or_recompute_cached_cooking_data':True,'physics_or_target_use':False,'colliders':{}}
    try:
        from pxr import UsdGeom,UsdPhysics,UsdUtils,PhysicsSchemaTools
        import omni.physx
        from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
        before=finite(world.current_time);evidence['sim_time_before_s']=before
        interface=omni.physx.get_physx_cooking_interface()
        stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        paths=sorted(str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(UsdPhysics.CollisionAPI)
                     and (str(prim.GetPath()).startswith(roots['pcb']+'/') or str(prim.GetPath()).startswith(roots['carrier']+'/')))
        evidence['expected_paths']=paths
        if not paths:raise ValueError('No payload colliders found')
        for path in paths:
            try:
                prim=stage.GetPrimAtPath(path);mesh=UsdGeom.Mesh(prim)
                matrix=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                actor=roots['pcb'] if path.startswith(roots['pcb']+'/') else roots['carrier']
                root_matrix=UsdGeom.Xformable(stage.GetPrimAtPath(actor)).ComputeLocalToWorldTransform(0)
                value={'actor':actor,'callback_count':0,'convexes':[],
                       'mesh_to_world_matrix':[list(row) for row in matrix],
                       'actor_to_world_matrix':[list(row) for row in root_matrix],
                       'mesh_to_actor_matrix':[list(row) for row in matrix*root_matrix.GetInverse()],
                       'authored_points_mesh_local':[vector(v) for v in mesh.GetPointsAttr().Get()]}
                evidence['colliders'][path]=value
                def receive(result,convexes):
                    value['callback_count']+=1;value['result_code']=int(result);value['result_repr']=repr(result)
                    value['result_valid']=result==PhysxCollisionRepresentationResult.RESULT_VALID
                    value['convexes']=[serialize_convex(c) for c in convexes]
                interface.request_convex_collision_representation(stage_id=stage_id,collision_prim_id=PhysicsSchemaTools.sdfPathToInt(path),run_asynchronously=False,on_result=receive)
                if value['callback_count']!=1 or not value.get('result_valid') or not value['convexes']:raise ValueError('Missing/invalid synchronous convex representation')
            except Exception as exc:
                evidence['complete']=False;evidence['errors'].append({'path':path,'type':type(exc).__name__,'message':str(exc)})
        evidence['sim_time_after_s']=finite(world.current_time)
        if evidence['sim_time_after_s']!=before:raise ValueError('Cooking representation request changed simulation time')
    except Exception as exc:
        evidence['complete']=False;evidence['errors'].append({'type':type(exc).__name__,'message':str(exc)})
    return evidence
