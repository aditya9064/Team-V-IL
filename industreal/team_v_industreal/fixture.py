"""Explicit immutable derived socket matching the task's declared bore size."""
import hashlib,json,math
from pathlib import Path

def verified_fixture(root=None):
    root=Path(root) if root else Path(__file__).parents[1]/'fixtures'
    record=json.loads((root/'fixture-patch.json').read_text())
    original=root/'factory_hole_original_9mm.usd';derived=root/'factory_hole_declared_8p1mm.usda'
    for p,key in [(original,'original_fixture_sha256'),(derived,'derived_fixture_sha256')]:
        if hashlib.sha256(p.read_bytes()).hexdigest()!=record[key]:raise ValueError('Fixture checksum mismatch: '+p.name)
    if record['original_inner_bore_diameter_m']!=.009 or record['derived_inner_bore_diameter_m']!=.0081:
        raise ValueError('Unexpected fixture dimensions')
    if record['inner_vertices_changed']!=432 or not record['outer_vertices_faces_and_topology_unchanged']:
        raise ValueError('Unqualified fixture mutation')
    clearance=.0081/2-record['peg_measured_max_radius_m']
    if not math.isclose(clearance,.000057,abs_tol=1e-8):raise ValueError('Measured fixture/peg clearance mismatch')
    return str(derived.resolve()),record


def refine_sdf_meshes(stage,resolution=1024):
    """Refine numerical contact fields; preserve all original mesh points/faces."""
    from pxr import UsdGeom,UsdPhysics,Sdf
    if not isinstance(resolution,int) or resolution<1024:raise ValueError('SDF spacing must resolve the57um fit')
    records=[];roles=set()
    for prim in stage.Traverse():
        parts=str(prim.GetPath()).split('/')
        role=next((r for r in ('HeldAsset','FixedAsset') if r in parts),None)
        if role is None or not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.MeshCollisionAPI):continue
        if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()!='sdf':raise ValueError('Unexpected collision approximation')
        api=prim.GetMetadata('apiSchemas')
        if api is None or 'PhysxSDFMeshCollisionAPI' not in api.GetAddedOrExplicitItems():raise ValueError('Native SDF API missing')
        old=prim.GetAttribute('physxSDFMeshCollision:sdfResolution').Get()
        points=UsdGeom.Mesh(prim).GetPointsAttr().Get();extent=max(max(float(p[i]) for p in points)-min(float(p[i]) for p in points) for i in range(3))
        prim.CreateAttribute('physxSDFMeshCollision:sdfResolution',Sdf.ValueTypeNames.Int,custom=False,variability=Sdf.VariabilityUniform).Set(resolution)
        prim.CreateAttribute('physxSDFMeshCollision:sdfSubgridResolution',Sdf.ValueTypeNames.Int,custom=False,variability=Sdf.VariabilityUniform).Set(6)
        blocked=[]
        for attr in prim.GetAttributes():
            if attr.GetName().startswith('physxCookedData:'):
                attr.Block();assert attr.Get() is None;blocked.append(attr.GetName())
        if extent/resolution>.000057:raise ValueError('SDF cell remains larger than final fit')
        records.append({'prim':str(prim.GetPath()),'role':role,'previous_authored_or_registered_resolution':old,'resolution':resolution,'sparse_subgrid_resolution':6,'maximum_local_cell_spacing_m':extent/resolution,'blocked_old_cooked_buffers':blocked,'geometry_unchanged':True})
        roles.add(role)
    if roles!={'HeldAsset','FixedAsset'}:raise ValueError('Both physical collision meshes must be refined')
    return records
