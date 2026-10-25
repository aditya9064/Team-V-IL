"""Recipe/CAD release pose, independent of simulator object coordinates."""
import numpy as np

def square_release_plan(support_origin,handle_xy_mm,tcp_mm,tool_axis,spread_axis,closing_axis):
    p=np.asarray(support_origin,float);h=np.asarray(handle_xy_mm,float);a=np.asarray(tool_axis,float);b=np.asarray(spread_axis,float)
    if p.shape!=(3,) or h.shape!=(2,) or a.shape!=(3,) or b.shape!=(3,) or not all(np.isfinite(v).all() for v in [p,h,a,b]) or not np.isfinite(tcp_mm) or tcp_mm<=0:raise ValueError('Invalid release recipe')
    basis=np.column_stack([a,b,np.cross(a,b)])
    if not np.allclose(basis.T@basis,np.eye(3),atol=1e-6) or closing_axis not in ('X','Y'):raise ValueError('Invalid release axes')
    down=np.array([0.,0.,-1.]);spread=np.array([1.,0.,0.]) if closing_axis=='X' else np.array([0.,1.,0.])
    rotation=np.column_stack([down,spread,np.cross(down,spread)])@basis.T
    return {'tip_world_m':(p+np.array([h[0],h[1],tcp_mm])*.001).tolist(),'tool_rotation_world':rotation.tolist(),'relaxed_per_jaw_m':.0023,'recipe_based':True,'source':'known fixed assembly datum + original CAD finger-release pose; no simulator object coordinates','stages':['relax clamp','square fingers','full open']}
