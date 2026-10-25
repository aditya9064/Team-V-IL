import unittest,json
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from held_pose import project,estimate,placement_from_held

def model(eye,centre,res=(1280,960)):
    eye,centre=map(lambda v:np.asarray(v,float),(eye,centre));z=(eye-centre);z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    matrix=np.eye(4);matrix[:3,:3]=np.array([x,y,z]);matrix[3,:3]=eye
    return {'resolution':list(res),'focal_length_mm':35.,'horizontal_aperture_mm':20.955,'vertical_aperture_mm':20.955*res[1]/res[0],'camera_to_world':matrix.tolist()}

def rendered(mesh,pos,R,m):
    w,h=m['resolution'];rgb=np.full((h,w,3),150,np.uint8);v=np.array(mesh['vertices_mm'])*.001@R.T+pos;uv=project(v,m)
    for face in mesh['faces']:cv2.fillConvexPoly(rgb,np.round(uv[face]*16).astype(np.int32),(35,110,180),shift=4)
    return rgb

class HeldPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mesh=json.loads(Path(__file__).with_name('trial-mesh.json').read_text())['parts']['base']['visual'];cls.nominal=np.array([.3,.1,1.05]);cls.models={n:model(cls.nominal+offset,cls.nominal+[0,0,.01]) for n,offset in [('x',[.19,-.08,-.05]),('y',[-.08,-.19,-.05])]}
    def test_rgb_reconstructs_non_nominal_held_pose(self):
        R=Rotation.from_euler('xyz',[.6,-1.4,.4],degrees=True).as_matrix();pos=self.nominal+[.00012,-.00008,-.0006]
        images={n:rendered(self.mesh,pos,R,m) for n,m in self.models.items()};fit=estimate(images,self.models,self.mesh['vertices_mm'],self.nominal)
        self.assertLess(np.linalg.norm((np.array(fit['position_world_m'])-pos)*1000),.15)
        self.assertLess(Rotation.from_matrix(np.array(fit['rotation_world']).T@R).magnitude()*180/np.pi,.25)
    def test_empty_images_rejected(self):
        with self.assertRaises(ValueError):estimate({n:np.zeros((960,1280,3),np.uint8) for n in self.models},self.models,self.mesh['vertices_mm'],self.nominal)
    def test_occluded_lower_edges_rejected(self):
        images={n:rendered(self.mesh,self.nominal,np.eye(3),m) for n,m in self.models.items()}
        images['x'][480:,:,:]=150
        with self.assertRaises(ValueError):estimate(images,self.models,self.mesh['vertices_mm'],self.nominal)
    def test_actual_transform_levels_and_positions_base(self):
        R=Rotation.from_euler('xyz',[1.5,-.2,.3],degrees=True).as_matrix();p=self.nominal.copy();tip=p+[.0165,0,.01382];T=Rotation.from_euler('xyz',[180,0,0],degrees=True).as_matrix();dest=np.array([.3,.1,.9])
        result=placement_from_held(p,R,tip,T,dest,self.mesh['vertices_mm']);newT=np.array(result['tool_rotation_world']);newtip=np.array(result['tip_world_m']);local=T.T@(p-tip)
        np.testing.assert_allclose(newtip+newT@local,dest+[0,0,.0001],atol=1e-12)
        np.testing.assert_allclose(newT@(T.T@R),np.eye(3),atol=1e-12)
    def test_nonfinite_and_bad_rotation_rejected(self):
        with self.assertRaises(ValueError):placement_from_held([np.nan,0,0],np.eye(3),[0,0,0],np.eye(3),[0,0,0],self.mesh['vertices_mm'])
        with self.assertRaises(ValueError):placement_from_held([0,0,0],np.zeros((3,3)),[0,0,0],np.eye(3),[0,0,0],self.mesh['vertices_mm'])
if __name__=='__main__':unittest.main()

class ControllerInvariants(unittest.TestCase):
    def test_provenance_and_protected_checks(self):
        import ast,hashlib
        root=Path(__file__).parent;p=json.loads((root/'held-placement-provenance.json').read_text());new=(root/'held_base_seating_trial.py').read_text()
        for a,b in reversed(p['replacements']):
            self.assertEqual(new.count(b),1,b[:80]);new=new.replace(b,a)
        self.assertEqual(new,(root/'holder_assembly_trial.py').read_text())
        for n,h in p['unchanged_files'].items():self.assertEqual(hashlib.sha256((root/n).read_bytes()).hexdigest(),h)
        new=new.replace('SIMULATION | base > loaded tray > lid | current fingers','SIMULATION | RGB held-pose base seating/release | current fingers')
        oldast=ast.parse(new);newast=ast.parse((root/'held_base_seating_trial.py').read_text())
        for name in ['on_contacts','fail_contacts','advance','command','feedback','save','poses','score_placed']:
            nodes=[next(n for n in ast.walk(a) if isinstance(n,ast.FunctionDef) and n.name==name) for a in [oldast,newast]]
            self.assertEqual(ast.dump(nodes[0]),ast.dump(nodes[1]),name)
    def test_estimator_accepts_no_truth_pose_api(self):
        import inspect
        self.assertEqual(list(inspect.signature(estimate).parameters),['images','models','vertices_mm','nominal_position'])
