"""Loading/RGB tests use a tiny explicit untrained fixture, never robot control."""
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from detector_contract import DetectorContractError

FAKE_ARCH='''import torch
class Fixture(torch.nn.Module):
    def __init__(self):
        super().__init__();self.bias=torch.nn.Parameter(torch.zeros(1))
    def forward(self,images):
        x=images[0]
        assert not self.training and not torch.is_grad_enabled()
        assert x.dtype==torch.float32 and float(x[0].mean())==1.0 and float(x[1:].max())==0.0
        return [dict(boxes=torch.tensor([[1.,2.,5.,6.]],device=x.device),labels=torch.tensor([1],device=x.device),scores=torch.tensor([.9],device=x.device))]
def make_model(**kwargs):return Fixture()
'''

class BackendFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        starter=os.environ.get('FANUC_STARTER')
        if not starter:raise unittest.SkipTest('FANUC_STARTER plus existing torch/OpenCV environment required')
        sys.path.insert(0,starter)
        import torch,numpy,vision
        cls.torch=torch;cls.np=numpy;cls.vision=vision
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        (self.root/'detector_model.py').write_text(FAKE_ARCH)
        self.config=dict(schema_version=1,architecture='fasterrcnn_resnet50_fpn_v2',classes=['yellow','blue','red'],
            foreground_labels=[1,2,3],full_resolution=[100,100],crop_rect_px=[10,20,30,40],
            inference_resolution=[20,20],score_threshold=.8,threshold_source='untrained-unit-fixture-only',
            checkpoint_sha256='a'*64,min_size=864,max_size=1920)
        self.trained=dict(model=self.config['architecture'],classes=self.config['classes'],foreground_labels=[1,2,3],min_size=864,max_size=1920)
        self.state={'bias':self.torch.zeros(1)};self.write()
    def write(self):
        self.path=self.root/'fixture.pt'
        self.torch.save(dict(model=self.state,config=self.trained,epoch=0,untrained_fixture=True),self.path)
        self.config['checkpoint_sha256']=hashlib.sha256(self.path.read_bytes()).hexdigest()
        self.cfg=self.root/'deployment.json';self.cfg.write_text(json.dumps(self.config))
    def load(self):
        from detector_backend import TorchvisionBackend
        return TorchvisionBackend(self.path,self.cfg,self.root,device='cpu')
    def test_rgb_conversion_normalization_eval_and_crop_mapping(self):
        backend=self.load();image=self.np.zeros((100,100,3),self.np.uint8);image[:,:,2]=255
        with patch.object(self.vision,'refine_proposals',side_effect=lambda image,camera,p:(p,[])):
            accepted,rejected=backend.predict(image,{'resolution':[100,100]})
        self.assertEqual(accepted[0]['bbox_full_px'],[11.,22.,15.,26.]);self.assertEqual(rejected,[])
        self.assertFalse(backend.model.training);self.assertTrue(backend.metadata['checkpoint_loaded'])
    def test_changed_checksum_is_rejected_without_fallback(self):
        self.path.write_bytes(self.path.read_bytes()+b'changed')
        with self.assertRaisesRegex(DetectorContractError,'checksum'):self.load()
    def test_checkpoint_class_mapping_must_match(self):
        self.trained['foreground_labels']=[0,1,2];self.write()
        with self.assertRaisesRegex(DetectorContractError,'config_mismatch'):self.load()
    def test_checkpoint_state_load_is_strict(self):
        self.state={};self.write()
        with self.assertRaises(RuntimeError):self.load()

if __name__=='__main__':unittest.main()
