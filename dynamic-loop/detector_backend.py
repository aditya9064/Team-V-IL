"""Explicit detector selection. Missing model/config never silently falls back."""
import hashlib
import importlib.util
import json
from pathlib import Path
from detector_contract import (CLASSES,DetectorContractError,validate_deployment,
                               deployment_id,map_predictions)

class GeometricBackend:
    model_id='geometric-hsv-silhouette-v1'
    metadata={'backend':'geometric','checkpoint_loaded':False}
    def predict(self,image,camera):
        from vision import detect
        return detect(image,camera)

class TorchvisionBackend:
    def __init__(self,checkpoint,config_path,training_code,device='cuda'):
        import torch
        self.torch=torch;self.config=validate_deployment(json.loads(Path(config_path).read_text()))
        checkpoint=Path(checkpoint)
        digest=hashlib.sha256(checkpoint.read_bytes()).hexdigest()
        if digest!=self.config['checkpoint_sha256']:raise DetectorContractError('checkpoint_checksum_mismatch')
        # Keep PyTorch's restricted loader. Never fall back to arbitrary pickle.
        state=torch.load(checkpoint,map_location='cpu',weights_only=True)
        if not isinstance(state,dict) or not isinstance(state.get('model'),dict) or not isinstance(state.get('config'),dict):
            raise DetectorContractError('unsupported_checkpoint_structure')
        trained=state['config']
        for key,expected in (('model',self.config['architecture']),('classes',list(CLASSES)),
                             ('foreground_labels',[1,2,3]),('min_size',self.config['min_size']),('max_size',self.config['max_size'])):
            if trained.get(key)!=expected:raise DetectorContractError('checkpoint_config_mismatch:'+key)
        architecture=Path(training_code)/'detector_model.py'
        spec=importlib.util.spec_from_file_location('team_v_detector_architecture',architecture)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.model=module.make_model(pretrained=False,min_size=self.config['min_size'],max_size=self.config['max_size'])
        self.model.load_state_dict(state['model'],strict=True)
        self.device=torch.device(device)
        if self.device.type=='cuda' and not torch.cuda.is_available():raise DetectorContractError('cuda_unavailable')
        self.model.to(self.device).eval()
        architecture_sha=hashlib.sha256(architecture.read_bytes()).hexdigest()
        self.model_id=deployment_id(self.config)+':'+architecture_sha
        self.metadata=dict(backend='torchvision',checkpoint_loaded=True,checkpoint_sha256=digest,
            architecture_sha256=architecture_sha,
            deployment=self.config,model_id=self.model_id,
            description='CNN box/colour proposals plus unchanged RGB silhouette pose fitting')

    def predict(self,image,camera):
        import cv2
        import numpy as np
        from vision import refine_proposals
        if image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:
            raise DetectorContractError('expected_uint8_bgr_image')
        if [image.shape[1],image.shape[0]]!=self.config['full_resolution'] or camera['resolution']!=self.config['full_resolution']:
            raise DetectorContractError('camera_image_deployment_resolution_mismatch')
        left,top,right,bottom=self.config['crop_rect_px']
        rgb=cv2.cvtColor(image[top:bottom,left:right],cv2.COLOR_BGR2RGB)
        size=self.config['inference_resolution']
        if [rgb.shape[1],rgb.shape[0]]!=size:rgb=cv2.resize(rgb,tuple(size),interpolation=cv2.INTER_LINEAR)
        tensor=self.torch.from_numpy(np.ascontiguousarray(rgb)).permute(2,0,1).to(self.device,dtype=self.torch.float32)/255.
        with self.torch.inference_mode():output=self.model([tensor])[0]
        payload={k:output[k].detach().cpu().tolist() for k in ('boxes','labels','scores')}
        proposals,rejected=map_predictions(payload,self.config['full_resolution'],self.config['crop_rect_px'],size,self.config['score_threshold'])
        fitted,fit_rejected=refine_proposals(image,camera,proposals)
        return fitted,rejected+fit_rejected
