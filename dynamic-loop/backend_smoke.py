"""Actual architecture/load/GPU inference smoke with UNTRAINED weights; no motion."""
import argparse,gc,hashlib,json,os,sys,tempfile,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--starter',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
a=p.parse_args();sys.path.insert(1,str(a.starter));training=Path(__file__).resolve().parent.parent/'mentor-training';sys.path.insert(1,str(training))
import torch,cv2
from detector_model import make_model
from detector_backend import TorchvisionBackend
started=time.monotonic()
with tempfile.TemporaryDirectory(prefix='team-v-untrained-smoke-') as tmp:
    root=Path(tmp);model=make_model(pretrained=False)
    cfg=dict(model='fasterrcnn_resnet50_fpn_v2',classes=['yellow','blue','red'],foreground_labels=[1,2,3],min_size=864,max_size=1920)
    checkpoint=root/'untrained.pt';torch.save(dict(model=model.state_dict(),config=cfg,epoch=0,untrained_smoke=True),checkpoint)
    del model;gc.collect()
    deployment=dict(schema_version=1,architecture=cfg['model'],classes=cfg['classes'],foreground_labels=[1,2,3],
       full_resolution=[3840,2160],crop_rect_px=[960,1080,2880,1944],inference_resolution=[1920,864],
       min_size=864,max_size=1920,score_threshold=1.0,threshold_source='UNTRAINED-SMOKE-ONLY-NOT-A-DEPLOYMENT-THRESHOLD',
       checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest())
    config=root/'deployment.json';config.write_text(json.dumps(deployment))
    backend=TorchvisionBackend(checkpoint,config,training,device='cuda')
    sample=Path.home()/'team-v-samples/pilot-001'
    image=cv2.imread(str(sample/'world_camera.png'));camera=json.loads((sample/'camera.json').read_text())
    if image is None:raise RuntimeError('Smoke input RGB missing')
    accepted,rejected=backend.predict(image,camera);torch.cuda.synchronize()
    assert not accepted,'Untrained smoke is not allowed to supply motion targets'
    report=dict(status='PASS',untrained_weights=True,training_steps=0,motion_commands=0,
                gpu=torch.cuda.get_device_name(0),torch=str(torch.__version__),elapsed_s=time.monotonic()-started,
                accepted_proposals=len(accepted),rejected_proposals=len(rejected),
                interpretation='Architecture loading/RGB/crop/GPU inference smoke only; no detector accuracy or trained-checkpoint claim')
    a.out.write_text(json.dumps(report,indent=2)+'\n');print('UNTRAINED_BACKEND_SMOKE_PASS',report,flush=True)
