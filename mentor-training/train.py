"""Initial detector fine-tuning recipe; mentor runs this on their H100."""
import argparse
import json
import math
import random
from pathlib import Path
import numpy as np
import torch
from torch.utils.data import DataLoader
from detector_data import CubeDataset, collate, CLASSES
from detector_model import make_model
from metrics import evaluate

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--data',type=Path,default=Path(__file__).resolve().parent.parent)
    p.add_argument('--out',type=Path,default=Path('runs/detector-pilot'))
    p.add_argument('--epochs',type=int,default=30)
    p.add_argument('--batch-size',type=int,default=2)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--lr',type=float,default=.0025)
    p.add_argument('--seed',type=int,default=20261014)
    p.add_argument('--device',default='cuda')
    p.add_argument('--amp',choices=['auto','bf16','fp16','none'],default='auto')
    a=p.parse_args()
    if min(a.epochs,a.batch_size)<=0 or a.workers<0: p.error('Invalid training counts')
    if a.out.exists(): p.error('Use a fresh output directory to preserve previous experiments')
    device=torch.device(a.device)
    if device.type=='cuda' and not torch.cuda.is_available(): raise RuntimeError('CUDA unavailable')
    random.seed(a.seed);np.random.seed(a.seed);torch.manual_seed(a.seed)
    if device.type=='cuda':torch.cuda.manual_seed_all(a.seed)
    torch.backends.cudnn.benchmark=False
    training=CubeDataset(a.data,'train',augment=True,verify_hashes=True)
    validation=CubeDataset(a.data,'val',verify_hashes=True)
    loader=DataLoader(training,batch_size=a.batch_size,shuffle=True,num_workers=a.workers,
                      collate_fn=collate,pin_memory=device.type=='cuda',
                      generator=torch.Generator().manual_seed(a.seed))
    model=make_model().to(device)
    optimizer=torch.optim.SGD(model.parameters(),lr=a.lr,momentum=.9,weight_decay=.0005)
    schedule=torch.optim.lr_scheduler.StepLR(optimizer,step_size=10,gamma=.1)
    amp=a.amp
    if amp=='auto': amp=('bf16' if torch.cuda.is_bf16_supported() else 'fp16') if device.type=='cuda' else 'none'
    if amp!='none' and device.type!='cuda':raise ValueError('This recipe uses AMP only on CUDA')
    if amp=='bf16' and not torch.cuda.is_bf16_supported():raise ValueError('BF16 unavailable')
    dtype=torch.bfloat16 if amp=='bf16' else torch.float16
    scaler=torch.amp.GradScaler('cuda',enabled=amp=='fp16')
    a.out.mkdir(parents=True)
    config={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()}
    config.update(classes=list(CLASSES),foreground_labels=[1,2,3],resolved_amp=amp,
                  torch_version=str(torch.__version__),model='fasterrcnn_resnet50_fpn_v2',
                  selection_metric='validation AP50',min_size=864,max_size=1920)
    (a.out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    best=-1.
    for epoch in range(1,a.epochs+1):
        model.train();losses=[]
        for images,targets in loader:
            images=[x.to(device) for x in images]
            targets=[{k:v.to(device) for k,v in t.items()} for t in targets]
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type,dtype=dtype,enabled=amp!='none'):
                parts=model(images,targets);loss=sum(parts.values())
            if not torch.isfinite(loss):raise RuntimeError(f'Non-finite loss at epoch {epoch}')
            scaler.scale(loss).backward();scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(),10.,error_if_nonfinite=True)
            scaler.step(optimizer);scaler.update();losses.append(float(loss.detach()))
        score=evaluate(model,validation,device,a.batch_size,a.workers,
                       a.out/'validation'/f'epoch-{epoch:03}')
        schedule.step()
        state=dict(model=model.state_dict(),optimizer=optimizer.state_dict(),
                   scheduler=schedule.state_dict(),scaler=scaler.state_dict(),epoch=epoch,
                   config=config,validation=score)
        torch.save(state,a.out/'last.pt')
        ap50=score['overall']['AP50']
        if ap50 is not None and ap50>best:
            best=ap50;torch.save(state,a.out/'best.pt')
        row=dict(epoch=epoch,mean_loss=float(np.mean(losses)),validation=score,best_AP50=best)
        with (a.out/'history.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps(row),flush=True)
    print('Training complete. Evaluate best.pt on the held-out test split once after model selection.',flush=True)

if __name__=='__main__':main()
