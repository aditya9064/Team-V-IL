"""One tiny, untrained T4 backward pass. No optimizer update or checkpoint."""
import argparse
import json
from pathlib import Path
import torch
from detector_data import CubeDataset
from detector_model import make_model

def main():
    p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    assert torch.cuda.is_available(),'CUDA unavailable'
    torch.manual_seed(8);torch.set_num_threads(4)
    dataset=CubeDataset(a.data,'train',verify_hashes=True)
    # Load every target, including empty annotations, to validate package contents.
    count=sum(len(dataset[i][1]['labels']) for i in range(len(dataset)))
    for split in ('val','test'):
        other=CubeDataset(a.data,split,verify_hashes=True)
        count+=sum(len(other[i][1]['labels']) for i in range(len(other)))
    assert count==432,count
    positive=next(i for i,r in enumerate(dataset.records) if r['cube_count']>0)
    empty=next(i for i,r in enumerate(dataset.records) if r['cube_count']==0)
    samples=[dataset[i] for i in (positive,empty)]
    images=[im.cuda() for im,_ in samples]
    targets=[{k:v.cuda() for k,v in t.items()} for _,t in samples]
    model=make_model(False,min_size=320,max_size=768).cuda().train()
    losses=model(images,targets);loss=sum(losses.values())
    assert torch.isfinite(loss),losses
    loss.backward()
    gradients=[p.grad for p in model.parameters() if p.grad is not None]
    assert gradients and all(torch.isfinite(g).all() for g in gradients)
    model.eval()
    with torch.inference_mode():predictions=model(images)
    assert len(predictions)==2
    assert all(p['boxes'].shape[1]==4 and torch.isfinite(p['boxes']).all() for p in predictions)
    assert all(((p['labels']>=1)&(p['labels']<=3)).all() for p in predictions)
    torch.cuda.synchronize()
    result=dict(passed=True,model='fasterrcnn_resnet50_fpn_v2',
                torch_version=str(torch.__version__),gpu=torch.cuda.get_device_name(0),
                scenes=78,labels_checked=count,losses={k:float(v.detach()) for k,v in losses.items()},
                empty_target_backward_passed=True,inference_passed=True,
                weights_downloaded=False,optimizer_steps=0,trained_model=False,
                smoke_resize=dict(min_size=320,max_size=768),training_resize=dict(min_size=864,max_size=1920))
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print('TEAM_V_DETECTOR_SMOKE_PASS',json.dumps(result),flush=True)

if __name__=='__main__':main()
