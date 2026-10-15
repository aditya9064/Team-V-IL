import argparse
from pathlib import Path
import torch
from detector_data import CubeDataset
from detector_model import make_model
from metrics import evaluate

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--data',type=Path,default=Path(__file__).resolve().parent.parent)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--split',choices=['val','test'],default='test')
    p.add_argument('--out',type=Path,default=Path('runs/test-evaluation'))
    p.add_argument('--device',default='cuda')
    p.add_argument('--batch-size',type=int,default=2)
    p.add_argument('--workers',type=int,default=4)
    a=p.parse_args()
    # Only load checkpoints produced by this trusted training script.
    checkpoint=torch.load(a.checkpoint,map_location='cpu',weights_only=True)
    config=checkpoint['config']
    model=make_model(False,config['min_size'],config['max_size'])
    model.load_state_dict(checkpoint['model']);model.to(a.device)
    result=evaluate(model,CubeDataset(a.data,a.split,verify_hashes=True),a.device,
                    a.batch_size,a.workers,a.out)
    print(result)

if __name__=='__main__':main()
