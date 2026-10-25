"""Fit the small actor on CPU; no simulator and no physical hardware."""
import argparse,json
from pathlib import Path
import torch
from team_v_industreal.learned_policy import Policy,warm_start,save_checkpoint
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(exist_ok=False)
torch.manual_seed(1000);torch.set_num_threads(4);m=Policy();metrics=warm_start(m,a.out)
sha=save_checkpoint(m,a.out/'warm-start.pt',{'method':'synthetic_teacher_least_squares_warm_start','fitted_policy':True,'training_seeds':[],'ppo_updates':0,'teacher_examples':metrics['training_examples']})
(a.out/'checkpoint.json').write_text(json.dumps({'sha256':sha,'checkpoint':str(a.out/'warm-start.pt'),'metrics':metrics},indent=2)+'\n');print(json.dumps(metrics))
