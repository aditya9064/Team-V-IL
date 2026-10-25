"""Goal-conditioned learned Cartesian correction policy; no simulator imports.

A learned linear Gaussian actor is deliberately small for this local assembly
servo. A deterministic phase supervisor supplies frozen-calibration pose goals.
This is not a learned task planner or the original Franka checkpoint.
"""
import hashlib,json,math
from pathlib import Path
import torch
from torch import nn
from .control import pose_error,quaternion_multiply
SCALE=(.002,.002,.002,.01,.01,.01)
OBSERVATION_SCHEMA='pose_error6/socket_table_force3/jaw_net_force3/previous_action6-v1'

class Policy(nn.Module):
    def __init__(self):
        super().__init__();self.actor=nn.Linear(18,6)
        self.critic=nn.Sequential(nn.Linear(18,64),nn.ELU(),nn.Linear(64,64),nn.ELU(),nn.Linear(64,1))
        self.register_buffer('sigma',torch.full((6,),.01))
    def mean(self,x):return self.actor(x).clamp(-1.,1.)
    def distribution(self,x):return torch.distributions.Normal(self.mean(x),self.sigma)
    def value(self,x):return self.critic(x).squeeze(-1)

def observation(tcp,quaternion,goal,goal_quaternion,forces,previous):
    error=pose_error(tcp,quaternion,goal,goal_quaternion)/torch.tensor(SCALE,device=tcp.device,dtype=tcp.dtype)
    if forces.shape!=(*tcp.shape[:-1],6) or previous.shape!=(*tcp.shape[:-1],6):raise ValueError('Wrong policy observation shape')
    out=torch.cat((error,forces.clamp(-50,50),previous),-1)
    if not torch.isfinite(out).all():raise ValueError('Nonfinite policy observation')
    return out

def action_target(tcp,quaternion,action,max_translation_m=.002):
    if action.shape!=(*tcp.shape[:-1],6) or not torch.isfinite(action).all():raise ValueError('Invalid policy action')
    a=action.clamp(-1,1)*torch.tensor(SCALE,device=tcp.device,dtype=tcp.dtype)
    if not math.isfinite(max_translation_m) or not 0<max_translation_m<=.002:raise ValueError('Invalid learned-action speed cap')
    norm=a[...,:3].norm(dim=-1,keepdim=True);a[...,:3]*=(max_translation_m/norm.clamp_min(1e-12)).clamp_max(1)
    r=a[...,3:];angle=r.norm(dim=-1,keepdim=True);half=angle/2
    q=torch.cat((half.cos(),r*torch.where(angle>1e-8,half.sin()/angle.clamp_min(1e-8),torch.full_like(angle,.5))),-1)
    return tcp+a[...,:3],quaternion_multiply(q,quaternion)

def warm_start(model,out,seed=1000,count=16384):
    """Fit weights from synthetic teacher pose-error examples, not physical demonstrations."""
    g=torch.Generator().manual_seed(seed)
    x=torch.randn((count,18),generator=g,dtype=torch.float64)
    x[:,:6]*=4.;x[:,12:]=x[:,12:].clamp(-1,1)
    y=x[:,:6].clone()  # teacher unsaturated Cartesian servo correction; actor clips at inference
    design=torch.cat((x,torch.ones(count,1,dtype=x.dtype)),1)
    coefficients=torch.linalg.lstsq(design,y).solution
    with torch.no_grad():
        model.actor.weight.copy_(coefficients[:-1].T.to(model.actor.weight));model.actor.bias.copy_(coefficients[-1].to(model.actor.bias))
    val=torch.randn((2048,18),generator=g);pred=model.mean(val);target=val[:,:6].clamp(-1,1)
    metrics={'kind':'synthetic_goal_error_teacher_regression','training_examples':count,'validation_examples':2048,'maximum_normalized_validation_error':float((pred-target).abs().max().detach()),'seed':seed,'simulator_demonstrations':False,'original_franka_checkpoint_used':False}
    if metrics['maximum_normalized_validation_error']>1e-5:raise RuntimeError('Warm-start fitting failed')
    out=Path(out);out.mkdir(exist_ok=True,parents=True)
    torch.save({'observations':x.float(),'teacher_unsaturated_actions':y.float(),'schema':OBSERVATION_SCHEMA},out/'teacher-data.pt')
    (out/'warm-start.json').write_text(json.dumps(metrics,indent=2)+'\n');return metrics

def gae(rewards,values,done,gamma=.995,tau=.95):
    if len(values)!=len(rewards)+1 or len(done)!=len(rewards):raise ValueError('GAE alignment error')
    advantages=torch.zeros_like(rewards);last=torch.zeros_like(rewards[0])
    for i in reversed(range(len(rewards))):
        live=1-done[i];delta=rewards[i]+gamma*values[i+1]*live-values[i]
        last=delta+gamma*tau*live*last;advantages[i]=last
    return advantages,advantages+values[:-1]

def ppo_update(model,rollout,seed=9000):
    """Clipped PPO, with teacher anchoring to preserve a useful warm start.

    Factory rewards come from actual simulation. The extra supervised term is
    explicit; this is a custom small-actor PPO profile, not the unmodified
    NVIDIA1024-unit recurrent training configuration.
    """
    torch.manual_seed(seed)
    x=torch.cat([r['obs'] for r in rollout]);actions=torch.cat([r['latent_action'] for r in rollout]);oldlog=torch.cat([r['log_prob'] for r in rollout]);rewards=torch.stack([r['reward'] for r in rollout]).reshape(-1)
    with torch.no_grad():values=model.value(x);adv,returns=gae(rewards,torch.cat((values,values.new_zeros(1))),torch.zeros_like(rewards).index_fill(0,torch.tensor([len(rewards)-1],device=rewards.device),1))
    adv=(adv-adv.mean())/adv.std().clamp_min(1e-6)
    before={k:v.detach().clone() for k,v in model.state_dict().items()};opt=torch.optim.Adam(model.parameters(),lr=1e-5)
    records=[]
    for epoch in range(4):
        dist=model.distribution(x);log=dist.log_prob(actions).sum(-1);ratio=(log-oldlog).exp();policy_loss=-torch.minimum(ratio*adv,ratio.clamp(.8,1.2)*adv).mean()
        anchor=(model.mean(x)-x[:,:6].clamp(-1,1)).square().mean()
        value_loss=(model.value(x)-returns).square().mean()
        loss=policy_loss+.5*value_loss+10*anchor
        opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(model.parameters(),1.);opt.step()
        records.append({'epoch':epoch,'policy_loss':float(policy_loss.detach()),'value_loss':float(value_loss.detach()),'teacher_anchor_loss':float(anchor.detach())})
    change=math.sqrt(sum(float((v-before[k]).square().sum()) for k,v in model.state_dict().items()))
    actor_change=math.sqrt(sum(float((v-before[k]).square().sum()) for k,v in model.state_dict().items() if k.startswith('actor.')))
    if not math.isfinite(change) or actor_change<=1e-10:raise RuntimeError('PPO did not update finite actor weights')
    return {'algorithm':'clipped_PPO_with_explicit_teacher_anchor','actual_simulator_transitions':len(rollout),'reward_sum':float(rewards.sum()),'parameter_l2_change':change,'actor_parameter_l2_change':actor_change,'optimizer_epochs':records}

def save_checkpoint(model,path,metadata):
    torch.save({'state_dict':{k:v.detach().cpu() for k,v in model.state_dict().items()},'observation_schema':OBSERVATION_SCHEMA,'metadata':metadata},path)
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_checkpoint(path,device):
    data=torch.load(path,map_location=device,weights_only=True)
    if data['observation_schema']!=OBSERVATION_SCHEMA:raise ValueError('Checkpoint observation mismatch')
    model=Policy().to(device);model.load_state_dict(data['state_dict']);model.eval();return model,data['metadata']
