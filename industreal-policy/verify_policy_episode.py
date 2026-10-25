"""CPU-only independent learned-action and strict released-insertion verifier."""
import json,hashlib,math,sys
from pathlib import Path
import torch

def mul(a,b):
 w,x,y,z=a;v,i,j,k=b
 return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]
def unit(q):
 n=math.sqrt(sum(x*x for x in q));assert n>1e-8;return [x/n for x in q]
def rotate(q,v):
 q=unit(q);r=mul(mul(q,[0.]+v),[q[0],-q[1],-q[2],-q[3]]);return r[1:]
def rot_error(q,goal):
 q=unit(q);d=mul(unit(goal),[q[0],-q[1],-q[2],-q[3]])
 if d[0]<0:d=[-v for v in d]
 n=math.sqrt(sum(v*v for v in d[1:]));s=2*math.atan2(n,max(0,d[0]))/n if n>1e-8 else 2
 return [v*s for v in d[1:]]
def norm(v):return math.sqrt(sum(x*x for x in v))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(directory,checkpoint):
 p=Path(directory);r=json.loads((p/'insertion-report.json').read_text());rows=[json.loads(x) for x in (p/'measured-state.jsonl').read_text().splitlines()][1:]
 assert r['status']=='LEARNED_POLICY_INSERTION_RELEASE_PASS',r.get('error')
 assert r['fixed_insertion_goal'] is True and r['deterministic_action_fallback'] is False
 assert r['policy_checkpoint_sha256']==sha(checkpoint)
 data=torch.load(checkpoint,map_location='cpu',weights_only=True);sd=data['state_dict'];w=sd['actor.weight'].tolist();b=sd['actor.bias'].tolist()
 assert w==r['input_actor_parameters']['weight'] and b==r['input_actor_parameters']['bias']
 assert data['metadata']['fitted_policy'] is True
 if r['split']!='train':assert data['metadata']['ppo_updates']>=2
 assert r['policy_trained'] is True and r['physical_hardware_tested'] is False
 assert len(rows)==331 and r['simulation_steps']==330 and r['physics_steps']==5280
 assert abs(r['provenance']['physics_dt_s']-1/240)<1e-10
 assert r['provenance']['maximum_table_displacement_m']<=1e-6
 calibration=json.loads((p/'privileged-frozen-calibration.json').read_text())['values'];goal_q=calibration['goal_tcp_quaternion_wxyz'][0]
 offset=rotate(goal_q,calibration['grasp_position_tcp_m'][0]);goal_p=[v+.00005*n-o for v,n,o in zip(calibration['fixture_position_m'][0],calibration['fixture_axis'][0],offset)]
 final=[];fixed=[];previous=[0.]*6;max_mean_error=0.;max_obs_error=0.;max_penetration=0.
 for i,s in enumerate(rows):
  assert s['step']==i and not s['terminated'] and not s['truncated'] and s['state_phase']=='CONTINUOUS_EPISODE'
  assert abs(s['elapsed_since_reset_s']-i/15)<1e-8
  for a in [s['contact_audit'],s['tool_contact_audit']]:
   assert not a['errors'] and not a['unexpected_contacts']
   assert a['tensor_frames']==16*i and a['force_crosscheck_frames']==a['tensor_frames']
   assert a['max_contact_depth_m']<=.00025
   max_penetration=max(max_penetration,a['max_contact_depth_m'])
  assert max(s['tool_contact_audit']['maximum_pair_normal_force_n'].values(),default=0.)<=.001
  if i:
   assert s['action_source']=='LEARNED_POLICY' and s['policy_checkpoint_sha256']==sha(checkpoint)
   x=s['actor_observation'];expected=[max(-1,min(1,sum(v*y for v,y in zip(ww,x))+bb)) for ww,bb in zip(w,b)]
   err=max(abs(a-c) for a,c in zip(expected,s['actor_mean']));max_mean_error=max(max_mean_error,err);assert err<2e-5
   assert all(abs(a-max(-1,min(1,c)))<1e-7 for a,c in zip(s['applied_action'],s['latent_action']))
   if r['split']!='train':assert not s['exploration'] and max(abs(a-c) for a,c in zip(s['applied_action'],s['actor_mean']))<1e-7
   before=rows[i-1];error=[(a-c)/.002 for a,c in zip(s['policy_goal_position_env_m'],before['tcp_position_env_m'])]+[v/.01 for v in rot_error(before['tcp_quaternion_wxyz'],s['policy_goal_quaternion_wxyz'])]
   err=max(abs(a-c) for a,c in zip(error,x[:6]));max_obs_error=max(max_obs_error,err);assert err<.002
   fs=before['contact_audit']['last_pair_normal_force_world_n'];forces=[sum(fs.get(k,[0.,0.,0.])[j] for k in names) for names in [('FixedAsset','Table'),('JawL','JawR')] for j in range(3)]
   assert max(abs(max(-50,min(50,a))-c) for a,c in zip(forces,x[6:12]))<1e-5
   assert max(abs(a-c) for a,c in zip(previous,x[12:]))<1e-7;previous=s['applied_action']
  if s['insertion_phase']=='LEARNED_INSERT':
   fixed.append(s['policy_goal_position_env_m']+s['policy_goal_quaternion_wxyz']);assert s['maximum_translation_step_m']==.0004
   assert max(abs(a-c) for a,c in zip(goal_p,s['policy_goal_position_env_m']))<1e-7
  if s['insertion_phase']!='RELEASED_VERIFY':continue
  q=s['fixed_quaternion_wxyz'];inv=[q[0],-q[1],-q[2],-q[3]]
  rel=rotate(inv,[a-c for a,c in zip(s['held_position_env_m'],s['fixed_position_env_m'])]);axis=rotate(inv,rotate(s['held_quaternion_wxyz'],[0.,0.,1.]));radial=math.hypot(*rel[:2]);tilt=math.acos(max(-1,min(1,axis[2])))
  assert radial<=.000057000001 and abs(rel[2])<=.001 and tilt<=math.pi/180
  joints=s['measured_joint_positions'];assert min(.006+joints[6],.006-joints[7])>=.0055
  a=s['contact_audit'];forces=a['last_pair_normal_force_world_n'];assert s['release_requested'] and a['socket_contact_observed']
  assert all(norm(forces[k])<=.001 for k in ['JawL','JawR'])
  support=sum(forces[k][2] for k in ['Table','FixedAsset']);assert .8*.019*9.81<=support<=1.2*.019*9.81
  assert norm(s['peg_linear_velocity_m_s'])<=.01 and norm(s['peg_angular_velocity_rad_s'])<=.1
  final.append({'time_s':s['elapsed_since_reset_s'],'radial_m':radial,'depth_m':rel[2],'tilt_rad':tilt,'support_n':support,'angular_rad_s':norm(s['peg_angular_velocity_rad_s'])})
 assert len(fixed)==150 and all(max(abs(a-c) for a,c in zip(t,fixed[0]))<1e-9 for t in fixed)
 assert len(final)==15 and final[-1]['time_s']-final[0]['time_s']>=.5
 assert len([s for s in rows if s['insertion_phase']=='RELEASE_SETTLE'])==45
 before=rows[180];assert before['insertion_phase']=='LEARNED_INSERT'
 axis=rotate(rows[-1]['fixed_quaternion_wxyz'],[0.,0.,1.]);withdraw=sum((a-c)*n for a,c,n in zip(rows[-1]['tcp_position_env_m'],before['tcp_position_env_m'],axis));assert withdraw>=.039
 return {'state':'INDEPENDENT_LEARNED_POLICY_CYCLE_PASS','seed':r['seed'],'split':r['split'],'checkpoint_sha256':sha(checkpoint),'trace_sha256':sha(p/'measured-state.jsonl'),'source_manifest_sha256':r['implementation_manifest_sha256'],'fixed_goal_steps':150,'physics_frames':5280,'maximum_final_radial_m':max(s['radial_m'] for s in final),'maximum_final_angular_rad_s':max(s['angular_rad_s'] for s in final),'maximum_monitored_penetration_m':max_penetration,'support_force_range_n':[min(s['support_n'] for s in final),max(s['support_n'] for s in final)],'measured_withdrawal_m':withdraw,'released_hold_s':final[-1]['time_s']-final[0]['time_s'],'maximum_actor_mean_reconstruction_error':max_mean_error,'maximum_actor_observation_reconstruction_error':max_obs_error,'physical_hardware_tested':False,'full_module_assembly_verified':False}
if __name__=='__main__':
 result=verify(sys.argv[1],sys.argv[2]);Path(sys.argv[1]+'/independent-verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
