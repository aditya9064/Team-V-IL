"""Separate camera-supervised flat-slot simulation, never a real robot driver."""
import argparse
import hashlib
import json
import math
import sys
import time
import traceback
from pathlib import Path

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--starter',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--scenario',choices=['nominal','empty','missing','moved','miss_once','miss_twice','stale','late_arrival','occupied_destination','placed_moved','occluded_view','missing_view'],default='nominal')
    p.add_argument('--colours',nargs='+',default=['yellow','yellow','blue','blue','red','red'])
    p.add_argument('--backend',choices=['geometric','torchvision'],default='geometric')
    p.add_argument('--checkpoint',type=Path)
    p.add_argument('--deployment',type=Path)
    p.add_argument('--training-code',type=Path,default=Path(__file__).resolve().parent.parent/'mentor-training')
    p.add_argument('--device',default='cuda')
    a=p.parse_args();a.out=a.out.resolve();a.starter=a.starter.resolve()
    if a.backend=='torchvision' and (a.checkpoint is None or a.deployment is None):
        p.error('Torchvision backend requires checkpoint and deployment config; no fallback')
    if a.backend=='geometric' and (a.checkpoint is not None or a.deployment is not None):
        p.error('Checkpoint/config provided but torchvision backend not selected')
    if a.out.exists():p.error('Use a fresh output directory; previous trials are preserved')
    a.out.mkdir(parents=True)
    sys.path.insert(1,str(a.starter))
    from supervisor import recipe
    slots=recipe(a.colours)
    source_hashes={n:hashlib.sha256((a.starter/n).read_bytes()).hexdigest()
                   for n in ('cell.py','scene_config.py','sort_scene.py','cube_pose.py','ik.py')}
    report=dict(scenario=a.scenario,task='adaptive_flat_slot_kitting',status='STARTING',
                task_complete=False,slots=slots,completed_slots=[],attempts=[],events=[],
                runtime_inputs='RGB, camera calibration, robot joint feedback and recipe only',
                detector_backend=a.backend,
                truth_used_for_runtime=False,source_sha256=source_hashes,
                limits=['simulation-only provisional thresholds','upright known 10mm cubes',
                        'at most one retry after camera-confirmed empty grasp',
                        'bounded wait; not an unattended production controller'])
    report['runtime_sha256']={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                             for name in ('runner.py','supervisor.py','vision.py','sim_plant.py','trial_harness.py','detector_backend.py','detector_contract.py')}
    def save():
        (a.out/'execution_report.json').write_text(json.dumps(report,indent=2)+'\n')
    def event(state,**details):
        row=dict(state=state,monotonic=time.monotonic(),**details)
        report['events'].append(row);report['status']=state;save()
        print('[dynamic]',json.dumps(row),flush=True)
    save();app=None;error=None
    try:
        # Strip our arguments before Kit parses its own command line.
        sys.argv=[sys.argv[0]]
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True})
        import scene_config as C
        import numpy as np
        from sim_plant import Plant
        from trial_harness import Harness
        from supervisor import (Hold,validate_frame,choose,recheck,matches,pickup_decision,
                                placement_decision)
        from vision import colour_evidence
        from detector_backend import GeometricBackend,TorchvisionBackend
        detector=(GeometricBackend() if a.backend=='geometric' else
                  TorchvisionBackend(a.checkpoint,a.deployment,a.training_code,a.device))
        expected_model_id=detector.model_id
        report['detector_metadata']=detector.metadata;report['model_id']=expected_model_id;save()
        harness=Harness(a.scenario,slots);plant=Plant(a.out,harness,detector=detector)
        report['physics_configuration']=plant.physics_configuration;save()
        completed=[];retry=0;replans=0;waits=0;previous=None
        def check_frame(frame,**kwargs):
            validate_frame(frame,time.monotonic(),expected_model_id=expected_model_id,**kwargs)
        def observe(label,perceive=True):
            nonlocal previous
            frame,images,models=plant.capture(label,perceive)
            check_frame(frame,previous=previous)
            previous=frame
            return frame,images,models
        while len(completed)<len(slots):
            event('OBSERVE',completed_slots=completed)
            frame,_,_=observe('source')
            if a.scenario=='stale':
                frame['captured_monotonic']-=60
                check_frame(frame)
            proposal,state=choose(frame,slots,completed)
            if state=='WAIT_FOR_PART':
                waits+=1;event('WAIT_FOR_PART',required_colour=next(s['colour'] for s in slots if s['slot_id'] not in completed),
                              observations=waits)
                harness.on_wait(plant)
                if waits>=2:break
                continue
            if state=='COMPLETE':break
            event('PROPOSED',proposal=proposal)
            harness.precheck(plant,proposal)
            fresh,_,_=observe('predispatch')
            if not recheck(proposal,fresh):
                replans+=1;event('REPLAN',reason='target_changed_after_proposal',replans=replans)
                if replans>2:raise Hold('target_replan_limit')
                continue
            # Also recheck completed/destination slots in the fresh snapshot.
            choose(fresh,slots,completed)
            target=matches(fresh['detections'],proposal['target']['xy'],proposal['target']['colour'],radius=.003)[0]
            plan=plant.prepare(target,proposal['slot'])
            check_frame(fresh)
            attempt=dict(number=len(report['attempts'])+1,retry=retry,target=target,slot=proposal['slot'],
                         source_observation_id=fresh['observation_id'],pickup_verified=False,
                         placement_verified=False,trajectory_ik_failures=0)
            report['attempts'].append(attempt);harness.begin_attempt()
            event('DISPATCH_PICK',attempt=attempt['number'],source_observation_id=fresh['observation_id'],
                  observation_age_s=time.monotonic()-fresh['captured_monotonic'])
            plant.move(plan,['approach'],attempt)
            open_ref,closed_ref=plant.reference()
            check_frame(open_ref[0],previous=previous)
            check_frame(closed_ref[0],previous=open_ref[0])
            previous=closed_ref[0]
            plant.move(plan,['descend','close','dwell','lift'],attempt)
            event('VERIFY_PICKUP',attempt=attempt['number'])
            harness.before_pickup_check(plant)
            decisions=[]
            for sample in range(2):
                check,images,models=observe('lift-check')
                source=bool(matches(check['detections'],target['xy'],target['colour'],radius=.006))
                _,tip=plant.tip()
                centre=[float(tip[0]),float(tip[1]),float(C.TABLE_TOP_Z+tip[2]+C.CUBE_EDGE/2-C.CUBE_TIP_CLEARANCE)]
                evidence={}
                for view in ('side','wrist'):
                    held=colour_evidence(images[view],models[view],centre,target['colour'],
                                         target['yaw_deg'],reference=closed_ref[1][view])
                    empty=colour_evidence(images[view],models[view],centre,target['colour'],
                                          target['yaw_deg'],reference=open_ref[1][view])
                    held['empty_reference_match']=empty['empty_reference_match']
                    evidence[view]=held
                verdict=pickup_decision(source,evidence);decisions.append(verdict)
                attempt.setdefault('pickup_evidence',[]).append(dict(observation_id=check['observation_id'],
                    source_present=source,held_centre_from_robot_feedback=centre,views=evidence,verdict=verdict))
                save()
            if decisions==['CONFIRMED_EMPTY']*2:
                event('CONFIRMED_EMPTY',attempt=attempt['number'])
                plant.retreat_empty(plan,attempt)
                if retry>=1:raise Hold('empty_grasp_retry_exhausted')
                retry+=1;event('RETRY',retry=retry);continue
            if decisions!=['VERIFIED']*2:raise Hold('pickup_camera_evidence_uncertain')
            attempt['pickup_verified']=True
            event('DISPATCH_PLACE',attempt=attempt['number'])
            plant.move(plan,['carry','lower','release','settle','retreat','home'],attempt)
            frames=[];sides=[]
            event('VERIFY_PLACEMENT',attempt=attempt['number'])
            for sample in range(2):
                check,images,models=observe('placement-check');frames.append(check)
                centre=[*proposal['slot']['xy'],C.TABLE_TOP_Z+C.SURFACE_DZ+C.CUBE_EDGE/2]
                sides.append(colour_evidence(images['side'],models['side'],centre,target['colour'],0.))
            attempt['placement_evidence']=dict(observation_ids=[f['observation_id'] for f in frames],side=sides)
            if not placement_decision(frames,proposal['slot'],sides):raise Hold('placement_camera_evidence_uncertain')
            attempt['placement_verified']=True;completed.append(proposal['slot']['slot_id'])
            report['completed_slots']=completed.copy();retry=0;replans=0;waits=0
            event('SLOT_VERIFIED',slot_id=proposal['slot']['slot_id'])
            harness.after_placement(plant,proposal['slot'])
        if len(completed)==len(slots):
            report['task_complete']=True;event('COMPLETE')
        report['simulator_score']=harness.score(slots,completed)
        report['motion_frames']=plant.command_frames
        report['verified_pickups']=sum(t['pickup_verified'] for t in report['attempts'])
        report['verified_placements']=sum(t['placement_verified'] for t in report['attempts'])
    except Exception as exception:
        from supervisor import Hold
        error=exception
        report['reason']=str(exception)
        event('HOLD_FOR_OPERATOR' if isinstance(exception,Hold) else 'ERROR',reason=str(exception))
        if not isinstance(exception,Hold):
            (a.out/'traceback.txt').write_text(traceback.format_exc());traceback.print_exc()
        if 'plant' in locals():
            try:
                plant.capture('held-final',perceive=False)
            except Exception as capture_error:report['held_capture_error']=str(capture_error)
            # A missing camera must not prevent independent post-run scoring.
            try:report['simulator_score']=harness.score(slots,completed)
            except Exception as scoring_error:report['evaluation_error']=str(scoring_error)
    finally:
        report['verified_pickups']=sum(t['pickup_verified'] for t in report['attempts'])
        report['verified_placements']=sum(t['placement_verified'] for t in report['attempts'])
        report['error_type']=type(error).__name__ if error else None
        save()
        print('TEAM_V_DYNAMIC_RESULT',json.dumps({k:report[k] for k in ('scenario','status','task_complete','verified_pickups','verified_placements')}),flush=True)
        if app is not None:app.close()

if __name__=='__main__':main()
