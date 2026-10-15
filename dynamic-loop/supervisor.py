"""Deterministic camera-only gates for the separate flat-slot kitting task."""
import math

COLOURS = ('yellow', 'blue', 'red')
CALIBRATION_ID = 'fanuc-v05-pinhole-sim-v1'
MODEL_ID = 'geometric-hsv-silhouette-v1'

class Hold(RuntimeError):
    pass

def yaw_error(a,b): return abs((a-b+45)%90-45)

def sensor_identifier(value):
    """Isaac 6 returns Fabric reference-time data instead of an integer."""
    if isinstance(value,dict):
        numerator=int(value['referenceTimeNumerator']);denominator=int(value['referenceTimeDenominator'])
        if denominator<=0:raise Hold('invalid_render_reference_time')
        result=numerator/denominator
    else:result=float(value)
    if not math.isfinite(result):raise Hold('invalid_render_frame_identifier')
    return result

def recipe(colours):
    if not colours or len(colours)>6 or any(c not in COLOURS for c in colours):
        raise ValueError('Recipe needs one to six known cube colours')
    return [dict(slot_id=f'slot-{i}',colour=c,xy=[.21+.044*i,-.27])
            for i,c in enumerate(colours)]

def validate_frame(frame,now,max_age=30.,previous=None):
    if frame.get('calibration_id')!=CALIBRATION_ID or frame.get('model_id')!=MODEL_ID:
        raise Hold('observation_version_mismatch')
    if frame.get('robot_idle') is not True: raise Hold('robot_not_stationary')
    age=now-frame['captured_monotonic']
    if not math.isfinite(age) or age<0 or age>max_age: raise Hold('stale_observation')
    views=frame.get('views',{})
    if set(views)!={'table','side','wrist'}:raise Hold('required_view_missing')
    times=[]
    for name,view in views.items():
        stamp=view.get('sensor_time');seq=view.get('sensor_frame')
        if not isinstance(stamp,(int,float)) or not math.isfinite(stamp):raise Hold('invalid_capture_timestamp')
        if seq is None:raise Hold('missing_frame_sequence')
        if previous and (seq<=previous['views'][name]['sensor_frame'] or
                         stamp<=previous['views'][name]['sensor_time']):
            raise Hold('reused_camera_frame')
        times.append(stamp)
    if max(times)-min(times)>.067:raise Hold('camera_time_skew')
    for obj in frame.get('detections',[]):
        values=[*obj['xy'],obj['yaw_deg'],obj['silhouette_iou']]
        if len(obj['xy'])!=2 or not all(math.isfinite(v) for v in values):raise Hold('invalid_detection')
        if obj['colour'] not in COLOURS or not .85<=obj['silhouette_iou']<=1:
            raise Hold('invalid_detection_quality')

def matches(objects,xy,colour=None,radius=.006):
    return [o for o in objects if (colour is None or o['colour']==colour)
            and math.dist(o['xy'],xy)<=radius]

def choose(frame,slots,completed):
    objects=frame['detections']
    for slot in slots:
        near=matches(objects,slot['xy'],radius=.012)
        if slot['slot_id'] in completed:
            if (len(near)!=1 or near[0]['colour']!=slot['colour'] or
                math.dist(near[0]['xy'],slot['xy'])>.010/3 or yaw_error(near[0]['yaw_deg'],0)>=8):
                raise Hold('previous_placement_lost_or_changed')
        elif near:
            raise Hold('unconfirmed_destination_occupied')
    pending=[s for s in slots if s['slot_id'] not in completed]
    if not pending:return None,'COMPLETE'
    slot=pending[0]
    candidates=[o for o in objects if o['colour']==slot['colour']
                and not any(math.dist(o['xy'],s['xy'])<.020 for s in slots)]
    if not candidates:return None,'WAIT_FOR_PART'
    candidates.sort(key=lambda o:(o['xy'][0],o['xy'][1]))
    target=candidates[0]
    if len(matches(objects,target['xy'],radius=.012))!=1:raise Hold('ambiguous_target')
    return dict(target=target,slot=slot,observation_id=frame['observation_id'],
                captured_monotonic=frame['captured_monotonic']),'PROPOSED'

def recheck(proposal,frame):
    candidates=matches(frame['detections'],proposal['target']['xy'],
                       proposal['target']['colour'],radius=.003)
    if len(candidates)!=1:return False
    return yaw_error(candidates[0]['yaw_deg'],proposal['target']['yaw_deg'])<=3.

def pickup_decision(source_present,held_by_view):
    """Positive held evidence is mandatory; absence alone never proves pickup."""
    if any(v.get('wrong_colour') for v in held_by_view.values()):return 'UNCERTAIN'
    positive=any(v.get('positive') is True for v in held_by_view.values())
    empty=any(v.get('empty_reference_match') is True for v in held_by_view.values())
    if source_present is False and positive:return 'VERIFIED'
    if source_present is True and empty and not positive:return 'CONFIRMED_EMPTY'
    return 'UNCERTAIN'

def placement_decision(frames,slot,side_evidence):
    if len(frames)<2 or len(side_evidence)!=len(frames):return False
    if len({f['observation_id'] for f in frames})!=len(frames):return False
    for before,after in zip(frames,frames[1:]):
        if any(after['views'][view]['sensor_frame']<=before['views'][view]['sensor_frame'] or
               after['views'][view]['sensor_time']<=before['views'][view]['sensor_time']
               for view in ('table','side','wrist')):return False
    for frame,side in zip(frames,side_evidence):
        near=matches(frame['detections'],slot['xy'],radius=.010/3)
        if len(near)!=1 or near[0]['colour']!=slot['colour'] or yaw_error(near[0]['yaw_deg'],0)>=8:
            return False
        if side.get('positive') is not True or side.get('wrong_colour'):return False
    return True
