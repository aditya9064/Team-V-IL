"""Pure coordinate/class contract for the mentor detector; no simulator inputs."""
import hashlib
import json
import math

CLASSES=('yellow','blue','red')
class DetectorContractError(ValueError):pass

def positive_size(value,name):
    if not isinstance(value,(list,tuple)) or len(value)!=2 or any(type(x) is not int or x<=0 for x in value):
        raise DetectorContractError('invalid_'+name)
    return tuple(value)

def validate_mapping(full_size,crop_rect,model_size):
    full_size=positive_size(full_size,'full_resolution');model_size=positive_size(model_size,'inference_resolution')
    if not isinstance(crop_rect,(list,tuple)) or len(crop_rect)!=4 or any(type(x) is not int for x in crop_rect):
        raise DetectorContractError('invalid_crop_rect')
    left,top,right,bottom=crop_rect
    if not (0<=left<right<=full_size[0] and 0<=top<bottom<=full_size[1]):
        raise DetectorContractError('crop_outside_frame')
    return full_size,tuple(crop_rect),model_size

def validate_deployment(config):
    if config.get('schema_version')!=1 or config.get('architecture')!='fasterrcnn_resnet50_fpn_v2':
        raise DetectorContractError('unsupported_deployment_schema_or_architecture')
    if config.get('classes')!=list(CLASSES) or config.get('foreground_labels')!=[1,2,3]:
        raise DetectorContractError('class_mapping_mismatch')
    validate_mapping(config.get('full_resolution'),config.get('crop_rect_px'),config.get('inference_resolution'))
    threshold=config.get('score_threshold')
    if isinstance(threshold,bool) or not isinstance(threshold,(int,float)) or not math.isfinite(threshold) or not 0<=threshold<=1:
        raise DetectorContractError('validation_selected_score_threshold_required')
    if not isinstance(config.get('threshold_source'),str) or not config['threshold_source'].strip():
        raise DetectorContractError('threshold_provenance_required')
    digest=config.get('checkpoint_sha256')
    if not isinstance(digest,str) or len(digest)!=64 or any(x not in '0123456789abcdef' for x in digest):
        raise DetectorContractError('checkpoint_sha256_required')
    for key in ('min_size','max_size'):
        if type(config.get(key)) is not int or config[key]<=0:raise DetectorContractError('invalid_'+key)
    return dict(config)

def deployment_id(config):
    digest=hashlib.sha256(json.dumps(config,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    return 'fasterrcnn-rgb-silhouette-v1:'+config['checkpoint_sha256']+':'+digest

def map_predictions(output,full_size,crop_rect,model_size,score_threshold):
    """Torchvision labels 1..3 and crop pixels -> full-frame box proposals.

    Resizing is explicit, with no guessed normalized/pixel convention. Invalid
    output fails the frame; valid low-score proposals are recorded as rejected.
    """
    full_size,crop_rect,model_size=validate_mapping(full_size,crop_rect,model_size)
    if isinstance(score_threshold,bool) or not isinstance(score_threshold,(int,float)) or not math.isfinite(score_threshold) or not 0<=score_threshold<=1:
        raise DetectorContractError('invalid_score_threshold')
    if not isinstance(output,dict) or set(('boxes','labels','scores'))-set(output):
        raise DetectorContractError('missing_prediction_fields')
    boxes,labels,scores=(output[k] for k in ('boxes','labels','scores'))
    if any(not isinstance(x,(list,tuple)) for x in (boxes,labels,scores)) or not len(boxes)==len(labels)==len(scores):
        raise DetectorContractError('prediction_shape_mismatch')
    left,top,right,bottom=crop_rect;sx=(right-left)/model_size[0];sy=(bottom-top)/model_size[1]
    accepted=[];rejected=[]
    for i,(box,label,score) in enumerate(zip(boxes,labels,scores)):
        if type(label) is not int or label not in (1,2,3):raise DetectorContractError('invalid_foreground_label')
        if isinstance(score,bool) or not isinstance(score,(int,float)) or not math.isfinite(score) or not 0<=score<=1:
            raise DetectorContractError('invalid_prediction_score')
        if not isinstance(box,(list,tuple)) or len(box)!=4 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in box):
            raise DetectorContractError('invalid_prediction_box')
        x0,y0,x1,y1=box
        if not (0<=x0<x1<=model_size[0] and 0<=y0<y1<=model_size[1]):
            raise DetectorContractError('prediction_box_outside_input')
        mapped=[left+x0*sx,top+y0*sy,left+x1*sx,top+y1*sy]
        item=dict(proposal_id=i,colour=CLASSES[label-1],detector_score=float(score),
                  bbox_full_px=mapped,pixel_xy=[(mapped[0]+mapped[2])/2,(mapped[1]+mapped[3])/2])
        if score<score_threshold:rejected.append(dict(item,rejection_reason='score_below_threshold'))
        else:accepted.append(item)
    return accepted,rejected
