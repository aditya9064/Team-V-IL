"""COCO metrics include every image, including empty scenes."""
import contextlib
import io
import json
from pathlib import Path
import numpy as np
import torch
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
from detector_data import CLASSES

def ground_truth(dataset):
    images, annotations = [], []
    for i, r in enumerate(dataset.records):
        _, target = dataset[i]
        w, h = r['crop_resolution']
        images.append(dict(id=i, width=w, height=h, file_name=r['image_path']))
        for box, label in zip(target['boxes'].tolist(), target['labels'].tolist()):
            x0,y0,x1,y1 = box
            annotations.append(dict(id=len(annotations)+1, image_id=i, category_id=label,
                                    bbox=[x0,y0,x1-x0,y1-y0], area=(x1-x0)*(y1-y0), iscrowd=0))
    return dict(info={'description':'Team-V synthetic detector pilot'}, images=images,
                annotations=annotations, categories=[dict(id=i+1,name=n) for i,n in enumerate(CLASSES)])

def coco_metrics(gt, predictions, image_ids=None):
    # loadRes([]) is not supported by every pycocotools version.
    with contextlib.redirect_stdout(io.StringIO()):
        actual = COCO(); actual.dataset = gt; actual.createIndex()
        if predictions:
            detected = actual.loadRes(predictions)
        else:
            detected = COCO()
            detected.dataset = dict(images=gt['images'], categories=gt['categories'], annotations=[])
            detected.createIndex()
        evaluator = COCOeval(actual, detected, 'bbox')
        if image_ids is not None: evaluator.params.imgIds = image_ids
        evaluator.evaluate(); evaluator.accumulate(); evaluator.summarize()
    stats = evaluator.stats
    result = {name: (float(value) if value >= 0 else None) for name,value in
              zip(('AP','AP50','AP75','AP_small','AP_medium','AP_large',
                   'AR1','AR10','AR100','AR_small','AR_medium','AR_large'), stats)}
    # Class AP50 is useful because similarly coloured distractors exist.
    precision = evaluator.eval['precision'][0, :, :, 0, -1]
    result['AP50_by_class'] = {}
    for i, name in enumerate(CLASSES):
        values = precision[:,i]; values = values[values >= 0]
        result['AP50_by_class'][name] = float(np.mean(values)) if values.size else None
    return result

@torch.inference_mode()
def evaluate(model, dataset, device, batch_size=2, workers=0, output=None):
    from torch.utils.data import DataLoader
    from detector_data import collate
    if dataset.augment: raise ValueError('Evaluation data cannot be augmented')
    model.eval(); predictions=[]
    loader = DataLoader(dataset, batch_size=batch_size, num_workers=workers,
                        collate_fn=collate, shuffle=False)
    for images, targets in loader:
        results = model([image.to(device) for image in images])
        for result, target in zip(results, targets):
            image_id = int(target['image_id'])
            for box,label,score in zip(result['boxes'].cpu().tolist(),
                                       result['labels'].cpu().tolist(), result['scores'].cpu().tolist()):
                if label not in (1,2,3): raise ValueError('Unexpected model class')
                x0,y0,x1,y1=box
                predictions.append(dict(image_id=image_id,category_id=label,score=score,
                                        bbox=[x0,y0,x1-x0,y1-y0]))
    gt=ground_truth(dataset)
    result=dict(overall=coco_metrics(gt,predictions), by_source={})
    for family in sorted({r['source_family'] for r in dataset.records}):
        ids=[i for i,r in enumerate(dataset.records) if r['source_family']==family]
        result['by_source'][family]=coco_metrics(gt,predictions,ids)
    empty={i for i,r in enumerate(dataset.records) if r['cube_count']==0}
    result['empty_scene_false_positives_at_0_5']=sum(
        p['image_id'] in empty and p['score']>=.5 for p in predictions)
    result['empty_scene_count']=len(empty)
    if output:
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        (output/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
        (output/'predictions.json').write_text(json.dumps(predictions,indent=2)+'\n')
    return result
