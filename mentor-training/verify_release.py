import argparse
import collections
import hashlib
import json
from pathlib import Path
from detector_data import CubeDataset

def main():
    p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);a=p.parse_args()
    manifest=json.loads((a.data/'manifest.json').read_text())
    seen=set();groups=set();hashes=set();counts={};labels={};empty={}
    for split in ('train','val','test'):
        dataset=CubeDataset(a.data,split,verify_hashes=True)
        counts[split]=len(dataset);labels[split]=empty[split]=0
        for i,r in enumerate(dataset.records):
            image,target=dataset[i]
            assert image.shape== (3,r['crop_resolution'][1],r['crop_resolution'][0])
            assert r['scene_id'] not in seen and r['group_id'] not in groups and r['source_image_sha256'] not in hashes
            seen.add(r['scene_id']);groups.add(r['group_id']);hashes.add(r['source_image_sha256'])
            labels[split]+=len(target['labels']);empty[split]+=len(target['labels'])==0
            metadata=json.loads((a.data/r['metadata_path']).read_text())
            assert len(metadata['objects'])==len(target['labels'])
            for box,obj,cls in zip(target['boxes'].tolist(),metadata['objects'],target['labels'].tolist()):
                assert cls==obj['class_id']+1
                assert max(abs(x-y) for x,y in zip(box,obj['bbox_crop_xyxy_pixels']))<.001
    assert counts=={'train':56,'val':11,'test':11},counts
    assert labels=={'train':324,'val':54,'test':54},labels
    assert empty=={'train':1,'val':1,'test':1},empty
    print('TEAM_V_RELEASE_VERIFY_PASS',json.dumps(dict(images=counts,labels=labels,empty=empty)),flush=True)

if __name__=='__main__':main()
