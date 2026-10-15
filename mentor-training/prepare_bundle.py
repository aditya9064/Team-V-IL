"""Build a portable detector pilot from accepted captures without changing them."""
import argparse
import collections
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path
from PIL import Image, ImageDraw

NAMES=('yellow','blue','red')
SPLITS=('train','val','test')

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def decode_rows(text,w,h):
    boxes=[]
    for row in text.splitlines():
        fields=list(map(float,row.split()))
        if len(fields)!=5 or not all(math.isfinite(v) for v in fields):
            raise ValueError('Malformed label')
        c,x,y,bw,bh=fields
        if c not in (0,1,2) or min(bw,bh)<=0:
            raise ValueError('Unknown class or degenerate box')
        box=[(x-bw/2)*w,(y-bh/2)*h,(x+bw/2)*w,(y+bh/2)*h]
        if not (-.001<=box[0]<box[2]<=w+.001 and -.001<=box[1]<box[3]<=h+.001):
            raise ValueError('Out-of-frame label')
        boxes.append((int(c),box))
    return boxes

def crop_boxes(boxes,rect):
    left,top,right,bottom=rect
    converted=[]
    for c,box in boxes:
        a,b,d,e=box
        if not (left<=a<d<=right and top<=b<e<=bottom):
            raise ValueError('Fixed ROI clips a cube; choose a broader versioned ROI')
        converted.append((c,[a-left,b-top,d-left,e-top]))
    return converted

def assign_diverse(folders):
    grouped=collections.defaultdict(list)
    for folder in folders:
        g=json.loads((folder/'generation.json').read_text())
        grouped[g['cube_count']].append(folder)
    if set(grouped)!={0,1,3,6,8}:
        raise ValueError('Diversity inventory coverage changed')
    assignment={}
    for count,group in grouped.items():
        group.sort(key=lambda p:hashlib.sha256(('split-v2:'+p.name).encode()).hexdigest())
        if len(group)<3:
            raise ValueError('Need independent examples for each inventory in every split')
        assignment[group[0]]='val';assignment[group[1]]='test'
        for folder in group[2:]:assignment[folder]='train'
    return assignment

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--legacy-dataset',type=Path,required=True)
    p.add_argument('--diverse',type=Path,nargs='+',required=True)
    p.add_argument('--tools',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--roi',type=float,nargs=4,default=[.25,.50,.75,.90])
    a=p.parse_args()
    if a.out.exists():raise ValueError('Use a fresh package directory')
    x0,y0,x1,y1=a.roi
    if not (0<=x0<x1<=1 and 0<=y0<y1<=1):raise ValueError('Invalid ROI')
    sys.path.insert(0,str(a.tools));from varied_labels import verify_labels
    legacy=json.loads((a.legacy_dataset/'manifest.json').read_text())
    audit=json.loads((a.legacy_dataset/'label_audit.json').read_text())
    if len(legacy)!=60 or not audit['passed'] or not audit.get('visual_review_complete'):
        raise ValueError('Legacy label acceptance not recorded')
    assignments=[]
    for record in legacy:
        assignments.append((a.samples/record['scene'],record['split'],'legacy',record['image_sha256']))
    diverse=[]
    for root in a.diverse:
        report=json.loads((root/'batch_report.json').read_text())
        review=json.loads((root/'visual_review.json').read_text())
        if not report['complete'] or not review['passed']:raise ValueError('Unaccepted diversity batch')
        diverse.extend(sorted(root.glob('seed-*')))
    if len(diverse)!=18:raise ValueError('Expected accepted 18-scene diversity pilot')
    splits=assign_diverse(diverse)
    assignments.extend((f,splits[f],'diverse',None) for f in diverse)
    validated=[];seen=set();groups=set();hashes=set()
    for folder,split,family,known_hash in assignments:
        g=json.loads((folder/'generation.json').read_text())
        g['cube_count']=len(g['cube_layout'])
        group=f"{family}:{g['seed']}";name=f"{family}-{g['seed']}"
        image_hash=digest(folder/'world_camera.png')
        if known_hash and known_hash!=image_hash:raise ValueError('Legacy image changed')
        if name in seen or group in groups or image_hash in hashes:raise ValueError('Duplicate scene/image or split leakage')
        seen.add(name);groups.add(group);hashes.add(image_hash)
        qa=verify_labels(folder,g,write_outputs=False)
        if family=='legacy':
            text=(a.legacy_dataset/'labels'/split/(folder.name+'.txt')).read_text()
        else:text=(folder/'cube_labels.txt').read_text()
        with Image.open(folder/'world_camera.png') as image:
            image.load();w,h=image.size
            if image.mode!='RGB':raise ValueError('Expected RGB source')
        boxes=decode_rows(text,w,h)
        objects=qa['annotations']['objects']
        if len(boxes)!=len(objects):raise ValueError('Label count changed')
        for (cls,box),obj in zip(boxes,objects):
            if cls!=obj['class_id'] or max(abs(v-t) for v,t in zip(box,obj['bbox_xyxy_pixels']))>.001:
                raise ValueError('Stored labels disagree with independently rendered boxes')
        rect=[int(x0*w),int(y0*h),int(x1*w),int(y1*h)]
        remapped=crop_boxes(boxes,rect)
        record=dict(scene_id=name,group_id=group,split=split,source_family=family,
            seed=g['seed'],cube_count=len(boxes),source_image_sha256=image_hash,
            source_folder_name=folder.name,full_resolution=[w,h],crop_rect_px=rect,
            crop_resolution=[rect[2]-rect[0],rect[3]-rect[1]],
            minimum_silhouette_iou=qa['minimum_silhouette_iou'],
            source_hashes={n:digest(folder/n) for n in ('camera.json','generation.json','scene_state.json','ground_truth.json')})
        validated.append((folder,record,objects,remapped))
        print('[preflight]',name,split,len(boxes),'labels',flush=True)
    # All original images, labels and crop extents validate before export.
    a.out.mkdir(parents=True)
    for split in SPLITS:
        for kind in ('images','labels'):(a.out/kind/split).mkdir(parents=True)
    (a.out/'metadata').mkdir();(a.out/'qa').mkdir()
    tiles={s:[] for s in SPLITS};patches=[];records=[]
    for folder,record,objects,boxes in validated:
        split,name=record['split'],record['scene_id'];rect=record['crop_rect_px']
        with Image.open(folder/'world_camera.png') as image:
            cropped=image.convert('RGB').crop(rect)
        cw,ch=cropped.size;target=a.out/'images'/split/(name+'.png');cropped.save(target)
        rows=[];overlay=cropped.copy();draw=ImageDraw.Draw(overlay)
        for (cls,box),obj in zip(boxes,objects):
            left,top,right,bottom=box
            rows.append(f'{cls} {(left+right)/2/cw:.9f} {(top+bottom)/2/ch:.9f} {(right-left)/cw:.9f} {(bottom-top)/ch:.9f}')
            draw.rectangle(box,outline='lime',width=2)
            draw.text((left,top-13),NAMES[cls],fill='black')
            u,v=round((left+right)/2),round((top+bottom)/2)
            patches.append((name+' '+NAMES[cls],cropped.crop((u-40,v-40,u+40,v+40))))
            obj['bbox_crop_xyxy_pixels']=box
        label=a.out/'labels'/split/(name+'.txt')
        label.write_text('\n'.join(rows)+('\n' if rows else ''))
        # Decode saved PNG/labels and verify exact pixel translation back to source.
        with Image.open(target) as saved:
            saved.load()
            if saved.mode!='RGB' or saved.size!=cropped.size or saved.tobytes()!=cropped.tobytes():
                raise ValueError('Cropped pixel data changed')
        decoded=decode_rows(label.read_text(),cw,ch)
        for (cls,box),obj in zip(decoded,objects):
            if cls!=obj['class_id'] or max(abs(v-t) for v,t in zip(box,obj['bbox_crop_xyxy_pixels']))>.001:
                raise ValueError('Crop label round-trip failed')
        record.update(image_path=f'images/{split}/{name}.png',label_path=f'labels/{split}/{name}.txt',
            crop_image_sha256=digest(target),label_sha256=digest(label))
        metadata=dict(record=record,full_frame_camera=json.loads((folder/'camera.json').read_text()),
            generation=json.loads((folder/'generation.json').read_text()),objects=objects,
            coordinate_rule='crop pixels + crop_rect_px left/top = full camera pixels',
            label_origin='exclusive visible-component xyxy pixel bounds; cubes have bottom-face USD origins')
        (a.out/'metadata'/(name+'.json')).write_text(json.dumps(metadata,indent=2)+'\n')
        record['metadata_path']=f'metadata/{name}.json';records.append(record)
        overlay.thumbnail((480,216));tile=Image.new('RGB',(480,246),'white')
        tile.paste(overlay,(0,30));ImageDraw.Draw(tile).text((5,7),f'{name} | {len(boxes)} cubes',fill='black')
        tiles[split].append(tile)
    for split in SPLITS:
        for start in range(0,len(tiles[split]),12):
            batch=tiles[split][start:start+12];sheet=Image.new('RGB',(1440,246*math.ceil(len(batch)/3)),'white')
            for i,tile in enumerate(batch):sheet.paste(tile,((i%3)*480,(i//3)*246))
            sheet.save(a.out/'qa'/f'{split}-overlays-{start//12+1:02}.png')
    for start in range(0,len(patches),60):
        batch=patches[start:start+60];sheet=Image.new('RGB',(960,110*math.ceil(len(batch)/6)),'white')
        draw=ImageDraw.Draw(sheet)
        for i,(name,patch) in enumerate(batch):
            x,y=(i%6)*160,(i//6)*110;sheet.paste(patch,(x+40,y+25));draw.text((x+5,y+5),name,fill='black')
        sheet.save(a.out/'qa'/f'cube-patches-{start//60+1:02}.png')
    counts={s:sum(r['split']==s for r in records) for s in SPLITS}
    object_counts={s:sum(r['cube_count'] for r in records if r['split']==s) for s in SPLITS}
    negatives={s:sum(r['cube_count']==0 and r['split']==s for r in records) for s in SPLITS}
    if counts!={'train':56,'val':11,'test':11} or sum(object_counts.values())!=432 or set(negatives.values())!={1}:
        raise ValueError('Unexpected release totals')
    (a.out/'manifest.json').write_text(json.dumps(dict(schema_version=2,classes=list(NAMES),records=records),indent=2)+'\n')
    (a.out/'dataset.yaml').write_text('path: .\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: yellow\n  1: blue\n  2: red\n')
    summary=dict(scenes=len(records),cube_labels=sum(object_counts.values()),splits=counts,
        labels_by_split=object_counts,empty_scenes_by_split=negatives,roi_normalized=a.roi,
        source_images_unchanged=True,pixel_and_label_checks_passed=True,visual_crop_review_complete=False,
        release_type='synthetic detector pilot; no real-arm transfer claim',
        labels='visible RGB components verified against bottom-origin USD projection',
        original_split_policy='preserved 48/6/6 legacy assignment',
        diverse_split_policy='each inventory has one val and one test scene; remaining scenes train',
        caveats=['78-scene pilot only','legacy yellow positions correlated across splits',
                 'same simulator/assets across splits','upright 10mm cubes; occlusions rejected'])
    (a.out/'data_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('TEAM_V_CROP_EXPORT_PASS',json.dumps(summary),flush=True)

if __name__=='__main__':main()
