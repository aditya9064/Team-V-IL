"""Create fixed-camera workspace crops, remapped labels, and QA sheets."""
import argparse
import json
import shutil
from pathlib import Path
from PIL import Image, ImageDraw

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    # Broad lower-table region selected from the fixed camera, not GT boxes.
    p.add_argument('--roi', nargs=4, type=float, default=[0.25,0.50,0.75,0.90])
    a = p.parse_args()
    x0,y0,x1,y1 = a.roi
    if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
        raise ValueError('Invalid ROI')
    if a.out.exists():
        raise SystemExit('Choose a fresh output directory')
    records = []
    seen = set()
    for split in ('train','val','test'):
        images = sorted((a.dataset/'images'/split).glob('*.png'))
        if not images:
            raise ValueError('Empty split: '+split)
        for path in images:
            if path.stem in seen:
                raise ValueError('Scene appears in more than one split')
            seen.add(path.stem)
            with Image.open(path) as image:
                image.load()
                w,h = image.size
            rect = (int(x0*w),int(y0*h),int(x1*w),int(y1*h))
            left,top,right,bottom = rect
            rows = []
            for line in (a.dataset/'labels'/split/(path.stem+'.txt')).read_text().splitlines():
                cls,cx,cy,bw,bh = map(float,line.split())
                if cls not in (0,1,2) or not all(0 <= t <= 1 for t in (cx,cy,bw,bh)) or bw*bh <= 0:
                    raise ValueError('Invalid label: '+path.name)
                xmin,ymin,xmax,ymax = (cx-bw/2)*w,(cy-bh/2)*h,(cx+bw/2)*w,(cy+bh/2)*h
                if not (left <= xmin < xmax <= right and top <= ymin < ymax <= bottom):
                    raise ValueError('ROI clips a labeled object: '+path.name+'; widen the fixed ROI')
                rows.append((int(cls), ((xmin+xmax)/2-left)/(right-left),
                             ((ymin+ymax)/2-top)/(bottom-top),
                             (xmax-xmin)/(right-left),(ymax-ymin)/(bottom-top)))
            if len(rows) != 6:
                raise ValueError('Expected six cube labels: '+path.name)
            records.append((split,path,rect,rows))
    a.out.mkdir(parents=True)
    qa = a.out/'qa'
    qa.mkdir()
    names = ('yellow','blue','red')
    counts = {}
    for split in ('train','val','test'):
        (a.out/'images'/split).mkdir(parents=True)
        (a.out/'labels'/split).mkdir(parents=True)
        tiles = []
        for record_split,path,rect,rows in records:
            if record_split != split:
                continue
            with Image.open(path) as image:
                cropped = image.convert('RGB').crop(rect)
            cropped.save(a.out/'images'/split/path.name)
            (a.out/'labels'/split/(path.stem+'.txt')).write_text(''.join(
                f'{cls} {cx:.9f} {cy:.9f} {bw:.9f} {bh:.9f}\n' for cls,cx,cy,bw,bh in rows))
            overlay = cropped.copy()
            draw = ImageDraw.Draw(overlay)
            w,h = overlay.size
            for cls,cx,cy,bw,bh in rows:
                box = ((cx-bw/2)*w,(cy-bh/2)*h,(cx+bw/2)*w,(cy+bh/2)*h)
                draw.rectangle(box,outline='lime',width=3)
                draw.text((box[0],box[1]-12),names[cls],fill='black')
            overlay.save(qa/(path.stem+'-labels.png'))
            overlay.thumbnail((480,240))
            tile = Image.new('RGB',(480,265),'white')
            tile.paste(overlay,(0,25))
            ImageDraw.Draw(tile).text((5,5),path.stem,fill='black')
            tiles.append(tile)
        counts[split] = len(tiles)
        for page in range(0,len(tiles),12):
            batch = tiles[page:page+12]
            sheet = Image.new('RGB',(480*3,265*((len(batch)+2)//3)),'white')
            for i,tile in enumerate(batch):
                sheet.paste(tile,((i%3)*480,(i//3)*265))
            sheet.save(qa/f'{split}-sheet-{page//12+1}.jpg',quality=95)
    (a.out/'dataset.yaml').write_text(
        f'path: {a.out.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: yellow\n  1: blue\n  2: red\n')
    shutil.copy2(a.dataset/'manifest.json',a.out/'source_manifest.json')
    (a.out/'preprocessing.json').write_text(json.dumps({
        'roi_normalized':a.roi,'splits':counts,'source':str(a.dataset.resolve()),
        'camera_specific':True,'labels':'projected amodal cube boxes',
        'visual_qa_complete':False,'pose_labels_remain_in_source':True
    },indent=2))
    print('CROP EXPORT PASSED:',counts)
    print('Inspect all QA sheets; label visibility is not automatically verified.')

if __name__ == '__main__':
    main()
